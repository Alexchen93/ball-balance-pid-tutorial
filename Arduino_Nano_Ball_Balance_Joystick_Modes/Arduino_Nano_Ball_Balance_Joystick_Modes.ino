/*
 * Ball-and-Plate controller: PC camera -> USB serial -> Arduino Nano PID
 *
 * PC-to-Nano protocol (ASCII, one command per LF-terminated line):
 *   POS,<x_mm>,<y_mm>,<camera_timestamp_ms>
 *   LOST
 *   RUN | READY | TARGET,<x_mm>,<y_mm>
 *   PIDX,<kp>,<ki>,<kd> | PIDY,<kp>,<ki>,<kd> | PING
 *
 * Nano telemetry (10 Hz):
 *   TEL,<state>,<link>,<x>,<y>,<error_x>,<error_y>,<u_x>,<u_y>,<servo_x>,<servo_y>,<age_ms>
 *
 * Hardware TEST extension: 5-pin analog joystick on A0/A1/D2.
 * Hold joystick SW for 1.5 s in WAIT/READY to enter TEST without a PC.
 * In TEST, each joystick deflection triggers exactly ONE movement. The stick
 * must return to center before another movement is accepted. Short-press SW
 * toggles MODE 1 = MICRO STEP (1 degree) and MODE 2 = ENDPOINT. In ENDPOINT
 * mode a single deflection drives the selected X or Y axis directly to
 * MIN/MAX. Long-press SW in TEST returns both axes to the mechanical center.
 *
 * Libraries to install from Arduino Library Manager:
 *   Servo
 */

#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t SERVO_X_PIN = 9;
constexpr uint8_t SERVO_Y_PIN = 10;
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;

// 5-pin analog joystick module (KY-023 / PS2-style): GND, +5V, VRx, VRy, SW.
constexpr uint8_t JOYSTICK_X_PIN = A0;
constexpr uint8_t JOYSTICK_Y_PIN = A1;
constexpr uint8_t JOYSTICK_SW_PIN = 2;
constexpr int JOYSTICK_X_DIRECTION = 1;
constexpr int JOYSTICK_Y_DIRECTION = 1;
constexpr int JOYSTICK_DEADZONE = 100;
constexpr uint16_t JOYSTICK_ACTION_DELAY_MS = 350;  // Minimum time between accepted moves.
constexpr uint16_t JOYSTICK_LONG_PRESS_MS = 1500;
constexpr uint16_t JOYSTICK_DEBOUNCE_MS = 30;

// Mechanical calibration: set these before enabling RUN on the real platform.
constexpr int SERVO_X_CENTER = 90;
constexpr int SERVO_Y_CENTER = 90;
constexpr int SERVO_MIN_ANGLE = 70;
constexpr int SERVO_MAX_ANGLE = 110;
constexpr int SERVO_X_DIRECTION = 1;  // Change to -1 if the X correction is reversed.
constexpr int SERVO_Y_DIRECTION = 1;  // Change to -1 if the Y correction is reversed.

constexpr float POSITION_LIMIT_MM = 150.0f;
constexpr float PID_OUTPUT_LIMIT_DEG = 8.0f;
constexpr float PID_INTEGRAL_LIMIT = 60.0f;
constexpr uint32_t POSITION_TIMEOUT_MS = 300UL;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100UL;
// Serial Monitor quiet by default. TELON/TELOFF can change this at runtime.
bool autoTelemetryEnabled = false;
constexpr uint32_t SATURATION_WARNING_MS = 2000UL;

// Conservative first-power-on values. Tune X and Y independently on hardware.
constexpr float DEFAULT_KP_X = 0.10f;
constexpr float DEFAULT_KI_X = 0.00f;
constexpr float DEFAULT_KD_X = 0.25f;
constexpr float DEFAULT_KP_Y = 0.10f;
constexpr float DEFAULT_KI_Y = 0.00f;
constexpr float DEFAULT_KD_Y = 0.25f;

enum ControllerState : uint8_t { WAIT_LINK, READY, RUN, TEST };
enum LinkState : uint8_t { LINK_WAIT, LINK_OK, BALL_LOST, LINK_LOST, POSITION_RANGE_ERROR };
enum TestMoveMode : uint8_t { TEST_MICRO_STEP, TEST_ENDPOINT };

struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousMeasurement;
  bool hasPreviousMeasurement;

  PIDController(float p, float i, float d)
      : kp(p), ki(i), kd(d), integral(0.0f), previousMeasurement(0.0f),
        hasPreviousMeasurement(false) {}

  void reset() {
    integral = 0.0f;
    hasPreviousMeasurement = false;
  }

  float update(float target, float measurement, float dtSeconds) {
    const float error = target - measurement;
    const float derivative = hasPreviousMeasurement
                                 ? -(measurement - previousMeasurement) / dtSeconds
                                 : 0.0f;
    const float candidateIntegral = constrain(
        integral + error * dtSeconds, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    const float unconstrained = kp * error + ki * candidateIntegral + kd * derivative;
    const float output = constrain(unconstrained, -PID_OUTPUT_LIMIT_DEG, PID_OUTPUT_LIMIT_DEG);

    // Conditional integration prevents windup while an output is angle-limited.
    if (output == unconstrained || error * output < 0.0f) {
      integral = candidateIntegral;
    }
    previousMeasurement = measurement;
    hasPreviousMeasurement = true;
    return output;
  }
};

Servo servoX;
Servo servoY;
PIDController pidX(DEFAULT_KP_X, DEFAULT_KI_X, DEFAULT_KD_X);
PIDController pidY(DEFAULT_KP_Y, DEFAULT_KI_Y, DEFAULT_KD_Y);

ControllerState controllerState = WAIT_LINK;
LinkState linkState = LINK_WAIT;
bool newPositionAvailable = false;
bool servoSaturated = false;

float ballX = 0.0f;
float ballY = 0.0f;
float targetX = 0.0f;
float targetY = 0.0f;
float errorX = 0.0f;
float errorY = 0.0f;
float outputX = 0.0f;
float outputY = 0.0f;
int servoAngleX = SERVO_X_CENTER;
int servoAngleY = SERVO_Y_CENTER;

uint32_t lastPositionMs = 0;
uint32_t lastPidUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t saturationStartedMs = 0;

int joystickCenterX = 512;
int joystickCenterY = 512;
int joystickRawX = 512;
int joystickRawY = 512;
uint32_t lastJoystickMoveMs = 0;
bool joystickMoveArmed = true;
bool joystickStablePressed = false;
bool joystickLastRawPressed = false;
bool joystickLongHandled = false;
uint32_t joystickDebounceChangedMs = 0;
uint32_t joystickPressStartedMs = 0;

// Manual hardware-test calibration values. These are RAM-only candidates;
// use SHOW to print the constants that should be copied into the production
// calibration section after the platform is mechanically level.
int testCenterX = SERVO_X_CENTER;
int testCenterY = SERVO_Y_CENTER;
TestMoveMode testMoveMode = TEST_MICRO_STEP;
constexpr int TEST_MICRO_STEP_DEG = 1;
constexpr int TEST_OFFSET_DEG = 3;
constexpr uint16_t TEST_HOLD_MS = 700;
constexpr uint16_t TEST_RETURN_MS = 500;

char serialLine[96];
uint8_t serialLineLength = 0;

const char *controllerStateName(ControllerState state) {
  switch (state) {
    case WAIT_LINK: return "WAIT";
    case READY: return "READY";
    case RUN: return "RUN";
    case TEST: return "TEST";
  }
  return "?";
}

const char *linkStateName(LinkState state) {
  switch (state) {
    case LINK_WAIT: return "WAIT";
    case LINK_OK: return "OK";
    case BALL_LOST: return "BALL";
    case LINK_LOST: return "LOST";
    case POSITION_RANGE_ERROR: return "RANGE";
  }
  return "?";
}


void resetControllers();
bool positionIsFresh();
const char *controllerStateName(ControllerState state);
void printTestStatus();
void enterTestMode();

void writeNeutralServos() {
  servoAngleX = constrain(SERVO_X_CENTER, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoAngleY = constrain(SERVO_Y_CENTER, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoX.write(servoAngleX);
  servoY.write(servoAngleY);
}

void resetControllers() {
  pidX.reset();
  pidY.reset();
  errorX = 0.0f;
  errorY = 0.0f;
  outputX = 0.0f;
  outputY = 0.0f;
  lastPidUs = 0;
  servoSaturated = false;
  saturationStartedMs = 0;
}

void stopControl(LinkState reason) {
  const bool wasRunning = controllerState == RUN;
  controllerState = (reason == LINK_WAIT) ? WAIT_LINK : READY;
  linkState = reason;
  newPositionAvailable = false;
  resetControllers();
  writeNeutralServos();
  if (wasRunning) {
    Serial.println(F("STATE,READY"));
  }
}

bool positionIsFresh() {
  return linkState == LINK_OK && (uint32_t)(millis() - lastPositionMs) <= POSITION_TIMEOUT_MS;
}


void writeManualServoX(int angle) {
  servoAngleX = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoX.write(servoAngleX);
}

void writeManualServoY(int angle) {
  servoAngleY = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoY.write(servoAngleY);
}

void writeManualBoth(int xAngle, int yAngle) {
  writeManualServoX(xAngle);
  writeManualServoY(yAngle);
}


void calibrateJoystickCenter() {
  long sumX = 0;
  long sumY = 0;
  constexpr uint8_t samples = 32;
  for (uint8_t i = 0; i < samples; ++i) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    delay(2);
  }
  joystickCenterX = (int)(sumX / samples);
  joystickCenterY = (int)(sumY / samples);
  joystickRawX = joystickCenterX;
  joystickRawY = joystickCenterY;
  Serial.print(F("JOYCAL,CX=")); Serial.print(joystickCenterX);
  Serial.print(F(",CY=")); Serial.println(joystickCenterY);
}

int joystickDirectionFromAxis(int raw, int center, int direction) {
  const int delta = (raw - center) * direction;
  if (abs(delta) <= JOYSTICK_DEADZONE) return 0;
  return delta > 0 ? 1 : -1;
}

const char *testMoveModeName(TestMoveMode mode) {
  switch (mode) {
    case TEST_MICRO_STEP: return "MODE1";
    case TEST_ENDPOINT: return "MODE2";
  }
  return "?";
}

void printTestModeGuide() {
  if (testMoveMode == TEST_MICRO_STEP) {
    Serial.println(F("MODE 1: MICRO STEP (1 degree)"));
  } else {
    Serial.print(F("MODE 2: ENDPOINT ("));
    Serial.print(SERVO_MIN_ANGLE);
    Serial.print(F("-"));
    Serial.print(SERVO_MAX_ANGLE);
    Serial.println(F(")"));
  }
  Serial.print(F("LIMITS,SERVO_MIN_ANGLE="));
  Serial.print(SERVO_MIN_ANGLE);
  Serial.print(F(",SERVO_MAX_ANGLE="));
  Serial.println(SERVO_MAX_ANGLE);
  Serial.println(F("JOY,ONE_DEFLECTION_ONE_AXIS,RETURN_CENTER_TO_REARM"));
  Serial.println(F("JOYBTN,SHORT=SWITCH_MODE,LONG=CENTER_BOTH_AXES"));
}

void selectTestMoveMode(TestMoveMode mode) {
  testMoveMode = mode;
}

void cycleTestMoveMode() {
  selectTestMoveMode(testMoveMode == TEST_MICRO_STEP ? TEST_ENDPOINT : TEST_MICRO_STEP);
  Serial.print(F("JOYMODE,"));
  Serial.println(testMoveModeName(testMoveMode));
  printTestModeGuide();
}

void applyOneJoystickMove(bool xAxis, int direction) {
  if (direction == 0) return;

  if (testMoveMode == TEST_ENDPOINT) {
    const int target = direction > 0 ? SERVO_MAX_ANGLE : SERVO_MIN_ANGLE;
    if (xAxis) writeManualServoX(target);
    else writeManualServoY(target);
  } else {
    const int delta = direction * TEST_MICRO_STEP_DEG;
    if (xAxis) writeManualServoX(servoAngleX + delta);
    else writeManualServoY(servoAngleY + delta);
  }

  Serial.print(F("JOYMOVE,"));
  Serial.print(xAxis ? 'X' : 'Y');
  Serial.print(direction > 0 ? F(",POS,") : F(",NEG,"));
  Serial.print(F("ANGLE="));
  Serial.println(xAxis ? servoAngleX : servoAngleY);
  printTestStatus();
}

void updateJoystickTestControl() {
  joystickRawX = analogRead(JOYSTICK_X_PIN);
  joystickRawY = analogRead(JOYSTICK_Y_PIN);
  if (controllerState != TEST) return;

  const int dirX = joystickDirectionFromAxis(joystickRawX, joystickCenterX, JOYSTICK_X_DIRECTION);
  const int dirY = joystickDirectionFromAxis(joystickRawY, joystickCenterY, JOYSTICK_Y_DIRECTION);
  const bool centered = dirX == 0 && dirY == 0;
  const uint32_t nowMs = millis();

  // A movement is not re-armed until the stick has returned to center.
  if (centered) {
    if (!joystickMoveArmed &&
        (uint32_t)(nowMs - lastJoystickMoveMs) >= JOYSTICK_ACTION_DELAY_MS) {
      joystickMoveArmed = true;
    }
    return;
  }

  if (!joystickMoveArmed) return;
  if ((uint32_t)(nowMs - lastJoystickMoveMs) < JOYSTICK_ACTION_DELAY_MS) return;

  // If the stick is diagonal, only the dominant axis is accepted so X/Y never
  // move at the same time during mechanical calibration.
  const int dx = abs(joystickRawX - joystickCenterX);
  const int dy = abs(joystickRawY - joystickCenterY);
  if (dx >= dy && dirX != 0) {
    applyOneJoystickMove(true, dirX);
  } else if (dirY != 0) {
    applyOneJoystickMove(false, dirY);
  }

  joystickMoveArmed = false;
  lastJoystickMoveMs = nowMs;
}

void updateJoystickButton() {
  const uint32_t nowMs = millis();
  const bool rawPressed = digitalRead(JOYSTICK_SW_PIN) == LOW;

  if (rawPressed != joystickLastRawPressed) {
    joystickLastRawPressed = rawPressed;
    joystickDebounceChangedMs = nowMs;
  }
  if ((uint32_t)(nowMs - joystickDebounceChangedMs) < JOYSTICK_DEBOUNCE_MS) return;

  if (rawPressed != joystickStablePressed) {
    joystickStablePressed = rawPressed;
    if (joystickStablePressed) {
      joystickPressStartedMs = nowMs;
      joystickLongHandled = false;
    } else if (controllerState == TEST && !joystickLongHandled) {
      cycleTestMoveMode();
      joystickMoveArmed = false;
      lastJoystickMoveMs = nowMs;
      Serial.println(F("JOYBTN,NEXT_MODE"));
      printTestStatus();
    }
  }

  if (joystickStablePressed && !joystickLongHandled &&
      (uint32_t)(nowMs - joystickPressStartedMs) >= JOYSTICK_LONG_PRESS_MS) {
    joystickLongHandled = true;
    if (controllerState == TEST) {
      writeManualBoth(testCenterX, testCenterY);
      joystickMoveArmed = false;
      lastJoystickMoveMs = nowMs;
      Serial.println(F("JOYBTN,CENTER"));
      printTestStatus();
    } else if (controllerState != RUN) {
      enterTestMode();
    }
  }
}

void printTestStatus() {
  Serial.print(F("TESTSTAT,X="));
  Serial.print(servoAngleX);
  Serial.print(F(",Y="));
  Serial.print(servoAngleY);
  Serial.print(F(",CX="));
  Serial.print(testCenterX);
  Serial.print(F(",CY="));
  Serial.print(testCenterY);
  Serial.print(F(",MODE="));
  Serial.print(testMoveModeName(testMoveMode));
  Serial.print(F(",STEP="));
  if (testMoveMode == TEST_ENDPOINT) Serial.print(F("ENDPOINT"));
  else Serial.print(TEST_MICRO_STEP_DEG);
  Serial.print(F(",ARM="));
  Serial.println(joystickMoveArmed ? 1 : 0);
}

void printTestCalibration() {
  Serial.println(F("----- COPY TO CALIBRATION CONSTANTS -----"));
  Serial.print(F("constexpr int SERVO_X_CENTER = "));
  Serial.print(testCenterX);
  Serial.println(F(";"));
  Serial.print(F("constexpr int SERVO_Y_CENTER = "));
  Serial.print(testCenterY);
  Serial.println(F(";"));
  Serial.print(F("constexpr int SERVO_MIN_ANGLE = "));
  Serial.print(SERVO_MIN_ANGLE);
  Serial.println(F(";"));
  Serial.print(F("constexpr int SERVO_MAX_ANGLE = "));
  Serial.print(SERVO_MAX_ANGLE);
  Serial.println(F(";"));
  Serial.println(F("-----------------------------------------"));
}

void enterTestMode() {
  resetControllers();
  controllerState = TEST;
  testCenterX = SERVO_X_CENTER;
  testCenterY = SERVO_Y_CENTER;
  selectTestMoveMode(TEST_MICRO_STEP);
  writeManualBoth(testCenterX, testCenterY);
  calibrateJoystickCenter();
  joystickMoveArmed = true;
  lastJoystickMoveMs = millis() - JOYSTICK_ACTION_DELAY_MS;
  Serial.println(F("STATE,TEST"));
  printTestModeGuide();
  printTestStatus();
}

void exitTestMode() {
  resetControllers();
  writeNeutralServos();
  controllerState = positionIsFresh() ? READY : WAIT_LINK;
  Serial.print(F("STATE,"));
  Serial.println(controllerStateName(controllerState));
}

void runManualAxisTest(bool xAxis) {
  const int center = xAxis ? testCenterX : testCenterY;
  const int positive = constrain(center + TEST_OFFSET_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  const int negative = constrain(center - TEST_OFFSET_DEG, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  Serial.print(F("MECHTEST,"));
  Serial.print(xAxis ? 'X' : 'Y');
  Serial.println(F(",START"));

  if (xAxis) writeManualServoX(center); else writeManualServoY(center);
  delay(TEST_RETURN_MS);
  if (xAxis) writeManualServoX(positive); else writeManualServoY(positive);
  delay(TEST_HOLD_MS);
  if (xAxis) writeManualServoX(center); else writeManualServoY(center);
  delay(TEST_RETURN_MS);
  if (xAxis) writeManualServoX(negative); else writeManualServoY(negative);
  delay(TEST_HOLD_MS);
  if (xAxis) writeManualServoX(center); else writeManualServoY(center);
  delay(TEST_RETURN_MS);

  Serial.print(F("MECHTEST,"));
  Serial.print(xAxis ? 'X' : 'Y');
  Serial.println(F(",DONE"));
}

bool parseFloat(const char *text, float &value) {
  if (text == NULL) {
    return false;
  }
  char *end = NULL;
  value = (float)strtod(text, &end);
  return end != text && *end == '\0' && !isnan(value) && !isinf(value);
}

bool parseUnsigned(const char *text, uint32_t &value) {
  if (text == NULL) {
    return false;
  }
  char *end = NULL;
  value = strtoul(text, &end, 10);
  return end != text && *end == '\0';
}

void acceptPosition(float x, float y, uint32_t cameraTimestampMs) {
  (void)cameraTimestampMs;  // Nano uses arrival time for its real PID delta-time.
  if (fabs(x) > POSITION_LIMIT_MM || fabs(y) > POSITION_LIMIT_MM) {
    Serial.println(F("ERROR,POSITION_OUT_OF_RANGE"));
    stopControl(POSITION_RANGE_ERROR);
    return;
  }

  ballX = x;
  ballY = y;
  lastPositionMs = millis();
  linkState = LINK_OK;
  newPositionAvailable = true;
  if (controllerState == WAIT_LINK) {
    controllerState = READY;
    Serial.println(F("STATE,READY"));
  }
}

void updateServos() {
  const float requestedX = SERVO_X_CENTER + SERVO_X_DIRECTION * outputX;
  const float requestedY = SERVO_Y_CENTER + SERVO_Y_DIRECTION * outputY;
  servoAngleX = constrain((int)round(requestedX), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoAngleY = constrain((int)round(requestedY), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoX.write(servoAngleX);
  servoY.write(servoAngleY);

  servoSaturated = servoAngleX == SERVO_MIN_ANGLE || servoAngleX == SERVO_MAX_ANGLE ||
                    servoAngleY == SERVO_MIN_ANGLE || servoAngleY == SERVO_MAX_ANGLE;
  if (servoSaturated && saturationStartedMs == 0) {
    saturationStartedMs = millis();
  }
  if (!servoSaturated) {
    saturationStartedMs = 0;
  }
}

void updatePidForPosition() {
  if (!newPositionAvailable || controllerState != RUN) {
    return;
  }
  newPositionAvailable = false;
  const uint32_t nowUs = micros();
  const float dtSeconds = lastPidUs == 0
                              ? 0.033f
                              : constrain((nowUs - lastPidUs) / 1000000.0f, 0.010f, 0.150f);
  lastPidUs = nowUs;
  errorX = targetX - ballX;
  errorY = targetY - ballY;
  outputX = pidX.update(targetX, ballX, dtSeconds);
  outputY = pidY.update(targetY, ballY, dtSeconds);
  updateServos();
}

void checkSafetyTimeout() {
  if (controllerState != TEST && linkState == LINK_OK && !positionIsFresh()) {
    stopControl(LINK_LOST);
  }
}

void printTelemetry() {
  const uint32_t ageMs = positionIsFresh() ? millis() - lastPositionMs : 9999UL;
  Serial.print(F("TEL,"));
  Serial.print(controllerStateName(controllerState));
  Serial.print(','); Serial.print(linkStateName(linkState));
  Serial.print(','); Serial.print(ballX, 1);
  Serial.print(','); Serial.print(ballY, 1);
  Serial.print(','); Serial.print(errorX, 1);
  Serial.print(','); Serial.print(errorY, 1);
  Serial.print(','); Serial.print(outputX, 2);
  Serial.print(','); Serial.print(outputY, 2);
  Serial.print(','); Serial.print(servoAngleX);
  Serial.print(','); Serial.print(servoAngleY);
  Serial.print(','); Serial.println(ageMs);
}


void updateStatusLed() {
  if (controllerState == TEST) {
    digitalWrite(STATUS_LED_PIN, (millis() / 200UL) % 2U);
  } else if (controllerState == RUN) {
    digitalWrite(STATUS_LED_PIN, (millis() / 500UL) % 2U);
  } else if (linkState == LINK_OK) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

void printHelp() {
  Serial.println(F("PC protocol: POS,x,y,timestamp | LOST | RUN | READY"));
  Serial.println(F("TARGET,x,y | PIDX,kp,ki,kd | PIDY,kp,ki,kd | PING"));
  Serial.println(F("Hardware test: TEST | EXITTEST | CENTER | STATUS | SHOW | JOYCAL | JOY | MODE,1|2"));
  Serial.println(F("Telemetry: TELNOW=once | TELON=auto 10Hz | TELOFF=quiet (default)"));
  Serial.println(F("Joystick TEST: one push = one axis move; return stick to center to re-arm"));
  Serial.println(F("Joystick SW: short=MODE 1 MICRO STEP / MODE 2 ENDPOINT, long=center"));
  Serial.println(F("TEST serial: X,n | Y,n | XY,x,y | X+ | X- | Y+ | Y- | TESTX | TESTY"));
}

void processCommand(char *line) {
  char *command = strtok(line, ",");
  if (command == NULL) {
    return;
  }

  // Telemetry control is available in every controller state.
  if (strcmp(command, "TELNOW") == 0 && strtok(NULL, ",") == NULL) {
    printTelemetry();
    return;
  }
  if (strcmp(command, "TELON") == 0 && strtok(NULL, ",") == NULL) {
    autoTelemetryEnabled = true;
    lastTelemetryMs = millis();
    Serial.println(F("TELEMETRY,ON"));
    return;
  }
  if (strcmp(command, "TELOFF") == 0 && strtok(NULL, ",") == NULL) {
    autoTelemetryEnabled = false;
    Serial.println(F("TELEMETRY,OFF"));
    return;
  }

  if (strcmp(command, "TEST") == 0 && strtok(NULL, ",") == NULL) {
    enterTestMode();
    return;
  }

  if (strcmp(command, "EXITTEST") == 0 && strtok(NULL, ",") == NULL) {
    if (controllerState == TEST) {
      exitTestMode();
    } else {
      Serial.println(F("ERROR,NOT_IN_TEST_MODE"));
    }
    return;
  }

  if (controllerState == TEST) {
    if (strcmp(command, "CENTER") == 0 && strtok(NULL, ",") == NULL) {
      writeManualBoth(testCenterX, testCenterY);
      printTestStatus();
      return;
    }
    if (strcmp(command, "STATUS") == 0 && strtok(NULL, ",") == NULL) {
      printTestStatus();
      return;
    }
    if (strcmp(command, "SHOW") == 0 && strtok(NULL, ",") == NULL) {
      printTestCalibration();
      return;
    }
    if (strcmp(command, "JOYCAL") == 0 && strtok(NULL, ",") == NULL) {
      calibrateJoystickCenter();
      return;
    }
    if (strcmp(command, "JOY") == 0 && strtok(NULL, ",") == NULL) {
      joystickRawX = analogRead(JOYSTICK_X_PIN);
      joystickRawY = analogRead(JOYSTICK_Y_PIN);
      Serial.print(F("JOY,X=")); Serial.print(joystickRawX);
      Serial.print(F(",Y=")); Serial.print(joystickRawY);
      Serial.print(F(",CX=")); Serial.print(joystickCenterX);
      Serial.print(F(",CY=")); Serial.println(joystickCenterY);
      return;
    }
    if (strcmp(command, "X+") == 0 && strtok(NULL, ",") == NULL) {
      applyOneJoystickMove(true, 1);
      return;
    }
    if (strcmp(command, "X-") == 0 && strtok(NULL, ",") == NULL) {
      applyOneJoystickMove(true, -1);
      return;
    }
    if (strcmp(command, "Y+") == 0 && strtok(NULL, ",") == NULL) {
      applyOneJoystickMove(false, 1);
      return;
    }
    if (strcmp(command, "Y-") == 0 && strtok(NULL, ",") == NULL) {
      applyOneJoystickMove(false, -1);
      return;
    }
    if (strcmp(command, "SETCX") == 0 && strtok(NULL, ",") == NULL) {
      testCenterX = servoAngleX;
      Serial.print(F("CENTER_X_SET,")); Serial.println(testCenterX);
      return;
    }
    if (strcmp(command, "SETCY") == 0 && strtok(NULL, ",") == NULL) {
      testCenterY = servoAngleY;
      Serial.print(F("CENTER_Y_SET,")); Serial.println(testCenterY);
      return;
    }
    if (strcmp(command, "SETC") == 0 && strtok(NULL, ",") == NULL) {
      testCenterX = servoAngleX;
      testCenterY = servoAngleY;
      printTestStatus();
      return;
    }
    if (strcmp(command, "STEP") == 0) {
      uint32_t stepValue;
      if (parseUnsigned(strtok(NULL, ","), stepValue) && strtok(NULL, ",") == NULL) {
        if (stepValue == 1UL) selectTestMoveMode(TEST_MICRO_STEP);
        else {
          Serial.println(F("ERROR,BAD_STEP,USE_1"));
          return;
        }
        printTestModeGuide();
        printTestStatus();
      } else {
        Serial.println(F("ERROR,BAD_STEP,USE_1"));
      }
      return;
    }
    if (strcmp(command, "MODE") == 0) {
      uint32_t modeValue;
      if (parseUnsigned(strtok(NULL, ","), modeValue) && strtok(NULL, ",") == NULL) {
        if (modeValue == 1UL) selectTestMoveMode(TEST_MICRO_STEP);
        else if (modeValue == 2UL) selectTestMoveMode(TEST_ENDPOINT);
        else {
          Serial.println(F("ERROR,BAD_MODE,USE_1_2"));
          return;
        }
        printTestModeGuide();
        printTestStatus();
      } else {
        Serial.println(F("ERROR,BAD_MODE,USE_1_2"));
      }
      return;
    }
    if (strcmp(command, "X") == 0 || strcmp(command, "Y") == 0) {
      uint32_t angle;
      if (parseUnsigned(strtok(NULL, ","), angle) && strtok(NULL, ",") == NULL &&
          angle >= (uint32_t)SERVO_MIN_ANGLE && angle <= (uint32_t)SERVO_MAX_ANGLE) {
        if (command[0] == 'X') writeManualServoX((int)angle);
        else writeManualServoY((int)angle);
        printTestStatus();
      } else {
        Serial.println(F("ERROR,BAD_ANGLE"));
      }
      return;
    }
    if (strcmp(command, "XY") == 0) {
      uint32_t xAngle, yAngle;
      if (parseUnsigned(strtok(NULL, ","), xAngle) &&
          parseUnsigned(strtok(NULL, ","), yAngle) &&
          strtok(NULL, ",") == NULL &&
          xAngle >= (uint32_t)SERVO_MIN_ANGLE && xAngle <= (uint32_t)SERVO_MAX_ANGLE &&
          yAngle >= (uint32_t)SERVO_MIN_ANGLE && yAngle <= (uint32_t)SERVO_MAX_ANGLE) {
        writeManualBoth((int)xAngle, (int)yAngle);
        printTestStatus();
      } else {
        Serial.println(F("ERROR,BAD_XY"));
      }
      return;
    }
    if (strcmp(command, "TESTX") == 0 && strtok(NULL, ",") == NULL) {
      runManualAxisTest(true);
      return;
    }
    if (strcmp(command, "TESTY") == 0 && strtok(NULL, ",") == NULL) {
      runManualAxisTest(false);
      return;
    }
    if (strcmp(command, "TESTBOTH") == 0 && strtok(NULL, ",") == NULL) {
      runManualAxisTest(true);
      runManualAxisTest(false);
      return;
    }

    // Keep these PC commands available even in TEST so the existing camera
    // application can immediately take control without any camera-side change.
    if (strcmp(command, "RUN") != 0 && strcmp(command, "READY") != 0 &&
        strcmp(command, "POS") != 0 && strcmp(command, "LOST") != 0 &&
        strcmp(command, "TARGET") != 0 && strcmp(command, "PIDX") != 0 &&
        strcmp(command, "PIDY") != 0 && strcmp(command, "PING") != 0 &&
        strcmp(command, "HELP") != 0) {
      Serial.println(F("ERROR,TEST_COMMAND_ONLY"));
      return;
    }
  }

  if (strcmp(command, "POS") == 0) {
    float x, y;
    uint32_t timestamp;
    if (parseFloat(strtok(NULL, ","), x) && parseFloat(strtok(NULL, ","), y) &&
        parseUnsigned(strtok(NULL, ","), timestamp) && strtok(NULL, ",") == NULL) {
      acceptPosition(x, y, timestamp);
    } else {
      Serial.println(F("ERROR,BAD_POS"));
    }
  } else if (strcmp(command, "LOST") == 0 && strtok(NULL, ",") == NULL) {
    stopControl(BALL_LOST);
  } else if (strcmp(command, "RUN") == 0 && strtok(NULL, ",") == NULL) {
    if (positionIsFresh()) {
      controllerState = RUN;
      resetControllers();
      Serial.println(F("STATE,RUN"));
    } else {
      Serial.println(F("ERROR,FRESH_POSITION_REQUIRED"));
    }
  } else if (strcmp(command, "READY") == 0 && strtok(NULL, ",") == NULL) {
    stopControl(positionIsFresh() ? LINK_OK : LINK_WAIT);
  } else if (strcmp(command, "TARGET") == 0) {
    float x, y;
    if (parseFloat(strtok(NULL, ","), x) && parseFloat(strtok(NULL, ","), y) &&
        strtok(NULL, ",") == NULL && fabs(x) <= POSITION_LIMIT_MM && fabs(y) <= POSITION_LIMIT_MM) {
      targetX = x;
      targetY = y;
      Serial.println(F("TARGET,OK"));
    } else {
      Serial.println(F("ERROR,BAD_TARGET"));
    }
  } else if (strcmp(command, "PIDX") == 0 || strcmp(command, "PIDY") == 0) {
    float kp, ki, kd;
    if (parseFloat(strtok(NULL, ","), kp) && parseFloat(strtok(NULL, ","), ki) &&
        parseFloat(strtok(NULL, ","), kd) && strtok(NULL, ",") == NULL) {
      PIDController &pid = strcmp(command, "PIDX") == 0 ? pidX : pidY;
      pid.kp = kp;
      pid.ki = ki;
      pid.kd = kd;
      pid.reset();
      Serial.println(F("PID,OK"));
    } else {
      Serial.println(F("ERROR,BAD_PID"));
    }
  } else if (strcmp(command, "PING") == 0 && strtok(NULL, ",") == NULL) {
    Serial.println(F("PONG"));
  } else if (strcmp(command, "HELP") == 0 && strtok(NULL, ",") == NULL) {
    printHelp();
  } else {
    Serial.println(F("ERROR,UNKNOWN_COMMAND"));
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      serialLine[serialLineLength] = '\0';
      processCommand(serialLine);
      serialLineLength = 0;
    } else if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = incoming;
    } else {
      serialLineLength = 0;
      Serial.println(F("ERROR,LINE_TOO_LONG"));
    }
  }
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  writeNeutralServos();
  Serial.println(F("BALL_CTRL,PC_VISION_MODE"));
  Serial.println(F("FEATURE,HARDWARE_TEST_MODE"));
  Serial.println(F("FEATURE,5PIN_JOYSTICK_DISCRETE_TEST,A0,A1,D2"));
  Serial.println(F("TIP,AUTO_ENTER_TEST_ON_BOOT"));
  Serial.println(F("TEST_MODES,MODE1=MICRO_STEP_1_DEG,MODE2=ENDPOINT"));
  Serial.println(F("TELEMETRY,QUIET_DEFAULT,USE_TELNOW_OR_TELON"));
  // Boot directly into TEST mode so the joystick controls the platform
  // immediately, without the 1.5 s long-press requirement.
  enterTestMode();
  printHelp();
}

void loop() {
  readSerialCommands();
  updateJoystickButton();
  updateJoystickTestControl();
  checkSafetyTimeout();
  updatePidForPosition();

  const uint32_t nowMs = millis();
  if (autoTelemetryEnabled &&
      (uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
  updateStatusLed();
}
