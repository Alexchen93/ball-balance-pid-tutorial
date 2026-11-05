/*
 * Maze controller: PC camera -> USB serial -> Arduino Nano PID + joystick target
 *
 * PC-to-Nano protocol (ASCII, one command per LF-terminated line):
 *   GEOM,<x_min_pct>,<x_max_pct>,<y_min_pct>,<y_max_pct>
 *   POS,<x_pct>,<y_pct>,<camera_timestamp_ms>
 *   LOST
 *   RUN | READY | TARGET,<x_pct>,<y_pct>
 *   PING | HELP
 *
 * Nano firmware is the only PID parameter source/control authority.
 * PIDX/PIDY commands are rejected with ERROR,PID_MANAGED_BY_NANO.
 *
 * POS replies with POS,OK after the Nano accepts a fresh in-range sample.
 * RUN must be immediately preceded by a fresh accepted POS; READY stops PID and returns servos to neutral.
 * Joystick updates the target only in RUN, after returning to neutral.
 * Target telemetry (5 Hz): TGT,<x_pct>,<y_pct>,<armed>
 * Nano telemetry (10 Hz):
 *   TEL,<state>,<link>,<x_pct>,<y_pct>,<e_x_pct>,<e_y_pct>,<u_x_pct>,<u_y_pct>,<tilt_x_deg>,<tilt_y_deg>,<servo_x>,<servo_y>,<age_ms>
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
// 搖桿控制目標位置（不是直接控制 Servo）。依實際搖桿調整方向及中心。
constexpr uint8_t JOYSTICK_X_PIN = A0;
constexpr uint8_t JOYSTICK_Y_PIN = A1;
constexpr int JOYSTICK_CENTER_X = 512;
constexpr int JOYSTICK_CENTER_Y = 512;
constexpr int JOYSTICK_DEADZONE = 100;
constexpr int JOYSTICK_X_DIRECTION = 1;
constexpr int JOYSTICK_Y_DIRECTION = 1;
constexpr float TARGET_SPEED_PCT_PER_SECOND = 20.0f;
// 0.8 表示目標不超過中心到邊界距離的 80%；不可保證球不掉落。
constexpr float TARGET_RANGE_FRACTION = 0.8f;
constexpr uint32_t JOYSTICK_PERIOD_MS = 20UL;
constexpr uint32_t TARGET_REPORT_PERIOD_MS = 200UL;
// ===== 學生校正參數（校正完成後只修改本區） =====
// 來源：校正程式的 SETC / SHOW 結果與學習單紀錄。
// 從正式 PID 韌體複製後須人工同步：數值不代表實機校正已完成。
// 安全限制：中心應落在各軸 MIN/MAX 之間，端點不可超出校正安全範圍。
// X 軸機構中心角度。校正完成後，從 SETC / SHOW / 學習單轉填。
constexpr int SERVO_X_CENTER = 77;
// Y 軸機構中心角度。校正完成後，從 SETC / SHOW / 學習單轉填。
constexpr int SERVO_Y_CENTER = 87;
// Servo 實體安全行程，也是 normalized PID 輸出 100% 時的最大安全機構要求。
// 調整 OFFSET 即可改每軸中心 ±安全端點，不需要手算端點；不要使用實體硬停點。
constexpr int SERVO_LIMIT_OFFSET_DEG = 20;
constexpr int SERVO_X_MIN_ANGLE = SERVO_X_CENTER - SERVO_LIMIT_OFFSET_DEG;
constexpr int SERVO_X_MAX_ANGLE = SERVO_X_CENTER + SERVO_LIMIT_OFFSET_DEG;
constexpr int SERVO_Y_MIN_ANGLE = SERVO_Y_CENTER - SERVO_LIMIT_OFFSET_DEG;
constexpr int SERVO_Y_MAX_ANGLE = SERVO_Y_CENTER + SERVO_LIMIT_OFFSET_DEG;
// ===== 學生校正參數結束 =====

constexpr int SERVO_X_DIRECTION = -1; // 實際方向依正式韌體與校正結果設定。
constexpr int SERVO_Y_DIRECTION = -1;  // 依實測設定；方向錯誤時停止並重新校正。

constexpr float MAX_PLATFORM_TILT_X_DEG = SERVO_LIMIT_OFFSET_DEG;
constexpr float MAX_PLATFORM_TILT_Y_DEG = SERVO_LIMIT_OFFSET_DEG;
constexpr float PID_INTEGRAL_LIMIT = 1.0f;
constexpr uint32_t POSITION_TIMEOUT_MS = 300UL;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100UL;
constexpr uint32_t SATURATION_WARNING_MS = 2000UL;
constexpr float POSITION_RANGE_EPSILON_PCT = 0.50f;

// Nano firmware is the single source of truth for PID constants. Camera Vision
// sends only vision/control-state commands and must not override these values.
// PID runs in normalized space: Kp=1.0 means a full-scale position error
// requests 100% of MAX_PLATFORM_TILT_*_DEG; Kp=0.5 would request 50%.
// Initial PD tune for a ball that carries outward momentum near an edge.
// Higher Kp starts the return tilt earlier; small Kd adds speed-proportional
// braking. Keep Ki at zero: it cannot brake a moving ball and can add overshoot.
constexpr float DEFAULT_KP_X = 1.25f;
constexpr float DEFAULT_KI_X = 0.00f;
constexpr float DEFAULT_KD_X = 0.5f;
constexpr float DEFAULT_KP_Y = 1.25f;
constexpr float DEFAULT_KI_Y = 0.00f;
constexpr float DEFAULT_KD_Y = 0.5f;

enum ControllerState : uint8_t { WAIT_LINK, READY, RUN };
enum LinkState : uint8_t { LINK_WAIT, LINK_OK, BALL_LOST, LINK_LOST, POSITION_RANGE_ERROR, GEOMETRY_REQUIRED };

struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousBall;
  bool hasPreviousBall;

  PIDController(float p, float i, float d)
      : kp(p), ki(i), kd(d), integral(0.0f), previousBall(0.0f),
        hasPreviousBall(false) {}

  void reset() {
    integral = 0.0f;
    hasPreviousBall = false;
  }

  float update(float errorNorm, float previousBallError, float ball, float dtSeconds) {
    // 用同一個當前目標比較前後球位置，目標移動不會造成 D 項突跳。
    const float derivative = hasPreviousBall ? (errorNorm - previousBallError) / dtSeconds : 0.0f;
    const float candidateIntegral = constrain(
        integral + errorNorm * dtSeconds, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    const float unconstrained = kp * errorNorm + ki * candidateIntegral + kd * derivative;
    const float outputNorm = constrain(unconstrained, -1.0f, 1.0f);

    // Conditional integration prevents windup while normalized output is saturated.
    if (outputNorm == unconstrained || errorNorm * outputNorm < 0.0f) {
      integral = candidateIntegral;
    }
    previousBall = ball;
    hasPreviousBall = true;
    return outputNorm;
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
bool geometryConfigured = false;

float ballX = 0.0f;
float ballY = 0.0f;
float targetX = 0.0f;
float targetY = 0.0f;
float positionMinX = 0.0f;
float positionMaxX = 0.0f;
float positionMinY = 0.0f;
float positionMaxY = 0.0f;
float errorNormX = 0.0f;
float errorNormY = 0.0f;
float outputNormX = 0.0f;
float outputNormY = 0.0f;
float requestedTiltX = 0.0f;
float requestedTiltY = 0.0f;
int servoAngleX = SERVO_X_CENTER;
int servoAngleY = SERVO_Y_CENTER;

uint32_t lastPositionMs = 0;
uint32_t lastPidUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t saturationStartedMs = 0;
uint32_t lastJoystickMs = 0;
uint32_t lastTargetReportMs = 0;
bool joystickArmed = false;

char serialLine[96];
uint8_t serialLineLength = 0;

const char *controllerStateName(ControllerState state) {
  switch (state) {
    case WAIT_LINK: return "WAIT";
    case READY: return "READY";
    case RUN: return "RUN";
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
    case GEOMETRY_REQUIRED: return "GEOM";
  }
  return "?";
}

void writeNeutralServos() {
  // 中心與安全端點統一使用上方「學生校正參數」，不要在其他地方重複硬編碼角度。
  servoAngleX = constrain(SERVO_X_CENTER, SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
  servoAngleY = constrain(SERVO_Y_CENTER, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
  servoX.write(servoAngleX);
  servoY.write(servoAngleY);
}

void resetControllers() {
  pidX.reset();
  pidY.reset();
  errorNormX = 0.0f;
  errorNormY = 0.0f;
  outputNormX = 0.0f;
  outputNormY = 0.0f;
  requestedTiltX = 0.0f;
  requestedTiltY = 0.0f;
  lastPidUs = 0;
  servoSaturated = false;
  saturationStartedMs = 0;
}

void stopControl(LinkState reason) {
  const bool wasRunning = controllerState == RUN;
  controllerState = (reason == LINK_WAIT) ? WAIT_LINK : READY;
  linkState = reason;
  newPositionAvailable = false;
  joystickArmed = false;
  resetControllers();
  writeNeutralServos();
  if (wasRunning) {
    Serial.println(F("STATE,READY"));
  }
}

bool positionIsFresh() {
  return linkState == LINK_OK && (uint32_t)(millis() - lastPositionMs) <= POSITION_TIMEOUT_MS;
}

bool positionWithinGeometry(float x, float y) {
  return geometryConfigured && isfinite(x) && isfinite(y) &&
         x >= positionMinX - POSITION_RANGE_EPSILON_PCT &&
         x <= positionMaxX + POSITION_RANGE_EPSILON_PCT &&
         y >= positionMinY - POSITION_RANGE_EPSILON_PCT &&
         y <= positionMaxY + POSITION_RANGE_EPSILON_PCT;
}

bool targetWithinGeometry(float x, float y) {
  return geometryConfigured && isfinite(x) && isfinite(y) &&
         x >= positionMinX && x <= positionMaxX &&
         y >= positionMinY && y <= positionMaxY;
}

bool configureGeometry(float xMin, float xMax, float yMin, float yMax) {
  if (!(xMin < 0.0f && xMax > 0.0f && yMin < 0.0f && yMax > 0.0f)) {
    return false;
  }
  if ((xMax - xMin) < 20.0f || (yMax - yMin) < 20.0f ||
      (xMax - xMin) > 250.0f || (yMax - yMin) > 250.0f) {
    return false;
  }
  positionMinX = xMin;
  positionMaxX = xMax;
  positionMinY = yMin;
  positionMaxY = yMax;
  geometryConfigured = true;
  if (!targetWithinGeometry(targetX, targetY)) {
    targetX = 0.0f;
    targetY = 0.0f;
  }
  return targetWithinGeometry(targetX, targetY);
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
  if (!geometryConfigured) {
    Serial.println(F("ERROR,GEOM_REQUIRED"));
    stopControl(GEOMETRY_REQUIRED);
    return;
  }
  if (!positionWithinGeometry(x, y)) {
    Serial.println(F("ERROR,POSITION_OUT_OF_RANGE"));
    stopControl(POSITION_RANGE_ERROR);
    return;
  }

  ballX = constrain(x, positionMinX, positionMaxX);
  ballY = constrain(y, positionMinY, positionMaxY);
  lastPositionMs = millis();
  linkState = LINK_OK;
  newPositionAvailable = true;
  if (controllerState == WAIT_LINK) {
    controllerState = READY;
    Serial.println(F("STATE,READY"));
  }
  Serial.println(F("POS,OK"));
}

float normalizedPositionError(float targetPct, float ballPct, float minPct, float maxPct) {
  const float errorPct = targetPct - ballPct;
  const float denominator = errorPct >= 0.0f ? targetPct - minPct : maxPct - targetPct;
  if (denominator <= 0.0f) {
    return 0.0f;
  }
  return constrain(errorPct / denominator, -1.0f, 1.0f);
}

void updateServos() {
  // Platform tilt requests come from normalized PID output; servo endpoints are separate physical safety bounds.
  const float requestedX = SERVO_X_CENTER + SERVO_X_DIRECTION * requestedTiltX;
  const float requestedY = SERVO_Y_CENTER + SERVO_Y_DIRECTION * requestedTiltY;
  servoAngleX = constrain((int)round(requestedX), SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
  servoAngleY = constrain((int)round(requestedY), SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
  servoX.write(servoAngleX);
  servoY.write(servoAngleY);

  servoSaturated = servoAngleX == SERVO_X_MIN_ANGLE || servoAngleX == SERVO_X_MAX_ANGLE ||
                    servoAngleY == SERVO_Y_MIN_ANGLE || servoAngleY == SERVO_Y_MAX_ANGLE;
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
  errorNormX = normalizedPositionError(targetX, ballX, positionMinX, positionMaxX);
  errorNormY = normalizedPositionError(targetY, ballY, positionMinY, positionMaxY);
  const float previousErrorX = normalizedPositionError(targetX, pidX.previousBall, positionMinX, positionMaxX);
  const float previousErrorY = normalizedPositionError(targetY, pidY.previousBall, positionMinY, positionMaxY);
  outputNormX = pidX.update(errorNormX, previousErrorX, ballX, dtSeconds);
  outputNormY = pidY.update(errorNormY, previousErrorY, ballY, dtSeconds);
  requestedTiltX = outputNormX * MAX_PLATFORM_TILT_X_DEG;
  requestedTiltY = outputNormY * MAX_PLATFORM_TILT_Y_DEG;
  updateServos();
}

// 搖桿偏移決定目標移動速度；放開時保持目標，並先回中立才能啟用。
float joystickInput(int raw, int center) {
  const int delta = raw - center;
  if (abs(delta) <= JOYSTICK_DEADZONE) return 0.0f;
  const int available = delta > 0 ? 1023 - center - JOYSTICK_DEADZONE
                                  : center - JOYSTICK_DEADZONE;
  if (available <= 0) return 0.0f;
  const float magnitude = constrain((float)(abs(delta) - JOYSTICK_DEADZONE) / available, 0.0f, 1.0f);
  return delta > 0 ? magnitude : -magnitude;
}

void updateJoystickTarget() {
  const uint32_t now = millis();
  if ((uint32_t)(now - lastJoystickMs) < JOYSTICK_PERIOD_MS) return;
  const float dt = constrain((now - lastJoystickMs) / 1000.0f, 0.0f, 0.05f);
  lastJoystickMs = now;
  if (controllerState != RUN || !positionIsFresh()) return;
  const float x = joystickInput(analogRead(JOYSTICK_X_PIN), JOYSTICK_CENTER_X);
  const float y = joystickInput(analogRead(JOYSTICK_Y_PIN), JOYSTICK_CENTER_Y);
  if (!joystickArmed) {
    if (x == 0.0f && y == 0.0f) joystickArmed = true;
    return;
  }
  targetX = constrain(targetX + x * JOYSTICK_X_DIRECTION * TARGET_SPEED_PCT_PER_SECOND * dt,
                      positionMinX * TARGET_RANGE_FRACTION, positionMaxX * TARGET_RANGE_FRACTION);
  targetY = constrain(targetY + y * JOYSTICK_Y_DIRECTION * TARGET_SPEED_PCT_PER_SECOND * dt,
                      positionMinY * TARGET_RANGE_FRACTION, positionMaxY * TARGET_RANGE_FRACTION);
}

void printTarget() {
  Serial.print(F("TGT,")); Serial.print(targetX, 1);
  Serial.print(','); Serial.print(targetY, 1);
  Serial.print(','); Serial.println(joystickArmed ? 1 : 0);
}

void checkSafetyTimeout() {
  if (linkState == LINK_OK && !positionIsFresh()) {
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
  Serial.print(','); Serial.print(errorNormX * 100.0f, 1);
  Serial.print(','); Serial.print(errorNormY * 100.0f, 1);
  Serial.print(','); Serial.print(outputNormX * 100.0f, 1);
  Serial.print(','); Serial.print(outputNormY * 100.0f, 1);
  Serial.print(','); Serial.print(requestedTiltX, 2);
  Serial.print(','); Serial.print(requestedTiltY, 2);
  Serial.print(','); Serial.print(servoAngleX);
  Serial.print(','); Serial.print(servoAngleY);
  Serial.print(','); Serial.println(ageMs);
}


void updateStatusLed() {
  if (controllerState == RUN) {
    digitalWrite(STATUS_LED_PIN, (millis() / 500UL) % 2U);
  } else if (linkState == LINK_OK) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

void printHelp() {
  Serial.println(F("PC protocol: GEOM,xmin_pct,xmax_pct,ymin_pct,ymax_pct -> GEOM,OK before POS/RUN"));
  Serial.println(F("POS,x_pct,y_pct,timestamp -> POS,OK | final POS immediately before RUN | LOST | READY"));
  Serial.println(F("TARGET,x,y | PING | HELP"));
  Serial.println(F("PID source: Nano firmware only; PIDX/PIDY return ERROR,PID_MANAGED_BY_NANO"));
}

void processCommand(char *line) {
  char *command = strtok(line, ",");
  if (command == NULL) {
    return;
  }

  if (strcmp(command, "GEOM") == 0) {
    float xMin, xMax, yMin, yMax;
    if (parseFloat(strtok(NULL, ","), xMin) && parseFloat(strtok(NULL, ","), xMax) &&
        parseFloat(strtok(NULL, ","), yMin) && parseFloat(strtok(NULL, ","), yMax) &&
        strtok(NULL, ",") == NULL && controllerState != RUN && configureGeometry(xMin, xMax, yMin, yMax)) {
      stopControl(positionIsFresh() ? LINK_OK : LINK_WAIT);
      Serial.println(F("GEOM,OK"));
    } else {
      geometryConfigured = false;
      Serial.println(F("ERROR,BAD_GEOM"));
      stopControl(GEOMETRY_REQUIRED);
    }
  } else if (strcmp(command, "POS") == 0) {
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
    if (!geometryConfigured) {
      Serial.println(F("ERROR,GEOM_REQUIRED"));
      stopControl(GEOMETRY_REQUIRED);
    } else if (positionIsFresh()) {
      // 從球的目前位置開始，避免啟動時直接拉向平台中心。
      targetX = constrain(ballX, positionMinX * TARGET_RANGE_FRACTION, positionMaxX * TARGET_RANGE_FRACTION);
      targetY = constrain(ballY, positionMinY * TARGET_RANGE_FRACTION, positionMaxY * TARGET_RANGE_FRACTION);
      joystickArmed = false;
      lastJoystickMs = millis();
      controllerState = RUN;
      resetControllers();
      Serial.println(F("STATE,RUN"));
    } else {
      Serial.println(F("ERROR,FRESH_POSITION_REQUIRED"));
    }
  } else if (strcmp(command, "READY") == 0 && strtok(NULL, ",") == NULL) {
    stopControl(positionIsFresh() ? LINK_OK : LINK_WAIT);
    Serial.print(F("STATE,"));
    Serial.println(controllerStateName(controllerState));
  } else if (strcmp(command, "TARGET") == 0) {
    float x, y;
    if (parseFloat(strtok(NULL, ","), x) && parseFloat(strtok(NULL, ","), y) &&
        strtok(NULL, ",") == NULL && targetWithinGeometry(x, y) &&
        x >= positionMinX * TARGET_RANGE_FRACTION && x <= positionMaxX * TARGET_RANGE_FRACTION &&
        y >= positionMinY * TARGET_RANGE_FRACTION && y <= positionMaxY * TARGET_RANGE_FRACTION) {
      targetX = x;
      targetY = y;
      Serial.println(F("TARGET,OK"));
    } else {
      Serial.println(F("ERROR,BAD_TARGET"));
    }
  } else if (strcmp(command, "PIDX") == 0 || strcmp(command, "PIDY") == 0) {
    Serial.println(F("ERROR,PID_MANAGED_BY_NANO"));
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
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  writeNeutralServos();
  Serial.println(F("BALL_CTRL,MAZE_JOYSTICK_PID"));
  printHelp();
}

void loop() {
  readSerialCommands();
  checkSafetyTimeout();
  updateJoystickTarget();
  updatePidForPosition();

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
  if ((uint32_t)(nowMs - lastTargetReportMs) >= TARGET_REPORT_PERIOD_MS) {
    lastTargetReportMs = nowMs;
    printTarget();
  }
  updateStatusLed();
}
