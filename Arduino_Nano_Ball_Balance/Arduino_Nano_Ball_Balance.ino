/*
 * Ball-and-Plate controller: PC camera -> USB serial -> Arduino Nano PID
 *
 * PC-to-Nano protocol (ASCII, one command per LF-terminated line):
 *   POS,<x_mm>,<y_mm>,<camera_timestamp_ms>
 *   LOST
 *   RUN | READY | TARGET,<x_mm>,<y_mm>
 *   PING | HELP
 *
 * Nano firmware is the only PID parameter source/control authority.
 * PIDX/PIDY commands are rejected with ERROR,PID_MANAGED_BY_NANO.
 *
 * POS replies with POS,OK after the Nano accepts a fresh in-range sample.
 * RUN must be immediately preceded by a fresh accepted POS; READY stops PID and returns servos to neutral.
 * Nano telemetry (10 Hz):
 *   TEL,<state>,<link>,<x>,<y>,<error_x>,<error_y>,<u_x>,<u_y>,<servo_x>,<servo_y>,<age_ms>
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
// ===== 學生校正參數（校正完成後只修改本區） =====
// 來源：校正程式的 SETC / SHOW 結果與學習單紀錄。
// 注意：本次 JOYMOVE / TESTSTAT 觀測到 X=71、Y=76，只代表測試當下的輸出角度，
// 不可當作正式 PID 的機構中心值。正式 PID 請只轉填/調整下列中心與安全行程常數。
// 安全限制：中心應落在 SERVO_MIN_ANGLE 到 SERVO_MAX_ANGLE 之間，端點不可超出機構安全範圍。
// X 軸機構中心角度。校正完成後，從 SETC / SHOW / 學習單轉填；本教案最終中心採 90 度。
constexpr int SERVO_X_CENTER = 74;
// Y 軸機構中心角度。校正完成後，從 SETC / SHOW / 學習單轉填；本教案最終中心採 90 度。
constexpr int SERVO_Y_CENTER = 76;
// X 中心左右可移動的安全範圍。目前端點是 X中心90 ±20，推導為 70～110 度；
// 調整 OFFSET 即可改安全行程，不需要手算端點。
// 若未來 X/Y 中心分開，須確認共用範圍仍適用，避免暗中改變 PID 行為。
constexpr int SERVO_LIMIT_OFFSET_DEG = 20;
constexpr int SERVO_MIN_ANGLE = SERVO_X_CENTER - SERVO_LIMIT_OFFSET_DEG;
constexpr int SERVO_MAX_ANGLE = SERVO_X_CENTER + SERVO_LIMIT_OFFSET_DEG;
// ===== 學生校正參數結束 =====

constexpr int SERVO_X_DIRECTION = -1; // 已依左右邊緣實測反轉；若球被推向同側，再改回 1。
constexpr int SERVO_Y_DIRECTION = 1;  // Change to -1 if the Y correction is reversed.

constexpr float POSITION_LIMIT_X_MM = 260.0f;  // 240 mm platform width + margin
constexpr float POSITION_LIMIT_Y_MM = 200.0f;  // 180 mm platform height + margin
constexpr float PID_OUTPUT_LIMIT_DEG = 8.0f;
constexpr float PID_INTEGRAL_LIMIT = 60.0f;
constexpr uint32_t POSITION_TIMEOUT_MS = 300UL;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100UL;
constexpr uint32_t SATURATION_WARNING_MS = 2000UL;

// Nano firmware is the single source of truth for PID constants. Camera Vision
// sends only vision/control-state commands and must not override these values.
constexpr float DEFAULT_KP_X = 0.10f;
constexpr float DEFAULT_KI_X = 0.00f;
constexpr float DEFAULT_KD_X = 0.00f;
constexpr float DEFAULT_KP_Y = 0.10f;
constexpr float DEFAULT_KI_Y = 0.00f;
constexpr float DEFAULT_KD_Y = 0.00f;

enum ControllerState : uint8_t { WAIT_LINK, READY, RUN };
enum LinkState : uint8_t { LINK_WAIT, LINK_OK, BALL_LOST, LINK_LOST, POSITION_RANGE_ERROR };

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
  }
  return "?";
}

void writeNeutralServos() {
  // 中心與安全端點統一使用上方「學生校正參數」，不要在其他地方重複硬編碼角度。
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
  if (fabs(x) > POSITION_LIMIT_X_MM || fabs(y) > POSITION_LIMIT_Y_MM) {
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
  Serial.println(F("POS,OK"));
}

void updateServos() {
  // PID 輸出只疊加到集中校正區的中心角度，端點限制也只引用同一組常數。
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
  Serial.print(','); Serial.print(errorX, 1);
  Serial.print(','); Serial.print(errorY, 1);
  Serial.print(','); Serial.print(outputX, 2);
  Serial.print(','); Serial.print(outputY, 2);
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
  Serial.println(F("PC protocol: POS,x,y,timestamp -> POS,OK | final POS immediately before RUN | LOST | READY"));
  Serial.println(F("TARGET,x,y | PING | HELP"));
  Serial.println(F("PID source: Nano firmware only; PIDX/PIDY return ERROR,PID_MANAGED_BY_NANO"));
}

void processCommand(char *line) {
  char *command = strtok(line, ",");
  if (command == NULL) {
    return;
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
    Serial.print(F("STATE,"));
    Serial.println(controllerStateName(controllerState));
  } else if (strcmp(command, "TARGET") == 0) {
    float x, y;
    if (parseFloat(strtok(NULL, ","), x) && parseFloat(strtok(NULL, ","), y) &&
        strtok(NULL, ",") == NULL && fabs(x) <= POSITION_LIMIT_X_MM && fabs(y) <= POSITION_LIMIT_Y_MM) {
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
  Serial.println(F("BALL_CTRL,PC_VISION_MODE"));
  printHelp();
}

void loop() {
  readSerialCommands();
  checkSafetyTimeout();
  updatePidForPosition();

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
  updateStatusLed();
}
