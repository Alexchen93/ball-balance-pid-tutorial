#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include "Servo.h"
uint32_t fakeMillis = 1000;
int fakeAnalog[2] = {512, 512};
FakeSerial Serial;
#include "../Arduino_Nano_Ball_Balance.ino"

void command(const char *text) {
  char line[96];
  std::strncpy(line, text, sizeof(line));
  line[sizeof(line) - 1] = '\0';
  processCommand(line);
}
bool closeEnough(float a, float b) { return std::fabs(a - b) < 0.0001f; }
int main() {
  PIDController p(0.5f, 0, 0);
  assert(closeEnough(p.update(0.4f, 0.1f), 0.2f));
  PIDController i(0, 1, 0);
  assert(closeEnough(i.update(0.2f, 0.5f), 0.1f));
  assert(closeEnough(i.update(0.2f, 0.5f), 0.2f));
  i.reset();
  assert(closeEnough(i.update(0.2f, 0.5f), 0.1f));
  PIDController d(0, 0, 0.5f);
  assert(closeEnough(d.update(0.2f, 0.1f), 0));
  assert(closeEnough(d.update(0.3f, 0.1f), 0.5f));
  PIDController limited(2, 1, 0);
  assert(closeEnough(limited.update(1, 1), 1));
  assert(closeEnough(limited.integral, 0)); // saturation inhibits further windup
  assert(closeEnough(normalizedPositionError(0, 50, -100, 100), -0.5f));
  setup();
  command("RUN");
  assert(controllerState != RUN);
  command("GEOM,-100,100,-100,100");
  command("POS,10,0,0");
  command("RUN");
  assert(controllerState == RUN);
  updatePidForPosition();
  // A stationary ball on the right requests the inverted X servo direction.
  assert(errorNormX < 0 && outputNormX < 0 && servoAngleX > SERVO_X_CENTER);
  assert(servoAngleY == SERVO_Y_CENTER);
  command("READY");
  command("POS,-10,0,0");
  command("RUN");
  updatePidForPosition();
  assert(errorNormX > 0 && outputNormX > 0 && servoAngleX < SERVO_X_CENTER);
  assert(servoAngleY == SERVO_Y_CENTER);
  command("READY"); command("POS,0,10,0"); command("RUN"); updatePidForPosition();
  assert(errorNormY < 0 && outputNormY < 0 && servoAngleY > SERVO_Y_CENTER);
  assert(servoAngleX == SERVO_X_CENTER);
  command("READY"); command("POS,0,-10,0"); command("RUN"); updatePidForPosition();
  assert(errorNormY > 0 && outputNormY > 0 && servoAngleY < SERVO_Y_CENTER);
  assert(servoAngleX == SERVO_X_CENTER);
  fakeMillis += POSITION_TIMEOUT_MS + 1;
  checkSafetyTimeout();
  assert(controllerState == READY && linkState == LINK_LOST);
  assert(servoX.angle == SERVO_X_CENTER && servoY.angle == SERVO_Y_CENTER);
  command("POS,0,0,0"); command("RUN"); command("LOST");
  assert(controllerState == READY && linkState == BALL_LOST);
  assert(servoX.angle == SERVO_X_CENTER && servoY.angle == SERVO_Y_CENTER);
  std::cout << "formal PID controller checks passed\n";
}
