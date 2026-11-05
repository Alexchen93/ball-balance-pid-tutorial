#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include "Servo.h"
uint32_t fakeMillis = 1000;
int fakeAnalog[2] = {512, 512};
FakeSerial Serial;
#include "../Arduino_Nano_Ball_Balance_Maze.ino"

void command(const char *text) {
  char line[96];
  std::strncpy(line, text, sizeof(line));
  line[sizeof(line) - 1] = '\0';
  processCommand(line);
}
int main() {
  setup();
  command("GEOM,-100,100,-100,100");
  command("POS,10,-15,0");
  command("RUN");
  assert(controllerState == RUN && targetX == 10 && targetY == -15);
  fakeAnalog[0] = 1023; fakeMillis += 20; updateJoystickTarget();
  assert(!joystickArmed && targetX == 10); // held stick cannot move target at startup
  fakeAnalog[0] = 512; fakeMillis += 20; updateJoystickTarget();
  assert(joystickArmed);
  fakeAnalog[0] = 1023; fakeMillis += 20; updateJoystickTarget();
  assert(targetX > 10 && targetY == -15);
  fakeAnalog[0] = 512; float held = targetX; fakeMillis += 20; updateJoystickTarget();
  assert(targetX == held);
  targetX = 79.9f; fakeAnalog[0] = 1023; fakeMillis += 20; updateJoystickTarget();
  assert(targetX <= 80.0f && targetX > 79.9f);
  // A moving setpoint with a stationary ball does not generate a D kick.
  PIDController dOnly(0, 0, 1);
  dOnly.update(normalizedPositionError(0, 0, -100, 100), 0, 0, .1f);
  float e = normalizedPositionError(20, 0, -100, 100);
  float oldBallError = normalizedPositionError(20, dOnly.previousBall, -100, 100);
  assert(std::fabs(dOnly.update(e, oldBallError, 0, .1f)) < 0.0001f);
  fakeMillis += POSITION_TIMEOUT_MS + 1; checkSafetyTimeout();
  assert(controllerState == READY && linkState == LINK_LOST && !joystickArmed);
  assert(servoX.angle == SERVO_X_CENTER && servoY.angle == SERVO_Y_CENTER);
  command("POS,0,0,0"); command("RUN");
  command("POS,10,10,0"); updatePidForPosition();
  assert(outputNormX < 0 && outputNormY < 0);
  assert(servoAngleX > SERVO_X_CENTER && servoAngleY > SERVO_Y_CENTER);
  command("LOST");
  assert(controllerState == READY && linkState == BALL_LOST && !joystickArmed);
  assert(servoX.angle == SERVO_X_CENTER && servoY.angle == SERVO_Y_CENTER);
  std::cout << "maze controller checks passed\n";
}
