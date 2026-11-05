#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include "Servo.h"
uint32_t fakeMillis = 1000;
int fakeAnalog[2] = {512, 512};
FakeSerial Serial;
#include "../Arduino_Nano_Ball_Balance_Joystick_Modes.ino"

void command(const char *text) {
  char line[96];
  std::strncpy(line, text, sizeof(line));
  line[sizeof(line) - 1] = '\0';
  processCommand(line);
}
bool printed(const char *fragment) { return Serial.text().find(fragment) != std::string::npos; }
int main() {
  setup();
  assert(controllerState == TEST);
  { const std::string output = Serial.text();
    assert(output.find("RESET/BOOT\n") == 0);
    assert(std::count(output.begin(), output.end(), '\n') == 4);
  }
  Serial.clear();
  command("DIR,X,UP");
  assert(printed("ERROR,DIR_REQUIRES_CENTER_MODE1_1_TO_3_POSITIVE_STEPS"));
  assert(recommendedDirectionX == 0);

  command("CENTER"); command("MODE,1"); Serial.clear(); command("X+");
  assert(directionCheckAxis == 'X' && servoAngleX == testCenterX + 1);
  assert(printed("SERVO_CHANGE=INCREASE"));
  { const std::string output = Serial.text(); assert(std::count(output.begin(), output.end(), '\n') == 3); }
  Serial.clear(); command("DIR,X,UP");
  assert(recommendedDirectionX == -1 && printed("SERVO_X_DIRECTION=-1"));
  Serial.clear(); command("DIR,Y,UP");
  assert(printed("ERROR,DIR_REQUIRES_CENTER_MODE1_1_TO_3_POSITIVE_STEPS"));
  assert(recommendedDirectionY == 0);

  command("CENTER"); command("MODE,1"); command("Y+"); command("Y+"); command("Y+");
  assert(directionCheckAxis == 'Y' && servoAngleY == testCenterY + 3);
  Serial.clear(); command("DIR,Y,DOWN");
  assert(recommendedDirectionY == 1 && printed("SERVO_Y_DIRECTION=1"));
  Serial.clear(); command("SHOW");
  assert(printed("SERVO_X_DIRECTION = -1;") && printed("SERVO_Y_DIRECTION = 1;"));
  command("Y+");
  Serial.clear(); command("DIR,Y,UP");
  assert(printed("ERROR,DIR_REQUIRES_CENTER_MODE1_1_TO_3_POSITIVE_STEPS"));
  assert(recommendedDirectionY == 1);

  command("CENTER"); command("MODE,2"); command("X+");
  Serial.clear(); command("DIR,X,UP");
  assert(printed("ERROR,DIR_REQUIRES_CENTER_MODE1_1_TO_3_POSITIVE_STEPS"));

  command("CENTER"); command("MODE,1");
  fakeAnalog[0] = 850; fakeAnalog[1] = 200;
  Serial.clear(); command("JOY");
  assert(printed("X_ADC=850") && printed("X_STATE=HIGH"));
  assert(printed("Y_ADC=200") && printed("Y_STATE=LOW"));
  fakeMillis += 500; Serial.clear(); loop();
  assert(printed("SOURCE=JOYSTICK") && printed("ARMED=0"));
  assert(printed("JOY_ADC=850,JOY_DIR=INCREASE,SERVO_CHANGE=INCREASE"));
  assert(printed("JOY_ADC=200,JOY_DIR=DECREASE,SERVO_CHANGE=NONE"));
  { const std::string output = Serial.text(); assert(std::count(output.begin(), output.end(), '\n') == 3); }
  assert(directionCheckAxis == 'X');
  // Holding or returning to centre must not print any extra status lines.
  fakeMillis += 500; Serial.clear(); loop();
  assert(Serial.text().empty());
  fakeAnalog[0] = 512; fakeAnalog[1] = 512;
  fakeMillis += 500; Serial.clear(); loop();
  assert(Serial.text().empty() && joystickMoveArmed);
  fakeAnalog[0] = 200;
  fakeMillis += 500; Serial.clear(); loop();
  assert(printed("JOY_ADC=200,JOY_DIR=DECREASE,SERVO_CHANGE=DECREASE"));
  { const std::string output = Serial.text(); assert(std::count(output.begin(), output.end(), '\n') == 3); }
  command("SETCX");
  assert(recommendedDirectionX == 0 && recommendedDirectionY == 0);
  Serial.clear(); command("SHOW");
  assert(printed("SERVO_X_DIRECTION: NOT TESTED") && printed("SERVO_Y_DIRECTION: NOT TESTED"));
  testCenterX = SERVO_MAX_ANGLE;
  command("CENTER"); command("MODE,1");
  Serial.clear(); command("X+");
  assert(printed("SERVO_CHANGE=NO_CHANGE_LIMIT") && directionCheckAxis == 0);
  command("DIR,X,UP");
  assert(printed("ERROR,DIR_REQUIRES_CENTER_MODE1_1_TO_3_POSITIVE_STEPS"));
  std::cout << "joystick calibration checks passed\n";
}
