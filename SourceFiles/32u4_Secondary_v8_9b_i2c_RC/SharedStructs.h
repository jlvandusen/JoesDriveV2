#ifndef SHAREDSTRUCTS_H
#define SHAREDSTRUCTS_H

struct ControllerButtons {
  bool cross, circle, up, down, left, right, ps, l1, l3, r1;
  int8_t leftStickX, leftStickY, rightStickX, rightStickY;
  int8_t l2;
};

struct IMUData {
  float pitch;
  float roll;
};

struct Send32u4Data {
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3, moveR3;
  int8_t leftStickX, leftStickY;
  int8_t soundcmd, psiFlash;
  float pitch, roll;
};

#endif