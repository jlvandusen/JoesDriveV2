// ControllerDefs.h
#ifndef CONTROLLER_DEFS_H
#define CONTROLLER_DEFS_H

struct ControllerButtons {
  bool cross, circle, up, down, left, right, ps, l1, l3, r1;
  int8_t leftStickX, leftStickY, rightStickX, rightStickY;
  int8_t l2;
};

#define EDGE_PRESSED(cur, prev) ((prev) == false && (cur) == true)

#endif
