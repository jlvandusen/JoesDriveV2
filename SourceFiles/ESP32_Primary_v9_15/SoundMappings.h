
#ifndef SOUND_MAPPINGS_H
#define SOUND_MAPPINGS_H
#include "ControllerDefs.h"

#define EDGE_PRESSED(cur, prev) ((prev) == false && (cur) == true)

// Sound IDs
enum : uint16_t {
    SOUND_NONE     = 0,
    SOUND_RANDOM   = 9,
    SOUND_STARTUP  = 1,

    // Toggle sounds
    SOUND_TOGGLE_DRIVE       = 60,
    SOUND_TOGGLE_REVERSE     = 61,
    SOUND_TOGGLE_DOME_SERVO  = 62,
    SOUND_TOGGLE_BALANCE     = 63,
    SOUND_TOGGLE_ESPNOW      = 64,
    SOUND_TOGGLE_OTA         = 65,
    SOUND_TOGGLE_ROLL_BIAS   = 66,
    SOUND_TOGGLE_TELEMETRY   = 67
};

static inline uint16_t pickRandom1to30() { return (uint16_t) random(2, 99); }

struct DPad {
    bool up, right, down, left;
    DPad(bool u, bool r, bool dwn, bool lft) : up(u), right(r), down(dwn), left(lft) {}
};

// Drive controller sound mapping
static uint16_t resolveDriveControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
    DPad d(cur.up, cur.right, cur.down, cur.left);

    // L1 + D-pad combos
    if (cur.l1) {
        if (EDGE_PRESSED(d.up, prev.up)) return 6;
        if (EDGE_PRESSED(d.right, prev.right)) return 7;
        if (EDGE_PRESSED(d.down, prev.down)) return 8;
        if (EDGE_PRESSED(d.left, prev.left)) return 9;
    }

    // R1 + D-pad combos
    if (cur.r1) {
        if (EDGE_PRESSED(d.up, prev.up)) return 10;
        if (EDGE_PRESSED(d.right, prev.right)) return 11;
        if (EDGE_PRESSED(d.down, prev.down)) return 12;
        if (EDGE_PRESSED(d.left, prev.left)) return 13;
    }

    // Single D-pad
    if (EDGE_PRESSED(cur.up, prev.up)) return 2;
    if (EDGE_PRESSED(cur.right, prev.right)) return 3;
    if (EDGE_PRESSED(cur.down, prev.down)) return 4;
    if (EDGE_PRESSED(cur.left, prev.left)) return 5;

    // Other buttons
    if (EDGE_PRESSED(cur.circle, prev.circle)) return 14;
    if (EDGE_PRESSED(cur.cross, prev.cross)) return 15;
    if (cur.l2 > 60 && prev.l2 <= 60) return 16;

    return SOUND_NONE;
}

// Dome controller sound mapping
static uint16_t resolveDomeControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
    DPad d(cur.up, cur.right, cur.down, cur.left);

    // L1 + D-pad combos
    if (cur.l1) {
        if (EDGE_PRESSED(d.up, prev.up)) return 24;
        if (EDGE_PRESSED(d.right, prev.right)) return 25;
        if (EDGE_PRESSED(d.down, prev.down)) return 26;
        if (EDGE_PRESSED(d.left, prev.left)) return 27;
    }

    // Single D-pad
    if (EDGE_PRESSED(cur.up, prev.up)) return 20;
    if (EDGE_PRESSED(cur.right, prev.right)) return 21;
    if (EDGE_PRESSED(cur.down, prev.down)) return 22;
    if (EDGE_PRESSED(cur.left, prev.left)) return 23;

    // Other buttons
    if (EDGE_PRESSED(cur.circle, prev.circle)) return 28;
    if (EDGE_PRESSED(cur.cross, prev.cross)) return 29;
    if (cur.l2 > 60 && prev.l2 <= 60) return 30;

    return SOUND_NONE;
}

// Combination sounds (both controllers)

// Combination sounds (both controllers)
static uint16_t resolveComboSound(const ControllerButtons& left, const ControllerButtons& right,
                                   const ControllerButtons& prevLeft, const ControllerButtons& prevRight) {
    // D-Pad combos
    if (EDGE_PRESSED(left.up, prevLeft.up) && EDGE_PRESSED(right.up, prevRight.up)) return 40;
    if (EDGE_PRESSED(left.down, prevLeft.down) && EDGE_PRESSED(right.down, prevRight.down)) return 41;
    if (EDGE_PRESSED(left.left, prevLeft.left) && EDGE_PRESSED(right.left, prevRight.left)) return 42;
    if (EDGE_PRESSED(left.right, prevLeft.right) && EDGE_PRESSED(right.right, prevRight.right)) return 43;

    // L1 combos
    if (EDGE_PRESSED(left.l1, prevLeft.l1) && EDGE_PRESSED(right.l1, prevRight.l1)) return 44;

    // R1 combos
    if (EDGE_PRESSED(left.r1, prevLeft.r1) && EDGE_PRESSED(right.r1, prevRight.r1)) return 45;

    // Circle + Circle
    if (EDGE_PRESSED(left.circle, prevLeft.circle) && EDGE_PRESSED(right.circle, prevRight.circle)) return 46;

    // Cross + Cross
    if (EDGE_PRESSED(left.cross, prevLeft.cross) && EDGE_PRESSED(right.cross, prevRight.cross)) return 47;

    // PS + PS
    if (EDGE_PRESSED(left.ps, prevLeft.ps) && EDGE_PRESSED(right.ps, prevRight.ps)) return 48;

    // L3 combos
    if (EDGE_PRESSED(left.l3, prevLeft.l3) && EDGE_PRESSED(right.l3, prevRight.l3)) return 49;

    // Mixed combos (D-Pad + Circle)
    if (EDGE_PRESSED(left.up, prevLeft.up) && EDGE_PRESSED(right.circle, prevRight.circle)) return 50;
    if (EDGE_PRESSED(left.down, prevLeft.down) && EDGE_PRESSED(right.circle, prevRight.circle)) return 51;
    if (EDGE_PRESSED(left.left, prevLeft.left) && EDGE_PRESSED(right.circle, prevRight.circle)) return 52;
    if (EDGE_PRESSED(left.right, prevLeft.right) && EDGE_PRESSED(right.circle, prevRight.circle)) return 53;

    // Mixed combos (D-Pad + Cross)
    if (EDGE_PRESSED(left.up, prevLeft.up) && EDGE_PRESSED(right.cross, prevRight.cross)) return 54;
    if (EDGE_PRESSED(left.down, prevLeft.down) && EDGE_PRESSED(right.cross, prevRight.cross)) return 55;
    if (EDGE_PRESSED(left.left, prevLeft.left) && EDGE_PRESSED(right.cross, prevRight.cross)) return 56;
    if (EDGE_PRESSED(left.right, prevLeft.right) && EDGE_PRESSED(right.cross, prevRight.cross)) return 57;

    // L1 + Circle combos
    if (EDGE_PRESSED(left.l1, prevLeft.l1) && EDGE_PRESSED(right.circle, prevRight.circle)) return 58;
    if (EDGE_PRESSED(left.l1, prevLeft.l1) && EDGE_PRESSED(right.cross, prevRight.cross)) return 59;

    // L2 triggers both pressed
    if ((left.l2 > 60 && prevLeft.l2 <= 60) && (right.l2 > 60 && prevRight.l2 <= 60)) return 60;

    // Add more combos as needed up to 99
    return SOUND_NONE;
}


#endif
