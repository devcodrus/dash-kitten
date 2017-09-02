#ifndef MS3_CAN_H
#define MS3_CAN_H

#include "display_types.h"

enum {
  ms3CanBase = 1520,
  
  // block 0
  secOffset = 0,
  pw1Offset = 2,
  pw2Offset = 4,
  rpmOffset = 6,

  // block 1
  advOffset = 0,
  afrTarget1Offset = 4,

  // block 2
  baroOffset = 0,
  mapOffset = 2,
  matOffset = 4,
  cltOffset = 6,

  // block 3
  tpsOffset = 0,
  battOffset = 2,
  ego1Offset = 4,
  ego2Offset = 6,

  // block 4
  knockValOffset = 0,

  // don't care about blocks 5 through 8

  // block 9
  dwellOffset = 4,

  // block 10
  status1Offset = 0,
  status2Offset = 1,
  status3Offset = 2,
  status4Offset = 3,
  status5Offset = 4,
  status6Offset = 5,
  status7Offset = 6,
  
  // block 13
  sensor1Offset = 0,
  sensor2Offset = 2,
  sensor3Offset = 4,
  sensor4Offset = 6,

  // FP is a generic sensor
  fuelPressOffset = sensor3Offset,

  // block 17
  boostTarget1Offset = 0,
  boostTarget2Offset = 1,
  boostDuty1Offset = 2,
  boostDuty2Offset = 3,

  // block 28
  clIdleTargetOffset = 0,

  // block 42
  vss1Offset = 0,

  // block 52
  knockRetardOffset = 2,
};

inline U16
canRead16( U8 * buf, U8 offset ) {
  return ntohs( *((U16 *) (buf + offset)) );
}

inline void
canWrite16( U8 * buf, U8 offset, U16 val ) {
  *((U16 *) (buf + offset)) = htons( val );
}

inline U8
canRead8( U8 * buf, U8 offset ) {
  return buf[ offset ];
}

inline void
canWrite8( U8 * buf, U8 offset, U8 val ) {
  buf[ offset ] = val;
}

#endif // MS3_CAN_H
