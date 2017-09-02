#ifndef DISPLAY_TYPES_H
#define DISPLAY_TYPES_H

#include <Arduino.h>

#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)


typedef unsigned char U8;
typedef unsigned short int U16;
typedef signed short int S16;
typedef unsigned long int U32;
typedef signed long int S32;

typedef S16 degF;

typedef U16 deciKpa;
typedef U16 deciRatio;
typedef U16 revs;
typedef U16 deciMph;
typedef S16 deciDegAngle;
typedef S16 deciDegF;
typedef S16 deciDegC;
typedef U16 deciVolt;
typedef U16 deciPercent;
typedef S16 degArc;
typedef U16 bar;

typedef U16 seconds;
typedef U16 milliSeconds;
typedef U16 deciPsiGauge;

typedef U32 canAddr;
typedef S16 deciMeters;
typedef U32 centiDegrees;

typedef U32 time_t;

class DegMinSec {
 public:
  DegMinSec() { deg = 0; min = 0; milliSec = 0; }
  
  U8 deg;
  U8 min;
  U32 milliSec;
  bool valid;
  bool present;
};

inline U8 u8FromBcd( char bcd ) {
  return U8( ((bcd >> 4) * 10) + (bcd & 0xf));
}

#endif // DISPLAY_TYPES_H
