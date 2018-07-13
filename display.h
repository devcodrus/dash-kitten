#ifndef DK_DISPLAY_H
#define DK_DISPLAY_H

#include <assert.h>
#include <Stream.h>
#include <RTClib.h>
#include "display_types.h"
#include "nextion.h"

// template is the right way to do this, but it sucks memory...
#define DISPLAY_VALUE( t, n, e ) \
  inline const t n() { return (t) value_[ e ]; } \
  inline void n ## Is( t x ) { value_[ e ] = x; dirtyIs( e, true ); }

enum {
  displayManPressure = 0, // argh 'map' is a keyword
  displayAfr,
  displayRpm,
  displayVss,
  displayAfrTarget,
  displaySpark,
  displayEgt,
  displayOilTempPan,
  displayOilTempCooler,
  displayCoolantTemp,
  displayMat,
  displayVoltage,
  displayWarn,
  displayRunTime,
  displayPw,
  displayBaroPressure,
  displayTps,
  displayKnock,
  displayDwell,
  displayFuelPressure,
  displayBoostTarget,
  displayClIdleTarget,
  displayKnockRetard,
  displayFreeRam,
  displayNumSat,
  displayGpsSpeed,
  displayAltitude,
  displayNmeaMsg,
  displayGpggaMsg,
  displayGpmrcMsg,
  displayBogusMsg,
  displayBrakePressure,
  displaySteeringAngle,
  
  displayLast,
};

class Display {
public:
  Display();

  // use the macro to define the cookie-cutter read & write accessors
  DISPLAY_VALUE( deciKpa, manPressure, displayManPressure );
  DISPLAY_VALUE( deciRatio, afr, displayAfr );
  DISPLAY_VALUE( revs, rpm, displayRpm );
  DISPLAY_VALUE( deciMph, vss, displayVss );
  DISPLAY_VALUE( deciRatio, afrTarget, displayAfrTarget );
  DISPLAY_VALUE( deciDegAngle, spark, displaySpark );
  DISPLAY_VALUE( deciDegF, egt, displayEgt );
  DISPLAY_VALUE( deciDegF, oilTempPan, displayOilTempPan );
  DISPLAY_VALUE( deciDegF, oilTempCooler, displayOilTempCooler );
  DISPLAY_VALUE( deciDegF, coolantTemp, displayCoolantTemp );
  DISPLAY_VALUE( deciDegF, mat, displayMat );
  DISPLAY_VALUE( deciVolt, voltage, displayVoltage );
  DISPLAY_VALUE( bool, warn, displayWarn );
  DISPLAY_VALUE( seconds, runTime, displayRunTime );
  DISPLAY_VALUE( milliSeconds, pw, displayPw );
  DISPLAY_VALUE( deciKpa, baroPressure, displayBaroPressure );
  DISPLAY_VALUE( deciPercent, tps, displayTps );
  DISPLAY_VALUE( U16, knock, displayKnock );
  DISPLAY_VALUE( deciDegAngle, dwell, displayDwell );
  DISPLAY_VALUE( deciPsiGauge, fuelPressure, displayFuelPressure );
  DISPLAY_VALUE( deciKpa, boostTarget, displayBoostTarget );
  DISPLAY_VALUE( deciDegAngle, knockRetard, displayKnockRetard );
  DISPLAY_VALUE( revs, clIdleTarget, displayClIdleTarget );
  DISPLAY_VALUE( U32, freeRam, displayFreeRam );
  DISPLAY_VALUE( U8, numSat, displayNumSat );
  DISPLAY_VALUE( deciMeters, altitude, displayAltitude );
  DISPLAY_VALUE( deciMph, gpsSpeed, displayGpsSpeed );
  DISPLAY_VALUE( U32, nmeaMsg, displayNmeaMsg );
  DISPLAY_VALUE( U32, gpggaMsg, displayGpggaMsg );
  DISPLAY_VALUE( U32, gpmrcMsg, displayGpmrcMsg );
  DISPLAY_VALUE( U32, bogusMsg, displayBogusMsg );
  DISPLAY_VALUE( degArc, steeringAngle, displaySteeringAngle );
  DISPLAY_VALUE( bar, brakePressure, displayBrakePressure );

  void printHigh( bool force = false );
  void printMedium( bool force = false );
  void printLow( bool force = false );
  void printAll( bool force = false );
  void maybePrintVal( U8 valueIndex, bool force );

  void refresh_labels();

  // write once
  void streamIs( Stream * );
  void nObjIs( U8 valueIndex, NextionObject * n );
  void initializedIs( bool );

  // debug
  void allDirtyIs( bool b ) {
    for( int i = 0; i < displayLast; i++ ) {
      dirtyIs( i, b );
    }
  }

  void pushColors() {
    for( int i = 0; i < displayLast; i++ ) {
      if( nObj[ i ] ) {
	nObj[ i ]->pushColor();
      }
    }
  }

private:

  U32 value_[ displayLast ];

  void dirtyIs( U8 valueIndex, bool );
  bool dirty( U8 valueIndex );
  U32 dirty_;

  // id, lid, suffix, scale, decimals, red_low, yellow_low, yellow_high,
  // red_high, ref
  NextionObject * nObj[ displayLast ];

  Stream * stream_;

  bool initialized_;
};

#endif // DK_DISPLAY_H
