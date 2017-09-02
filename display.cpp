#include "display.h"

#define assert( val )


Display::Display() {
  memset( this, 0, sizeof( *this ) );

  initialized_ = false;
}

void
Display::nObjIs( U8 valueIndex, NextionObject * n ) {
  //  assert( !nObj[ valueIndex ] );
  nObj[ valueIndex ] = n;
  if( stream_ ) {
    nObj[ valueIndex ]->streamIs( stream_ );
  }
}

void
Display::streamIs( Stream * s ) {
  //  assert( !stream_ );
  stream_ = s;
  for( int i = 0; i < displayLast; i++ ) {
    if( nObj[ i ] ) {
      nObj[ i ]->streamIs( stream_ );
    }
  }
}

void
Display::initializedIs( bool b ) {
  //  assert( !initialized_ && b );
  initialized_ = b;

  //  assert( stream_ );
  
  for( int i = 0; i < displayLast; i++ ) {
    dirtyIs( i, true );
  }
  printAll();
}

void
Display::dirtyIs( U8 valueIndex, bool newDirty ) {
  if( newDirty ) {
    dirty_ |= 1 << valueIndex;
  } else {
    dirty_ &= ~(1 << valueIndex);
  }
}

bool
Display::dirty( U8 valueIndex ) {
  return (dirty_ & (1 << valueIndex)) ? true : false;
}
  
void
Display::printHigh( bool force ) {
  maybePrintVal( displayManPressure, force );
  maybePrintVal( displayRpm, force );
}

void
Display::printMedium( bool force ) {
  maybePrintVal( displayAfr, force );
  maybePrintVal( displayVss, force );
  maybePrintVal( displayAfrTarget, force );
  maybePrintVal( displaySpark, force );
}

void
Display::printLow( bool force ) {
  maybePrintVal( displayEgt, force );
  maybePrintVal( displayCoolantTemp, force );
  maybePrintVal( displayMat, force );
  maybePrintVal( displayVoltage, force );
  maybePrintVal( displayWarn, force );
  maybePrintVal( displayRunTime, force );
  maybePrintVal( displayOilTemp, force );
  maybePrintVal( displayFuelPressure, force );
  maybePrintVal( displayFreeRam, force );
  maybePrintVal( displayGpsSpeed, force );
  maybePrintVal( displayAltitude, force );
  maybePrintVal( displayNumSat, force );

  maybePrintVal( displayGpggaMsg, force );
  maybePrintVal( displayGpmrcMsg, force );
  maybePrintVal( displayNmeaMsg, force );
  maybePrintVal( displayBogusMsg, force );

  maybePrintVal( displaySteeringAngle, force );
  maybePrintVal( displayBrakePressure, force );
}

void
Display::printAll( bool force ) {
//  Serial.println( "Display::printAll" );
  printHigh( force );
  printMedium( force );
  printLow( force );
}

void
Display::maybePrintVal( U8 valueIndex, bool force ) {
  if( !nObj[ valueIndex ] ) {
//    Serial.print( "nothing registered for value " );
//    Serial.print( valueIndex );
    return;
  }
  if( force || dirty( valueIndex ) ) {
#if 0
    if( valueIndex == displayFuelPressure ) {
      Serial.print( "pushing value " );
      Serial.print( valueIndex );
      Serial.print( " to " );
      Serial.println( value_[ valueIndex ] );
    }
#endif
    nObj[ valueIndex ]->val( value_[ valueIndex ] );
    dirtyIs( valueIndex, false );
  }
}

void
Display::refresh_labels() {
  nObj[ displayManPressure ]->label( "MAP kPa" );
  nObj[ displayAfr ]->label("AFR");
  nObj[ displayRpm ]->label("RPM");
  nObj[ displayAfrTarget ]->label("AFR trgt");
  nObj[ displayVss ]->label("VSS mph");
  nObj[ displaySpark ]->label("Advance");
  nObj[ displayEgt ]->label("EGT degF");
  //  nObj[ displayRunTime ]->label("MSticks");
}
