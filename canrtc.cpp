#include <Wire.h>
#include <Stream.h>

#include "canrtc.h"

#define DEBUG_RTC 0

CanRtc::CanRtc() {
  txAddrIs( 0 );
  txCanFn_ = 0;
}

void
CanRtc::init() {
  rtc_.begin();

  if( !rtc_.isrunning() ) {
    //    Serial.println( F( "RTC is NOT running!" ) );
    // following line sets the RTC to the date & time this sketch was compiled
    rtc_.adjust( DateTime( __DATE__, __TIME__ ));
  }

  rtc_.BBSQWEnable( false );
  rtc_.enable32kHz( true );
  rtc_.enable32kHz( false );

  char datastr[100];
  rtc_.getControlRegisterData( datastr[0]  );
  Serial.print( F( "control register data for rtc_: " ) );
  Serial.println( datastr );

  Serial.print( F( "RTC is currently:  " ) );
  displayTime( now() );
  Serial.print( F( "RTC temp is:  " ) );
  Serial.println( temp() );

  // clear /EOSC bit
  // Sometimes necessary to ensure that the clock
  // keeps running on just battery power. Once set,
  // it shouldn't need to be reset but it's a good
  // idea to make sure.
  Wire.beginTransmission(0x68); // address DS3231
  Wire.write(0x0E); // select register
  Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
  Wire.endTransmission();
}
  

void
CanRtc::handleCanReq( void * buf, U8 len ) {
  DateTime t = now();
#if DEBUG_RTC
  Serial.print( F( "Read RTC per-request as: " ) );
  displayTime( t );
#endif

  txCanRsp( &t );
}

void
CanRtc::handleCanWrite( void * buf, U8 len ) {
  DateTime dt = convertCanClock( (U8 *) buf, len );
#if DEBUG_RTC
  Serial.print( F( "RTC from MS3 as: " ) );
  displayTime( dt );
#endif

  rtc_.adjust( dt );
}

DateTime
CanRtc::convertCanClock( U8 * buf, U8 len ) {
  // the MS3 sends us 8 bytes, but the last one is always zero (should
  // be the century).

  DateTime ret( u8FromBcd( buf[ 6 ] ) + 2000,
		u8FromBcd( buf[ 5 ] ),
		u8FromBcd( buf[ 4 ] ),
		u8FromBcd( buf[ 2 ] ),
		u8FromBcd( buf[ 1 ] ),
		u8FromBcd( buf[ 0 ] ) );
  return ret;
}  

void
CanRtc::txCanRsp( DateTime * dt ) {
  if( !txCanFn_ ) {
    Serial.println( F( "No tx can function vector!!!" ) );
    return;
  }
  
  U8 txBuf[ 8 ];
  txBuf[ 0 ] = dt->second();
  txBuf[ 1 ] = dt->minute();
  txBuf[ 2 ] = dt->hour();
  txBuf[ 3 ] = dt->dayOfWeek();
  txBuf[ 4 ] = dt->day();
  txBuf[ 5 ] = dt->month();
  int year = dt->year();
  txBuf[ 6 ] = (year) >> 8;
  txBuf[ 7 ] = (year) & 0xff;
  
  (*txCanFn_)( txAddr_, 1, 8, txBuf );
}

DateTime
CanRtc::now() {
  return rtc_.now();
}
  
void
CanRtc::displayTime( DateTime const & dt ) {
  Serial.print( dt.year(), DEC );
  Serial.print( '/');
  Serial.print( dt.month(), DEC );
  Serial.print( '/');
  Serial.print( dt.day(), DEC );
  Serial.print( ' ');
  Serial.print( dt.hour(), DEC) ;
  Serial.print( ':');
  Serial.print( dt.minute(), DEC );
  Serial.print( ':');
  Serial.print( dt.second(), DEC );
  Serial.println();

  Serial.print( F( " since midnight 1/1/1970 = " ) );
  Serial.print( dt.unixtime() );
  Serial.print( "s = " );
  Serial.print( dt.unixtime() / 86400L) ;
  Serial.println( "d" );
}

void
CanRtc::timeAsText( DateTime const & dt, char * buf ) {
  int h = dt.hour();
  int m = dt.minute();
  int s = dt.second();

  int i = 0;
  buf[ i++ ] = ( h / 10 ) + '0';
  buf[ i++ ] = ( h % 10 ) + '0';
  buf[ i++ ] = ':';
  buf[ i++ ] = ( m / 10 ) + '0';
  buf[ i++ ] = ( m % 10 ) + '0';
  buf[ i++ ] = ':';
  buf[ i++ ] = ( s / 10 ) + '0';
  buf[ i++ ] = ( s % 10 ) + '0';
  buf[ i++ ] = 0;
}

  
deciDegF
CanRtc::temp() {
  deciDegF ret;
  rtc_.forceTempConv( true );
  U16 t = rtc_.getTempAsWord();

  // the U16 is a U8 for degrees and a second U8 for centiDegrees, so convert it
  ret = (t & 0xff) % 10;
  ret += 10 * ( t >> 8 );
  return ret;
}

