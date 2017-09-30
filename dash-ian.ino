#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>
#include <RTClib.h>
#include <Adafruit_MAX31855.h>
#include "thermistor.h"
#include "display_types.h"
#include "display.h"
#include "nextion.h"
#include "ms3Can.h"
#include "canrtc.h"
#include "dash-gps.h"

#define LCD_TX 18
#define LCD_RX 5
#define CAN_CS 9
#define CAN_INT 2

#define DEBUG_CAN_LOOP 0

#define MAX_NMEA_LEN 120

//SoftwareSerial lcd( LCD_RX, LCD_TX );
HardwareSerial * lcd = &Serial1;
HardwareSerial * nmea = &Serial2;
//HardwareSerial * rltc = &Serial2; // &Serial3;
NextionObject map_g("v0", "l0", "", 10, 0, 0, 0, 2600, 2800, 20);
NextionObject afr_g("v1", "l1", "", 10, 1, 100, 105, 150, 155, 100);
NextionObject rpm_g("v2", "l2", "", 1, 0, 0, 0, 6500, 7000, 100);
NextionObject vss_g("v3", "l3", "", 1, 0, -32768, -32768, 32767,
		    32767, 200);
NextionObject art_g("v4", "l4", "", 10, 1, -32768, -32768, 32767,
		    32767, 200);
NextionObject spk_g("v5", "l5", "", 10, 1, -32768, -32768, 32767,
		    32767, 50);
NextionObject egt_g("v6", "l6", "", 1, 0, 500, 700, 1600, 1800, 100);
  // hack, need to rework display layout for this
NextionObject oit_g("c0", "", "oilF", 10, 0, -32768, 1700, 2400,
		    2600, 1000);
NextionObject clt_g("c1", "", "cltF", 10, 0, 600, 1700, 2050, 2200,
		    1010);
NextionObject mat_g("c2", "", "matF", 10, 0, 200, 400, 1400, 1600, 200);
NextionObject bat_g("b3", "", "v", 10, 1, 120, 130, 147, 150, 500);
//NextionObject warn_g("warn", "", "", 0, 0, 0, 0, 0, 0, 50);
NextionObject time_g("b0", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject fp_g("b1", "", "fpP", 10, 1, 20, 30, 65, 70, 1);
NextionObject runTime_g("c3", "", "t",  1, 0, -32768, -32768, 32767,
			32767, 50);
NextionObject freeRam_g("b2", "", "B", 1, 0, 100, 200, 32767, 32767, 50 );
NextionObject numSat_g( "ll5", "", "sat", 1, 0, 3, 5, 100, 100, 50 );
NextionObject altitude_g( "ll6", "", "m", 10, 1, -100, -100, 1000, 3000, 50 );
NextionObject gpsSpeed_g( "ll7", "", "mph", 10, 1, 0, 0, 100, 120, 50 );
NextionObject gpsTime_g("ll4", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject nmeaMsg_g( "ll0", "", "msg", 1, 0, 0, 0, 1000000, 1000000, 50 );
NextionObject gpmrcMsg_g( "ll1", "", "mrc", 1, 0, 0, 0, 1000000, 1000000, 50 );
NextionObject gpggaMsg_g( "ll2", "", "gga", 1, 0, 0, 0, 1000000, 1000000, 50 );
NextionObject bogusMsg_g( "ll3", "", "bog", 1, 0, 0, 0, 1, 1, 50 );

Display MainDisplay;
CanRtc MainCanRtc;

NextionObject lat_g( "lat", "latl", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject long_g( "long", "longl", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject fix_g( "fix", "fixl", "", 0, 0, 0, 0, 0, 0, 50 );

#define NUM_DEBUG 6

NextionObject debug0_g( "debug0", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject debug1_g( "debug1", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject debug2_g( "debug2", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject debug3_g( "debug3", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject debug4_g( "debug4", "", "", 0, 0, 0, 0, 0, 0, 50 );
NextionObject debug5_g( "debug5", "", "", 0, 0, 0, 0, 0, 0, 50 );

NextionObject steeringAngle_g( "data0", "", "", 1, 1, 0, 0, 2000, 2000, 100 );
//NextionObject steeringAngle_g( "data0", "", "deg", 1, 0, 0, 0, 0, 100 );
NextionObject brakePressure_g( "data1", "", "deg", 1, 0, 0, 0, 2000, 2000, 100 );

char * debugStr[ NUM_DEBUG ];
NextionObject * debugObj[ NUM_DEBUG ];
bool debugDirty[ NUM_DEBUG ];

void initDebug();
void maybeUpdateDebug();
void addDebug( int, char * );
void rmDebug( int );

// ugh
#define MS3_RTC_REQ_ADDR 28869304
#define MS3_RTC_WRITE_ADDR 644
#define MS3_RTC_RSP_ADDR 0x9352838

#define PIN_THERMO_CS 6
Adafruit_MAX31855 egtThermo( PIN_THERMO_CS );
//Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

MCP_CAN CAN0( CAN_CS );

#define MAX_CAN_VECTORS 64
typedef void (*canVector)( U8 * );
canVector canVectorSet[ MAX_CAN_VECTORS ];
void initializeCanVectors();
void handleCan();
U32 advanceLed( U32 );
int freeRam();

void
canTx( canAddr a, U8 extended, U8 len, void * buf ) {
  CAN0.sendMsgBuf( a, extended, len, buf );
}

#define LED_WARN_PIN 11
#define LED_CRIT_PIN 12
#define LED_KIT_PIN 13

#define OIL_TEMP_PIN A12
THERMISTOR oilTempTherm( OIL_TEMP_PIN, 3075, 3820, 220 );

#define FUEL_PRESSURE_PIN A7
#define STEERING_ANGLE_PIN A6

void readFuelPressure();
void readSteeringAngle();
void readOilTemp();
void readTime();
void readEgt();
void canSendAdc( U16 canId, U8 a, U8 b, U8 c, U8 d );
void canSendOtherData( U16 canId );

#define ADC_PAGE_1_ID 0x20            // Destination CAN ID for Page 1
#define ADC_PAGE_2_ID 0x21            // and page 2
#define ADC_PAGE_3_ID 0x30            // and page 3

void maybeUpdateLeds();
bool kittenLedConfig = false;
bool kittenLedStatus = false;
bool critLedConfig = false;
bool critLedStatus = false;
bool warnLedConfig = false;
bool warnLedStatus = false;

void handleNmea();
void handleRltc();
void refreshGps();
void displayField( String * lcd, String id, String text );

GpsRx gpsRx;

void
setup() {
  lcd->begin( 115200 );
  lcd->print( EOC );

  nmea->begin( 4800 );
  //rltc->begin( 57600 );
  
  Serial.begin( 115200 );
  Serial.println( F( "boot" ) );

  MainDisplay.streamIs( lcd );
  MainDisplay.nObjIs( displayManPressure, &map_g );
  MainDisplay.nObjIs( displayAfr, &afr_g );
  MainDisplay.nObjIs( displayRpm, &rpm_g );
  MainDisplay.nObjIs( displayVss, &vss_g );
  MainDisplay.nObjIs( displayAfrTarget, &art_g );
  MainDisplay.nObjIs( displaySpark, &spk_g );
  MainDisplay.nObjIs( displayEgt, &egt_g );
  MainDisplay.nObjIs( displayOilTemp, &oit_g );
  MainDisplay.nObjIs( displayCoolantTemp, &clt_g );
  MainDisplay.nObjIs( displayMat, &mat_g );
  MainDisplay.nObjIs( displayVoltage, &bat_g );
  //  MainDisplay.nObjIs( displayWarn, &warn_g );
  MainDisplay.nObjIs( displayRunTime, &runTime_g );
  MainDisplay.nObjIs( displayFuelPressure, &fp_g );
  MainDisplay.nObjIs( displayFreeRam, &freeRam_g );
  MainDisplay.nObjIs( displayAltitude, &altitude_g );
  MainDisplay.nObjIs( displayNumSat, &numSat_g );
  MainDisplay.nObjIs( displayGpsSpeed, &gpsSpeed_g );
  MainDisplay.nObjIs( displayNmeaMsg, &nmeaMsg_g );
  MainDisplay.nObjIs( displayGpggaMsg, &gpggaMsg_g );
  MainDisplay.nObjIs( displayGpmrcMsg, &gpmrcMsg_g );
  MainDisplay.nObjIs( displayBogusMsg, &bogusMsg_g );
  //MainDisplay.nObjIs( displaySteeringAngle, &steeringAngle_g );
  //MainDisplay.nObjIs( displayBrakePressure, &brakePressure_g );
  
  CAN0.begin( CAN_1000KBPS );

  initializeCanVectors();

  Wire.begin();
  digitalWrite( A4, HIGH );     // pull them up so we can use for I2C
  digitalWrite( A5, HIGH );     // ditto

  // Serial.println( "before RTC init" );
  MainCanRtc.init();
  MainCanRtc.txAddrIs( MS3_RTC_RSP_ADDR );
  MainCanRtc.txCanFnIs( canTx );
  // Serial.println( "after RTC init" );

  time_g.streamIs( lcd );
  gpsTime_g.streamIs( lcd );
  lat_g.streamIs( lcd );
  long_g.streamIs( lcd );
  fix_g.streamIs( lcd );

  //  delay( 500 );
  lcd->print( "page main0" EOC );

  pinMode( LED_WARN_PIN, OUTPUT );
  pinMode( LED_CRIT_PIN, OUTPUT );
  pinMode( LED_KIT_PIN, OUTPUT );
  
  digitalWrite( LED_WARN_PIN, HIGH );
  digitalWrite( LED_CRIT_PIN, HIGH );
  digitalWrite( LED_KIT_PIN, HIGH );

  pinMode( FUEL_PRESSURE_PIN, INPUT );
  pinMode( OIL_TEMP_PIN, INPUT );

  gpsRx.debugStreamIs( &Serial );

  initDebug();
}

const U32 lowInterval = 250;
const U32 medInterval = 100;
const U32 highInterval = 50;
const U32 analogInterval = 50;
const U32 ledInterval = 200;
const U32 colorInterval = 1000;

void
loop() {
  static bool first = true;
  if( first ) {
    // Serial.println( F( "first loop" ) );
    MainDisplay.initializedIs( true );
    first = false;
  }

  static U32 nextLow = 0;
  static U32 nextMed = 0;
  static U32 nextHi = 0;
  static U32 nextLed = 0;
  static U32 nextAnalog = 0;
  static U32 nextColor = 0;

  U32 now = millis();

  if( now > nextAnalog ) {
    canSendAdc( ADC_PAGE_1_ID, A0, A1, A2, A3 );
    canSendAdc( ADC_PAGE_2_ID, A1, A5, A6, A7 );
    nextAnalog += analogInterval;
  }
  
  if( now > nextHi ) {
    MainDisplay.printHigh( true );
    readFuelPressure();
    readSteeringAngle();
    canSendOtherData( ADC_PAGE_3_ID );
    nextHi += highInterval;
  }
  
  if( now > nextMed ) {
    MainDisplay.printMedium( true );
    nextMed += medInterval;
  }
  
  if( now > nextLow ) {
    readTime();
    readOilTemp();
    readEgt();

    MainDisplay.freeRamIs( freeRam() );

    MainDisplay.printLow( true );
    nextLow += lowInterval;
    MainDisplay.refresh_labels();

    refreshGps();
  }

  if( now > nextColor ) {
    MainDisplay.pushColors();
    nextColor += colorInterval;
    maybeUpdateDebug();
  }

  if( !digitalRead( CAN_INT ) ) {
    handleCan();
  }

#if 0
  if( now > nextLed ) {
    nextLed = advanceLed( now );
  }
#endif

  maybeUpdateLeds();

  handleNmea();
  //handleRltc();

  return;
}

U32
advanceLed( U32 now ) {
  static U8 ledState = 0;
  U32 nextLed = now;

  switch( ledState ) {
  case 0:
    digitalWrite( LED_WARN_PIN, LOW );
    digitalWrite( LED_CRIT_PIN, HIGH );
    digitalWrite( LED_KIT_PIN, HIGH );
    ledState = 1;
    nextLed += 300;
    break;
  case 1:
    digitalWrite( LED_WARN_PIN, HIGH );
    digitalWrite( LED_CRIT_PIN, LOW );
    digitalWrite( LED_KIT_PIN, HIGH );
    ledState = 2;
    nextLed += 200;
    break;
  case 2:
    digitalWrite( LED_WARN_PIN, HIGH );
    digitalWrite( LED_CRIT_PIN, HIGH );
    digitalWrite( LED_KIT_PIN, LOW );
    ledState = 3;
    nextLed += 300;
    break;
  case 3:
    digitalWrite( LED_WARN_PIN, HIGH );
    digitalWrite( LED_CRIT_PIN, LOW );
    digitalWrite( LED_KIT_PIN, HIGH);
    ledState = 0;
    nextLed += 200;
    break;
  }
  return nextLed;
}

  
void
handleCan() {
  U8 len;
  static U8 buf[ 8 ];

  CAN0.readMsgBuf( &len, buf );
  U32 canId = CAN0.getCanId();
  U32 ms3CanId = canId - ms3CanBase;

#if DEBUG_CAN_LOOP
  Serial.print( F( "handleCan, id is " ) );
  Serial.println( canId, HEX );
#endif

  switch( canId ) {
    case MS3_RTC_REQ_ADDR:
      MainCanRtc.handleCanReq( buf, len );
      break;
    case MS3_RTC_WRITE_ADDR:
      MainCanRtc.handleCanWrite( buf, len );
      break;
    default:
      if( canVectorSet[ ms3CanId ] && ms3CanId < MAX_CAN_VECTORS ) {
	canVectorSet[ ms3CanId ]( buf );
      }
      break;
  }
}

void
handleCanBlock0( U8 * buf ) {
  MainDisplay.runTimeIs( canRead16( buf, secOffset ) );
  MainDisplay.pwIs( canRead16( buf, pw1Offset ) );
  MainDisplay.rpmIs( canRead16( buf, rpmOffset ) );
}

void
handleCanBlock1( U8 * buf ) {
  MainDisplay.sparkIs( canRead16( buf, advOffset ) );
  MainDisplay.afrTargetIs( canRead8( buf, afrTarget1Offset ) );
}

void
handleCanBlock2( U8 * buf ) {
  MainDisplay.baroPressureIs( canRead16( buf, baroOffset ) );
  MainDisplay.manPressureIs( canRead16( buf, mapOffset ) );
  MainDisplay.matIs( canRead16( buf, matOffset ) );
  MainDisplay.coolantTempIs( canRead16( buf, cltOffset ) );
}
void
handleCanBlock3( U8 * buf ) {
  MainDisplay.tpsIs( canRead16( buf, tpsOffset ) );
  MainDisplay.voltageIs( canRead16( buf, battOffset ) );
  MainDisplay.afrIs( canRead16( buf, ego1Offset ) );
}
void
handleCanBlock4( U8 * buf ) {
  MainDisplay.knockIs( canRead16( buf, knockValOffset ) );
}
void
handleCanBlock9( U8 * buf ) {
  MainDisplay.dwellIs( canRead16( buf, dwellOffset ) );
}
void
handleCanBlock13( U8 * buf ) {
  MainDisplay.boostTargetIs( canRead16( buf, boostTarget1Offset ) );
  //  MainDisplay.boostTarget2Is( canRead16( buf, boostTarget2Offset ) );
}
void
handleCanBlock17( U8 * buf ) {
  MainDisplay.dwellIs( canRead16( buf, dwellOffset ) );
}
void
handleCanBlock28( U8 * buf ) {
  MainDisplay.clIdleTargetIs( canRead16( buf, clIdleTargetOffset ) );
}
void
handleCanBlock42( U8 * buf ) {
  // hack
  MainDisplay.vssIs( (canRead16( buf, vss1Offset ) / 4.4) );
}
void
handleCanBlock52( U8 * buf ) {
  MainDisplay.knockRetardIs( canRead16( buf, knockRetardOffset ) );
}

void
initializeCanVectors() {
  memset( canVectorSet, 0, MAX_CAN_VECTORS * sizeof( canVector ) );
  canVectorSet[ 0 ] = handleCanBlock0;
  canVectorSet[ 1 ] = handleCanBlock1;
  canVectorSet[ 2 ] = handleCanBlock2;
  canVectorSet[ 3 ] = handleCanBlock3;
  canVectorSet[ 4 ] = handleCanBlock4;
  canVectorSet[ 9 ] = handleCanBlock0;
  canVectorSet[ 13 ] = handleCanBlock13;
  canVectorSet[ 17 ] = handleCanBlock17;
  canVectorSet[ 28 ] = handleCanBlock28;
  canVectorSet[ 42 ] = handleCanBlock42;
  canVectorSet[ 52 ] = handleCanBlock52;
}

char debugBuf[ 33 ];
void
readFuelPressure() {
  U16 fpAdc = analogRead( FUEL_PRESSURE_PIN );

  // sensor outputs from 0.5 to 4.5 out of a nominal 5, for 0-100 psi
  // gauge.  ADC is 10 bit, so max value is 1023, but we will only
  // read 0.5*1024 to 4.5*1024 (102.4 to 921.6).  Subtract out 102.4
  // so that we have 0 to 819.2 for 0 - 100 psi.
  float fpPsig = (fpAdc - 102.4) * (100 / 819.2);
  //  float fpKpg = (fpPsig / 14.5) * 100;
  //  float fpKpa = fpKpg + (MainDisplay.baroPressure() / 10);
  MainDisplay.fuelPressureIs( fpPsig * 10 );

#if 0
  static U32 lastDebug = 0;

  const U32 fpDeltaConst = 303 - ( MainDisplay.baroPressure() / 10 );
  U32 manPress = MainDisplay.manPressure() / 10;
  S32 fpError = (fpKpa - manPress) - fpDeltaConst;
  bool oldKlc = kittenLedConfig;
  U32 now = millis();
  //  kittenLedConfig = ( MainDisplay.rpm() > 1000 ) && ( abs( fpError ) > 100 );
  kittenLedConfig = ( abs( fpError ) > 100 );
  if( kittenLedConfig ) {
    if( lastDebug == 0 || (now - lastDebug) > 2000) {
      snprintf( debugBuf, 11, "FPer %4d,", fpError );
      snprintf( debugBuf + 10, 9, "MAP %3d,", manPress );
      snprintf( debugBuf + 18, 8, "FP %3d,", (U32) fpKpa );
      snprintf( debugBuf + 25, 9, "del %3d", fpDeltaConst );
      debugBuf[ 32 ] = 0;
      addDebug( 0, debugBuf );
      lastDebug = now;
    }
  } else if( oldKlc ) {
    rmDebug( 0 );
  }
#endif
}

// all the way left     200
// one turn left        270
// half turn left       377
// center               485
// half turn right      596
// one right right      700
// all the way right    800

// center == 485, one turn == 108, .3 per degree
#define SA_CENTER_ADC     485
#define SA_ADC_PER_DEGREE 1.67

void
readSteeringAngle() {
  S32 saAdc = analogRead( STEERING_ANGLE_PIN );
  float saDegrees = ((saAdc - SA_CENTER_ADC) * SA_ADC_PER_DEGREE);
  MainDisplay.steeringAngleIs( saDegrees );
}

void
readEgt() {
  double egtC = egtThermo.readCelsius();
  if( isnan( egtC ) ) {
    MainDisplay.egtIs( -5 );
  } else {
    deciDegF eftF = ((egtC * 18) + 320)/10;
    MainDisplay.egtIs( eftF );
  }
}

void
readOilTemp() {
  U16 serialRes = 217;
  U16 nominalRes = 3075;
  float bCoeff = 3820;
  
  U16 otAdc = analogRead( OIL_TEMP_PIN );
  float resistance = serialRes * ( ( 1024.0 / otAdc ) - 1 );
  float steinhart = log( resistance / nominalRes ) / bCoeff;
  steinhart += 1.0 / ( 25 + 273.15 );
  steinhart = 1 / steinhart;
  deciDegC otc = (steinhart - 273.15) * 10;

  deciDegF otf = (otc * 9.0 / 5) + 320;
  MainDisplay.oilTempIs( otf );
}

void
readTime() {
  char buf[ 10 ];
  MainCanRtc.timeAsText( MainCanRtc.now(), buf );
  time_g.txt( buf );
  MainCanRtc.timeAsText( gpsRx.now(), buf );
  gpsTime_g.txt( buf );
}

void
canSendAdc( U16 canId, U8 a, U8 b, U8 c, U8 d ) {
  U16 samples[ 4 ];
  samples[ 0 ] = htons( analogRead( a ) );
  samples[ 1 ] = htons( analogRead( b ) );
  samples[ 2 ] = htons( analogRead( c ) );
  if( canId == ADC_PAGE_2_ID ) {
    samples[ 3 ] = htons( analogRead( d ) - 30 );
  } else {
    samples[ 3 ] = htons( analogRead( d ) );
  }
  CAN0.sendMsgBuf( canId, 0, 8, (U8 *) &samples );
}

void
canSendOtherData( U16 canId ) {
  U16 data[ 4 ];
  data[ 0 ] = htons( MainDisplay.oilTemp() );
  data[ 1 ] = htons( MainDisplay.fuelPressure() );
  data[ 2 ] = htons( MainDisplay.egt() );
  data[ 3 ] = htons( MainDisplay.steeringAngle() );
  CAN0.sendMsgBuf( canId, 0, 8, (U8 *) &data );
}


int
freeRam()  {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void
maybeUpdateLeds() {
  U32 afrError = abs( (MainDisplay.afr() - MainDisplay.afrTarget()) );
  //  critLedConfig = (MainDisplay.rpm() > 1500) && (afrError > 10);

  bool oldWlc = warnLedConfig;
  warnLedConfig = (MainDisplay.knockRetard() != 0);
  if( warnLedConfig != oldWlc ) {
    if( warnLedConfig ) {
      addDebug( 1, "MS3 is retarding ignition due to knock" );
    } else {
      rmDebug( 1 );
    }
  }

  if( kittenLedConfig != kittenLedStatus ) {
    digitalWrite( LED_KIT_PIN, kittenLedConfig ? LOW : HIGH );
    kittenLedStatus = kittenLedConfig;
  }

  if( warnLedConfig != warnLedStatus ) {
    digitalWrite( LED_WARN_PIN, warnLedConfig ? LOW : HIGH );
    warnLedStatus = warnLedConfig;
  }

  if( critLedConfig != critLedStatus ) {
    digitalWrite( LED_CRIT_PIN, critLedConfig ? LOW : HIGH );
    critLedStatus = critLedConfig;
  }

}

void
handleNmea() {
  static char buf[ MAX_NMEA_LEN ];
  static int bufPos = 0;
  const int maxToRead = 10;

  for( int i = 0; ( i < maxToRead ) && ( nmea->available() > 0 ); i++ ) {
    char c = nmea->read();
    switch( c ) {
    case '\r':
      break;
    case '\n':
      buf[ bufPos ] = 0;
      gpsRx.newMsg( buf );
      bufPos = 0;
      break;
    default:
      buf[ bufPos ] = c;
      bufPos++;
      break;
    }
    if( bufPos >= MAX_NMEA_LEN ) {
      memset( buf, 0, sizeof( buf ) );
      bufPos = 0;
    }
  }

  MainDisplay.numSatIs( gpsRx.numSats() );
  MainDisplay.altitudeIs( gpsRx.altitude() );
  MainDisplay.gpsSpeedIs( gpsRx.speed() );

  MainDisplay.nmeaMsgIs( gpsRx.nmeaMsgCount() );
  MainDisplay.bogusMsgIs( gpsRx.bogusNmeaMsgCount() );
  MainDisplay.gpmrcMsgIs( gpsRx.gpmrcCount() );
  MainDisplay.gpggaMsgIs( gpsRx.gpggaCount() );
}

void
refreshGps() {
  char buf[ 60 ];
  lat_g.txt( gpsRx.latString( buf, sizeof( buf ) ) );
  lat_g.label( "LAT" );
  long_g.txt( gpsRx.longString( buf, sizeof( buf ) ) );
  long_g.label( "LONG" );
  fix_g.txt( gpsRx.fixTypeString() );
  fix_g.label( "FIX" );
}

#if 0
void
handleRltc() {
  for( int i = 0; i < 20 && rltc->available(); i++ ) {
    char c = rltc->read();
    Serial.print( c );
  }
}
#endif

void
initDebug() {
  debugObj[ 0 ] = &debug0_g;
  debugObj[ 1 ] = &debug1_g;
  debugObj[ 2 ] = &debug2_g;
  debugObj[ 3 ] = &debug3_g;
  debugObj[ 4 ] = &debug4_g;
  debugObj[ 5 ] = &debug5_g;

  for( int i = 0; i < NUM_DEBUG; i++ ) {
    debugObj[ i ]->streamIs( lcd );
    debugStr[ i ] = "--";
    debugDirty[ i ] = true;
  }
}

void
maybeUpdateDebug() {
  for( int i = 0; i < NUM_DEBUG; i++ ) {
    if( debugDirty[ i ] ) {
      debugObj[ i ]->txt( debugStr[ i ] );
    }
  }
}

void
addDebug( int i, char * str ) {
  debugStr[ i ] = str;
  debugDirty[ i ] = true;
  maybeUpdateDebug();
}

void
rmDebug( int i ) {
  addDebug( i, "--" );
}
