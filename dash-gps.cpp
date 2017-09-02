#include "dash-gps.h"

#define GPS_DEBUG 0

#if GPS_DEBUG
#define DEBUGLOG( x )\
  do { \
    if( debugStream_ ) { \
      debugStream_->println( (x) ); \
    } \
  } while( 0 )
#else
#define DEBUGLOG( x )
#endif

GpsRx::GpsRx() {
  now_ = DateTime( 0, 0, 0, 0, 0, 0 );
  north_ = false;
  west_ = false;
  altitude_ = 0;
  speed_ = 0;
  course_ = 0;
  numSats_ = 0;
  nmeaMsgCount_ = 0;
  gpmrcCount_ = 0;
  gpggaCount_ = 0;
  otherCount_ = 0;
  bogusNmeaMsgCount_ = 0;

  fixType_ = GpsFixNone;
}

void
GpsRx::newMsg( const char * msg ) {
  // only doing GPMRC and GPGGA right now
  static bool seenOneGoodMsg = false;

  if( (msg[ 0 ] != '$') || (msg[ 1 ] != 'G') || (msg[ 2 ] != 'P') ) {
    if( seenOneGoodMsg ) {
      nmeaMsgCount_++;
      bogusNmeaMsgCount_++;
    }
    DEBUGLOG( F( "bogus stream" ) );
    return;
  }

  if( !seenOneGoodMsg ) {
    seenOneGoodMsg = true;
  }
  nmeaMsgCount_++;
  if( msg[ 3 ] == 'R' ) {
    gpmrcCount_++;
    newGprmc( msg );
  } else if( msg[ 3 ] == 'G' ) {
    gpggaCount_++;
    newGpgga( msg );
  } else {
    otherCount_++;
  }
}

#define charToInt( c ) ((c) - '0')
const U8 maxFloatLen = 20;

StupidFloat
GpsRx::readStupidFloat( const char * pos, U8 exp ) {
  StupidFloat ret;
  ret.val = 0;
  ret.exp = 0;
  ret.valid = false;
  bool negative = false;

  if( *pos == '-' ) {
    negative = true;
    pos++;
  }

  U8 i = 0;
  bool foundDot = false;
  while( *pos && (*pos != ',') && (i < maxFloatLen) ) {
    i++;
    if( *pos == '.' ) {
      foundDot = true;
      pos++;
      continue;
    }
    if( foundDot ) {
      ret.exp++;
      if( ret.exp > exp ) {
	// we got to max precision, truncate here
	break;
      }
    }
    ret.val *= 10;
    ret.val += charToInt( *pos );
    pos++;
  }
  if( i >= maxFloatLen ) {
    // bogus
    return ret;
  }
  // if we didn't find the desired precision, then extend with zeros
  while( ret.exp < exp ) {
    ret.val *= 10;
    ret.exp++;
  }
  // in case we truncated, find end of string
  while( *pos && *pos != ',' && (i < maxFloatLen) ) {
    i++;
    pos++;
  }
  if( i >= maxFloatLen ) {
    // also bogus
    return ret;
  }
  ret.comma = pos;

  if( negative ) {
    ret.val *= -1;
  }
  ret.valid = true;
  return ret;
}

Gps3Val
GpsRx::read3Val( const char * pos ) {
  Gps3Val val;
  val.val1 = (10 * charToInt( *(pos+0) )) + charToInt( *(pos+1) );
  val.val2 = (10 * charToInt( *(pos+2) )) + charToInt( *(pos+3) );
  val.val3 = (10 * charToInt( *(pos+4) )) + charToInt( *(pos+5) );
  return val;
}

DateTime
GpsRx::readDateTime( const char * hmsPos, const char * dmyPos ) {
  Gps3Val hms = read3Val( hmsPos );
  Gps3Val dmy = read3Val( dmyPos );
  return DateTime( 2000 + dmy.val3, dmy.val2, dmy.val1,
		   hms.val1, hms.val2, hms.val3 );
}

DegMinSec
GpsRx::readDegMinSec( char **posPtr ) {
  DegMinSec dms;
  dms.valid = false;
  dms.present = false;
  char * pos = *posPtr;

  // This means there's no data
  if( *pos == ',' ) {
    // skip the comma
    (*posPtr)++;
    return dms;
  }
  dms.present = true;

  if( pos[ 4 ] == '.' ) {
    dms.deg = (10 * charToInt( *pos )) + charToInt( *(pos+1) );
    pos += 2;
    dms.min = (10 * charToInt( *pos )) + charToInt( *(pos+1) );
    pos += 2;
  } else {
    if( pos[ 5 ] != '.' ) {
      DEBUGLOG( F( "bogus DMS" ) );
    }
    dms.deg = ( (100 * charToInt( *pos )) +
		(10 * charToInt( *(pos+1) )) +
		charToInt( *(pos+1) ) );
    pos += 3;
    dms.min = (10 * charToInt( *pos )) + charToInt( *(pos+1) );
    pos += 2;
  }
  pos++;
  
  StupidFloat temp = readStupidFloat( pos, 0 );
  if( !temp.valid ) {
    return dms;
  }
  dms.milliSec = temp.val;
  (*posPtr) = temp.comma;

  dms.valid = true;
  return dms;
}


// $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
// 01234567890123456789
//           1        LAT     N LONG     W speed
//                                             course
//                                                 date   magnetVar

#define RMC_TIMESTAMP_OFFSET 7
#define RMC_VALID_OFFSET 17
#define RMC_LAT_OFFSET 19

void
GpsRx::newGprmc( const char * msg ) {

  DegMinSec newLat, newLng;
  bool newNorth, newWest;
  StupidFloat temp;
  deciMph newSpeed;
  centiDegrees newCourse;
  DateTime newNow;
  GpsFixType newFixType;

  DEBUGLOG( msg );

  if( msg[ RMC_VALID_OFFSET ] != 'A' ) {
    // ignore invalid messages;
    DEBUGLOG( F( "RMC invalid fix" ) );
    return;
  }
  if( fixType_ == GpsFixNone ) {
    newFixType = GpsFixNormal;
  }

  char * cur = msg + RMC_LAT_OFFSET;
  newLat = readDegMinSec( &cur );
  if( !newLat.valid ) {
    DEBUGLOG( F( "bogus lat in rmc" ) );
    goto error;
  }
  cur++;
  newNorth = ( *cur == 'N' );
  cur += (*cur == ',') ? 1 : 2;
  newLng = readDegMinSec( &cur );
  if( !newLng.valid ) {
    DEBUGLOG( F( "bogus long in rmc" ) );
    goto error;
  }
  cur++;
  newWest = ( *cur == 'W' );
  cur += (*cur == ',') ? 1 : 2;

  temp = readStupidFloat( cur, 3 );
  if( !temp.valid ) {
    DEBUGLOG( F( "bogus speed in rmc" ) );
    goto error;
  }
  // speed was in knots, convert to deciMph
  newSpeed = temp.val / 899;
  cur = temp.comma + 1;
  
  temp = readStupidFloat( cur, 2 );
  if( !temp.valid ) {
    DEBUGLOG( F( "bogus course in rmc" ) );
    goto error;
  }
  newCourse = temp.val;
  cur = temp.comma + 1;
  
  // time is fixed length, so there's no valid field
  newNow = readDateTime( msg + RMC_TIMESTAMP_OFFSET, cur );

  // we don't care about the rest.  if we got here it's valid, so
  // store it
  lat_ = newLat;
  north_ = newNorth;
  lng_ = newLng;
  west_ = newWest;
  speed_ = newSpeed;
  course_ = newCourse;
  now_ = newNow;
  fixType_ = newFixType;
  
#ifdef GPS_DEBUG
  if( debugStream_ ) {
    char timeBuf[ 24 ];
    debugStream_->print( "GPMRC: " );
    debugStream_->print( now_.toString( timeBuf, sizeof( timeBuf ) ) );
    debugStream_->print( ", position: " );
    printDebugLatLong( false );
    debugStream_->print( ", speed (mph): " );
    debugStream_->print( speed_ );
    debugStream_->print( ", course: " );
    debugStream_->print( course_ );
    debugStream_->println();
  }
#endif

  return;

 error:
  bogusNmeaMsgCount_++;
  return;
}

// $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
// 012345678901234567
//           1      LAT     N LONG     W Quality
//                                         numSat
//                                            dilution
//                                                alt M geoidHeight
//                                                          M DGPSInterval

#define GGA_TIMESTAMP_OFFSET 7
#define GGA_LAT_OFFSET 17

void
GpsRx::newGpgga( const char * msg ) {

  DEBUGLOG( msg );

  DegMinSec newLat, newLng;
  bool newNorth, newWest;
  GpsFixType newFixType;
  StupidFloat temp;
  U8 newNumSats;
  deciMeters newAltitude;

  // we just have time in here, not date, so keep date the same
  Gps3Val hms = read3Val( msg + GGA_TIMESTAMP_OFFSET );
  DateTime newNow( now_.year(), now_.month(), now_.day(),
		    hms.val1, hms.val2, hms.val3 );

  char * cur = msg + GGA_LAT_OFFSET;
  newLat = readDegMinSec( &cur );
  if( newLat.present && !newLat.valid ) {
    DEBUGLOG( F( "bogus lat in gga" ) );
    goto error;
  }
  cur++;
  newNorth = ( *cur == 'N' );
  cur += (*cur == ',') ? 1 : 2;
  newLng = readDegMinSec( &cur );
  if( newLng.present && !newLng.valid ) {
    DEBUGLOG( F( "bogus lng in gga" ) );
    goto error;
  }
  cur++;
  newWest = ( *cur == 'W' );
  cur += (*cur == ',') ? 1 : 2;

  newFixType = (*cur) - '0';
  if( newFixType > GpsFixMax ) {
    DEBUGLOG( F( "bogus fix in gga" ) );
    goto error;
  }

  if( (newFixType != GpsFixNormal) && (newFixType != GpsFixDiff) ) {
    // this isn't an error, just a message indicating that we don't
    // have a fix.  Since we don't have that, we don't want to do
    // anything with this data, so return
    return;
  }
  // since fix field was correct, we don't need to check for comma here
  cur += 2;

  temp = readStupidFloat( cur, 0 );
  if( !temp.valid ) {
    DEBUGLOG( F( "bogus num sats in gga" ) );
    goto error;
  }
  newNumSats = temp.val;
  cur = temp.comma + 1;

  // don't care about the dilution, but we need to consume it to find the rest
  temp = readStupidFloat( cur, 0 );
  if( !temp.valid ) {
    DEBUGLOG( F( "bogus dilution in gga" ) );
    goto error;
  }
  cur = temp.comma + 1;

  // this is the altitude, in meters
  temp = readStupidFloat( cur, 1 );
  if( !temp.valid ) {
    DEBUGLOG( F( "bogus altitude in gga" ) );
    goto error;
  }
  newAltitude = temp.val;

  // skip the rest, we don't care.  Since we're done, we can save the
  // data
  now_ = newNow;
  lat_ = newLat;
  north_ = newNorth;
  lng_ = newLng;
  west_ = newWest;
  fixType_ = newFixType;
  numSats_ = newNumSats;
  altitude_ = newAltitude;

#if GPS_DEBUG
  if( debugStream_ ) {
    char timeBuf[ 24 ];
    debugStream_->print( "GPGGA: " );
    debugStream_->print( now_.toString( timeBuf, sizeof( timeBuf ) ) );
    debugStream_->print( ", position: " );
    printDebugLatLong( false );
    debugStream_->print( ", numSats: " );
    debugStream_->print( numSats_ );
    debugStream_->print( ", altitude: " );
    debugStream_->print( altitude_ );
    debugStream_->println();
  }
#endif
  
  return;

 error:
  bogusNmeaMsgCount_++;
  return;
}

void
GpsRx::printDebugLatLong( bool terminate ) {
  char buf[ 60 ];
  debugStream_->print( latString( buf, sizeof( buf ) ) );
  debugStream_->print( " " );
  debugStream_->print( longString( buf, sizeof( buf ) ) );
  if( terminate ) {
    debugStream_->println();
  }
}

char *
GpsRx::latString( char * buf, int bufLen ) {
  return latLongString( buf, bufLen, lat_, north_ ? 'N' : 'S' );
}

char *
GpsRx::longString( char * buf, int bufLen ) {
  return latLongString( buf, bufLen, lng_, west_ ? 'W' : 'E' );
}

char *
GpsRx::latLongString( char * buf, int bufLen, const DegMinSec & ll, char dir ) {

  if( fixType() == GpsFixNone ) {
    return "";
  }

  memset( buf, 0, bufLen );
  // snprintf appears to have bugs...
  snprintf( buf, bufLen, "%3d %2d %2d.", ll.deg, ll.min, (ll.milliSec / 1000) );
  snprintf( buf + 10, bufLen - 10, "%03d", (ll.milliSec % 1000) );
  buf[ 13 ] = dir;
  return buf;
}

char *
GpsRx::fixTypeString() {
  switch( fixType_ ) {
  case GpsFixNone:
    return "None";
  case GpsFixNormal:
    return "Standard GPS";
  case GpsFixDiff:
    return "Diff GPS";
  case GpsFixPps:
    return "PPS";
  case GpsFixRtk:
    return "RTK";
  case GpsFixFloatRtk:
    return "Float RTK";
  case GpsFixEstimated:
    return "Estimated";
  case GpsFixManual:
    return "Manual";
  case GpsFixSim:
    return "Simulated";
  default:
    return "Unknown";
  }
}
