#ifndef DASH_GPS_H
#define DASH_GPS_H

#include "display_types.h"
#include <RTClib.h>

typedef enum {
  GpsFixNone,
  GpsFixNormal,
  GpsFixDiff,
  GpsFixPps,
  GpsFixRtk,
  GpsFixFloatRtk,
  GpsFixEstimated,
  GpsFixManual,
  GpsFixSim,
  GpsFixMax = GpsFixSim,
} GpsFixType;

typedef struct {
  U32 val;
  U8 exp;
  char *comma;
  bool valid;
} StupidFloat;

typedef struct {
  U8 val1;
  U8 val2;
  U8 val3;
} Gps3Val;

class GpsRx {
 public:
  GpsRx();
  ~GpsRx() {};

  GpsFixType fixType() { return fixType_; }

  void newMsg( const char * );
  void debugStreamIs( Stream * s ) { debugStream_ = s; }

  const char * positionString(); // good until next time it's called

  DateTime now() const { return now_; }
  DegMinSec lat() const { return lat_; }
  DegMinSec lng() const { return lng_; }
  bool north() const { return north_; }
  bool west() const { return west_; }
  deciMeters altitude() const { return altitude_; }
  deciMph speed() const { return speed_; }
  centiDegrees course() const { return course_; }
  U8 numSats() const { return numSats_; }

  char * latString( char *, int );
  char * longString( char *, int );
  char * fixTypeString();

  U32 nmeaMsgCount() const { return nmeaMsgCount_; }
  U32 gpmrcCount() const { return gpmrcCount_; }
  U32 gpggaCount() const { return gpggaCount_; }
  U32 otherCount() const { return otherCount_; }
  U32 bogusNmeaMsgCount() { return bogusNmeaMsgCount_; }
    
 private:

  void newGprmc( const char * );
  void newGpgga( const char * );

  GpsFixType fixType_;
  DateTime now_;
  DegMinSec lat_;
  DegMinSec lng_;
  bool north_;
  bool west_;
  deciMeters altitude_;
  deciMph speed_;
  centiDegrees course_;
  U8 numSats_;
  
  U32 nmeaMsgCount_;
  U32 gpmrcCount_;
  U32 gpggaCount_;
  U32 otherCount_;
  U32 bogusNmeaMsgCount_;

  Stream * debugStream_;

  StupidFloat readStupidFloat( const char *, U8 exp );
  Gps3Val read3Val( const char * );
  DateTime readDateTime( const char * hmsPos, const char * dmyPos );
  DegMinSec readDegMinSec( char ** );
  void printDebugLatLong( bool terminate );

  char * latLongString( char * buf, int bufLen, const DegMinSec &, char );

};


#endif // DASH_GPS_H
