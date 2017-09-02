#ifndef DK_CANRTC_H
#define DK_CANRTC_H

#include <RTClib.h>
#include <RTC_DS3231.h>

#include "display_types.h"

typedef void ( *txCanFn )( canAddr addr, U8 extended, U8 len, void * buf );

class CanRtc {
 public:
  CanRtc();

  void init();
  
  void txAddrIs( canAddr a ) { txAddr_ = a; }
  canAddr txAddr() const { return txAddr_; }

  void txCanFnIs( txCanFn f ) { txCanFn_ = f; }

  void handleCanWrite( void * buf, U8 len );
  void handleCanReq( void * buf, U8 len );

  DateTime now();
  void displayTime( DateTime const & );
  void timeAsText( DateTime const &, char * buf );
  deciDegF temp();

 private:
  canAddr txAddr_;
  txCanFn txCanFn_;
  RTC_DS3231 rtc_;

  DateTime convertCanClock( U8 * buf, U8 len );
  void txCanRsp( DateTime * );
};

#endif // DK_CANRTC_H
