#ifndef _MAVLINK_H_
#define _MAVLINK_H_

#include "system.h"
extern char battery[40];
extern char gps[37];
extern char imu[37];
extern const uint8_t mavlink_message_crcs[];

 void crc_accumulate(uint8_t data, uint16_t *crcAccum);


  void crc_init(uint16_t* crcAccum);
 
 
  uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
 
 
  void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);
 

#endif
