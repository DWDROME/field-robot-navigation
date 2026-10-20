#include "mavlink.h"

char battery[40]   = {0xFE,0x1F,0x61,0x01,0x01,0x01,0x2F,0x00,0x06,0x00,0x0F,0x00,0x21,0x00,0x2F,0x00,0x06,0x00,0xF3,0x01,0xF4,0x2D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0x92,0xE7};
char gps[37] =       {0xFE,0x1C,0x01,0x01,0x01,0x21,0xE8,0x03,0x00,0x00,0x80,0xBB,0xE5,0x42,0xA0,0x40,0x66,0x10,0xA0,0x86,0x01,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0,0x00,0x00,0xEB,0x3E};
char imu[37] =		 {0xFE,0x1C,0x01,0x01,0x01,0x1E,0xE8,0x03,0x00,0x00,0xD8,0x0F,0xC9,0x3F,0xD8,0x0F,0xC9,0x3F,0x00,0x00,0x00,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x56,0x6E};

#define DL_MAVLINK_MESSAGE_CRCS {50, 124, 137, 0, 237, 217, 104, 119, 117, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 137, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 132, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 14, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 208, 0, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}
const uint8_t mavlink_message_crcs[] = DL_MAVLINK_MESSAGE_CRCS;


 void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
   uint8_t tmp;
   tmp = data ^ (uint8_t)(*crcAccum & 0xff);
 tmp ^= (tmp << 4);
 *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
 }

 inline void crc_init(uint16_t* crcAccum)
 {
	 *crcAccum = 0xffff;
 }
 
  uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
 {
	 uint16_t crcTmp;
	 crc_init(&crcTmp);
	 while (length--) {
		 crc_accumulate(*pBuffer++, &crcTmp);
	 }
	 return crcTmp;
 }
 
  void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
 {
	 const uint8_t *p = (const uint8_t *)pBuffer;
	 while (length--) {
		 crc_accumulate(*p++, crcAccum);
	 }
 }

