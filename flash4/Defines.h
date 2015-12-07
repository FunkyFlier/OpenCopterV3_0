enum LOG_STATES{
  CHECK_4K,
  ERASE,
  WRITE_READY,
  COMPLETE_PAGE,
  START_NEW_LOG,
  END_CURRENT_LOG,
  COMPLETE_LAST_PAGE,
  UPDATE_FIRST_PAGE,
  BOUND_CHECK
};


#define GyroSSOutput() DDRL |= 1<<0 
#define GyroSSHigh() PORTL |= 1<<0 
#define GyroSSLow() PORTL &= ~(1<<0)

#define AccSSOutput() DDRL |= 1<<1 
#define AccSSHigh() PORTL |= 1<<1 
#define AccSSLow() PORTL &= ~(1<<1)

#define BaroSSOutput() DDRL |= 1<<2 
#define BaroSSHigh() PORTL |= 1<<2 
#define BaroSSLow() PORTL &= ~(1<<2)

#define MagSSOutput() DDRL |= 1<<3 
#define MagSSHigh() PORTL |= 1<<3
#define MagSSLow() PORTL &= ~(1<<3)

#define FlashSSOutput() DDRL |= 1<<4
#define FlashSSHigh() PORTL |= 1<<4
#define FlashSSLow() PORTL &= ~(1<<4)


#define READ_ARRAY 0x03
#define ERASE_4K 0x20
#define ERASE_32K 0x52
#define ERASE_64K 0xD8
//#define ERASE_CHIP 0xC7
#define ERASE_CHIP 0x60
#define PROGRAM_PAGE 0x02

#define WRITE_ENABLE 0x06

#define READ_STATUS_REG 0x05

#define STATUS_WRITE 0x01

#define TOP_ADDRESS 0x3FF000

#define WRITE_ERROR_MASK 0x20

//first byte flags
#define ERASED 0XFF
#define WRITE_STARTED 0x7F
#define WRITE_STARTED_REC_START 0x5F
#define WRITE_STARTED_REC_END 0x6F
#define WRITE_COMPLETE 0x3F
#define WRITE_COMPLETE_REC_START 0x1F
#define WRITE_COMPLETE_REC_END 0x2F
#define WRITE_COMPLETE_REC_START_END 0x0F
#define TO_ERASE 0x00
#define START_OF_REC_LEN 5

typedef union{
  uint16_t val;
  uint8_t buffer[2];
}
uint16_u;

typedef union{
  uint32_t val;
  uint8_t buffer[4];
}
uint32_u;

typedef union {
  float val;
  uint8_t buffer[4];
}
float_u;
