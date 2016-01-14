#ifndef Definitions_h
#define Definitions_h

#define HIGH_RATE_INTERVAL 13
#define MED_RATE_INTERVAL 50
#define LOW_RATE_INTERVAL 200

#define Z_RATE_LIMIT 45.0

#define SPEED_LIMIT

#define RAW_PRES_FILT

#define BARO_ERR_LIMIT 0.5
#define BARO_ERR_COUNTS_OFF 25
#define BARO_ERR_COUNTS_ON 35
#define LAND_DET_LIM -0.75

#define BARO_GLITCH_TIME 5000


#define NEW_BARO_FEEDBACK
//------------------------

//#define AUX3_WP_DEBUG

//#define AUX3_RTB
#define AUX3_YAW_SP
//#define AUX3_FS_TESTS
//#define AUX3_RATEX
//#define AUX3_ROLL
//#define AUX3_VEL
//#define AUX3_POS
//#define AUX3_FORCE_ATT_RESET
//#define AUX3_RESET_BIAS 
//------------------------
#ifdef AUX3_RATEX
#define STEP_RATE 50.0
#define STEP_DURATION 500
#endif
//------------------------
#ifdef AUX3_ROLL
#define STEP_ATT 10.0
#endif
//------------------------
#ifdef AUX3_VEL
#define STEP_VEL 1.0
#endif
//------------------------
#ifdef AUX3_POS
#define STEP_DIST 1.0
#endif
//------------------------

#define LOIT_VEL_MAX 4.0
#define LOIT_VEL_MIN -4.0
#define RAMP_DOWN_ALPHA 0.95
#define RAMP_DOWN_VEL_RTB 1.0
#define LOIT_RAMP_MIN 0.5

#define LOW_SPEED_RADIUS 15
#define MIN_RTB_DIST 1.5

#define LOW_RATE_DIVIDER 3

#define LOIT_TILT_MAX 30
#define LOIT_TILT_MIN -30
#define LOIT_YAW_MAX 75
#define LOIT_YAW_MIN -75
#define ATT_ERR_MAX 500
#define BATT_FS_PERCENT 30

#define PATTERN_TIME 200000//in mS
#define PATTERN_TIME_FAST 40000

#define BIAS_MAX 0.3
#define BIAS_MIN -0.3

//#define EXT_MAG

//#define V1
#define V2

#ifdef V2
#ifdef V1
#undef V1
#endif
#endif

#define COUNTS_TO_AMPS 0.23339658
#define _3_V_PER_CELL 750
#define LOW_VOLTAGE_COUNT 750
#define HIGH_VOLTAGE_COUNT 1024
#define VOLT_COUNTS_TO_CELL_VOLTAGE 0.0041015625 //   4.2/1024

#define CEILING 6.0
#define FLOOR 1.0
#define TAKE_OFF_ALT 1.5



#define LAND_VEL -0.5
#define RTB_VEL 1

#define HH_ON 0
#define HH_OFF 1



#define MAX_Z_RATE 3.0f
#define MIN_Z_RATE -3.0f
//common defines


//LED defines GREEN, YELLOW, BLUE, RED
#define GREEN 42
#define YELLOW 40
#define BLUE 13
#define RED 38

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines - ST L3G2
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_X_L 0x28
#define L3G_WHO_AM_I 0x0F

//mag defines ST HMC5983DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define HMC5983_CRA_REG (uint8_t)0x00
#define HMC5983_CRB_REG 0x01
#define HMC5983_MR_REG 0x02
#define HMC5983_OUT_X_H 0x03

#define HMC5983_ID_A 0x0A
#define HMC5983_ID_B 0x0B
#define HMC5983_ID_C 0x0C

#define D22Output() DDRA |= 1<<0
#define D22High() PORTA |= 1<<0
#define D22Low() PORTA &= ~(1<<0)
#define D22Toggle() PORTA ^= (1<<0)

#define D23Output() DDRA |= 1<<1
#define D23High() PORTA |= 1<<1
#define D23Low() PORTA &= ~(1<<1)
#define D23Toggle() PORTA ^= (1<<1)

#define D24Output() DDRA |= 1<<2
#define D24High() PORTA |= 1<<2
#define D24Low() PORTA &= ~(1<<2)
#define D24Toggle() PORTA ^= (1<<2)

#define D25Output() DDRA |= 1<<3
#define D25High() PORTA |= 1<<3
#define D25Low() PORTA &= ~(1<<3)
#define D25Toggle() PORTA ^= (1<<3)

#define D26Output() DDRA |= 1<<4
#define D26High() PORTA |= 1<<4
#define D26Low() PORTA &= ~(1<<4)
#define D26Toggle() PORTA ^= (1<<4)

#define D27Output() DDRA |= 1<<5
#define D27High() PORTA |= 1<<5
#define D27Low() PORTA &= ~(1<<5)
#define D27Toggle() PORTA ^= (1<<5)

#define D28Output() DDRA |= 1<<6
#define D28High() PORTA |= 1<<6
#define D28Low() PORTA &= ~(1<<6)
#define D28Toggle() PORTA ^= (1<<6)

#define D29Output() DDRA |= 1<<7
#define D29High() PORTA |= 1<<7
#define D29Low() PORTA &= ~(1<<7)
#define D29Toggle() PORTA ^= (1<<7)

//however digitalWrite will work when using SPI
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



#define Port0 Serial
#define RC_SERIAL_PORT Serial1
#define Port2 Serial2
#define gpsPort Serial3

#define RADIO_BUF_SIZE 256
#define NUM_WAY_POINTS 0x14
//end common defines





#define BLOCK_MASK_4K 0x000F
#define BLOCK_MASK_32K 0x007F
#define BLOCK_MASK_64K 0x00FF

#define READ_ARRAY 0x03
#define ERASE_4K 0x20
#define ERASE_32K 0x52
#define ERASE_64K 0xD8
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


//V1 defines
#ifdef V1
//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

//barometer defines
#define BMP085_ADDRESS 0x77
#define POLL_RATE 20
/*#define OSS 0x00
 #define CONV_TIME 5*/
/*#define OSS 0x01
 #define CONV_TIME 8*/
/*#define OSS 0x02
 #define CONV_TIME 14*/
#define OSS 0x03
#define CONV_TIME 27

#define Motor1WriteMicros(x) OCR3B = x * 2
#define Motor2WriteMicros(x) OCR3C = x * 2
#define Motor3WriteMicros(x) OCR3A = x * 2
#define Motor4WriteMicros(x) OCR4A = x * 2
#define Motor5WriteMicros(x) OCR4B = x * 2
#define Motor6WriteMicros(x) OCR4C = x * 2
#define Motor7WriteMicros(x) OCR1A = x * 2
#define Motor8WriteMicros(x) OCR1B = x * 2
#endif//#ifdef V1
//end V1 defines

//V2 defines
#ifdef V2
//acc defines
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define OUT_X_L_A 0x28

//baro defines
#define MS5611_RESET 0x1E
#define MS5611_PROM_Setup 0xA0
#define MS5611_PROM_C1 0xA2
#define MS5611_PROM_C2 0xA4
#define MS5611_PROM_C3 0xA6
#define MS5611_PROM_C4 0xA8
#define MS5611_PROM_C5 0xAA
#define MS5611_PROM_C6 0xAC
#define MS5611_PROM_CRC 0xAE
#define CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CONVERT_D2_OSR4096 0x58   // Maximun resolution

#define ADC_READ 0x00

#define BARO_CONV_TIME 50


#define Motor1WriteMicros(x) OCR3A = x * 2
#define Motor2WriteMicros(x) OCR3B = x * 2
#define Motor3WriteMicros(x) OCR3C = x * 2
#define Motor4WriteMicros(x) OCR4A = x * 2
#define Motor5WriteMicros(x) OCR4B = x * 2
#define Motor6WriteMicros(x) OCR4C = x * 2
#define Motor7WriteMicros(x) OCR1A = x * 2
#define Motor8WriteMicros(x) OCR1B = x * 2
#endif//#ifdef V2
//end V2 defines


#define FC_ADC 1
#define RC_CONST_ADC 1/(2.0 * 3.14 * FC_ADC)

#define FC_ACC 5.0
#define RC_CONST_ACC 1.0/(2.0 * 3.14 * FC_ACC)

#define FC_BARO 3.75
//#define FC_BARO 2.0
#define RC_CONST_BARO 1.0/(2.0 * 3.14 * FC_BARO)

#define FC_PRESS 1.0
#define RC_CONST_PRESS 1.0/(2.0 * 3.14 * FC_PRESS)


#define RC_SS 44

#define RC_SS_Output() DDRH |= 1<<7 
#define RC_SSHigh() PORTH |= 1<<7 
#define RC_SSLow() PORTH &= ~(1<<7)

#define CAL_FLAGS 0
#define HS_FLAG 1

#define PKT_LOCAL_ORD_L 2
#define PKT_LOCAL_ORD_M 3

#define PKT_LOCAL_UN_L 4
#define PKT_LOCAL_UN_M 5
#define PR_FLAG 6

#define ESC_CAL_FLAG 7

#define ACC_CALIB_START 8
#define ACC_S_X_INDEX 11
#define ACC_S_Y_INDEX 15
#define ACC_S_Z_INDEX 19
#define ACC_O_X_INDEX 23
#define ACC_O_Y_INDEX 27
#define ACC_O_Z_INDEX 31
#define ACC_CALIB_END 31

#define MAG_CALIB_START 32
#define MAG_OFF_X_INDEX 35
#define MAG_OFF_Y_INDEX 39
#define MAG_OFF_Z_INDEX 43
#define W_00_INDEX 47
#define W_01_INDEX 51
#define W_02_INDEX 55
#define W_10_INDEX 59
#define W_11_INDEX 63
#define W_12_INDEX 67
#define W_20_INDEX 71
#define W_21_INDEX 75
#define W_22_INDEX 79
#define MAG_CALIB_END 79

#define PITCH_OFFSET_START 80
#define PITCH_OFFSET_END 83

#define ROLL_OFFSET_START 84
#define ROLL_OFFSET_END 87

#define GAINS_START 88
#define GAINS_END 327

#define DEC_START 328
#define DEC_END 331

#define RC_DATA_START 332
#define MAX_INDEX 333
#define MIN_INDEX 335
#define MID_INDEX 337
#define CHAN_INDEX 338
#define SCALE_INDEX 342
#define REV_INDEX 343
#define RC_DATA_END 427

#define VER_FLAG_1 428
#define VER_FLAG_2 429

#define PWM_LIM_HIGH_START 430
#define PWM_LIM_HIGH_END 431

#define PWM_LIM_LOW_START 432
#define PWM_LIM_LOW_END 433

#define PWM_FLAG 434
#define PROP_IDLE 435
#define PROP_IDLE_FLAG 436

#define HOVER_THRO 437
#define HOVER_THRO_FLAG 438

#define TX_FS_FLAG 439
#define TX_FS 440

#define MODE_FLAG 441
#define MODE_START 442
#define MODE_END 450

#define GS_ID_INDEX 451

#define CEILING_FLOOR_FLAG 452
#define CEILING_START 453
#define CEILING_END 454
#define FLOOR_START 455
#define FLOOR_END 456

#define MIX_FLAG 457
#define MIX_START 458
#define MIX_END 553

#define EST_FLAG 554
#define EST_GAIN_START 555
#define EST_GAIN_END 606

#define SWIFT_X_FLAG 607
#define ROT_45 608

#define PWM_HIGH_MAX 2000
#define PWM_LOW_MIN 1000

#define PWM_HIGH_MIN 1900
#define PWM_LOW_MAX 1200

#define ESC_CALIBRATION_DELAY 4000

#define PROTOCOL_VER_NUM 1
#define PROTOCOL_VER_SUB_NUM 1

#define VER_NUM_1 0x01
#define VER_NUM_2 0x03

#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)



















#endif//#ifndef Definitions.h

