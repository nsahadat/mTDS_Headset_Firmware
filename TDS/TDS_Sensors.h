/**************************************************************************//**
    @file       magneticsensor.h

    @brief      Header file for LSM303D. @Note This header file
                does not include all register addresses for the LSM303D.

******************************************************************************/
#ifndef TDS_Sensors_H
#define TDS_Sensors_H


/******************************************************************************
 * INCLUDES
 */


/******************************************************************************
 * DEFINES
 */

// BMA250 addressing space
#define MGT_CHIPID                  0x00    // Always 0x03
#define MGT_X_LSB                   0x08    // ACC_X_LSB[7:6] = 2 LSb of X acceleration data sahadat
#define MGT_X_MSB                   0x09    // ACC_X_MSB[7:0] = 8 MSb of X data sahadat
#define MGT_Y_LSB                   0x0A        //sahadat
#define MGT_Y_MSB                   0x0B        // sahadat
#define MGT_Z_LSB                   0x0C        // sahadat
#define MGT_Z_MSB                   0x0D        // sahadat

#define ACC_X_LSB                   0x28    // LSM303D accelerometer data registers
#define ACC_X_MSB                   0x29    
#define ACC_Y_LSB                   0x2A        
#define ACC_Y_MSB                   0x2B        
#define ACC_Z_LSB                   0x2C        
#define ACC_Z_MSB                   0x2D       
   
   
   
#define MGT_CTRL0                   0x1F
#define MGT_CTRL1                   0x20
#define MGT_CTRL2                   0x21
#define MGT_CTRL3                   0x22
#define MGT_CTRL4                   0x23
#define MGT_CTRL5                   0x24
#define MGT_CTRL6                   0x25
#define MGT_CTRL7                   0x26
#define MGT_INT_CTRL_M              0x12

#define MGT_CTRL0_VAL               0x00        // CHECK THE VALUES IN THE DATASHEET OF LSM303D TO CHANGE THE CONFIGURATION
#define MGT_CTRL1_VAL               0x6F        //0x5F        // not updated untill the value is read--- 50Hz power mode 0110(100Hz)1(contineous updat)1(AZEN)1(AYEN)1(AZEN)
#define MGT_CTRL2_VAL               0x40        //0xC0        // SPI 4 wire interface 50Hz antialias filter for accelerometer, +-2g // 01(194Hz anti alias)000(+-2G)0(default)0(selftest disable)0(SPI 4 wire)
#define MGT_CTRL3_VAL               0x00        // nothing no interrupt
#define MGT_CTRL4_VAL               0x00        // no interrupt
#define MGT_CTRL5_VAL               0x74            //0x70        // 50Hz magnetic resolution ; high resoulution // 0 (temp dissable)11(high resolution magnetic data)101(100Hz magnetic data)0(LIR2)0(LIR1)
#define MGT_CTRL6_VAL               0x40        // +- 8gauss for the magnetic resolution //0(default)10(+-8Gauss)00000(default)
#define MGT_CTRL7_VAL               0x00        // contineous magnetic sensor mode
#define MGT_INT_CTRL_M_VAL          0x00           //0xE0

#define MGT_INT_ENABLE0             0x16  // change it if you want to use it as interrupt basis
#define MGT_INT_ENABLE1             0x17  // change it if you want to use it as interrupt basis

// Range selection definitions
#define ACC_RANGE_2G                0x18    //  3.91 mg/LSB sahadat
#define ACC_RANGE_4G                0x05    //  7.81 mg/LSB
#define ACC_RANGE_8G                0x08    // 15.62 mg/LSB
#define ACC_RANGE_16G               0x0C    // 31.25 mg/LSB

// Filtered bandwidth selection (delta_t = time between successive acc samples)
#define ACC_BW_7_81HZ               0x08    // 7.81Hz bandwidth (delta_t = 64 ms)
#define ACC_BW_15_63HZ              0x09    // delta_t = 32   ms
#define ACC_BW_31_25HZ              0x0A    // delta_t = 16   ms
#define ACC_BW_62_5HZ               0x0B    // delta_t =  8   ms
#define ACC_BW_125HZ                0x0C    // delta_t =  4   ms
#define ACC_BW_250HZ                0x00    // delta_t =  2   ms sahadat
#define ACC_BW_500HZ                0x0E    // delta_t =  1   ms
#define ACC_BW_1000HZ               0x0F    // delta_t =  0.5 ms

#define ACC_PM_SUSP                 0x80    // Power mode register (0x11), bit 7
#define ACC_PM_LP                   0x40    // Low power mode
#define ACC_PM_SLEEP_10MS           0x14
#define ACC_PM_SLEEP_25MS           0x16
#define ACC_PM_SLEEP_50MS           0x18

// Interrupt enable bitmasks (for use with registers ACC_INT_ENABLEx [x=0,1] )
#define ACC_INT_FLAT_EN             0x80    // Bit in register 0x16
#define ACC_INT_ORIENT_EN           0x40    //          "
#define ACC_INT_S_TAP_EN            0x20    //          "
#define ACC_INT_D_TAP_EN            0x10    //          "
#define ACC_INT_SLOPE_Z_EN          0x04    //          "
#define ACC_INT_SLOPE_Y_EN          0x02    //          "
#define ACC_INT_SLOPE_X_EN          0x01    //          "
#define ACC_INT_DATA_EN             0x10    // Bit in register 0x17
#define ACC_INT_LOW_EN              0x08    //          "
#define ACC_INT_HIGH_Z_EN           0x04    //          "
#define ACC_INT_HIGH_Y_EN           0x02    //          "
#define ACC_INT_HIGH_X_EN           0x01    //          "

// Interrupt mapping bitmasks (for use with registers ACC_INT_MAPPINGx [x=0,1,2] )
#define ACC_INT_MAP_FLAT            0x80    // For pin INT1 (INT2), bit in register 0x19 (0x1B)
#define ACC_INT_MAP_ORIENT          0x40    //                   "
#define ACC_INT_MAP_S_TAP           0x20    //                   "
#define ACC_INT_MAP_D_TAP           0x10    //                   "
#define ACC_INT_MAP_SLOPE           0x04    //                   "
#define ACC_INT_MAP_HIGH            0x02    //                   "
#define ACC_INT_MAP_LOW             0x01    //                   "
#define ACC_INT1_MAP_DATA           0x01    // New data IRQ to pin INT1, bit in register 0x1A
#define ACC_INT2_MAP_DATA           0x80    // New data IRQ to pin INT2, bit in register 0x1A

// Interrupt source bitmasks (for use with register ACC_INT_SOURCE)
#define ACC_INT_SRC_DATA_FILT       0x20
#define ACC_INT_SRC_TAP_FILT        0x01
#define ACC_INT_SRC_SLOPE_FILT      0x04
#define ACC_INT_SRC_HIGH_FILT       0x02
#define ACC_INT_SRC_LOW_FILT        0x01

// Interrupt pin behavior bitmasks (for use with register (Open drive/push-pull and active level 0/1)
#define ACC_INT2_OD                 0x08
#define ACC_INT2_LVL                0x04
#define ACC_INT1_OD                 0x02
#define ACC_INT1_LVL                0x01

// Perform soft reset
#define ACC_SOFTRESET_EN            0xB6    // Soft reset by writing 0xB6 to softreset register


//-------LSM9DS1 registors and values------------//
/////////////////////////////////////////
// LSM9DS1_ Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define LSM9DS1_ACT_THS				0x04
#define LSM9DS1_ACT_DUR				0x05
#define LSM9DS1_INT_GEN_CFG_XL		0x06
#define LSM9DS1_INT_GEN_THS_X_XL	0x07
#define LSM9DS1_INT_GEN_THS_Y_XL	0x08
#define LSM9DS1_INT_GEN_THS_Z_XL	0x09
#define LSM9DS1_INT_GEN_DUR_XL		0x0A
#define LSM9DS1_REFERENCE_G			0x0B
#define LSM9DS1_INT1_CTRL			0x0C
#define LSM9DS1_INT2_CTRL			0x0D
#define LSM9DS1_WHO_AM_I_XG			0x0F
#define LSM9DS1_CTRL_REG1_G			0x10
#define LSM9DS1_CTRL_REG2_G			0x11
#define LSM9DS1_CTRL_REG3_G			0x12
#define LSM9DS1_ORIENT_CFG_G		0x13

/****************gyroscoper configuration values************/
#define LSM9DS1_CTRL_REG1_G_VAL		0XA3 //0XA0    	//0b10100000 //0b10100011 <--filter @100Hz

/* ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | 0 | BW_G1 | BW_G0
101(476Hz)00(245 dps)0(default)11() change to 00 for more noise*/

#define LSM9DS1_CTRL_REG2_G_VAL         0X00     //0b00000000

/* 0(1) | 0(1) | 0(1) | 0(1) | INT_SEL1 | INT_SEL0 | OUT_SEL1 | OUT_SEL0 

*/
#define LSM9DS1_CTRL_REG3_G_VAL	        0X49 //0X00		//0b00000000 //0b01001001<-highpass filter with 0.05 cut off

/*
LP_mode | HP_EN | 0(1) | 0(1) | HPCF3_G | HPCF2_G | HPCF1_G |HPCF0_G

*/
#define LSM9DS1_ORIENT_CFG_G_VAL        0X00		//0b00000000

/*

0(1)| 0(1) | SignX_G | SignY_G | SignZ_G | Orient_2 | Orient_1 | Orient_0
*/


#define LSM9DS1_CTRL_REG4_VAL   0X38		//0b00111000

/*
0(1) |0(1)|Zen_G | Yen_G | Xen_G | 0(1) | LIR_XL1 | 4D_XL1

*/

/*******************gyroscope configuration values ends******/



#define LSM9DS1_INT_GEN_SRC_G		0x14
#define LSM9DS1_OUT_TEMP_L			0x15
#define LSM9DS1_OUT_TEMP_H			0x16
#define LSM9DS1_STATUS_REG_0		0x17
#define LSM9DS1_OUT_X_L_G			0x18
#define LSM9DS1_OUT_X_H_G			0x19
#define LSM9DS1_OUT_Y_L_G			0x1A
#define LSM9DS1_OUT_Y_H_G			0x1B
#define LSM9DS1_OUT_Z_L_G			0x1C
#define LSM9DS1_OUT_Z_H_G			0x1D
#define LSM9DS1_CTRL_REG4			0x1E
#define LSM9DS1_CTRL_REG5_XL		0x1F
#define LSM9DS1_CTRL_REG6_XL		0x20
#define LSM9DS1_CTRL_REG7_XL		0x21

/******************** values for accel configuration*************************/
#define LSM9DS1_CTRL_REG5_XL_VAL        0X38		//0b00111000

/*
DEC_1 | DEC_0 | Zen_XL | Yen_XL | Xen_XL | 0(1) | 0(1) | 0(1)

*/
#define LSM9DS1_CTRL_REG6_XL_VAL	0X86//0XA2 //0X62    	//0b01100010 //0b10100010 // 0b100(odr 238)00(+-2g)1(BW_scale)10

/*
ODR_XL2 | ODR_XL1 | ODR_XL0 | FS1_XL | FS0_XL | BW_SCAL_ODR | BW_XL1 | BW_XL0
101(476Hz)00(+-2G) 0(default) 10(105Hz)
011(119Hz)00(+-2G) 0(default) 10(105Hz)
*/

#define LSM9DS1_CTRL_REG7_XL_VAL	0X20 //0XA0	//0b10100000	// 0b00100000

/*
HR | DCF1 | DCF0 | 0(1) | 0(1) | FDS | 0(1) | HPIS1

1 | 01(100Hz)|00(default)|0|0
*/

/************end of accel configuration********************************/

#define LSM9DS1_CTRL_REG8			0x22
#define LSM9DS1_CTRL_REG9			0x23
#define LSM9DS1_CTRL_REG10			0x24
#define LSM9DS1_INT_GEN_SRC_XL		        0x26
#define LSM9DS1_STATUS_REG_1		        0x27
#define LSM9DS1_OUT_X_L_XL			0x28
#define LSM9DS1_OUT_X_H_XL			0x29
#define LSM9DS1_OUT_Y_L_XL			0x2A
#define LSM9DS1_OUT_Y_H_XL			0x2B
#define LSM9DS1_OUT_Z_L_XL			0x2C
#define LSM9DS1_OUT_Z_H_XL			0x2D
#define LSM9DS1_FIFO_CTRL			0x2E
#define LSM9DS1_FIFO_SRC			0x2F
#define LSM9DS1_INT_GEN_CFG_G		0x30
#define LSM9DS1_INT_GEN_THS_XH_G	0x31
#define LSM9DS1_INT_GEN_THS_XL_G	0x32
#define LSM9DS1_INT_GEN_THS_YH_G	0x33
#define LSM9DS1_INT_GEN_THS_YL_G	0x34
#define LSM9DS1_INT_GEN_THS_ZH_G	0x35
#define LSM9DS1_INT_GEN_THS_ZL_G	0x36
#define LSM9DS1_INT_GEN_DUR_G		0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define LSM9DS1_OFFSET_X_REG_L_M	0x05
#define LSM9DS1_OFFSET_X_REG_H_M	0x06
#define LSM9DS1_OFFSET_Y_REG_L_M	0x07
#define LSM9DS1_OFFSET_Y_REG_H_M	0x08
#define LSM9DS1_OFFSET_Z_REG_L_M	0x09
#define LSM9DS1_OFFSET_Z_REG_H_M	0x0A
#define LSM9DS1_WHO_AM_I_M			0x0F
#define LSM9DS1_CTRL_REG1_M			0x20
#define LSM9DS1_CTRL_REG2_M			0x21
#define LSM9DS1_CTRL_REG3_M			0x22
#define LSM9DS1_CTRL_REG4_M			0x23
#define LSM9DS1_CTRL_REG5_M			0x24

/*****************configuration values for magnetometer*************/
#define LSM9DS1_CTRL_REG1_M_VAL		0X7E //0X7C	//0b01111110
/*
TEMP_COMP | OM1 | OM0 | DO2 | DO1 | DO0 | 0(1) | ST
*/
#define LSM9DS1_CTRL_REG2_M_VAL		0X20	//0b00100000

/*
0(1) | FS1 | FS0 | 0(1) | REBOOT | SOFT_RST | 0(1) | 0(1)
0 | 01 (8 Gauss) | 0 (normal mode)| 0 (soft reset) | 0 | 0
*/

#define LSM9DS1_CTRL_REG3_M_VAL		0x00//0X84	//0b10000100

/*
I2C_DISABLE | 0(1)|LP | 0(1) | 0(1) | SIM | MD1 | MD0

*/

#define LSM9DS1_CTRL_REG4_M_VAL		0X0C	//0b00001100

/*
0(1)|0(1)|0(1) |0(1) |OMZ1 |OMZ0 |BLE |0

*/

#define LSM9DS1_CTRL_REG5_M_VAL		0x00//0X40	//0b00000000

/*
0(1)| BDU | 0(1) | 0(1) | 0(1) | 0(1) | 0(1) | 0(1)

*/

/**********end of magnetometer configuration**************/



#define LSM9DS1_STATUS_REG_M		        0x27
#define LSM9DS1_OUT_X_L_M			0x28
#define LSM9DS1_OUT_X_H_M			0x29
#define LSM9DS1_OUT_Y_L_M			0x2A
#define LSM9DS1_OUT_Y_H_M			0x2B
#define LSM9DS1_OUT_Z_L_M			0x2C
#define LSM9DS1_OUT_Z_H_M			0x2D
#define LSM9DS1_INT_CFG_M			0x30
#define LSM9DS1_INT_SRC_M			0x30
#define LSM9DS1_INT_THS_L_M			0x32
#define LSM9DS1_INT_THS_H_M			0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define LSM9DS1_WHO_AM_I_AG_RSP		0x68
#define LSM9DS1_WHO_AM_I_M_RSP		0x3D


//-------LSM9DS1-----register and values ends here-------------//

/*sahadat 
*/




#define CS_DISABLED(pin)	digitalWrite(pin, HIGH)
#define CS_ENABLED(pin)     digitalWrite(pin, LOW)

#define DISABLED(pin)	digitalWrite(pin, LOW)
#define ENABLED(pin)    digitalWrite(pin, HIGH)

#define SPI_BAUD_M  34
#define SPI_BAUD_E  13                  //230400 baud rate. change it to 12 if you want 115200 baud rate



/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void setupPins(void);
void Init_Mag_Sensors(void);
void InitLSM9DS1(void);
//void GetSensorOutput(uint16_t *SEN_OUT);
void GetSensorOutput(void);
void spiWriteByte(uint8_t write);
void spiReadByte(uint8_t *read, uint8_t write);
void mgtWriteReg(uint8_t reg, uint8_t val, uint8_t sennum);
void LSM9DWriteReg(uint8_t reg, uint8_t val, uint8_t sennum);
int16_t * mgtReadAcc( uint8_t sennum);
int16_t * LSM3DAcc( uint8_t sennum);
int16_t * ReadLSM9DS1(uint8_t sennum, uint8_t reg_add); 


/******************************************************************************

written by Md Nazmus Sahadat
GTBionics Laboratory
******************************************************************************/


#endif
