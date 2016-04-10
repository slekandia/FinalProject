/*
 * MPU9150.h
 *
 * Created: 12.1.2016 17:59:14
 *  Author: Metin
 */ 

int16_t MX_MAX, MX_MIN, MY_MAX, MY_MIN, MZ_MAX,MZ_MIN;
#include "board.h"
#include "compiler.h"
#include "gpio.h"
#include <math.h>
#include <fastmath.h>
#include "conf_at42qt1060.h"
#include "at42qt1060.h"
#include "cycle_counter.h"
#include "libraries/AP_HAL/AP_HAL.h"
#include "libraries/AP_HAL_Empty/AP_HAL_Empty.h"
#if (UC3A3 || UC3A4 || UC3C || UC3L)
#include "twim.h"
#else
#include "twi.h"
#endif

#define WHO_AM_I_AK8975A 0x00 // should return 0x48
#define INFO             0x01
#define AK8975A_ST1      0x02  // data ready status bit 0
#define AK8975A_ADDRESS  0x0C
#define AK8975A_XOUT_L   0x03  // data
#define AK8975A_XOUT_H   0x04
#define AK8975A_YOUT_L   0x05
#define AK8975A_YOUT_H   0x06
#define AK8975A_ZOUT_L   0x07
#define AK8975A_ZOUT_H   0x08
#define AK8975A_ST2      0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975A_ASTC     0x0C  // Self test control
#define AK8975A_ASAX     0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY     0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ     0x12  // Fuse ROM z-axis sensitivity adjustment value

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope, populate with calibration routine
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19

#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9150 0x75 // Should return 0x68
#define MPU9150_ADDRESS 0x68

#define PI M_PI
static uint32_t cpu_hz;
extern const AP_HAL::HAL& hal;
enum Ascale {
	AFS_2G,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float converted_ax, converted_ay, converted_az, converted_gx, converted_gy, converted_gz, converted_mx, converted_my, converted_mz; // variables to hold latest sensor data values
int16_t tempCount;   //Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float mag_Hard_Bias[3], mag_Scale_Bias[3];
int delt_t = 0; // used to control display output rate
int count1 = 0;  // used to control display output rate
//new variables

int32_t GYRO_XOUT_OFFSET_1000SUM = 0;
int32_t GYRO_YOUT_OFFSET_1000SUM = 0;
int32_t GYRO_ZOUT_OFFSET_1000SUM = 0;

int32_t ACCL_XOUT_OFFSET_1000SUM = 0;
int32_t ACCL_YOUT_OFFSET_1000SUM = 0;
int32_t ACCL_ZOUT_OFFSET_1000SUM = 0;

float GYRO_XOUT_OFFSET = 0;
float GYRO_YOUT_OFFSET = 0;
float GYRO_ZOUT_OFFSET = 0;

float ACCL_XOUT_OFFSET = 0;
float ACCL_YOUT_OFFSET = 0;
float ACCL_ZOUT_OFFSET = 0;
// parameters for 6 DoF sensor fusion calculations

#define GyroMeasError  3.14159265358979323846 * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3

#define GyroMeasDrift  3.14159265358979323846 * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; 
float pitch, yaw, roll;
float deltat = 0.01f;                             // integration interval for both filter schemes
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method
float mgPerDigit = 0.92f;

class MPU9150 {
	public:
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
 {
	 char data_write[2];
	 data_write[0] = subAddress;
	 data_write[1] = data;
	 at42qt1060_write_reg(subAddress, data_write[1]);
	
 }
char readByte(uint8_t address, uint8_t subAddress)
{
	char data; // `data` will store the register data
	data = at42qt1060_read_reg(subAddress);
	return data;
}
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
twi_package_t twi_package;

	twi_package.chip = address;
	twi_package.addr_length = 0;
	twi_package.buffer = &subAddress;
	twi_package.length = 1;
	while(twi_master_write(AT42QT1060_TWI, &twi_package)!=TWI_SUCCESS);
	/* We need a delay here to make this work although this is not
	* specified in the datasheet.
	* Also there seems to be a bug in the TWI module or the driver
	* since some delay here (code or real delay) adds about 500us
	* between the write and the next read cycle.
	*/
	cpu_delay_us(20, cpu_hz);

	twi_package.chip = address;
	twi_package.addr_length = 0;
	twi_package.buffer = &dest;
	twi_package.length = count;
	while(twi_master_read(AT42QT1060_TWI, &twi_package)!=TWI_SUCCESS);

}
void getGres() {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
		gRes = 250.0/32768.0;
		break;
		case GFS_500DPS:
		gRes = 500.0/32768.0;
		break;
		case GFS_1000DPS:
		gRes = 1000.0/32768.0;
		break;
		case GFS_2000DPS:
		gRes = 2000.0/32768.0;
		break;
	}
}


void getAres() {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
		aRes = 2.0/32768.0;
		break;
		case AFS_4G:
		aRes = 4.0/32768.0;
		break;
		case AFS_8G:
		aRes = 8.0/32768.0;
		break;
		case AFS_16G:
		aRes = 16.0/32768.0;
		break;
	}
}


void readAccelData(int16_t * destination)
{
			
	uint8_t axh = at42qt1060_read_reg(0x3B);
	uint8_t axl = at42qt1060_read_reg(0x3C);
	
	uint8_t ayh = at42qt1060_read_reg(0x3D);
	uint8_t ayl = at42qt1060_read_reg(0x3E);
	
	uint8_t azh = at42qt1060_read_reg(0x3F);
	uint8_t azl = at42qt1060_read_reg(0x40);
	destination[0] = (int16_t) (axh <<8) | axl;
	destination[1] = (int16_t) (ayh <<8) | ayl;
	destination[2] = (int16_t) (azh <<8) | azl;

}

void readGyroData(int16_t * destination)
{
	uint8_t gxh = at42qt1060_read_reg(0x43);
	uint8_t gxl = at42qt1060_read_reg(0x44);
	
	uint8_t gyh = at42qt1060_read_reg(0x45);
	uint8_t gyl = at42qt1060_read_reg(0x46);
	
	uint8_t gzh = at42qt1060_read_reg(0x47);
	uint8_t gzl = at42qt1060_read_reg(0x48);
	
	destination[0] = (int16_t) (gxh <<8) | gxl;
	destination[1] = (int16_t) (gyh <<8) | gyl;
	destination[2] = (int16_t) (gzh <<8) | gzl;

}

void readMagData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	mag_write_reg(AK8975A_CNTL, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
	cpu_delay_ms(10,cpu_hz);
	// Only accept a new magnetometer data read if the data ready bit is set and
	// if there are no sensor overflow or data read errors
	uint16_t mx,my,mz;
	uint8_t mxh = mag_read_reg(0x03);
	uint8_t mxl = mag_read_reg(0x04);

	uint8_t myh = mag_read_reg(0x05);
	uint8_t myl = mag_read_reg(0x06);

	uint8_t mzh = mag_read_reg(0x07);
	uint8_t mzl = mag_read_reg(0x08);
	mx = (int16_t) (mxh <<8) | mxl;
	destination[0] = mx;
	my = (int16_t) (myh <<8) | myl;
	destination[1]=  my;
	mz = (int16_t) (mzh <<8) |mzl;
	destination[2]=  mz;

}
void readHMC58Data(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	uint16_t mx,my,mz;
	uint8_t mxh = mag_read_reg(0x03);
	uint8_t mxl = mag_read_reg(0x04);

	uint8_t mzh = mag_read_reg(0x05);
	uint8_t mzl = mag_read_reg(0x06);

	uint8_t myh = mag_read_reg(0x07);
	uint8_t myl = mag_read_reg(0x08);
	mx = (int16_t) (mxh <<8) | mxl;
	destination[0] = mx;
	my = (int16_t) (myh <<8) | myl;
	destination[1]=  my;
	mz = (int16_t) (mzh <<8) |mzl;
	destination[2]=  mz;

}

void update() {
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
	int16_t magData[3];
	
	uint8_t gxh = at42qt1060_read_reg(0x43);
	uint8_t gxl = at42qt1060_read_reg(0x44);
	
	uint8_t gyh = at42qt1060_read_reg(0x45);
	uint8_t gyl = at42qt1060_read_reg(0x46);
	
	uint8_t gzh = at42qt1060_read_reg(0x47);
	uint8_t gzl = at42qt1060_read_reg(0x48);
	
	uint8_t axh = at42qt1060_read_reg(0x3B);
	uint8_t axl = at42qt1060_read_reg(0x3C);
	
	uint8_t ayh = at42qt1060_read_reg(0x3D);
	uint8_t ayl = at42qt1060_read_reg(0x3E);
	
	uint8_t azh = at42qt1060_read_reg(0x3F);
	uint8_t azl = at42qt1060_read_reg(0x40);
	

	//G_Dt = deltat;
	
	gx = (int16_t) (gxh <<8) | gxl;
	converted_gx = (float) ((gx-GYRO_XOUT_OFFSET)*250/32768)+4;
	gy = (int16_t) (gyh <<8) | gyl;
	converted_gy = (float) (gy-GYRO_YOUT_OFFSET)*250/32768+0.3;
	gz = (int16_t) (gzh <<8) | gzl;
	converted_gz = (float) (gz-GYRO_ZOUT_OFFSET)*250/32768+1;

	ax = (int16_t) (axh <<8) | axl;
	converted_ax = (float) (ax-ACCL_XOUT_OFFSET)/16384;
	ay = (int16_t) (ayh <<8) | ayl;
	converted_ay = (float) (ay-ACCL_YOUT_OFFSET)/16384;
	az = (int16_t) (azh <<8) | azl;
	converted_az = (float) (az+1-ACCL_ZOUT_OFFSET)/16384-0.07;
// 	mag_write_reg(AK8975A_CNTL, 0x00); // toggle enable data read from magnetometer, no continuous read mode!
// 	mag_write_reg(AK8975A_CNTL, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
// 	cpu_delay_ms(10,cpu_hz);
// 	uint8_t mxh = mag_read_reg(0x03);
// 	uint8_t mxl = mag_read_reg(0x04);
// 
// 	uint8_t myh = mag_read_reg(0x05);
// 	uint8_t myl = mag_read_reg(0x06);
// 
// 	uint8_t mzh = mag_read_reg(0x07);
// 	uint8_t mzl = mag_read_reg(0x08);
	this->readHMC58Data(magData);
	mx = magData[0];
	my = magData[1];
	mz = magData[2];

// 	mx = (int16_t) (mxh <<8) | mxl;
// 	my = (int16_t) (myh <<8) | myl;
// 	mz = (int16_t) (mzh <<8) |mzl;
	//hal.uartB->println("initialization done");
// 	converted_mx = (float) ((((mx-127.0)*3)*magCalibration[0]))*0.796;
// 	converted_my = (float) ((((my-11263.0)*3)*magCalibration[1]))*1.238;
// 	converted_mz = (float) ((((mz+3584.0)*3)*magCalibration[2]))*1.068;
	
	converted_mx = (float) ((mx+44)*0.92f);
	converted_my = (float) ((my+170)*0.92f);
	converted_mz = (float) ((mz-158)*0.92f);

	MahonyQuaternionUpdate(converted_ax, converted_ay, converted_az, converted_gx*PI/180.0f, converted_gy*PI/180.0f, converted_gz*PI/180.0f,  converted_my,  converted_mx, converted_mz);

	yaw   = (atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
	pitch  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	roll = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	yaw   += 0; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	roll  *= 180.0f / PI;	
}
//Calibration should be called before this method
void initial_RollPitch(float* destination){
	float ax,ay,az,gx,gy,gz;
	float mx,my,mz;

	uint8_t axh = at42qt1060_read_reg(0x3B);
	uint8_t axl = at42qt1060_read_reg(0x3C);
	
	uint8_t ayh = at42qt1060_read_reg(0x3D);
	uint8_t ayl = at42qt1060_read_reg(0x3E);
	
	uint8_t azh = at42qt1060_read_reg(0x3F);
	uint8_t azl = at42qt1060_read_reg(0x40);
	

	//G_Dt = deltat;
	
	ax = (int16_t) (axh <<8) | axl;
	converted_ax = (float) ((int16_t)ax)/16384;
	ay = (int16_t) (ayh <<8) | ayl;
	converted_ay = (float) ((int16_t)ay)/16384;
	az = (int16_t) (azh <<8) | azl;
	converted_az = (float) ((int16_t)az+1)/16384-0.07;
	
// 	mag_write_reg(AK8975A_CNTL, 0x00);
// 	mag_write_reg(AK8975A_CNTL, 0x01);
// 	cpu_delay_ms(10,FOSC0);
	
// 	uint8_t mxh = mag_read_reg(0x03);
// 	uint8_t mxl = mag_read_reg(0x04);
// 
// 	uint8_t mzh = mag_read_reg(0x05);
// 	uint8_t mzl = mag_read_reg(0x06);
// 
// 	uint8_t myh = mag_read_reg(0x07);
// 	uint8_t myl = mag_read_reg(0x08);
// 
// 	//hal.uartB->println("initialization done");
// 	mx = (int16_t) (mxh <<8) | mxl;
// 	my = (int16_t) (myh <<8) | myl;
// 	mz = (int16_t) (mzh <<8) |mzl;
// 	
// 	converted_mx = (float) ((mx+44)*0.92f);
// 	converted_my = (float) ((my+170)*0.92f);
// 	converted_mz = (float) ((mz)*0.92f);
// 	
	float acc[3];
	float gyro[3];
	float mag[3];
	
	acc[0] = converted_ax;
	acc[1] = converted_ay;
	acc[2] = converted_az;
	
	gyro[0] = converted_gx;
	gyro[1] = converted_gy;
	gyro[2] = converted_gz;
	
	destination[0] = atan(acc[1]/(acc[0]*acc[0]+acc[2]*acc[2]))*180.0/M_PI;  //roll
	destination[1] = (atan(acc[0]/(acc[1]*acc[1]+acc[2]*acc[2]))*180.0/M_PI);//pitch
	//destination[2] = atan2(converted_my,converted_mx)*180.0/M_PI;
	
	
}
void update_6050() {
	float ax,ay,az,gx,gy,gz;
	float mx,my,mz;
	//hal.uartB->println("selam");
	uint8_t gxh = at42qt1060_read_reg(0x43);
	uint8_t gxl = at42qt1060_read_reg(0x44);
	
	uint8_t gyh = at42qt1060_read_reg(0x45);
	uint8_t gyl = at42qt1060_read_reg(0x46);
	
	uint8_t gzh = at42qt1060_read_reg(0x47);
	uint8_t gzl = at42qt1060_read_reg(0x48);
	
	uint8_t axh = at42qt1060_read_reg(0x3B);
	uint8_t axl = at42qt1060_read_reg(0x3C);
	
	uint8_t ayh = at42qt1060_read_reg(0x3D);
	uint8_t ayl = at42qt1060_read_reg(0x3E);
	
	uint8_t azh = at42qt1060_read_reg(0x3F);
	uint8_t azl = at42qt1060_read_reg(0x40);
	

	//G_Dt = deltat;
	
	gx = (int16_t) ((int16_t)gxh <<8) | gxl;
	converted_gx = (float) ((gx-GYRO_XOUT_OFFSET)*250/32768)+4;
	gy = (int16_t) ((int16_t)gyh <<8) | gyl;
	converted_gy = (float) (gy-GYRO_YOUT_OFFSET)*250/32768+0.3;
	gz = (int16_t) ((int16_t)gzh <<8) | gzl;
	converted_gz = (float) (gz-GYRO_ZOUT_OFFSET)*250/32768+1;

	ax = (int16_t) (axh <<8) | axl;
	converted_ax = (float) ((int16_t)ax-ACCL_XOUT_OFFSET)/16384;
	ay = (int16_t) (ayh <<8) | ayl;
	converted_ay = (float) ((int16_t)ay-ACCL_YOUT_OFFSET)/16384;
	az = (int16_t) (azh <<8) | azl;
	converted_az = (float) ((int16_t)az+1-ACCL_ZOUT_OFFSET)/16384-0.07;
	float acc[3];
	float gyro[3];

	acc[0] = converted_ax;
	acc[1] = converted_ay;
	acc[2] = converted_az;
	
	gyro[0] = converted_gx;
	gyro[1] = converted_gy;
	gyro[2] = converted_gz;
	float pitch_acc = (atan(acc[0]/(acc[1]*acc[1]+acc[2]*acc[2]))*180.0/M_PI);
	float roll_acc = atan(acc[1]/(acc[0]*acc[0]+acc[2]*acc[2]))*180.0/M_PI;
	

	pitch = pitch + (gyro[1]*deltat);
	pitch = pitch*0.95 + pitch_acc*0.05;
	roll = roll + (gyro[0]*deltat);
	roll = roll*0.95 + roll_acc*0.05;
}


void initAK8975A(float * destination)
{
	mRes = 3;
	uint8_t rawData[3];  // x/y/z gyro register data stored here
	mag_write_reg(AK8975A_CNTL, 0x00); // Power down
	cpu_delay_ms(10,cpu_hz);
 	mag_write_reg(AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
 	cpu_delay_ms(10,cpu_hz);
	rawData[0] = mag_read_reg(0x10);  // Read the x-, y-, and z-axis calibration values
	rawData[1] = mag_read_reg(0x11);
	rawData[2] = mag_read_reg(0x12);
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f; // Return x-axis sensitivity adjustment values
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
}

void initHMC58()
{
	mag_write_reg(0x01,0x20); // SetMagGain
	cpu_delay_ms(10,cpu_hz);
	uint8_t value;
	
	value = mag_read_reg(0x02);
	value &= 0b11111100;
	value |= 0x00;
	mag_write_reg(0x02, value); // Continous Mode
	cpu_delay_ms(10,cpu_hz);
	
	value = mag_read_reg(0x00);
	value &= 0b11100011;
	value |= 0b00010100;
	mag_write_reg(0x00, value); // Data rate is 0b101
	cpu_delay_ms(10,cpu_hz);
	
	value = mag_read_reg(0x00);
	value &= 0b10011111;
	value |= 0b01100000;
	mag_write_reg(0x00, value); // Sample rate is 8
	cpu_delay_ms(10,cpu_hz);
}

void selfTestAK8975A(float * destination)
{
	float calibratedValues[3];
	mag_write_reg(AK8975A_CNTL, 0x00);
	cpu_delay_ms(10,cpu_hz);
	mag_write_reg(AK8975A_ASTC, 0x40);
	cpu_delay_ms(10,cpu_hz);
	mag_write_reg(AK8975A_CNTL, 0x08);
	cpu_delay_ms(10,cpu_hz);
	
	uint8_t tmxh = mag_read_reg(0x03);
	uint8_t tmxl = mag_read_reg(0x04);

	uint8_t tmyh = mag_read_reg(0x05);
	uint8_t tmyl = mag_read_reg(0x06);

	uint8_t tmzh = mag_read_reg(0x07);
	uint8_t tmzl = mag_read_reg(0x08);
	
	uint8_t tmx = (int16_t) (tmxh <<8) | tmxl;
	destination[0] = (float) (tmx)*magCalibration[0];
	uint8_t tmy = (int16_t) (tmyh <<8) | tmyl;
	destination[1] = (float) (tmy)*magCalibration[1];
	uint8_t tmz = (int16_t) (tmzh <<8) |tmzl;
	destination[2] = (float) (tmz)*magCalibration[2];
	
	mag_write_reg(AK8975A_ASTC, 0x00);
}
void calibrateQuickAK8975A(float* destination1, float* destination2){
	uint8_t result_t[4];
	dataflash_multiple_read(0x00,4,result_t);
	destination1[0] = cast_8_32_array(result_t);
	dataflash_multiple_read(0x04,4,result_t);
	destination1[1] = cast_8_32_array(result_t);
	dataflash_multiple_read(0x08,4,result_t);
	destination1[2]= cast_8_32_array(result_t);
	dataflash_multiple_read(0x12,4,result_t);
	destination2[0] = (float) cast_8_32_array(result_t)*0.01;
	dataflash_multiple_read(0x16,4,result_t);
	destination2[1] = (float) cast_8_32_array(result_t)*0.01;
	dataflash_multiple_read(0x20,4,result_t);
	destination2[2]= (float) cast_8_32_array(result_t)*0.01;
}
void calibrateAK8975A(float* destination1, float* destination2)
{
	hal.uartB->println("Start calibration of the xy axis");
	hal.scheduler->delay(4000);
	int16_t ii = 0, sample_count = 0, x_max = INT16_MIN, y_max = INT16_MIN, z_max = INT16_MIN, x_min = INT16_MAX, y_min = INT16_MAX, z_min =INT16_MAX;
	int16_t mag_Datas[3];
	int32_t mag_Scale[3];
	sample_count = 1000;
	 for(ii = 0; ii < sample_count; ii++) {
		
		readMagData(mag_Datas);
		
		hal.uartB->print("Magnetometer XYZ: ");
		hal.uartB->print(mag_Datas[0]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[1]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[2]);
		hal.uartB->println(" ");
		hal.uartB->println(" ");
		
		if(mag_Datas[0] > x_max)
			x_max = mag_Datas[0];
		if(mag_Datas[0]< x_min)
			x_min = mag_Datas[0];
		if(mag_Datas[1] > y_max)
			y_max = mag_Datas[1];
		if(mag_Datas[1]< y_min)
			y_min = mag_Datas[1];
		
	 }
	 hal.uartB->println("Start calibration of the z axis");
	 hal.scheduler->delay(4000);
	  for(ii = 0; ii < sample_count; ii++) {
		
		readMagData(mag_Datas);
		
		hal.uartB->print("Magnetometer XYZ: ");
		hal.uartB->print(mag_Datas[0]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[1]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[2]);
		hal.uartB->println(" ");
		hal.uartB->println(" ");
		
		if(mag_Datas[2] > z_max)
			z_max = mag_Datas[2];
		if(mag_Datas[2]< z_min)
			z_min = mag_Datas[2];
		
	 }
	destination1[0]= (float) ((x_max+x_min)/2);
	destination1[1]= (float) ((y_max+y_min)/2);
	destination1[2]= (float) ((z_max+z_min)/2);
	
	mag_Scale[0]  = (x_max - x_min)/2;  // get average x axis max chord length in counts
	mag_Scale[1]  = (y_max - y_min)/2;  // get average y axis max chord length in counts
	mag_Scale[2]  = (z_max - z_min)/2;  // get average z axis max chord length in counts

	float avg_rad = mag_Scale[0] + mag_Scale[1] + mag_Scale[2];
	avg_rad /= 3.0;

	destination2[0] = avg_rad/((float)mag_Scale[0]);
	destination2[1] = avg_rad/((float)mag_Scale[1]);
	destination2[2] = avg_rad/((float)mag_Scale[2]);
	
	MX_MAX = x_max;
	MX_MIN = x_min;
	
	MY_MAX = y_max;
	MY_MIN = y_min;
	
	MZ_MAX = z_max;
	MZ_MIN = z_min;
	
	//Multiply with 100 for the numbers that can be less than 1
	uint8_t result[4];
// 	cast_32_8_array(destination1[0],result);
// 	dataflash_multiple_write(0x00,result,4);
// 	cast_32_8_array(destination1[1],result);
// 	dataflash_multiple_write(0x04,result,4);
// 	cast_32_8_array(destination1[2],result);
// 	dataflash_multiple_write(0x08,result,4);
// 	cast_32_8_array(destination2[0]*100,result);
// 	dataflash_multiple_write(0x12,result,4);
// 	cast_32_8_array(destination2[1]*100,result);
// 	dataflash_multiple_write(0x16,result,4);
// 	cast_32_8_array(destination2[2]*100,result);
// 	dataflash_multiple_write(0x20,result,4);
	
}

void calibrateHMC58(float* destination1, float* destination2)
{
	hal.uartB->println("Start calibration of the xy axis");
	hal.scheduler->delay(4000);
	int16_t ii = 0, sample_count = 0, x_max = INT16_MIN, y_max = INT16_MIN, z_max = INT16_MIN, x_min = INT16_MAX, y_min = INT16_MAX, z_min =INT16_MAX;
	int16_t mag_Datas[3];
	int32_t mag_Scale[3];
	sample_count = 1000;
	for(ii = 0; ii < sample_count; ii++) {
		
		readMagData(mag_Datas);
		
		hal.uartB->print("Magnetometer XYZ: ");
		hal.uartB->print(mag_Datas[0]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[1]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[2]);
		hal.uartB->println(" ");
		hal.uartB->println(" ");
		
		if(mag_Datas[0] > x_max)
		x_max = mag_Datas[0];
		if(mag_Datas[0]< x_min)
		x_min = mag_Datas[0];
		if(mag_Datas[1] > y_max)
		y_max = mag_Datas[1];
		if(mag_Datas[1]< y_min)
		y_min = mag_Datas[1];
		
	}
	hal.uartB->println("Start calibration of the z axis");
	hal.scheduler->delay(4000);
	for(ii = 0; ii < sample_count; ii++) {
		
		readMagData(mag_Datas);
		
		hal.uartB->print("Magnetometer XYZ: ");
		hal.uartB->print(mag_Datas[0]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[1]);
		hal.uartB->print("\t");
		hal.uartB->print(mag_Datas[2]);
		hal.uartB->println(" ");
		hal.uartB->println(" ");
		
		if(mag_Datas[2] > z_max)
		z_max = mag_Datas[2];
		if(mag_Datas[2]< z_min)
		z_min = mag_Datas[2];
		
	}
	destination1[0]= (float) ((x_max+x_min)/2);
	destination1[1]= (float) ((y_max+y_min)/2);
	destination1[2]= (float) ((z_max+z_min)/2);
	
	mag_Scale[0]  = (x_max - x_min)/2;  // get average x axis max chord length in counts
	mag_Scale[1]  = (y_max - y_min)/2;  // get average y axis max chord length in counts
	mag_Scale[2]  = (z_max - z_min)/2;  // get average z axis max chord length in counts

	float avg_rad = mag_Scale[0] + mag_Scale[1] + mag_Scale[2];
	avg_rad /= 3.0;

	destination2[0] = avg_rad/((float)mag_Scale[0]);
	destination2[1] = avg_rad/((float)mag_Scale[1]);
	destination2[2] = avg_rad/((float)mag_Scale[2]);
	
	MX_MAX = x_max;
	MX_MIN = x_min;
	
	MY_MAX = y_max;
	MY_MIN = y_min;
	
	MZ_MAX = z_max;
	MZ_MIN = z_min;
	
	//Multiply with 100 for the numbers that can be less than 1
	uint8_t result[4];
	// 	cast_32_8_array(destination1[0],result);
	// 	dataflash_multiple_write(0x00,result,4);
	// 	cast_32_8_array(destination1[1],result);
	// 	dataflash_multiple_write(0x04,result,4);
	// 	cast_32_8_array(destination1[2],result);
	// 	dataflash_multiple_write(0x08,result,4);
	// 	cast_32_8_array(destination2[0]*100,result);
	// 	dataflash_multiple_write(0x12,result,4);
	// 	cast_32_8_array(destination2[1]*100,result);
	// 	dataflash_multiple_write(0x16,result,4);
	// 	cast_32_8_array(destination2[2]*100,result);
	// 	dataflash_multiple_write(0x20,result,4);
	
}

int16_t readTempData()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(0x68, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void resetMPU9150() {
	// reset device
	writeByte(0x68, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	cpu_delay_ms(10,cpu_hz);
}

void initMPU9150(uint32_t fcpu)
{
	cpu_hz = fcpu;
	
	irq_initialize_vectors();
	cpu_irq_enable();
		const gpio_map_t AT42QT1060_TWI_GPIO_MAP =
	{
		{AT42QT1060_TWI_SCL_PIN, AT42QT1060_TWI_SCL_FUNCTION},
		{AT42QT1060_TWI_SDA_PIN, AT42QT1060_TWI_SDA_FUNCTION}
	};

	twi_options_t AT42QT1060_TWI_OPTIONS;
	
	AT42QT1060_TWI_OPTIONS.pba_hz = 16000000;
	AT42QT1060_TWI_OPTIONS.speed = AT42QT1060_TWI_MASTER_SPEED;
	AT42QT1060_TWI_OPTIONS.chip = AT42QT1060_TWI_ADDRESS;
	

	// Assign I/Os to SPI.
	gpio_enable_module(AT42QT1060_TWI_GPIO_MAP,
	sizeof(AT42QT1060_TWI_GPIO_MAP) / sizeof(AT42QT1060_TWI_GPIO_MAP[0]));
	sysclk_enable_peripheral_clock(AT42QT1060_TWI);
	// Initialize as master.
	twi_master_init(AT42QT1060_TWI, &AT42QT1060_TWI_OPTIONS);
	cpu_delay_ms(230, cpu_hz);
	// Initialize MPU9150 device
	// wake up device

	writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	cpu_delay_ms(100,cpu_hz); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	writeByte(MPU9150_ADDRESS, 0x1A, 0x03);
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c =  readByte(MPU9150_ADDRESS, GYRO_CONFIG);
	writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9150_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	// Set accelerometer configuration
	c =  readByte(MPU9150_ADDRESS, ACCEL_CONFIG);
	writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrate_Gyros()
{
	for (int i = 0; i < 50; i++)
	{
		uint8_t GYRO_X_H = readByte(MPU9150_ADDRESS, 0x43);
		uint8_t GYRO_X_L = readByte(MPU9150_ADDRESS, 0x44);
		uint8_t GYRO_Y_H = readByte(MPU9150_ADDRESS, 0x45);
		uint8_t GYRO_Y_L = readByte(MPU9150_ADDRESS, 0x46);
		uint8_t GYRO_Z_H = readByte(MPU9150_ADDRESS, 0x47);
		uint8_t GYRO_Z_L = readByte(MPU9150_ADDRESS, 0x48);

		GYRO_XOUT_OFFSET_1000SUM += (int32_t)(((int16_t)GYRO_X_H << 8) | GYRO_X_L);
		GYRO_YOUT_OFFSET_1000SUM += (int32_t)(((int16_t)GYRO_Y_H << 8) | GYRO_Y_L);
		GYRO_ZOUT_OFFSET_1000SUM += (int32_t)(((int16_t)GYRO_Z_H << 8) | GYRO_Z_L);

		//System.Threading.Thread.Sleep(1);
	}

	GYRO_XOUT_OFFSET = (float)(GYRO_XOUT_OFFSET_1000SUM * 0.02);
	GYRO_YOUT_OFFSET = (float)(GYRO_YOUT_OFFSET_1000SUM * 0.02);
	GYRO_ZOUT_OFFSET = (float)(GYRO_ZOUT_OFFSET_1000SUM * 0.02);
}

void calibrate_Accel()
{
	for (int i = 0; i < 50; i++)
	{
		uint8_t ACCL_X_H = readByte(MPU9150_ADDRESS, 0x3B);
		uint8_t ACCL_X_L = readByte(MPU9150_ADDRESS, 0x3C);
		uint8_t ACCL_Y_H = readByte(MPU9150_ADDRESS, 0x3D);
		uint8_t ACCL_Y_L = readByte(MPU9150_ADDRESS, 0x3E);
		uint8_t ACCL_Z_H = readByte(MPU9150_ADDRESS, 0x3F);
		uint8_t ACCL_Z_L = readByte(MPU9150_ADDRESS, 0x40);

		ACCL_XOUT_OFFSET_1000SUM += (int16_t)(ACCL_X_H << 8) | ACCL_X_L;
		ACCL_YOUT_OFFSET_1000SUM += (int16_t)(ACCL_Y_H << 8) | ACCL_Y_L;
		ACCL_ZOUT_OFFSET_1000SUM += (int16_t)(ACCL_Z_H << 8) | ACCL_Z_L;

		//System.Threading.Thread.Sleep(1);
	}

	ACCL_XOUT_OFFSET = (float)(ACCL_XOUT_OFFSET_1000SUM * 0.02);
	ACCL_YOUT_OFFSET = (float)(ACCL_YOUT_OFFSET_1000SUM * 0.02);
	ACCL_ZOUT_OFFSET = (float)(ACCL_ZOUT_OFFSET_1000SUM * 0.02);
}
void calibrate_mpu9150 (float* destination){
	float count = 0;
	while(count < 1000){
		update();
		count = count + 1;
	}
	destination[0] = roll;
	destination[1] = pitch;
	destination[2] = yaw;
	
}
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9150SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4] = {0, 0, 0, 0};
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU9150_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU9150_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   cpu_delay_ms(250,cpu_hz);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU9150_ADDRESS, SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU9150_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU9150_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU9150_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation
   
 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
}
float calculateHeading(float roll, float pitch, float mag_x, float mag_y, float mag_z)
{
	float heading;
    float headX;
    float headY;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
	
    cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
    sin_roll = 1  - (cos_roll * cos_roll);
    cos_pitch = cos(pitch);
    sin_pitch = 1  - (cos_pitch * cos_pitch);

    // Tilt compensated magnetic field X component:
    headX = mag_x*cos_pitch+mag_y*sin_roll*sin_pitch+mag_z*cos_roll*sin_pitch;
    // Tilt compensated magnetic field Y component:
    headY = mag_y*cos_roll-mag_z*sin_roll;
    // magnetic heading
    heading = atan2(-headY,headX);

//     // Declination correction (if supplied)
//     if( fabs(_declination) > 0.0 )
//     {
//         heading = heading + _declination;
//         if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
//             heading -= (2.0 * M_PI);
//         else if (heading < -M_PI)
//             heading += (2.0 * M_PI);
//     }
	heading = heading + 5.29f;
	if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
        heading -= (2.0 * M_PI);
    else if (heading < -M_PI)
        heading += (2.0 * M_PI);
	
	return heading*180.0/M_PI;
}


	// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
	// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
	// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
	// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
	// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
	// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f/norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * deltat;
		q2 += qDot2 * deltat;
		q3 += qDot3 * deltat;
		q4 += qDot4 * deltat;
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f/norm;
		q[0] = q1 * norm;
		q[1] = q2 * norm;
		q[2] = q3 * norm;
		q[3] = q4 * norm;

	}



	// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
	// measured ones.
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
		float norm;
		float hx, hy, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float pa, pb, pc;

		// Auxiliary variables to avoid repeated arithmetic
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
		hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2.0f * (q2q4 - q1q3);
		vy = 2.0f * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
		wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
		wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		if (Ki > 0.0f)
		{
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
		}
		else
		{
			eInt[0] = 0.0f;     // prevent integral wind up
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
		}

		// Apply feedback terms
		gx = gx + Kp * ex + Ki * eInt[0];
		gy = gy + Kp * ey + Ki * eInt[1];
		gz = gz + Kp * ez + Ki * eInt[2];

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
		q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
		q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
		q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

		// Normalise quaternion
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		q[0] = q1 * norm;
		q[1] = q2 * norm;
		q[2] = q3 * norm;
		q[3] = q4 * norm;
		
	}

};