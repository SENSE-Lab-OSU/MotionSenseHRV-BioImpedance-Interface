#ifndef IMUSENSOR_H_
#define IMUSENSOR_H_
#include <zephyr.h>
#include <drivers/spi.h>
#include "arm_const_structs.h"
#include "arm_math.h"
#include <drivers/spi.h>

#define READMASTER 0x80
#define WRITEMASTER 0x00
#define MAGNETOADDRESS 0x0C
#define READBIT_MAGNET 0x01
/// IMU registers
#define WHO_AM_I 0x00
#define IMU_GYRO_XOUT_H 0x33
#define I2C_SLV0_DO_IMU 0x06      // UB3
#define INT_PIN_CFG_IMU 0x0F      // UB0
#define I2C_MST_CTRL_IMU 0x01     // UB3
#define I2C_SLV0_ADDR_IMU 0x03    // UB3
#define I2C_SLV0_REG_IMU 0x04     // UB3
#define I2C_SLV0_CTRL_IMU 0x05    // UB3
#define EXT_SENSE_DATA_0_IMU 0x3B // UB0
#define REG_BANK_SEL 0x7F
#define MAGNETOMETER_CNTL2 0x31
#define MAGNETOMETER_HXL 0x11
#define MAGNETOMETER_ST1 0x10
#define ACCEL_XOUT_H 0x2D
#define INT_STAATUS_1 0x1A
#define PWR_MGMT_1 0x06
#define USER_CTRL 0x03
#define PWR_MGMT_2 0x07
#define LP_CONFIG 0x05
#define INT_PIN_CFG 0x0F
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_DELAY_CTRL 0x02
// Settings
#define REG_BANK_0 0x00
#define REG_BANK_1 0x10
#define REG_BANK_2 0x20
#define REG_BANK_3 0x30
#define I2C_SLV_EN 0x80
#define SPI_FILL 0xFF
#define MAGNETO_SINGLE_MEASUREMENT 0x01
#define IMU_SLEEP 0x40
#define IMU_CLK_SEL_BEST_SEL 0x01
#define MAGNETOMETER_POWERDOWN 0x00
#define IMU_TEMP_DISABLE 0x08
#define I2C_MST_EN 0x20
#define I2C_IF_DIS 0x10
#define DISABLE_GYRO 0x07
#define ENABLE_GYRO 0x00
#define DISABLE_ACC 0x07<<3
#define ENABLE_ACC 0x00<<3
#define I2C_MST_CYCLE 0x40
#define GYRO_FCHOICE_DLPF_EN 0x01
#define GYRO_FS_SEL_250 0x00
#define GYRO_FS_SEL_500 0x02
#define GYRO_FS_SEL_1000 0x04
#define GYRO_FS_SEL_2000 0x06
#define GYRO_DLPFCFG_196HZ  0x00<<3
#define GYRO_DLPFCFG_151HZ  0x01<<3
#define GYRO_DLPFCFG_120HZ  0x02<<3
#define GYRO_DLPFCFG_51HZ  0x03<<3
#define GYRO_DLPFCFG_24HZ  0x04<<3
#define GYRO_DLPFCFG_11HZ  0x05<<3
#define GYRO_DLPFCFG_6HZ  0x06<<3
#define GYRO_DLPFCFG_361HZ  0x07<<3
#define ACCEL_FCHOICE_DLPF_ENABLE 0x01
#define ACCEL_FS_SEL_2g 0x00 //+- 2g fullscale
#define ACCEL_FS_SEL_4g 0x02 //+- 4g fullscale
#define ACCEL_FS_SEL_8g 0x04 //+- 8g fullscale
#define ACCEL_FS_SEL_16g 0x06 //+- 16g fullscale
#define ACCEL_DLPFCFG_246HZ 0x00 <<3
#define ACCEL_DLPFCFG_111HZ 0x02 <<3
#define ACCEL_DLPFCFG_50HZ 0x03 <<3
#define ACCEL_DLPFCFG_24HZ 0x04 <<3
#define ACCEL_DLPFCFG_12HZ 0x05 <<3
#define ACCEL_DLPFCFG_6HZ 0x06 <<3
#define ACCEL_DLPFCFG_473HZ 0x07 <<3
#define DELAY_ES_SHADOW 0x80
#define I2C_MST_P_NSR_STOP_READS 0x10
#define I2C_MST_CLK_370KHZ_50DUTY 0x00
#define I2C_MST_CLK_432KHZ_50DUTY 0x03
#define I2C_MST_CLK_370KHZ_42DUTY 0x04
#define I2C_MST_CLK_345KHZ_40DUTY 0x07
#define I2C_MST_CLK_304KHZ_47DUTY 0x08
#define I2C_MST_CLK_432KHZ_41DUTY 0x0A
#define I2C_MST_CLK_471KHZ_45DUTY 0x0C
#define I2C_MST_CLK_345KHZ_46DUTY 0x0E
#define MAGNETOMETER_ID 0x00
#define GYRO_SAMPLING_RATE 200
#define MAGNETO_SAMPLING_RATE 25
 
extern const struct device *spi_dev_imu;
extern struct spi_config spi_cfg_imu;
struct motionInfo {
  struct k_work work;
  uint16_t pktCounter;
  uint8_t magneto_first_read;
  uint8_t gyro_first_read;
}; 
struct magnetoInfo {
  struct k_work work;
  uint8_t *dataPacket;
  uint8_t packetLength;
}; 
struct orientationInfo {
  struct k_work work;
  uint8_t *dataPacket;
  uint8_t packetLength;
}; 
extern struct motionInfo my_motionSensor; 
extern struct magnetoInfo my_magnetoSensor; 
extern struct orientationInfo my_orientaionSensor;
struct accData {
  int16_t accx;
  int16_t accy;
  int16_t accz;
  float accx_val;
  float accy_val;
  float accz_val;
};
struct gyroData {
  uint16_t quaternion_1;
  uint16_t quaternion_2;
  uint16_t quaternion_3;
  float quaternion_1_val;
  float quaternion_2_val;
  float quaternion_3_val;
  float quaternion_4_val;
  float gyrox_val;
  float gyroy_val;
  float gyroz_val;
  bool movingFlag;
};
struct magnetoData {
  int16_t Hx;
  int16_t Hy;
  int16_t Hz;
  float Hx_val;
  float Hy_val;
  float Hz_val;
};
struct orientationData {
  float q0;
  float q1;
  float q2;
  float q3;
};
typedef enum{
  MAGNETOMETER_SINGLE,
  MAGNETOMETER_SET_EXT_SENSOR,
  MAGNETOMETER_SET_EXT_TOREAD,
} magneto_sample_config_t;

struct accel_config{
  bool isEnabled;
  bool txPacketEnable;
  uint8_t sensitivity;
  uint8_t sample_bw;
};


struct gyro_config{
  bool isEnabled;
  bool txPacketEnable;
  uint8_t sensitivity;
  uint8_t tot_samples;
  uint8_t sampling_time;
};
struct orientation_config{
  bool isEnabled;
  bool txPacketEnable;
};

struct magneto_config{
  bool isEnabled;
  bool txPacketEnable;
};

extern struct accData accData1;
extern struct gyroData gyroData1;
extern struct magnetoData magnetoData1;  
extern struct orientationData orientationData1; 
                    
extern struct accel_config accelConfig;
extern struct gyro_config gyroConfig;
extern struct magneto_config magnetoConfig;
extern struct orientation_config orientationConfig;

void motion_config(void);
void spiRead_registerIMU(uint8_t * tx_buffer, uint8_t txLen, uint8_t * rx_buffer, uint8_t rxLen);
void motion_data_timeout_handler(struct k_work *);
void motionSensitivitySampling_config(void);
void motion_data_orientation_timeout_handler(uint16_t );
void motion_sleep(void);
static void magnetometer_data_read_send(bool validMeasurement , uint16_t pktCounter);
#endif




