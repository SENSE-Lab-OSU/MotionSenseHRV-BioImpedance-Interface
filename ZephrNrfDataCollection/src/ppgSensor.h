#ifndef PPGSENSOR_H_
#define PPGSENSOR_H_
#include "DSP/Include/arm_const_structs.h"
#include "DSP/Include/arm_math.h"
#include <zephyr.h>
#include <drivers/spi.h>
#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>

#define READMASTER 0x80
#define WRITEMASTER 0x00
// Register Addresses
#define PPG_INT_STAT_1 0x00
#define PPG_INT_STAT_2 0x01
#define PPG_INT_EN_1 0x02
#define PPG_FIFO_DATA_COUNTER 0x07
#define PPG_FIFO_DATA 0x08
#define PPG_FIFO_CONFIG_1 0x09
#define PPG_FIFO_CONFIG_2 0x0A
#define PPG_SYS_CTRL 0x0D
#define PPG_CONFIG_1 0x11
#define PPG_CONFIG_2 0x12
#define PPG_CONFIG_3 0x13
#define PPG_PHOTODIODE_BIAS 0x15
#define PPG_LED_SEQ_1 0x20
#define PPG_LED1_PA 0x23
#define PPG_LED2_PA 0x24
#define PPG_LED3_PA 0x25
#define PPG_LED_RANGE_1 0x2A
#define PPG_CHIP_ID_1 0xFF
// Settings
#define PPG_RESET 0x01
#define PPG_SHUTDOWN 0x02
#define PPG_LP_MODE 0x04
#define PPG_SINGLE 0x08
#define SPI_FILL 0xFF
#define PPG2_ADC_RGE_4096nA 0x00 
#define PPG2_ADC_RGE_8192nA 0x10 
#define PPG2_ADC_RGE_16384nA 0x20 
#define PPG2_ADC_RGE_32768nA 0x30 
#define PPG1_ADC_RGE_4096nA 0x00 
#define PPG1_ADC_RGE_8192nA 0x04 
#define PPG1_ADC_RGE_16384nA 0x08 
#define PPG1_ADC_RGE_32768nA 0x0C 
#define PPG_TINT_14_8us 0x00
#define PPG_TINT_29_4us 0x01
#define PPG_TINT_58_7us 0x02
#define PPG_TINT_117_3us 0x03
#define PPG_SR_25_1 0x00 <<3
#define PPG_SR_50_1 0x01 <<3
#define PPG_SR_84_1 0x02 <<3
#define PPG_SR_100_1 0x03 <<3
#define PPG_SR_200_1 0x04 <<3
#define PPG_SR_400_1 0x05 <<3
#define PPG_SR_25_2 0x06 <<3
#define PPG_SR_50_2 0x07 <<3
#define PPG_SR_84_2 0x08 <<3
#define PPG_SR_100_2 0x09 <<3
#define PPG_SR_8_1 0x0A <<3
#define PPG_SR_16_1 0x0B <<3
#define PPG_SR_32_1 0x0C <<3
#define PPG_SR_64_1 0x0D <<3
#define PPG_SR_128_1 0x0E <<3
#define PPG_SR_256_1 0x0F <<3
#define PPG_SR_512_1 0x10 <<3
#define PPG_SR_1024_1 0x11 <<3
#define PPG_SR_2048_1 0x12 <<3
#define PPG_SR_4096_1 0x13 <<3
#define PPG_SMP_AVE_1 0x00
#define PPG_SMP_AVE_2 0x01
#define PPG_SMP_AVE_4 0x02
#define PPG_SMP_AVE_8 0x03
#define PPG_SMP_AVE_16 0x04
#define PPG_SMP_AVE_32 0x05
#define PPG_SMP_AVE_64 0x06
#define PPG_SMP_AVE_128 0x07
#define PPG_LED_SETLNG_4us 0x00
#define PPG_LED_SETLNG_6us 0x40
#define PPG_LED_SETLNG_8us 0x80
#define PPG_LED_SETLNG_12us 0xC0
#define PPG_PDBIAS_65pF 0x01
#define PPG_PDBIAS_130pF 0x05
#define PPG_PDBIAS_260pF 0x06
#define PPG_PDBIAS_520pF 0x07
#define PPG_LED_CURRENT_31mA 0x00
#define PPG_LED_CURRENT_62mA 0x01
#define PPG_LED_CURRENT_93mA 0x02
#define PPG_LED_CURRENT_124mA 0x03
#define PPG_FIFO_PUSH_ENABLE 0x02
#define PPG_INT_A_FULL_EN 0x80
#define PPG_LEDC2_LED2_LED3_SIMULT 0x60
#define PPG_LEDC1_LED1 0x01

// New PPG tag types
#define PPG1_LEDC1_DATA    0x01
#define PPG1_LEDC2_DATA    0x02
#define PPG2_LEDC1_DATA    0x07
#define PPG2_LEDC2_DATA    0x08

// FIR Filter Globabl variables
#define NUM_TAPS  222
#define BLOCK_SIZE  1 
#define NUM_TAPS_MA_FIL  2000
#define FLOAT_CON_16UA   (16000.0/524287.0)


// these structs are responsible for sending
struct ppgInfo {
  struct k_work work;
  bool movingFlag;
  uint16_t pktCounter;
  bool ppgTFPass;
}; 
struct ppgDataInfo {
  struct k_work work;
  //this is the pointer to the actual packets being sent
  uint8_t* dataPacket;
  uint8_t packetLength;
}; 
extern struct ppgInfo my_ppgSensor;
extern struct ppgDataInfo my_ppgDataSensor;

extern const struct device *spi_dev_ppg;
extern struct spi_config spi_cfg_ppg;
/*and also here */
struct ppgData {
float green_ch1;
float green_ch2;
float infraRed_ch1;
float infraRed_ch2;
float meanChanIR_1;
float meanChanIR_2;
float meanChanGreen_1;
float meanChanGreen_2;
float stdChanIR_1;
float stdChanIR_2;
float stdChanGreen_1;
float stdChanGreen_2;
float green_ch1_buffer[500];
float green_ch2_buffer[500];
float infraRed_ch1_buffer[500];
float infraRed_ch2_buffer[500];
uint32_t bufferIndex;
bool dataReadyTF;
};

struct ppg_configData {
  bool isEnabled;
  bool txPacketEnable;
  uint8_t sample_avg;
  uint8_t sampling_time;
  uint8_t green_intensity;
  uint8_t infraRed_intensity;
  uint8_t numCounts;
};
struct tfMicro_configData {
  bool isEnabled;
  bool txPacketEnable;
};


extern struct ppg_configData ppgConfig;
extern struct ppgData ppgData1;
extern struct tfMicro_configData tfMicroCoonfig;
void spiRead_registerPPG(uint8_t * tx_buffer, uint8_t txLen, uint8_t * rx_buffer, uint8_t rxLen);
void spiWrite_registerPPG(uint8_t * tx_buffer, uint8_t txLen);
void ppgsensor_config(void);
void ppgsensor_sleep(void);
void high_pass_filter_init_25(void);        
void high_pass_filter_init_50(void); 
void high_pass_filter_init_100(void); 
void high_pass_filter_init_200(void);                    
void ppg_config(void);
void ppg_changeIntensity(void);
void ppg_changeSamplingRate(void);
void read_ppg_fifo_buffer(struct k_work *item);
void ppg_sleep(void);
void fs_umountFilesys(void);
void getFileSysSize(void);
void fs_mount_init(void);
void fileOpenRead(void);
void fileClose(void);
void fileOpenAppend(void);
void fileOpen(void);
void fileWrite(uint32_t dataFlash);
uint32_t fileRead(void);
#endif