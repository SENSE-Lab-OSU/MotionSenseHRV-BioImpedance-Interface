/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <nrfx_timer.h>
#include <nrfx.h>
#include <nrfx_uarte.h>

#include "batteryMonitor.h"
#include "ppgSensor.h"
#include "imuSensor.h"
#include "common.h"
#include "BLEService.h"
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <settings/settings.h>


//dfu configuration
#define INCLUDE_DFU

#ifdef INCLUDE_DFU
#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include "os_mgmt/os_mgmt.h"
#include "img_mgmt/img_mgmt.h"
#include "stats/stats.h"
#include "stat_mgmt/stat_mgmt.h"

#include <mgmt/mcumgr/smp_bt.h>
#endif
#endif




/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
uint8_t gyro_first_read = 0;
uint8_t magneto_first_read = 0;  
uint8_t ppgRead = 0;
bool ppgTFPass = false;
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define READMASTER 0x80
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif


#define WORKQUEUE_STACK_SIZE 1024

const struct device *gpioHandle_CS_IMU;
const struct device *gpioHandle_CS_ppg;

struct spi_cs_control imu_cs = {
.delay = 0, 
.gpio_pin = 23 , 
.gpio_dt_flags=GPIO_ACTIVE_LOW,
};
struct spi_cs_control ppg_cs = {
.delay = 0, 
.gpio_pin = 15 , 
.gpio_dt_flags=GPIO_ACTIVE_LOW,
};
/*
------------------------------------------------------------------------------------
SPI Mode    CPOL 	CPHA 	Clock Polarity  Clock Phase Used to 
                                in Idle State 	Sample and/or Shift the Data
------------------------------------------------------------------------------------
0               0         0 	Logic low 	Data sampled on rising edge and
                                                shifted out on the falling edge
1               0         1 	Logic low 	Data sampled on the falling edge and
                                                shifted out on the rising edge
2               1         1 	Logic high 	Data sampled on the falling edge and
                                                shifted out on the rising edge
3               1         0 	Logic high 	Data sampled on the rising edge and
                                                shifted out on the falling edge
-------------------------------------------------------------------------------------
*/
// SPI Mode-3 IMU
struct spi_config spi_cfg_imu = {
        .frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
		     SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
	.cs=&imu_cs,
};
// SPI Mode-3 PPG
struct spi_config spi_cfg_ppg = {
        .frequency = 4000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
		     SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
	.cs=&ppg_cs,
};

struct ppg_configData ppgConfig;
struct ppgData ppgData1;
static const nrfx_timer_t timer_global = NRFX_TIMER_INSTANCE(1); // Using TIMER1 as TIMER 0 is used by RTOS for blestruct device *spi_dev_imu;
const struct device *spi_dev_ppg,*spi_dev_imu;
const struct device *i2c_dev;
struct bq274xx_data batteryMonitor;
struct bq274xx_config batteryMonitorConfig;

struct ppgInfo my_ppgSensor;
uint8_t blePktTFMicro[ble_tfMicroPktLength];
struct batteryInfo my_battery ;  // work-queue instance for batter level

struct TfMicroInfo my_HeartRateEncoder;  // work-queue instance for tflite notifications
struct motionInfo my_motionSensor; // work-queue instance for motion sensor
struct magnetoInfo my_magnetoSensor; // work-queue instance for magnetometer
struct orientationInfo my_orientaionSensor; // work-queue instance for orientation
struct ppgDataInfo my_ppgDataSensor;

struct accData accData1;
struct gyroData gyroData1;
struct magnetoData magnetoData1;  
                   
struct accel_config accelConfig;
struct gyro_config gyroConfig;
struct magneto_config magnetoConfig;
struct orientation_config orientationConfig;

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define DIS_FW_REV_STR		 CONFIG_BT_DIS_FW_REV_STR
#define DIS_FW_REV_STR_LEN 	 (sizeof(DIS_FW_REV_STR))

#define DIS_HW_REV_STR 	 CONFIG_BT_DIS_HW_REV_STR
#define DIS_HW_REV_STR_LEN 	 (sizeof(DIS_HW_REV_STR))

#define DIS_MANUF		 CONFIG_BT_DIS_MANUF
#define DIS_MANUF_LEN 	 	 (sizeof(DIS_MANUF))

#define DIS_MODEL		 CONFIG_BT_DIS_MODEL
#define DIS_MODEL_LEN		 (sizeof(DIS_MODEL))

#define TIMER_MS 5
#define TIMER_PRIORITY 1
#define WORKQUEUE_PRIORITY 1

#define RUN_STATUS_LED          DK_LED1

K_THREAD_STACK_DEFINE(my_stack_area, WORKQUEUE_STACK_SIZE);


struct k_work_q my_work_q;

void timer_handler(nrf_timer_event_t, void*);

static K_SEM_DEFINE(ble_init_ok, 0, 1);

uint16_t global_counter;

static bool connectedFlag=false;
static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
  BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
  BT_DATA_BYTES(BT_DATA_UUID128_ALL, TFMICRO_SERVICE_UUID),
};

struct bt_conn *my_connection;


// Setting up the device information service
static int settings_runtime_load(void){
  settings_runtime_set("bt/dis/model",
    DIS_MODEL,DIS_MODEL_LEN);
  settings_runtime_set("bt/dis/manuf",
    DIS_MANUF,DIS_MANUF_LEN);
  settings_runtime_set("bt/dis/fw",
    DIS_FW_REV_STR,DIS_FW_REV_STR_LEN);
  settings_runtime_set("bt/dis/hw",
    DIS_HW_REV_STR,DIS_HW_REV_STR_LEN);
  return 0;
}

static void timer_deinit(void){
  nrfx_timer_disable(&timer_global);
  nrfx_timer_uninit(&timer_global);	
  motion_sleep();
  ppg_sleep();
}

static void timer_init(void){
printk("timer init\n");
  uint32_t time_ticks;
  nrfx_err_t          err;
  nrfx_timer_config_t timer_cfg = {
          .frequency = NRF_TIMER_FREQ_1MHz,
          .mode      = NRF_TIMER_MODE_TIMER,
          .bit_width = NRF_TIMER_BIT_WIDTH_24,
          .interrupt_priority = TIMER_PRIORITY,
          .p_context = NULL,
  };  

  err = nrfx_timer_init(&timer_global, &timer_cfg, timer_handler);
  if (err != NRFX_SUCCESS) {
          printk("nrfx_timer_init failed with: %d\n", err);
  }
  else
          printk("nrfx_timer_init success with: %d\n", err);
  time_ticks = nrfx_timer_ms_to_ticks(&timer_global, TIMER_MS);
  printk("time ticks = %d\n", time_ticks);


  nrfx_timer_extended_compare(&timer_global, NRF_TIMER_CC_CHANNEL0, time_ticks \ 
   , NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrfx_timer_enable(&timer_global);
  printk("timer initialized\n");
}

static void connected(struct bt_conn *conn, uint8_t err){
  struct bt_conn_info info; 
  char addr[BT_ADDR_LE_STR_LEN];

  my_connection = conn;
  if (err) {
    printk("Connection failed (err %u)\n", err);
    return;
  }
  else if(bt_conn_get_info(conn, &info))
    printk("Could not parse connection info\n");
  else{  
  // Start the timer and stop advertising and initialize all the modules
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Connection established!		\n\
      Connected to: %s					\n\
      Role: %u							\n\
      Connection interval: %u				\n\
      Slave latency: %u					\n\
      Connection supervisory timeout: %u	\n"
      , addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
		
    ppg_config();
    motion_config();
    
    connectedFlag=true;
    
    timer_init();
    global_counter = 0;
    gyro_first_read = 0;
    magneto_first_read = 0;  
    ppgRead = 0;
    ppgTFPass = false;
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
  // Stop timer and do all the cleanup
  printk("Disconnected (reason %u)\n", reason);
  timer_deinit();	
  connectedFlag=false;
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param){
  //If acceptable params, return true, otherwise return false.
  if((param->interval_min > 9) && (param->interval_max > 20))
    return false;
  else
    return true; 
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout){
  struct bt_conn_info info; 
  char addr[BT_ADDR_LE_STR_LEN];
	
  if(bt_conn_get_info(conn, &info))
    printk("Could not parse connection info\n");
  else{
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Connection parameters updated!	\n\
      Connected to: %s						\n\
      New Connection Interval: %u				\n\
      New Slave Latency: %u					\n\
      New Connection Supervisory Timeout: %u	\n"
      , addr, info.le.interval, info.le.latency, info.le.timeout);
  }
}

static struct bt_conn_cb conn_callbacks = {
  .connected			= connected,
  .disconnected   		= disconnected,
  .le_param_req			= le_param_req,
  .le_param_updated		= le_param_updated
};


static void bt_ready(int err){
  if (err){
    printk("BLE init failed with error code %d\n", err);
    return;
  }
  else 	
    printk("BLE init success\n");
	
       
  settings_load();
	 
  settings_runtime_load();

  //Configure connection callbacks
  bt_conn_cb_register(&conn_callbacks);

  //Initalize services
  err = tfMicro_service_init();
        
  if (err)
    return;
	
  //Start advertising
  const struct bt_le_adv_param v={ \
    .id = BT_ID_DEFAULT, \
    .sid = 0, \
    .secondary_max_skip = 0, \
    .options = BT_LE_ADV_OPT_CONNECTABLE,\
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2, \
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2, \
    .peer = NULL
  };
	
  err = bt_le_adv_start(&v, ad, ARRAY_SIZE(ad),sd, ARRAY_SIZE(sd));
  if (err) 
    printk("Advertising failed to start (err %d)\n", err);
  else{
    printk("Advertising successfully started\n");
    }

  k_sem_give(&ble_init_ok);
}

// Initialize BLE
static void ble_init(void){
  int err;
  err = bt_enable(bt_ready);
  if (err){
    printk("BLE initialization failed\n");
  }
	
  
  if (!err)
    printk("Bluetooth initialized\n");
  else{
    printk("BLE initialization did not complete in time\n");
  }
  if (err) 
    printk("Bluetooth init failed (err %d)\n", err);	
}

// Timer handler that periodically executes commands with a period, 
// which is defined by the macro-variable TIMER_MS
static void spi_init(void){
  uint32_t dataFlash;
  const char* const spiName_imu = "SPI_1";
  const char* const spiName_ppg = "SPI_4";
 
  spi_dev_imu = device_get_binding(spiName_imu);
  gpioHandle_CS_IMU= device_get_binding("GPIO_0");
        
  spi_dev_ppg = device_get_binding(spiName_ppg);
  gpioHandle_CS_ppg= device_get_binding("GPIO_1");
               
  if (gpioHandle_CS_IMU == NULL) {
    printk("Could not get GPIO_0\n");
    return;
  }
	
  if (gpioHandle_CS_ppg == NULL) {
    printk("Could not get GPIO_1\n");
    return;
  }
	
  imu_cs.gpio_dev=gpioHandle_CS_IMU;
  if (spi_dev_imu == NULL) {
    printk("Could not get %s device\n", spiName_imu);
    return;
  }
  ppg_cs.gpio_dev=gpioHandle_CS_ppg;
  if (spi_dev_ppg == NULL) {
    printk("Could not get %s device\n", spiName_ppg);
    return;
  }
  ppgConfig.isEnabled = true;
  ppgConfig.sample_avg=0x08;
  ppgConfig.green_intensity = 0x28;
  ppgConfig.infraRed_intensity = 0x28;
  ppgConfig.sampling_time = 0x28;
  ppgConfig.numCounts = 8;
  ppgConfig.txPacketEnable = true;
  high_pass_filter_init_25();

  fileOpen();
  dataFlash = (((uint32_t)ppgConfig.green_intensity)<<8) + ((uint32_t)ppgConfig.infraRed_intensity);
  printk("data combo = %d,%d,%d\n",dataFlash,((uint32_t)ppgConfig.green_intensity)<<8,(uint32_t)ppgConfig.infraRed_intensity );
  fileWrite(dataFlash);
  fileClose();


  accelConfig.isEnabled = true;
  accelConfig.txPacketEnable = true;
  accelConfig.sample_bw = ACCEL_DLPFCFG_12HZ;
  accelConfig.sensitivity = ACCEL_FS_SEL_4g;

  gyroConfig.isEnabled = true;
  gyroConfig.txPacketEnable = true;
  gyroConfig.tot_samples = 8;
  gyroConfig.sensitivity = GYRO_FS_SEL_250;
  orientationConfig.isEnabled = true;
  orientationConfig.txPacketEnable = true;
  magnetoConfig.isEnabled = true;
  magnetoConfig.txPacketEnable = true;
  tfMicroCoonfig.isEnabled = true;
  tfMicroCoonfig.txPacketEnable = true;

  configRead[1] = IMU_ENABLE | MAGNETOMETER_ENABLE | PPG_ENABLE | 
    ORIENTATION_ENABLE | TFMICRO_ENABLE;
  configRead[0] = MOTION_BLE_ENABLE | MAGNETOMETER_BLE_ENABLE | 
    PPG_BLE_ENABLE | ORIENTATION_BLE_ENABLE | TFMICRO_BLE_ENABLE;
  configRead[2] = ppgConfig.green_intensity;
  configRead[3] = ppgConfig.infraRed_intensity;
  configRead[4] = 0x12;
  configRead[5] = 0x44;
}
static void i2c_init(void){
  printk("The I2C Init started\n");
  i2c_dev = device_get_binding("I2C_0");
  if (!i2c_dev) {
    printk("Binding failed to i2c.");
    return;
  }
  i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
   
  batteryMonitorConfig.design_capacity = 0x00AA; // 170 mAHour
  batteryMonitorConfig.taper_current = 0x0015; //21 mA
  batteryMonitorConfig.terminate_voltage = 0x0C1C; // 3100 mV
  bq274xx_gauge_init( &batteryMonitorConfig);
}


// Timer handler that periodically executes commands with a period, 
// which is defined by the macro-variable TIMER_MS

void timer_handler(nrf_timer_event_t event_type, void* p_context){
  if(connectedFlag == true){
    switch (event_type){
      case NRF_TIMER_EVENT_COMPARE0:

        // submit work to read gyro, acc, magnetometer and orientation
        my_motionSensor.magneto_first_read = magneto_first_read;
        my_motionSensor.pktCounter = global_counter;
        my_motionSensor.gyro_first_read = gyro_first_read;
        k_work_submit(&my_motionSensor.work);

        // Executes every 2 seconds to send battery level
        if(global_counter % 400 ==0) 
          k_work_submit(&my_battery.work);
    
        if(ppgRead  == 0){
          my_ppgSensor.pktCounter = global_counter;
          my_ppgSensor.movingFlag = gyroData1.movingFlag;
          my_ppgSensor.ppgTFPass = ppgTFPass;
          k_work_submit(&my_ppgSensor.work);
        }  
        // Executes every 8 seconds to send compressed signal
        
        ppgRead = (ppgRead+1) % ppgConfig.numCounts;
        magneto_first_read = (magneto_first_read +1) % (GYRO_SAMPLING_RATE/MAGNETO_SAMPLING_RATE);
        gyro_first_read = (gyro_first_read + 1) % (gyroConfig.tot_samples);
        break;

      default:
              //Do nothing.
        break;
    }
  }
}

void main(void){

  //this initializes FOTA
  #ifdef INCLUDE_DFU
  os_mgmt_register_group();
  
  img_mgmt_register_group();
  smp_bt_register();
  #endif


  const struct device *dev;
  bool led_is_on = true;
  int ret;

  IRQ_CONNECT(TIMER1_IRQn, TIMER_PRIORITY,
    nrfx_timer_1_irq_handler, NULL, 0);

  dev = device_get_binding(LED0);
  if (dev == NULL)
    return;
	
  ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
  if (ret < 0)
    return;

  fs_mount_init();
  spi_init();
  uint8_t tx_buffer[4],rx_buffer[4];

  tx_buffer[0] = READMASTER | 0x00;
  tx_buffer[1] =0xFF;
  uint8_t txLen=2,rxLen=2;
  spiRead_registerIMU(tx_buffer, txLen, rx_buffer, rxLen);
  printk("Chip ID from motion sensor=%x\n",rx_buffer[1]);
	
  tx_buffer[0] = 0xFF;
  tx_buffer[1] = READMASTER;
  tx_buffer[2] = 0x00;
	
  txLen=3;
  rxLen=3;
  spiRead_registerPPG(tx_buffer, txLen, rx_buffer, rxLen);
  printk("Chip ID from ppg sensor=%x,%x,%x\n",rx_buffer[0],rx_buffer[1],rx_buffer[2]);
  
  ppg_config();
  motion_config();
  i2c_init(); 
  
  k_work_q_start(&my_work_q, my_stack_area,
    K_THREAD_STACK_SIZEOF(my_stack_area), WORKQUEUE_PRIORITY);
               
  k_work_init(&my_battery.work, bas_notify);     
  k_work_init(&my_motionSensor.work, motion_data_timeout_handler);
  k_work_init(&my_ppgSensor.work, read_ppg_fifo_buffer);
  k_work_init(&my_motionData.work, motion_notify);
  k_work_init(&my_magnetoSensor.work, magneto_notify);
  k_work_init(&my_orientaionSensor.work, orientation_notify);
  k_work_init(&my_ppgDataSensor.work, ppgData_notify);
  ble_init();
  


  while (1) {
    printk("%d\n", connectedFlag); 
    if(!connectedFlag)
      led_is_on = !led_is_on;
    else
      led_is_on = 0;
    gpio_pin_set(dev, PIN, (int)led_is_on);
    k_msleep(SLEEP_TIME_MS);
  }
}
