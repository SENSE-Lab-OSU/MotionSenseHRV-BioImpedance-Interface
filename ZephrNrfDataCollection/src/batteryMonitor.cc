

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>

#include "batteryMonitor.h"
#define BATTERY_MONITOR_ADDR 0x55
#define BQ274XX_SUBCLASS_DELAY 5 /* subclass 64 & 82 needs 5ms delay */


int bq274xx_gauge_init(struct bq274xx_config* batterMonitorConfig);
int bq274xx_sample_fetch(struct bq274xx_data *bq274xx, enum sensor_channel chan);

int bq274xx_command_reg_read( uint8_t reg_addr,uint16_t *val){
  uint8_t buf[3],read_buf[3];
  int status;
  buf[0]=reg_addr;

  read_buf[0]=0x00;
  read_buf[1]=0x00;
  read_buf[2]=0x00;

  status = i2c_write_read(i2c_dev, BATTERY_MONITOR_ADDR,buf, 1,read_buf, 2);
  if (status < 0) {
    printk("Unable to read register internal\n");
    return status;
  }

  *val = (read_buf[1] << 8) | read_buf[0];
  return 0;
}

int bq274xx_control_reg_write(uint16_t subcommand){
  uint8_t i2c_writedata[3];
  int status = 0;

  i2c_writedata[0] = BQ274XX_COMMAND_CONTROL_LOW;
  i2c_writedata[1] = (uint8_t)((subcommand)&0x00FF);
  i2c_writedata[2] = (uint8_t)((subcommand >> 8) & 0x00FF);
  status = i2c_write(i2c_dev,i2c_writedata,3, BATTERY_MONITOR_ADDR);
  
  if (status < 0) {
    printk("Failed to write into control low register");
    return status;
  }
  return 0;
}

int bq274xx_command_reg_write(uint8_t command,
				     uint8_t data){
  uint8_t i2c_data, reg_addr;
  int status = 0;

  reg_addr = command;
  i2c_data = data;

  status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR, reg_addr,
				    i2c_data);
  if (status < 0) {
    printk("Failed to write into control register");
    return status;
  }
  return 0;
}

int bq274xx_read_data_block( uint8_t offset,
				   uint8_t *data, uint8_t bytes){
  uint8_t i2c_data;
  int status = 0;
  i2c_data = BQ274XX_EXTENDED_BLOCKDATA_START + offset;

  status = i2c_burst_read(i2c_dev, BATTERY_MONITOR_ADDR, i2c_data,
    data, bytes);
  if (status < 0) {
    printk("Failed to read block");
    return status;
  }
  return 0;
}

int bq274xx_get_device_type( uint16_t *val){
  int status;
  status = bq274xx_control_reg_write( BQ274XX_CONTROL_DEVICE_TYPE);
  if (status < 0) {
    printk("Unable to write control register");
    return status;
  }

  bq274xx_command_reg_read(BQ274XX_COMMAND_CONTROL_LOW,val);
  return 0;
}

/**
 * @brief sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
 /*
static int bq274xx_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct bq274xx_data *bq274xx = dev->data;
	float int_temp;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = ((bq274xx->voltage / 1000));
		val->val2 = ((bq274xx->voltage % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		val->val1 = ((bq274xx->avg_current / 1000));
		val->val2 = ((bq274xx->avg_current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
		val->val1 = ((bq274xx->stdby_current / 1000));
		val->val2 = ((bq274xx->stdby_current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT:
		val->val1 = ((bq274xx->max_load_current / 1000));
		val->val2 = ((bq274xx->max_load_current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_TEMP:
		int_temp = (bq274xx->internal_temperature * 0.1);
		int_temp = int_temp - 273.15;
		val->val1 = (int32_t)int_temp;
		val->val2 = (int_temp - (int32_t)int_temp) * 1000000;
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = bq274xx->state_of_charge;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
		val->val1 = bq274xx->state_of_health;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		val->val1 = (bq274xx->full_charge_capacity / 1000);
		val->val2 = ((bq274xx->full_charge_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		val->val1 = (bq274xx->remaining_charge_capacity / 1000);
		val->val2 =
			((bq274xx->remaining_charge_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
		val->val1 = (bq274xx->nom_avail_capacity / 1000);
		val->val2 = ((bq274xx->nom_avail_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY:
		val->val1 = (bq274xx->full_avail_capacity / 1000);
		val->val2 = ((bq274xx->full_avail_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_AVG_POWER:
		val->val1 = (bq274xx->avg_power / 1000);
		val->val2 = ((bq274xx->avg_power % 1000) * 1000U);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
*/

int bq274xx_sample_fetch(struct bq274xx_data *bq274xx, enum sensor_channel chan){
	
  int status = 0;
  switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_VOLTAGE, 
        &bq274xx->voltage);
      if (status < 0) {
        printk("Failed to read voltage\n");
        return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_AVG_CURRENT:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_AVG_CURRENT,
        &bq274xx->avg_current);
      if (status < 0) {
        printk("Failed to read average current\n ");
	return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_TEMP:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_INT_TEMP,
        &bq274xx->internal_temperature);
      if (status < 0) {
        printk("Failed to read internal temperature\n");
	return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_STDBY_CURRENT,
        &bq274xx->stdby_current);
      if (status < 0) {
        printk("Failed to read standby current\n");
        return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_MAX_CURRENT,
        &bq274xx->max_load_current);
      if (status < 0) {
        printk("Failed to read maximum current\n");
	return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_SOC,
        &bq274xx->state_of_charge);
      if (status < 0) {
        printk("Failed to read state of charge\n");
	return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_FULL_CAPACITY,
        &bq274xx->full_charge_capacity);
      if (status < 0) {
        printk("Failed to read full charge capacity\n");
	return -EIO;
      }
      break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_REM_CAPACITY,
        &bq274xx->remaining_charge_capacity);
      if (status < 0) {
        printk("Failed to read remaining charge capacity\n");
	return -EIO;
      }
      break;

    case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_NOM_CAPACITY,
        &bq274xx->nom_avail_capacity);
      if (status < 0) {
        printk("Failed to read nominal available capacity\n");
        return -EIO;
      }
      break;

    case SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_AVAIL_CAPACITY,
        &bq274xx->full_avail_capacity);
      if (status < 0) {
        printk("Failed to read full available capacity\n");
        return -EIO;
      }
      break;

    case SENSOR_CHAN_GAUGE_AVG_POWER:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_AVG_POWER,
        &bq274xx->avg_power);
      if (status < 0) {
        printk("Failed to read battery average power\n");
        return -EIO;
      }
      break;

    case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
      status = bq274xx_command_reg_read(BQ274XX_COMMAND_SOH,
        &bq274xx->state_of_health);

      bq274xx->state_of_health = (bq274xx->state_of_health) & 0x00FF;
      if (status < 0) {
        printk("Failed to read state of health\n");
        return -EIO;
      }
      break;
    
    default:
      return -ENOTSUP;
  }

  return 0;
}

/**
 * @brief initialise the fuel gauge
 *
 * @return 0 for success
 */
int bq274xx_gauge_init(struct bq274xx_config* config){
  int status = 0; 
  uint8_t tmp_checksum = 0, checksum_old = 0, checksum_new = 0;
  uint16_t flags = 0, designenergy_mwh = 0, taperrate = 0, id;
  uint8_t designcap_msb, designcap_lsb, designenergy_msb, designenergy_lsb,
    terminatevolt_msb, terminatevolt_lsb, taperrate_msb,
    taperrate_lsb;
  uint8_t block[32];

  designenergy_mwh = (uint16_t)3.7 * config->design_capacity;
  taperrate = 
    (uint16_t)config->design_capacity / (0.1 * config->taper_current);


  status = bq274xx_get_device_type(&id);
  if (status < 0) {
    printk("Unable to get device ID");
    return status;
  }
        
  printk("data from i2c data1=%x\n",id);
  if (id != BQ274XX_DEVICE_ID) {
    printk("Invalid Device");
    return -10;
  }
  /** Unseal the battery control register **/
        
  status = bq274xx_control_reg_write(BQ274XX_UNSEAL_KEY);
  if (status < 0) {
    printk("Unable to unseal the battery");
    return status;
  }

  status = bq274xx_control_reg_write( BQ274XX_UNSEAL_KEY);
  if (status < 0) {
    printk("Unable to unseal the battery");
    return status;
  }

  /* Send CFG_UPDATE */
        
  status = bq274xx_control_reg_write(BQ274XX_CONTROL_SET_CFGUPDATE);
  if (status < 0) {
    printk("Unable to set CFGUpdate");
    return status;  
  }
        
  /** Step to place the Gauge into CONFIG UPDATE Mode **/
  do {
    status = bq274xx_command_reg_read(BQ274XX_COMMAND_FLAGS, &flags);
    if(status <0){
      printk("Unable to transfer to CONFIG UPDATE MODE\n");
      return status;
    }
    if (!(flags & 0x0010)) 
      k_msleep(BQ274XX_SUBCLASS_DELAY * 10);
  } while (!(flags & 0x0010));
  
  status = bq274xx_command_reg_write(BQ274XX_EXTENDED_DATA_CONTROL, 0x00);
  if (status < 0) {
    printk("Failed to enable block data memory");
    return status;
  }
        
  /* Access State subclass */
  status = bq274xx_command_reg_write( BQ274XX_EXTENDED_DATA_CLASS,
					   0x52);
  if (status < 0) {
    printk("Failed to update state subclass");
    return status;
  }

  /* Write the block offset */
  status = bq274xx_command_reg_write( BQ274XX_EXTENDED_DATA_BLOCK,
					   0x00);
  if (status < 0) {
    printk("Failed to update block offset");
    return status;
  }

  for (uint8_t i = 0; i < 32; i++) 
    block[i] = 0;
	

  status = bq274xx_read_data_block( 0x00, block, 32);
  if (status < 0) {
    printk("Unable to read block data");
    return status;
  }

  tmp_checksum = 0;
  for (uint8_t i = 0; i < 32; i++) 
    tmp_checksum += block[i];
	
  tmp_checksum = 255 - tmp_checksum;

  /* Read the block checksum */
  status = i2c_reg_read_byte(i2c_dev, BATTERY_MONITOR_ADDR,
    BQ274XX_EXTENDED_CHECKSUM, &checksum_old);
  if (status < 0) {
    printk("Unable to read block checksum");
    return status;
  }

  designcap_msb = config->design_capacity >> 8;
  designcap_lsb = config->design_capacity & 0x00FF;
  designenergy_msb = designenergy_mwh >> 8;
  designenergy_lsb = designenergy_mwh & 0x00FF;
  terminatevolt_msb = config->terminate_voltage >> 8;
  terminatevolt_lsb = config->terminate_voltage & 0x00FF;
  taperrate_msb = taperrate >> 8;
  taperrate_lsb = taperrate & 0x00FF;

  status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
    BQ274XX_EXTENDED_BLOCKDATA_DESIGN_CAP_HIGH,designcap_msb);
  if (status < 0) {
    printk("Failed to write designCAP MSB");
    return status;
  }

  status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
    BQ274XX_EXTENDED_BLOCKDATA_DESIGN_CAP_LOW,designcap_lsb);
  if (status < 0) {
    printk("Failed to erite designCAP LSB");
    return status;
  }

  status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
    BQ274XX_EXTENDED_BLOCKDATA_DESIGN_ENR_HIGH,
    designenergy_msb);
  if (status < 0) {
    printk("Failed to write designEnergy MSB");
    return status;
  }

	status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
				    BQ274XX_EXTENDED_BLOCKDATA_DESIGN_ENR_LOW,
				    designenergy_lsb);
	if (status < 0) {
		printk("Failed to erite designEnergy LSB");
		return status;
	}

	status = i2c_reg_write_byte(
		i2c_dev, BATTERY_MONITOR_ADDR,
		BQ274XX_EXTENDED_BLOCKDATA_TERMINATE_VOLT_HIGH,
		terminatevolt_msb);
	if (status < 0) {
		printk("Failed to write terminateVolt MSB");
		return status;
	}

	status = i2c_reg_write_byte(
		i2c_dev, BATTERY_MONITOR_ADDR,
		BQ274XX_EXTENDED_BLOCKDATA_TERMINATE_VOLT_LOW,
		terminatevolt_lsb);
	if (status < 0) {
		printk("Failed to write terminateVolt LSB");
		return status;
	}

	status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
				    BQ274XX_EXTENDED_BLOCKDATA_TAPERRATE_HIGH,
				    taperrate_msb);
	if (status < 0) {
		printk("Failed to write taperRate MSB");
		return status;
	}

	status = i2c_reg_write_byte(i2c_dev, BATTERY_MONITOR_ADDR,
				    BQ274XX_EXTENDED_BLOCKDATA_TAPERRATE_LOW,
				    taperrate_lsb);
	if (status < 0) {
		printk("Failed to erite taperRate LSB");
		return status;
	}

	for (uint8_t i = 0; i < 32; i++) {
		block[i] = 0;
	}

	status = bq274xx_read_data_block( 0x00, block, 32);
	if (status < 0) {
		printk("Unable to read block data");
		return status;
	}

	checksum_new = 0;
	for (uint8_t i = 0; i < 32; i++) {
		checksum_new += block[i];
	}
	checksum_new = 255 - checksum_new;

	status = bq274xx_command_reg_write( BQ274XX_EXTENDED_CHECKSUM,
					   checksum_new);
	if (status < 0) {
		printk("Failed to update new checksum");
		return status;
	}

	tmp_checksum = 0;
	status = i2c_reg_read_byte(i2c_dev, BATTERY_MONITOR_ADDR,
				   BQ274XX_EXTENDED_CHECKSUM, &tmp_checksum);
	if (status < 0) {
		printk("Failed to read checksum");
		return status;
	}

	status = bq274xx_control_reg_write( BQ274XX_CONTROL_BAT_INSERT);
	if (status < 0) {
		printk("Unable to configure BAT Detect");
		return status;
	}

	status = bq274xx_control_reg_write( BQ274XX_CONTROL_SOFT_RESET);
	if (status < 0) {
		printk("Failed to soft reset the gauge");
		return status;
	}
   
	flags = 0;
	/* Poll Flags   */
	do { 
		status = bq274xx_command_reg_read(
			 BQ274XX_COMMAND_FLAGS, &flags);
		if (status < 0) {
			printk("Unable to read flags");
			return status;
		}
                
		if (!(flags & 0x0010)) {
                  k_msleep(BQ274XX_SUBCLASS_DELAY * 100);
		}
	} while ((flags & 0x0010));
        
 
	/* Seal the gauge */
	status = bq274xx_control_reg_write( BQ274XX_CONTROL_SEALED);
	if (status < 0) {
		printk("Failed to seal the gauge");
		return status;
	}


	return 0;
}

// Function that periodically extracts the battery level and needs to be called in a timer routine 
void bas_notify(struct k_work *item){
  bq274xx_sample_fetch(&batteryMonitor, 
                        SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY);
  bt_bas_set_battery_level(batteryMonitor.remaining_charge_capacity);
}


