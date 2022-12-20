#include <zephyr.h>
#include <cstdio>
#include "common.h"
#include "imuSensor.h"
//#include "DSP/Include/arm_const_structs.h"
//#include "DSP/Include/arm_math.h"
#include "BLEService.h"
float32_t accMagStaticThrehsold =0.015f;
static float32_t mxBiax_arr[500]={0.0f},myBiax_arr[500]={0.0f},mzBiax_arr[500]={0.0f};
static float32_t gyXBiax_arr[300]={0.0f},gyYBiax_arr[300]={0.0f},gyZBiax_arr[300]={0.0f};
uint8_t blePktOrientation[ble_orientationPktLength];
static float32_t gx_offset=0.0f,gy_offset=0.0f,gz_offset=0.0f;
static float32_t mx_offset=0.0f,my_offset=0.0f,mz_offset=0.0f;
static float32_t mx_scale=1.0f,my_scale=1.0f,mz_scale=1.0f;

uint16_t pointerMag=0,pointer_g=0;
static uint8_t magFilled=0;

static float32_t beta=0.2;				// algorithm gain
struct orientationData orientationData1; 
//---------------------------------------------------------------------------------------------------
// Variable definitions

float32_t invSqrt(float32_t x) {
  float32_t y;
  arm_sqrt_f32(x,&y);
  if(y!=0)
    y = 1.0f/y;
  return y;
}


// AHRS algorithm update
void MadgwickAHRSupdateIMU(float32_t gx, float32_t gy, 
float32_t gz, float32_t ax, float32_t ay, float32_t az) {
  float32_t recipNorm;
  float32_t s0, s1, s2, s3;
  float32_t qDot1, qDot2, qDot3, qDot4;
  float32_t _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-orientationData1.q1 * gx - orientationData1.q2 * gy 
    - orientationData1.q3 * gz);
  qDot2 = 0.5f * (orientationData1.q0 * gx + orientationData1.q2 * gz 
    - orientationData1.q3 * gy);
  qDot3 = 0.5f * (orientationData1.q0 * gy - orientationData1.q1 * gz 
    + orientationData1.q3 * gx);
  qDot4 = 0.5f * (orientationData1.q0 * gz + orientationData1.q1 * gy 
    - orientationData1.q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   
    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * orientationData1.q0;
    _2q1 = 2.0f * orientationData1.q1;
    _2q2 = 2.0f * orientationData1.q2;
    _2q3 = 2.0f * orientationData1.q3;
    _4q0 = 4.0f * orientationData1.q0;
    _4q1 = 4.0f * orientationData1.q1;
    _4q2 = 4.0f * orientationData1.q2;
    _8q1 = 8.0f * orientationData1.q1;
    _8q2 = 8.0f * orientationData1.q2;
    q0q0 = orientationData1.q0 * orientationData1.q0;
    q1q1 = orientationData1.q1 * orientationData1.q1;
    q2q2 = orientationData1.q2 * orientationData1.q2; 
    q3q3 = orientationData1.q3 * orientationData1.q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * orientationData1.q1 - 
      _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * orientationData1.q2 + _2q0 * ax + _4q2 * q3q3 - 
      _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * orientationData1.q3 - _2q1 * ax + 
      4.0f * q2q2 * orientationData1.q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step  
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  orientationData1.q0 += qDot1 * (1.0f / sampleFreq);
  orientationData1.q1 += qDot2 * (1.0f / sampleFreq);
  orientationData1.q2 += qDot3 * (1.0f / sampleFreq);
  orientationData1.q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(orientationData1.q0 * orientationData1.q0 + 
  orientationData1.q1 * orientationData1.q1 + 
  orientationData1.q2 * orientationData1.q2 + 
  orientationData1.q3 * orientationData1.q3);
  orientationData1.q0 *= recipNorm;
  orientationData1.q1 *= recipNorm;
  orientationData1.q2 *= recipNorm;
  orientationData1.q3 *= recipNorm;
}

void MadgwickAHRSupdate(float32_t gx, float32_t gy, float32_t gz, 
float32_t ax, float32_t ay, float32_t az, float32_t mx, 
float32_t my, float32_t mz) {
  float32_t recipNorm;
  float32_t s0, s1, s2, s3;
  float32_t qDot1, qDot2, qDot3, qDot4;
  float32_t hx, hy;
  float32_t _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, 
  _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, 
  q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }
	
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-orientationData1.q1 * gx - orientationData1.q2 * gy 
    - orientationData1.q3 * gz);
  qDot2 = 0.5f * (orientationData1.q0 * gx + orientationData1.q2 * gz 
    - orientationData1.q3 * gy);
  qDot3 = 0.5f * (orientationData1.q0 * gy - orientationData1.q1 * gz 
    + orientationData1.q3 * gx);
  qDot4 = 0.5f * (orientationData1.q0 * gz + orientationData1.q1 * gy 
    - orientationData1.q2 * gx);

  // Compute feedback only if accelerometer measurement 
  //valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * orientationData1.q0 * mx;
    _2q0my = 2.0f * orientationData1.q0 * my;
    _2q0mz = 2.0f * orientationData1.q0 * mz;
    _2q1mx = 2.0f * orientationData1.q1 * mx;
    _2q0 = 2.0f * orientationData1.q0;
    _2q1 = 2.0f * orientationData1.q1;
    _2q2 = 2.0f * orientationData1.q2;
    _2q3 = 2.0f * orientationData1.q3;
    _2q0q2 = 2.0f * orientationData1.q0 * orientationData1.q2;
    _2q2q3 = 2.0f * orientationData1.q2 * orientationData1.q3;
    q0q0 = orientationData1.q0 * orientationData1.q0;
    q0q1 = orientationData1.q0 * orientationData1.q1;
    q0q2 = orientationData1.q0 * orientationData1.q2;
    q0q3 = orientationData1.q0 * orientationData1.q3;
    q1q1 = orientationData1.q1 * orientationData1.q1; 
    q1q2 = orientationData1.q1 * orientationData1.q2;
    q1q3 = orientationData1.q1 * orientationData1.q3;
    q2q2 = orientationData1.q2 * orientationData1.q2;
    q2q3 = orientationData1.q2 * orientationData1.q3;
    q3q3 = orientationData1.q3 * orientationData1.q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * orientationData1.q3 + _2q0mz * orientationData1.q2 
      + mx * q1q1 + _2q1 * my * orientationData1.q2 + _2q1 * mz * orientationData1.q3 
      - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * orientationData1.q3 + my * q0q0 - _2q0mz * orientationData1.q1 
      + _2q1mx * orientationData1.q2 - my * q1q1 + my * q2q2 + 
      _2q2 * mz * orientationData1.q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * orientationData1.q2 + _2q0my * orientationData1.q1 + 
      mz * q0q0 + _2q1mx * orientationData1.q3 - mz * q1q1 + 
      _2q2 * my * orientationData1.q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) 
      - _2bz * orientationData1.q2 * (_2bx * (0.5f - q2q2 - q3q3) +
       _2bz * (q1q3 - q0q2) - mx) + (-_2bx * orientationData1.q3 + 
       _2bz * orientationData1.q1) * (_2bx * (q1q2 - q0q3) + 
       _2bz * (q0q1 + q2q3) - my) + _2bx * orientationData1.q2 * (_2bx * (q0q2 + q1q3) 
       + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 
      + _2q2q3 - ay) - 4.0f * orientationData1.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) 
      + _2bz * orientationData1.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
      + (_2bx * orientationData1.q2 + _2bz * orientationData1.q0) * (_2bx * (q1q2 - q0q3) 
      + _2bz * (q0q1 + q2q3) - my) + (_2bx * orientationData1.q3 - 
      _4bz * orientationData1.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) 
      - 4.0f * orientationData1.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) 
      + (-_4bx * orientationData1.q2 - _2bz * orientationData1.q0) * (_2bx * (0.5f - q2q2 - q3q3)
      + _2bz * (q1q3 - q0q2) - mx) + (_2bx * orientationData1.q1 + _2bz * orientationData1.q3)
      * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
      (_2bx * orientationData1.q0 - _4bz * orientationData1.q2) * (_2bx * (q0q2 + q1q3) 
      + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) 
      + (-_4bx * orientationData1.q3 + _2bz * orientationData1.q1) * 
      (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
      + (-_2bx * orientationData1.q0 + _2bz * orientationData1.q2) * (_2bx * (q1q2 - q0q3)
      + _2bz * (q0q1 + q2q3) - my) + _2bx * orientationData1.q1 * (_2bx * (q0q2 + q1q3) 
      + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion 
  orientationData1.q0 += qDot1 * (1.0f / sampleFreq);
  orientationData1.q1 += qDot2 * (1.0f / sampleFreq);
  orientationData1.q2 += qDot3 * (1.0f / sampleFreq);
  orientationData1.q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(orientationData1.q0 * orientationData1.q0 + 
  orientationData1.q1 * orientationData1.q1 + 
  orientationData1.q2 * orientationData1.q2 + 
  orientationData1.q3 * orientationData1.q3);
  orientationData1.q0 *= recipNorm;
  orientationData1.q1 *= recipNorm;
  orientationData1.q2 *= recipNorm;
  orientationData1.q3 *= recipNorm;
}



void motion_data_orientation_timeout_handler(uint16_t pktCounter){
  float32_t norm_acc;
  float32_t gyro_orientation_x,gyro_orientation_y,gyro_orientation_z;
  float32_t acc_orientation_x,acc_orientation_y,acc_orientation_z;
  float32_t mag_orientation_x,mag_orientation_y,mag_orientation_z;
  float_cast tempBuff;
  // Compute the angular velocity in radians/second
  gyro_orientation_x = gyroData1.gyrox_val; 
  gyro_orientation_y = gyroData1.gyroy_val; 
  gyro_orientation_z = gyroData1.gyroz_val;
					
  acc_orientation_x = accData1.accx_val;  
  acc_orientation_y = accData1.accy_val;  
  acc_orientation_z = accData1.accz_val;
  //printf("accx= %f,accy=%f,accz=%f, gx=%f,gy=%f,gz=%f\n",
  //  acc_orientation_x,acc_orientation_y,acc_orientation_z,
  //  gyro_orientation_x,gyro_orientation_y,gyro_orientation_z);
  arm_sqrt_f32(acc_orientation_x*acc_orientation_x +
  acc_orientation_y*acc_orientation_y+
  acc_orientation_z*acc_orientation_z, &norm_acc );
  if( norm_acc -1 < accMagStaticThrehsold && 
  norm_acc -1 > -accMagStaticThrehsold){   
    gyXBiax_arr[pointer_g]=gyro_orientation_x;
    gyYBiax_arr[pointer_g]=gyro_orientation_y;
    gyZBiax_arr[pointer_g]=gyro_orientation_z;
    pointer_g = (pointer_g + 1)%(200);
    arm_mean_f32( gyXBiax_arr,200, &gx_offset);
    arm_mean_f32( gyYBiax_arr,200, &gy_offset);
    arm_mean_f32( gyZBiax_arr,200, &gz_offset);
  }	
//  gyro_orientation_x = angularVelX -gx_offset ; 
//  gyro_orientation_y = angularVelY-gy_offset; 
//  gyro_orientation_z = angularVelZ -gz_offset;

  MadgwickAHRSupdate(gyro_orientation_x, gyro_orientation_y, 
    gyro_orientation_z, acc_orientation_x, acc_orientation_y,
    acc_orientation_z, mag_orientation_x, mag_orientation_y, 
    mag_orientation_z);

  tempBuff.float_val = orientationData1.q0;
  blePktOrientation[0] =tempBuff.floatcast[0];
  blePktOrientation[1] =tempBuff.floatcast[1];
  blePktOrientation[2] =tempBuff.floatcast[2];
  blePktOrientation[3] =tempBuff.floatcast[3];

  tempBuff.float_val = orientationData1.q1;
  blePktOrientation[4] =tempBuff.floatcast[0];
  blePktOrientation[5] =tempBuff.floatcast[1];
  blePktOrientation[6] =tempBuff.floatcast[2];
  blePktOrientation[7] =tempBuff.floatcast[3];

  tempBuff.float_val = orientationData1.q2;
  blePktOrientation[8] =tempBuff.floatcast[0];
  blePktOrientation[9] =tempBuff.floatcast[1];
  blePktOrientation[10] =tempBuff.floatcast[2];
  blePktOrientation[11] =tempBuff.floatcast[3];

  tempBuff.float_val = orientationData1.q3;
  blePktOrientation[12] =tempBuff.floatcast[0];
  blePktOrientation[13] =tempBuff.floatcast[1];
  blePktOrientation[14] =tempBuff.floatcast[2];
  blePktOrientation[15] =tempBuff.floatcast[3];

  blePktOrientation[16] = (pktCounter&0xFF00) >> 8;
  blePktOrientation[17] = (pktCounter&0x00FF);
  
  if(orientationConfig.txPacketEnable == true){
    my_orientaionSensor.dataPacket = blePktOrientation;
    my_orientaionSensor.packetLength = ORIENTATION_DATA_LEN;
    k_work_submit(&my_orientaionSensor.work);
  }

  mag_orientation_x = magnetoData1.Hx_val;
  mag_orientation_y = magnetoData1.Hy_val;
  mag_orientation_z = magnetoData1.Hz_val;
  if( norm_acc > 1.3f ){   
    mxBiax_arr[pointerMag]=mag_orientation_x;
    myBiax_arr[pointerMag]=mag_orientation_y;
    mzBiax_arr[pointerMag]=mag_orientation_z;
    pointerMag = (pointerMag + 1)%(500);
    if(pointerMag ==0){
      float32_t temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9,temp10;
      magFilled=1;
      arm_max_f32(mxBiax_arr, 500, &temp1,NULL);
      arm_min_f32(mxBiax_arr, 500, &temp2,NULL);
      mx_offset = (temp2 +temp1)*0.5;
      arm_max_f32(myBiax_arr, 500, &temp3,NULL);
      arm_min_f32(myBiax_arr, 500, &temp4,NULL);
      my_offset = (temp3 +temp4)*0.5;
      arm_max_f32(mzBiax_arr, 500, &temp5,NULL);
      arm_min_f32(mzBiax_arr, 500, &temp6,NULL);
      mz_offset = (temp5 +temp6)*0.5;
     
      temp7 = 0.5f*(temp1-temp2);
      temp8 = 0.5f*(temp3-temp4);
      temp9 = 0.5f*(temp5-temp6);
      temp10 = 1/3.0f*(temp7+temp8+temp9);
      if(temp7 > 0.0001f && temp8 > 0.0001f && temp9 >0.0001f){	
        mx_scale = temp10/temp7;
        my_scale = temp10/temp8;
        mz_scale = temp10/temp9;
      }
      else{ 
        mx_scale = 1.0f;
	my_scale = 1.0f;
        mz_scale = 1.0f;
      }
    }
                    
    mag_orientation_x= mx_scale*(mag_orientation_x - mx_offset);
    mag_orientation_y= my_scale*(mag_orientation_y - my_offset);
    mag_orientation_z= mz_scale*(mag_orientation_z - mz_offset);	
  }
}
