//プログラム 2023.10.16~
#include <SD.h>
#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MadgwickAHRS.h>

Madgwick madgwick;

volatile uint8_t highbit;
volatile uint8_t lowbit;
volatile char tag;
volatile int16_t value;

volatile float x_volatile, y_volatile, z_volatile;
volatile float v_mag_volatile, yawrate_volatile;

float x2,y2,z2,x_pre,y_pre,z_pre;

union sendint {
  int i;
  uint8_t ui[2];
};
volatile sendint send1, send2;

AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int32_t offset_x, offset_y, offset_z;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_shenzhen = -2.2;

const float pi = 3.14159265358979323846264338327950288;
const float g = 9.81;

float hard_offset_x ;
float hard_offset_y ;
float hard_offset_z ;
float field_x ;
float field_y ;
float field_z ;

uint8_t IN[7];
uint8_t CHANNEL[7];

HardwareSerial SbusSerial2(2);

//SBUS
#define SBUS_STARTBYTE 0x0F
#define SBUS_ENDBYTE_1 0b00100000
#define SBUS_ENDBYTE_2 0b00101000
#define SBUS_ENDBYTE_3 0b00100100
#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE 1

unsigned int error_sbus_start;
unsigned int error_sbus_end;
unsigned int sbus_frame_loss;
unsigned int sbus_failsafe;
int16_t ch[18];

//Control
const float control_sample_time = 0.04;

const float kp_ail = 40.0;const float kd_ail = 0.0;const float ki_ail = 0.0;
float kp_ele = 10.0; //離陸25
const float kp_ele_turn = 22.0; //旋回22
const float kd_ele = 0.0;//2
const float kd1_ele = 0.0;
float ki_ele = 5.0;//16,8
const float ki_ele_turn = 18.0;
const float kp_rud_right = 25.0;
const float kd_rud_right = 0.0;
const float ki_rud_right = 16.0;
const float kp_rud_left = 20.0;
const float kd_rud_left = 0.0;
const float ki_rud_left = 15.0;
const float kp_sff = 10.0;
const float kd_sff = 0.0;
const float ki_sff = 0.0;
float kp_thrust = 38.0;
float kd_thrust = 3.0;
float ki_thrust = 5.0;

const float roll_ref = 0.0;
float pitch_ref = 20; //離陸,25で飛ぶ
const float pitch_ref_turn = 15.0; //旋回
const float sideslip_ref = 15.0;
const float radius_ref = 6.0;
float h_ref ;
float hh_count = 0;

float yawrate;
float ut;

float radius;
float radius_pre;
float roll_integral, pitch_integral,h_integral, radius_integral, sideslip_integral,kiatu_integral,pitch_def;
float pitch_pre,yaw1,yaw2,pitch1,pitch2,roll1;

int16_t ch_default[8];
int16_t ch_correction[8];
int16_t ch_temp[8];
int servo[7];
unsigned int flag_roll, flag_pitch,flag_h, flag_radius, flag_radius_left, flag_radius_right;
unsigned int count_pitch, count_radius, count_h, count_r;
float ail, ele, rud, sff, flap;

int flag_sbus = 0;

float h,h1, dh, h_ref_pre1, dh_ref1, dh_ref_pre1, h_ref1, dh_count;
float h_pre,h_ref_pre, dh_ref, dh_ref_pre, dh_temp, h_temp_pre, h_t_pre, yaw_mag_correction,h_temp0,count_htime,edh;


float h_0, h_time = -0.1;
//float h_te=0.7;//指数関数の底9s
//float h_te=0.65;//指数関数の底
//float h_te = 0.6; //指数関数の底6s//hurea2
//
//float h_te=0.45;//動画ふれあ１

//float h_te = 0.95;
float h_te = 0.35;

float kiatu_ref;
float kiatu_pre;
float kiatu1, kiatu0, kiatu0_temp;
uint8_t oversamplingOne = 0.05;
int16_t retOne,retTwo;
unsigned int count_kiatu,flag_kiatu;
const float kp_kiatu=2;
const float ki_kiatu=5;
const float kd_kiatu=5;



//GPS
float latitude;
float longitude;
float alt;

//loop
#define MOUNTING_ANGLE_IMU 14.8 //PSJA搭載型横力板UAV_IMU取付角[deg]

float temp_float;

unsigned int loop_count = 0;
unsigned long time1 = 0;

float h0_count, h0_camera, h_temp;
float ax, ay, az, p, q, r, mx, my, mz;
float ax_g, ay_g, az_g;
float roll, pitch, yaw;
float roll_magnetism, pitch_magnetism, yaw_magnetism;
float v_sen_x, v_sen_y, v_sen_z, u, v, w, v_mag,v_def, v_mag_pre, attack_angle, sideslip_angle;


//旋回PID
float sff_ref = 0;
float rud_ref = 0;
float sff_integral = 0;
float rud_integral = 0;
float flag_sff = 0;
float sff_pre = 0,rud_pre = 0;


void SBUS(void){
  uint8_t buf[25];
  unsigned int buf_index = 0;
  
  error_sbus_start = 0;
  error_sbus_end = 0;
  
  while(SbusSerial2.available()){
    buf[buf_index] = SbusSerial2.read();
    
    if(buf_index == 0 && buf[0] != SBUS_STARTBYTE) error_sbus_start++;
    
    else if(buf_index == 24){
      if(buf[24] != SBUS_ENDBYTE_1 && buf[24] != SBUS_ENDBYTE_2 && buf[24] != SBUS_ENDBYTE_3){
        error_sbus_end++;
        buf_index = 0;
      }
      
      //else{
        ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
        ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
        ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
        ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
        ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
        ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
        ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
        ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
        ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
        ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
        ch[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
        ch[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
        ch[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
        ch[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
        ch[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
        ch[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);
        
        if((buf[23] >> 3) & 0x0001) sbus_failsafe = SBUS_FAILSAFE_ACTIVE;
        else sbus_failsafe = SBUS_FAILSAFE_INACTIVE;
        
        if((buf[23] >> 2) & 0x0001) sbus_frame_loss = 0;
        else  sbus_frame_loss = 1;
        
        for(buf_index = 0; buf_index < 12; buf_index++) ch[buf_index] = 0.89286 * (float)ch[buf_index] + 585.7;
        for(buf_index = 12; buf_index > 0; buf_index--) ch[buf_index] = ch[buf_index-1];
      //}
    }
    
    else buf_index++;
  }
}
void get_ACK6200(void){
  // get acceleration
    acc_x = icm20600.getAccelerationX();
    acc_y = icm20600.getAccelerationY();
    acc_z = icm20600.getAccelerationZ();

    q = icm20600.getGyroscopeX();   
    p = icm20600.getGyroscopeY();
    r = icm20600.getGyroscopeZ();

    ak09918.getData(&x, &y, &z);
    mx = x ;
    my = y ;
    mz = z ;
    /*
    Serial.print("M:  ");
    Serial.print(x);
    Serial.print(",  ");
    Serial.print(y);
    Serial.print(",  ");
    Serial.print(z);
    Serial.println(" uT");
    */

    // roll/pitch in radian
    roll1 = -atan2((float)acc_y, (float)acc_z)*57.3;
    pitch1 = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z))* 57.3;
   // pitch1 = pitch;
    //pitch2 = pitch2-pitch1;
   // if(pitch2>20)pitch=(pitch+pitc1)/2;

    
    


    //double Xheading = x * cos(pitch) + y * sin(roll) * sin(pitch) + z * cos(roll) * sin(pitch);
    //double Yheading = y * cos(roll) - z * sin(pitch);


     //yaw = 180 + 57.3 * atan2(Yheading, Xheading) + declination_shenzhen;
     //yaw1+=r*control_sample_time-0.048*0.04;
    //yawrate = (yaw-yaw1)/0.03;

      /*if (yaw < -179)
    yaw += 360;
  else if (yaw > 179)
    yaw1 -= 360;
   yaw1 = yaw;*/

        
  mx = (mx - hard_offset_x) * temp_float / field_x;
  my = -(my - hard_offset_y) * temp_float / field_y;
  mz = -(mz - hard_offset_z) * temp_float / field_z;
  madgwick.update(q, p, r, acc_x, acc_y, acc_z, mx, my, mz);
  roll  = -madgwick.getRoll();
  pitch = madgwick.getPitch();
  yaw   = -madgwick.getYaw()+360;
/*
    Serial.print(roll);
    Serial.print(",  ");
    Serial.print(pitch);
    Serial.print(",  ");
    Serial.println(yaw);
    */


     if(yaw2<-300)yaw1 =yaw1 -360;
     yawrate = (yaw-yaw1)/0.04;
     yaw2 = yaw-yaw1;
     yaw1 =yaw;
     if(yawrate>100)yawrate=20;

}

int Control(void){
  unsigned int i;
  //int radius_temp;

  if(sbus_frame_loss == 0){ //緊急時、墜落させる
    ch[1] = 800; //ch_default[1];
    ch[2] = 800;
    ch[3] = 800; //ch_default[3];
    ch[4] = 800; //ch_default[4];
    ch[5] = 800; //ch_default[5];
    ch[6] = 800; //ch_default[6];
    ch[7] = 800;
    
    ail = (float)(ch[1] - ch_default[1]) / 600.0 * 25.0; //[deg]
    ele = (float)(ch[2] - ch_default[2]) / 600.0 * 40.0;
    sff = (float)(ch[4] - ch_default[4]) / 600.0 * 32.5;
    rud = (float)(ch[5] - ch_default[5]) / 600.0 * 32.5;
    ail = (float)(ch[7] - ch_default[7]) / 600.0 * 25.0;
    // flap = (float)(ch[7] - ch_default[7]) / 600.0 * 30.0;
    
    for(i = 0; i < 7; i++){
      servo[i] = (float)ch[i+1] * 1024.0 / 20000.0;
      if(servo[i] > 108) servo[i] = 108;
      if(servo[i] < 51) servo[i] = 51;
      ledcWrite(CHANNEL[i], servo[i]);
    }
    
    return -1;
  }
  
  if(ch[9] >= 1400){ //ロール自動
    
    if(flag_roll == 0){
      roll_integral = 0.0;
      for(i = 1; i <= 7; i++) ch_default[i] = ch[i];
      flag_roll++;
    }
    
    yawrate = -((r * cos(roll / 180.0 * pi) + q * sin(roll / 180.0 * pi)) / cos(pitch / 180.0 * pi)) / pi * 180.0;
    ut = -0.1 * yawrate / 180.0 * pi;

    // ch[3] = (1.0 - ut) * (float)(ch[3] - ch_default[3]) - 0.5 * (float)(ch[1] - ch_default[1]) + ch_default[3]; //右推力
    // ch[6] = (1.0 + ut) * (float)(ch[6] - ch_default[6]) + 0.5 * (float)(ch[1] - ch_default[1]) + ch_default[6]; //左推力
    
    roll_integral = roll_integral + (roll - roll_ref) * control_sample_time;
    ch[1] = -(kp_ail * (roll - roll_ref) + kd_ail * q + ki_ail * roll_integral) + ch_default[1];
    ch[7] = -(kp_ail * (roll - roll_ref) + kd_ail * q + ki_ail * roll_integral) + ch_default[7];
        
    // ch[7] = ch_default[7] + 500; //フラップ
    
    if(ch[8] >= 1400){ //ピッチ自動
      
      if(flag_pitch == 0){
        pitch_integral = 0.0;
        ch_correction[2] = ch[2] - ch_default[2];
        flag_pitch++;
        count_pitch = 0;
      }
      
      if(count_pitch == 0){
        pitch_integral = pitch_integral + (pitch - pitch_ref) * control_sample_time;
        ch_temp[2] = -kp_ele * (pitch - pitch_ref) - kd_ele * p - ki_ele * pitch_integral + ch_correction[2] + ch_default[2];
      }
      ch[2] = ch_temp[2];
      count_pitch++;
      if(count_pitch >= 1) count_pitch = 0;
    }
    
    else flag_pitch = 0;
  }
  
  else flag_roll = 0;
  
  ail = (float)(ch[1] - ch_default[1]) / 600.0 * 25.0; //[deg]
  ele = (float)(ch[2] - ch_default[2]) / 600.0 * 40.0;
  sff = (float)(ch[4] - ch_default[4]) / 600.0 * 32.5;
  rud = (float)(ch[5] - ch_default[5]) / 600.0 * 32.5;
  ail = (float)(ch[7] - ch_default[7]) / 600.0 * 25.0;
//   flap = (float)(ch[7] - ch_default[7]) / 600.0 * 30.0;

  if(ch[1] > 2300) ch[1] = 2300;
  if(ch[1] < 1000) ch[1] = 1000;
  if(ch[2] > 2200) ch[2] = 2200;
  if(ch[2] < 900) ch[2] = 900;
  if(ch[3] > 2100) ch[3] = 2100;
  if(ch[4] > 2200) ch[4] = 2200;
  if(ch[4] < 900) ch[4] = 900;
  if(ch[5] > 2200) ch[5] = 2200;
  if(ch[5] < 1000) ch[5] = 1000;
  if(ch[6] > 2100) ch[6] = 2100;
  if(ch[7] > 2300) ch[7] = 2300;
  if(ch[7] < 1000) ch[7] = 1000;
  
  for(i = 0; i < 7; i++){
    servo[i] = (float)ch[i+1] * 1024.0 / 20000.0;
    if(servo[i] > 108) servo[i] = 108;
    if(servo[i] < 51) servo[i] = 51;
    ledcWrite(CHANNEL[i], servo[i]);
  }

  return 0;
}


void SerialPrint(void){
  Serial.print(time1);
  Serial.print(" , ");
  Serial.print(roll);
  Serial.print(" , ");
  Serial.print(roll1);
  Serial.print(" , ");
  Serial.print(pitch);
  Serial.print(" , ");
  Serial.print(pitch1);
  Serial.print(" , ");
  Serial.print(yaw);
  Serial.print(" , ");
  Serial.print(yaw1);
  Serial.print(" , ");
 /* Serial.print(roll_magnetism);
  Serial.print(" , ");
  Serial.print(pitch_magnetism);
  Serial.print(" , ");
  Serial.print(yaw_magnetism);
  Serial.print(" , ");
  Serial.print(mx);
  Serial.print(" , ");
  Serial.print(my);
  Serial.print(" , ");
  Serial.print(mz);
  Serial.print(" , ");
  Serial.print(v_sen_x);
  Serial.print(" , ");
  Serial.print(v_sen_y);
  Serial.print(" , ");
  Serial.print(v_sen_z);
  Serial.print(" , ");
  Serial.print(v_mag);
  Serial.print(" , ");*/
  //Serial.print(temperatureOne);
  //Serial.print(" , ");
  //Serial.print(yawrate);
  //Serial.print(" , ");
  //Serial.print(pressureOne);  
  //Serial.print(" , ");
  //Serial.print(ch[3]);
  //Serial.print(" , ");
  //Serial.print(h1);
//  Serial.println();
}

void SDPrint(void){
  File dataFile;
  
  dataFile = SD.open("/flight.txt", FILE_APPEND);
  if(dataFile){
    dataFile.print(time1);
    dataFile.print(" , ");
    dataFile.print(acc_y);
    dataFile.print(" , ");
    dataFile.print(acc_x);
    dataFile.print(" , ");
    dataFile.print(acc_z);
    dataFile.print(" , ");
    dataFile.print(p);
    dataFile.print(" , ");
    dataFile.print(q);
    dataFile.print(" , ");
    dataFile.print(r);
    dataFile.print(" , ");
    dataFile.print(roll);
    dataFile.print(" , ");
    dataFile.print(pitch);
    dataFile.print(" , ");
    dataFile.print(yaw);
    dataFile.print(" , ");
    dataFile.print(roll_magnetism);
    dataFile.print(" , ");
    dataFile.print(pitch_magnetism);
    dataFile.print(" , ");
    dataFile.print(yaw_magnetism);
    dataFile.print(" , ");
    dataFile.print(v_mag);
    dataFile.print(" , ");
    dataFile.print(attack_angle);
    dataFile.print(" , ");
    dataFile.print(sideslip_angle);
    dataFile.print(" , ");
    dataFile.print(v_sen_x);
    dataFile.print(" , ");
    dataFile.print(v_sen_y);
    dataFile.print(" , ");
    dataFile.print(v_sen_z);
    dataFile.print(" , ");
    dataFile.print(mx);
    dataFile.print(" , ");
    dataFile.print(my);
    dataFile.print(" , ");
    dataFile.print(mz);
    dataFile.print(" , ");
    dataFile.print(yawrate);
    dataFile.print(" , ");
    dataFile.print(ut);
    dataFile.print(" , ");
    dataFile.print(radius);
    dataFile.print(" , ");
    dataFile.print(ail);
    dataFile.print(" , ");
    dataFile.print(ele);
    dataFile.print(" , ");
    dataFile.print(rud);
    dataFile.print(" , ");
    dataFile.print(sff);
    dataFile.print(" , ");
    dataFile.print(ch[3]);
    dataFile.print(" , ");
    dataFile.print(ch[6]);
    dataFile.print(" , ");
    dataFile.print(ch[9]);
    dataFile.print(" , ");
    dataFile.print(ch[8]);
    dataFile.print(" , ");
    dataFile.print(ch[10]);
    dataFile.print(" , ");
    dataFile.print(ch[12]);
    dataFile.print(" , ");
    dataFile.print(error_sbus_end);
    dataFile.print(" , ");
    dataFile.print(sbus_frame_loss);
    dataFile.print(" , ");
    dataFile.print(sbus_failsafe);
    dataFile.print(" , "); 
    dataFile.print(ch[8]);
    dataFile.print(" , ");  
    dataFile.print(x2);
    dataFile.print(" , ");
    dataFile.print(y2);
    dataFile.print(" , ");
    dataFile.print(z2);
    /*dataFile.print(" , ");
    dataFile.print(latitude);
    dataFile.print(" , ");
    dataFile.print(longitude);
    dataFile.print(" , ");
    dataFile.print(pitch_def);*/
    dataFile.println();
    dataFile.close();
  }
}

void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    ak09918.getData(&x, &y, &z);

    value_x_min = x;
    value_x_max = x;
    value_y_min = y;
    value_y_max = y;
    value_z_min = z;
    value_z_max = z;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        ak09918.getData(&x, &y, &z);

        /* Update x-Axis max/min value */
        if (value_x_min > x) {
            value_x_min = x;
        } else if (value_x_max < x) {
            value_x_max = x;
        }

        /* Update y-Axis max/min value */
        if (value_y_min > y) {
            value_y_min = y;
        } else if (value_y_max < y) {
            value_y_max = y;
        }

        /* Update z-Axis max/min value */
        if (value_z_min > z) {
            value_z_min = z;
        } else if (value_z_max < z) {
            value_z_max = z;
        }

        Serial.print(".");
        delay(100);

    }
    hard_offset_x = value_x_min + (value_x_max - value_x_min) / 2;
    hard_offset_y = value_y_min + (value_y_max - value_y_min) / 2;
    hard_offset_z = value_z_min + (value_z_max - value_z_min) / 2;

     field_x = (value_x_max - value_x_min) / 2.0;
     field_y = (value_y_max - value_y_min) / 2.0;
     field_z = (value_z_max - value_z_min) / 2.0;
     temp_float = (field_x + field_y + field_z) / 3.0;
}


void setup(void) {
    unsigned int i;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(115200);
    Serial.println("1");
    // ros2_Initialize(ssid,pass,ip,port);
    madgwick.begin(25); //100Hz
    Wire.begin();

    temp_float = (field_x + field_y + field_z) / 3.0;
    Serial.println("2");

    
    err = ak09918.initialize();
    icm20600.initialize();
    Serial.println("3");
    ak09918.switchMode(AK09918_POWER_DOWN);
    Serial.println("4");
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  Serial.println("5");
    err = ak09918.isDataReady();
    while (err != AK09918_ERR_OK) {
        Serial.println("Waiting Sensor");
        delay(100);
        err = ak09918.isDataReady();
    }

    Serial.println("Start figure-8 calibration after 2 seconds.");
    delay(2000);
    calibrate(10000, &offset_x, &offset_y, &offset_z);
    Serial.println("");

    File dataFile;
    
    SD.begin(5, SPI, 24000000, "/sd");

    if(!SD.exists("/gain.txt")){
    dataFile = SD.open("/gain.txt", FILE_APPEND);
    if(dataFile){
      dataFile.println("ail ,,, ele ,,, rud_right ,,, rud_left ,,, roll_ref , pitch_ref , radius_ref");
      dataFile.println("kp , kd , ki , kp , kd , ki , kp , kd , ki , kp , kd , ki");
      dataFile.close();
      }
  }
  dataFile = SD.open("/gain.txt", FILE_APPEND);
  if(dataFile){
    dataFile.print(kp_ail);
    dataFile.print(" , ");
    dataFile.print(kd_ail);
    dataFile.print(" , ");
    dataFile.print(ki_ail);
    dataFile.print(" , ");
    dataFile.print(kp_ele);
    dataFile.print(" , ");
    dataFile.print(kd_ele);
    dataFile.print(" , ");
    dataFile.print(ki_ele);
    dataFile.print(" , ");
    dataFile.print(kp_rud_right);
    dataFile.print(" , ");
    dataFile.print(kd_rud_right);
    dataFile.print(" , ");
    dataFile.print(ki_rud_right);
    dataFile.print(" , ");
    dataFile.print(kp_rud_left);
    dataFile.print(" , ");
    dataFile.print(kd_rud_left);
    dataFile.print(" , ");
    dataFile.print(ki_rud_left);
    dataFile.print(" , ");
    dataFile.print(roll_ref);
    dataFile.print(" , ");
    dataFile.print(pitch_ref);
    dataFile.print(" , ");
    dataFile.print(radius_ref);
    dataFile.println();
    dataFile.close();
  }
  
  if(!SD.exists("/flight.txt")){
    dataFile = SD.open("/flight.txt", FILE_APPEND);
    if(dataFile){
      dataFile.print("time");
      dataFile.print(" , ");
      dataFile.print("ax , ay , az , p , q , r");
      dataFile.print(" , ");
      dataFile.print("roll , pitch , yaw");
      dataFile.print(" , ");
      dataFile.print("roll_magnetism , pitch_magnetism , yaw_magnetism");
      dataFile.print(" , ");
      dataFile.print("v_mag , attack , sideslip, v_sen_x, v_sen_y, v_sen_z");
      dataFile.print(" , ");
      dataFile.print("x , y , z ");
      dataFile.print(" , ");
      dataFile.print("yawrate , ut , radius");
      dataFile.print(" , ");
      dataFile.print("ail , ele , rad , sff , ThrustRight(ch[3]) , ThrustLeft(ch[6])");
      dataFile.print(" , ");
      dataFile.print("AutoRoll(ch[9]) , AutoPitch(ch[8]) , AutoRadius(ch[10]) , RightOrLeft(ch[12])");
      dataFile.print(" , ");
      dataFile.print("error sbus end , sbus frame loss , sbus failsafe");
      dataFile.print(" , null , null , null , ");
      dataFile.println();
      dataFile.close();
    }
  }
  SbusSerial2.begin(100000, SERIAL_8E2);
  
  IN[1-1] = 17;
  IN[2-1] = 33;
  IN[3-1] = 27;
  IN[4-1] = 25;
  IN[5-1] = 14;
  IN[6-1] = 26;
  IN[7-1] = 13;
  
  for(i = 0; i < 7; i++){
    CHANNEL[i] = (uint8_t)i;
    ledcSetup(CHANNEL[i], 50.0, 10);
    ledcAttachPin(IN[i], CHANNEL[i]);
  }

}

bool sd_flag = false;


void loop(void) {
    SBUS();
    //Serial.println("1");
    for(int i = 0; i < 16; i++){
      Serial.print(ch[i]);
      Serial.print("---");
    }
    Serial.print("\n");
//    SerialPrint();
    h1 = -z2;
    get_ACK6200();
    //Serial.println("ACK6200_OK!");
    Control();
    //Serial.println("Control_OK!");
    
//    SerialPrint();
    //if(ch[9]>1400){
        // SDPrint();
    // } //ロール自動時にmicroSDカードに書き込み

    if(ch[11] > 2000){
      sd_flag = true;
    }
    if(sd_flag){
     SDPrint(); 
    }

    while(millis() < time1);
    time1 = (unsigned long)(millis() + 40);
    // Serial.println("Loop_OK!");
}
