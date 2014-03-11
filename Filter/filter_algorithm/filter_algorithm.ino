#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "TimerOne.h"

MPU6050 Gyro6050;

#define Acc 0x1D
#define Gyr 0x69
#define Mag 0x1E
#define Gry_offset 0    // 陀螺仪偏移量
#define Gyr_Gain 0.00763     // 满量程250dps时灵敏度(dps/digital)
#define pi 3.14159

float Com_angle;
float y1, Com2_angle;
float Klm_angle;

#define Q_angle 0.01      // 角度数据置信度
#define Q_omega 0.0003    // 角速度数据置信度
#define R_angle 0.01      // 方差噪声
float bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float angleG = 0;
long timer = 0;  // 采样时间
float dt=0.05;                 //0.01=10ms, dt為kalman濾波器採樣時間;
long task_dt = 50000;             //10=10ms, task_dt為主循環執行時間

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int blink_led = 0;
int timer1_sw = 0;

void Timer1_callback(void)
{
  blink_led = !blink_led;
  digitalWrite(led, blink_led);
  timer1_sw = 1;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  //Initialize serial and wait for port to open:
  Serial.begin(57600);
  // initialize device
  Serial.println("Initializing MPU6050 with I2C ...");
  // FS_SEL = MPU6050_GYRO_FS_250 0x0;
  // AFS_SEL = MPU6050_ACCEL_FS_2 0x0;
  Gyro6050.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(Gyro6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Initializing car's parameter ...");
  Serial.println("System Ready!");
  Serial.println("t,Acc,1-step-comp,2-step-comp,Kalmam;");
  Timer1.initialize(task_dt); //1s
  Timer1.attachInterrupt(Timer1_callback);  // attaches callback() as a timer overflow interrupt
}

void loop() {
  
  if (timer1_sw)
  {
    timer = millis();                       // 当前时间(ms)
    float X_Accelerometer = Gyro6050.getAccelerationX();   // 获取向前的加速度
    float Z_Accelerometer = Gyro6050.getAccelerationZ();;   // 获取向下的加速度
    float angleA = atan(X_Accelerometer / Z_Accelerometer) * 180 / pi;
                                            // 根据加速度分量得到的角度(degree)
    float omega =  Gyr_Gain * (Gyro6050.getRotationY() +  Gry_offset);
    //angleG = angleG + omega * dt;           // 对角速度积分得到的角度(degree)
// 一阶互补算法
    float K;
    //K = 0.075;                              // 对加速度计取值的权重
    K = 0.08;                              // 对加速度计取值的权重
    float A = K / (K + dt);
    Com_angle = A * (Com_angle + omega * dt) + (1-A) * angleA;
// 二阶互补算法
    K = 0.5;
    float x1 = (angleA - Com2_angle) * K * K;
    y1 = y1 + x1 * dt;
    float x2 = y1 + 2 * K *(angleA - Com2_angle) + omega;
    Com2_angle = Com2_angle + x2 * dt;
// 卡尔曼滤波
    Klm_angle += (omega - bias) * dt;          // 先验估计
    P_00 += -(P_10 + P_01) * dt + Q_angle *dt;
    P_01 += -P_11 * dt;
    P_10 += -P_11 * dt;
    P_11 += +Q_omega * dt;                     // 先验估计误差协方差
    
    float K_0 = P_00 / (P_00 + R_angle);
    float K_1 = P_10 / (P_00 + R_angle);
    
    bias += K_1 * (angleA - Klm_angle);
    Klm_angle += K_0 * (angleA - Klm_angle);   // 后验估计
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;                        // 后验估计误差协方差

    Serial.print(timer);
    Serial.print(",");
    Serial.print(angleA, 3);
    Serial.print(",");
    //Serial.print(angleG, 3);
    //Serial.print(",");
    Serial.print(Com_angle, 3);
    Serial.print(",");
    Serial.print(Com2_angle, 3);
    Serial.print(",");
    Serial.print(Klm_angle, 3);
    Serial.println(";");      
    timer1_sw= 0;
  }
}

