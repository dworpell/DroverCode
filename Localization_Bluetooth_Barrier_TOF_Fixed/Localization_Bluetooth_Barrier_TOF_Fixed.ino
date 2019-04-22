#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

#include "defines.h"
#include "drive_commands.h"
#include "MPU6050.h"
#include "MovingAverageFilter.h"

#define PI 3.1416
#define D2R PI/180.0

#define LONG_RANGE
//#define HIGH_ACCURACY
#define BARRIER_DETECTED 1
#define EDGE_DETECTED 2
#define UNDEFINED 0
#define INITIAL 0
#define OUTERPERIM 1
#define SWEEP 2
#define BARRIERCROSS 3

MPU6050 mpu;
VL53L0X tof_width;
VL53L0X tof_length;

Servo servo_horn;
Servo fan_control;

MovingAverageFilter tof_width_filter(1);
MovingAverageFilter tof_length_filter(1);
MovingAverageFilter yaw_filter(1);

MovingAverageFilter gyroX_filter(5);

int servo_horn_pin = 10;

int fan_esc_pwm = 9;
int s = 150;

int motor_left_pwm = 6;
int motor_left_in_1 = 7;
int motor_left_in_2 = 8;

int motor_right_pwm = 5;
int motor_right_in_1 = 12;
int motor_right_in_2 = 11;

int barrier_flag = 0;
int edge_flag = 0;
uint8_t TOFCounter=0;

int fanspeed = 50;

// Timers
unsigned long timer = 0, timer_d = 0;
float timeStep = 0.016;//0.00576;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
volatile float yaw = 0;
float d2r = PI/180.0;

int num_iter = 10000;

int servo_init = 150;
int servo_final = 15;

int move_speed = 0;
int fixed_speed = 170;

volatile byte timerRoll=0;
volatile byte I2C_Lock=0;
volatile byte rollCounter=0;
float rad_to_deg = 180/3.141592654;
volatile byte path_state;
byte top_flag=0;
byte bottom_flag=0;
byte left_flag=0;
byte right_flag=0;
void setup() 
{
  Serial.begin(115200);
  Serial.println("Q");
  fan_control.attach(fan_esc_pwm);
  fan_control.writeMicroseconds(1000);
  delay(3000);
  path_state = INITIAL;
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  while (mpu.getClockSource()!=MPU6050_CLOCK_PLL_XGYRO){mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);}
  while (mpu.getScale()==0){mpu.setScale(3);}
  while (mpu.getRange()!= MPU6050_RANGE_2G) {mpu.setRange(MPU6050_RANGE_2G);}
  mpu.setDLPFMode(MPU6050_DLPF_6);

  delay(500);
  pinMode(A3,OUTPUT);
  digitalWrite(A3,LOW);
  Serial.println("d");
  tof_width.init();
  tof_width.setAddress(0x30);
  tof_width.setTimeout(500);

  digitalWrite(A3,HIGH);
  Serial.println("o");
  tof_length.init();
  tof_length.setAddress(0x31);
  tof_length.setTimeout(500);
  delay(100);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro(100);
  delay(1000);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);
  Serial.println("o");

  pinMode(motor_left_pwm,OUTPUT);
  pinMode(motor_left_in_1,OUTPUT);
  pinMode(motor_left_in_2,OUTPUT);
  
  pinMode(motor_right_pwm,OUTPUT);
  pinMode(motor_right_in_1,OUTPUT); 
  pinMode(motor_right_in_2,OUTPUT);

  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);

  pinMode(10, INPUT); //PCINT5
  pinMode(2, INPUT); //INT0
  pinMode(3, INPUT); //INT1
  pinMode(4, INPUT); //PCINT20
  
  Encoder0B=digitalRead(2);
  Encoder0A=digitalRead(3);
  Encoder1A=digitalRead(10);
  Encoder1B=digitalRead(4);
  
  /***Configure INT0 and INT1 Pins***/
  //*EIFR=0x03;  //Clear the INT0 and INT1 digitFlags
  //*EICRA=0x05; //Set INT0 and INT1 to trigger on any edge
  //*EIMSK=0x03; //Enable INT0 and INT1 interrupt vectors
  
  
  /***Configure PCINT pins***/
  //*PCIFR=0x07;  //Clear all PCINT flags
  *PCMSK2=0x00; //Enable PCINT20 Pin
  *PCMSK1=0x00; //Ensure all PCINT1 pins are disabled
  *PCMSK0=0x00; //Enable PCINT2 pin
  //*PCICR=0x05; //Enable PCINT2 and PCINT0 banks

  servo_horn.attach(servo_horn_pin);
  servo_horn.write(servo_init);
  
  #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    tof_width.setSignalRateLimit(0.17);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tof_width.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    tof_width.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);

    tof_length.setSignalRateLimit(0.25);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tof_length.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    tof_length.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  #endif
  //Initalize everything to zero
  tof_width.setMeasurementTimingBudget(37000);
  Serial.println("g");
  timerRoll=0;
  I2C_Lock=0;
  rollCounter=0;
  yaw=0;
  roll=0;
  pitch=0;
  *TCNT2 =0x00; //Set Timer Counter to 0 for safety
  *TIFR2=0xff; //Clear interrupt flags for safety
  *TCCR2A=0x02; //Set to Clear on Timer Compare
  *TCCR2B=0x00;
  *OCR2A=249;   
  *TIMSK2 = 0x02; // Enables Interrupts on OCRA = TCNT2
  *TCCR2B = 0x07; // Starts the timer
  
  tof_width.startContinuous();
  tof_length.startContinuous();
  //delay(500);
  //fan_control.write(50);
  //delay(5000);
  TOFCounter=0;
}


char a;
int flag = 0;
volatile uint16_t y_t=0;
Vector norm, accel;

float x_base = 0.0, y_base = 0.0, theta = 0.0, d_prev_l = 0.0, d_prev_r = 0.0;
float d_l, d_r, del_l, del_r, heading, rad_wheel = 2/2.54, width = 15/2.54;
float x_g, y_g, x_new = 400,y_new = 250, yaw_goal = 0, yaw_error_prev = 0.0;
float alpha = 0.1, yaw3 = 0.0;
float dlpf_freq=3;
volatile float filtered_accelZ=0; float dlpf_alpha = 2*PI*dlpf_freq*timeStep/(2*PI*timeStep*dlpf_freq + 1); float dlpf_alpha_m=1-dlpf_alpha;
volatile float filtered_accelY=0;
volatile float filtered_accelX=0;

/***************************************************************************/
byte current_direction =0;
/*void getDirection()
{
  if (filtered_accelZ > 2000)
    return NORTH;
  else if (filtered_accelZ<2000)
}*/

//########################################################################//
/********STEPS TO CALIBRATE**********************/
//1.) Set the define statements below to be 0 (e.g. #define XAccel_Offset 0)
//2.) Upload this code.
//3.) Put Moolander 2: Electric Boogaloo on the window
//4.) Align the side of Moolander 2 with right angles relative to the window frame
//5.) Look at what the range of acceleromerter values is and guess the average value of that (Values printed are in order X Y Z roll)
//6.) Repeat for the other 2 Axis
//7.) The offset is the difference between (+ or -) 16384 and the value you noted down. 
//8.) Replace the define statements below with the appropriate values.
//########################################################################//
#define XAccel_Offset -1920
#define YAccel_Offset 484
#define ZAccel_Offset 4264

ISR (TIMER2_COMPA_vect)
{
  sei();
  if (I2C_Lock==0)
  {
    //should just be hardset to 0.016 s (with a small difference in millis)
    float del_t = timeStep;
    float e_t = timeStep*rollCounter + timeStep;
    //Gyro value gets filtered by MPU as well as accel values. This is the way the MPU works, so we would need to turn off DLPF for both
    norm = mpu.readNormalizeGyro();
    //Roughly 16 cycles to read from MPU for a single vector.
    accel = mpu.readRawAccel();
    accel.XAxis +=  XAccel_Offset;
    accel.YAxis +=  YAccel_Offset;
    accel.ZAxis +=  ZAccel_Offset;

    filtered_accelZ = accel.ZAxis*dlpf_alpha + filtered_accelZ*dlpf_alpha_m;
    filtered_accelY = accel.YAxis*dlpf_alpha + filtered_accelY*dlpf_alpha_m;
    filtered_accelX = accel.XAxis*dlpf_alpha + filtered_accelX*dlpf_alpha_m;

         
    // Calculate Pitch, Roll and Yaw
     float accelX= atan2((filtered_accelY/16384.0),sqrt(pow((filtered_accelX/16384.0),2) + pow((filtered_accelZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     float accelY= atan2(-1*(filtered_accelX/16384.0),sqrt(pow((filtered_accelY/16384.0),2) + pow((filtered_accelZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
    //Serial.println(accel.YAxis);
    float alpha=0.1;
    if (!isnan(accelX))
    pitch = (1-alpha)*(pitch + norm.YAxis * timeStep)+(alpha*accelY);
    if (!isnan(accelY))
    roll =  (1-alpha)*(roll + norm.XAxis * timeStep)+(alpha*accelX);
    //Serial.print(norm.XAxis);
    //Serial.print(" ");
    //Serial.print(filtered_accelX);
    //Serial.print(" ");
    /*Serial.print(filtered_accelY);
    Serial.print(" ");
    Serial.print(filtered_accelZ);
    Serial.print(" ");
    Serial.println(roll);*/
    timerRoll=1;
    rollCounter=0;
  }
  else
  {
    rollCounter++;
  }
  TOFCounter++;
  //must have 8 timer ticks in order to avoid I2C conflict
  if (TOFCounter==6 && TOFCounter>=4)
  {
    //Serial.println("Yeet");
    y_t = tof_length.readRangeContinuousMillimeters();
    Serial.println(y_t);
    TOFCounter=0;
  }
}
void turn(int dir)
{
  yaw_goal = yaw_goal+dir;
}
void waitForNextTimer()
{
  timerRoll=0;
  while (timerRoll==0){}

}
/**********************************************************/
void face(){
  float yaw_error = yaw_goal - yaw;
  float kp = 15;                                //, kd = 2;

  float control = kp * yaw_error;               // + kd * (yaw_error - yaw_error_prev) + 10;

  set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed - control);
  set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, move_speed + control);
}
byte startflag=0;
//volatile int16_t prev_filteredX=0;
//volatile int16_t prev_filteredY=0;
//volatile int16_t prev_filteredZ=0;
void loop()
{ 
  //Serial.println("1");
  fan_control.writeMicroseconds(2000);
  delay(5000);
  //forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
  

  //ENU frame +90deg x_g to x_base
  //If power is lost to the TOF sesnor. All the arduino bricks.
  //TOF sensors work in the way I expected them to. Continous will instantly read given
  //The time-alotted period has passed. The Timing budget of 37ms therefore means that after
  //37ms since the previous read, the timer will "instantly" read the next call.
  //delay(10000);
  int control=0;
  if (path_state==INITIAL)
  {
    //Serial.println("Test");
    while (fabs(roll)>6 || filtered_accelZ<0)
    {
      fixed_speed=110;
      control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      //Serial.println(y_t);
    }
    //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    //Waitfornext timer, read TOF Width and decide which quadrant you are in.
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    } 
    //Serial.println("l");
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(300);
    int horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
      top_flag = BARRIER_DETECTED;
    else
      top_flag = EDGE_DETECTED;   
    servo_horn.write(servo_init);
    path_state=OUTERPERIM;
  }
  if (path_state == OUTERPERIM)
  {
    while (fabs(roll-90)>8)
    {
      fixed_speed=-170;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    } 
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(300);
    int horn_limit_switch = digitalRead(A2);
    Serial.println(horn_limit_switch);
    if(horn_limit_switch == LOW)
      left_flag = BARRIER_DETECTED;
    else
      left_flag = EDGE_DETECTED;
    servo_horn.write(servo_init);
    //It gets this far
    Serial.println("F Down");
    while (y_t <200)
    {
      fixed_speed=-170;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (fabs(roll)>8)
    {
      
      fixed_speed=110;
      control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    Serial.println("Done Turning");
    //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    //delay(2000);
    Serial.println("Moving");
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=110;
      control =face_down(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    Serial.println("Hit bottom");
    servo_horn.write(servo_final);
    delay(300);
    horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
      bottom_flag = BARRIER_DETECTED;
    else
      bottom_flag = EDGE_DETECTED;
    servo_horn.write(servo_init);
    while (y_t <200)
    {
      fixed_speed=-170;
      control =face_down(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    Serial.println("Turning Right");
    while (fabs(roll+90)>5)
    {
      fixed_speed=110;
      control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    Serial.println("Yurned Right");
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_right(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    servo_horn.write(servo_final);
    delay(300);
    horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
      right_flag = BARRIER_DETECTED;
    else
      right_flag = EDGE_DETECTED;
    servo_horn.write(servo_init);
    Serial.println("yeet");

    
    while (fabs(roll)>5 || filtered_accelZ<0)
    {
      fixed_speed=-170;
      control =face_up(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_up(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
  }

  path_state = BARRIERCROSS;

  if (path_state == BARRIERCROSS)
  {
    if(top_flag == BARRIER_DETECTED)
    {
      //Face UP First
      while (fabs(roll)>6 || filtered_accelZ<0)
      {
        fixed_speed=110;
        control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
        //Serial.println(y_t);
      }

      //Move up to barrier
      while (!digitalRead(A0) && !digitalRead(A1))
      {
        fixed_speed=170;
        control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      } 

      //Cross
      for(int i = 0; i<1500; i++)
        forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
    }
    
    else if(bottom_flag == BARRIER_DETECTED)
    {
      //Face Down First
      while (fabs(roll)>8)
      {
        fixed_speed=110;
        control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Move down to the barrier
      while (!digitalRead(A0) && !digitalRead(A1))
      {
        fixed_speed=110;
        control =face_down(filtered_accelY, filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Cross
      for(int i = 0; i<1500; i++)
        forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
    }
    
    else if(left_flag == BARRIER_DETECTED)
    {
      //Face Left
      while (fabs(roll-90)>8)
      {
        fixed_speed=-170;
        control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Move left to barrier
      while (!digitalRead(A0) && !digitalRead(A1))
      {
        fixed_speed=170;
        control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Cross
      for(int i = 0; i<1500; i++)
        forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
    }
    
    else if(right_flag == BARRIER_DETECTED)
    {
      //Face Right
      while (fabs(roll+90)>5)
      {
        fixed_speed=110;
        control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Move right to barrier
      while (!digitalRead(A0) && !digitalRead(A1))
      {
        fixed_speed=170;
        control =face_right(filtered_accelY, filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }

      //Cross
      for(int i = 0; i<1500; i++)
        forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
    }
    
  }



  if(path_state == SWEEP){
    //Subroutine for Cleaning
    
  }

 
  
  /*while (y_t>200)
  {
    move_speed=150;
    face();
    //Serial.println(y_t);
    delay(20);
  }
  turn(RIGHT);else
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face();
    delay(20);
  }
  
  while (y_t>200)
  {
    move_speed=150;
    face();
    //Serial.println(y_t);
    delay(20);
  }
  turn(RIGHT);
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face();
    delay(20);
  }
  while (y_t>200)
  {
    move_speed=150;
    face();
    //Serial.println(y_t);
    delay(20);
  }
  turn(RIGHT);
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face();
    delay(20);
  }
  while (y_t>200)
  {
    move_speed=150;
    face();
    //Serial.println(y_t);
    delay(20);
  }*/
  //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
  //delay(7000);
  /*
  if(fabs(roll) < 4)
    up_flag=1;
    fixed_speed = 0;
  else
    fixed_speed = 170;
  
  int control = face_left(accel.YAxis,accel.ZAxis,roll);
  set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
  set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
  Serial.println(control);
  */
  
  
  //delay(4000);
  
  /*while (fabs(roll-90)>4)
  {
    fixed_speed=170;
    control = face_left(accel.YAxis,accel.ZAxis,roll);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
  }
  
  while (y_t < 200)
  {
    fixed_speed=170;
    control =face_left(accel.YAxis,accel.ZAxis,roll);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
  }*/
  for(int i=0; i<1; i++) {
  //ZAxis is positive when robot faces up.
    /*while (fabs(roll)>3 || accel.ZAxis<0)
    {
      fixed_speed=110;
      control =face_up(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);
    }
    while (y_t > 200)
    {
      fixed_speed=170;
      control =face_up(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);  
    }
    
    while (fabs(roll+90)>4)
    {
      fixed_speed=170;
      control =face_right(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (y_t > 200)
    {
      fixed_speed=170;
      control =face_right(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }*/
    /*while (fabs(roll-90)>6)
    {
      fixed_speed=170;
      control =face_left(filtered_accelY,filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (y_t > 200)
    {
      fixed_speed=170;
      control =face_left(filtered_accelY,filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }*/
    /*
    while (fabs(roll)>4 || accel.ZAxis >0)
    {
      fixed_speed=170;
      control =face_down(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);
    }
    while (y_t > 200)
    {
      fixed_speed=170;
      control =face_down(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);
    }*/
    /*while (fabs(roll+90)>4)
    {
      fixed_speed=170;
      control =face_right(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    while (y_t < 200)
    {
      fixed_speed=170;
      control =face_right(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }*/
    
    
    
    /*
    while (fabs(roll)>3 || accel.ZAxis<0)
    {
      fixed_speed=110;
      control =face_up(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);
    }
    while (y_t > 200)
    {
      fixed_speed=170;
      control =face_up(accel.YAxis,accel.ZAxis,roll);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      Serial.println(y_t);  
    }*/
  }
  //delay(10000);
  brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
  delay(2000);
  while(1)
  {
    fan_control.writeMicroseconds(1000);
    Serial.println(y_t);
  }
  /*
  Serial.println("DOWN");
  delay(500);
    for (int i=0; i<500; i++)
  {
    face(DOWN); 
    delay(20);
  }
  Serial.println("LEFT");
  delay(500);
    for (int i=0; i<500; i++)
  {
    face(LEFT);
    delay(20);
  }*/
  /*
  if(flag == 0 && startflag==1)
    {
      //forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
      move_speed = fixed_speed;
      if (y_g<300)
      {
        brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
        flag = 1;
        startflag=0;
        yaw_goal = 90;
      }
    }
  
  if(flag == 1){

    move_speed = 0;
    
    if(fabs(yaw_goal - yaw) < 5.0)
      flag = 2;
  }

  
  if(flag == 2)
    {
      move_speed = fixed_speed;
      if (y_g<300)
      {
        brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
        flag = 3;
        startflag=0;
        move_speed = 0;
      }
    }
  
  if(flag == 3){
    move_speed = 0;
    
  }

  if(flag == 4){
    forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 170);
   if (y_g<300)
      {
        brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 170);
        flag = 5;
      }
  }
*/
  /* 
  float yaw_error = yaw_goal - yaw, kp = 10, kd = 2;

  float control = kp * yaw_error + kd * (yaw_error - yaw_error_prev) + 10;

  set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed + control);
  set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, move_speed - control);

  yaw_error_prev = yaw_error;
  timer++;

  if(timer > 20)
    move_speed = 0;
  */
  
  


   

  /*
  if(fabs(del_l - del_r)  < 0.01){
    x_base += del_l * cos(yaw2);
    y_base += del_r * sin(yaw2);
  }
    
  else{

    float R = width * (del_l + del_r) / (2 * (del_r - del_l)), wd = (del_r - del_l) / width; 

    x_base += R * sin(wd + yaw2) - R * sin(yaw2);
    y_base += R * cos(wd + yaw2) - R * cos(yaw2);
  }
  */

  
  

  //Serial.println("R");


  /*
  int right_limit_switch = digitalRead(A0);
  int left_limit_switch = digitalRead(A1);
  
  if(right_limit_switch == HIGH || left_limit_switch == HIGH){
    if(barrier_flag == 0 && edge_flag ==0)
      servo_horn.write(servo_final);
    else
      servo_horn.write(servo_init);
    
    delay(300);
    int horn_limit_switch = digitalRead(A2);
    Serial.print(" ");
    Serial.println(horn_limit_switch);
    if(horn_limit_switch == LOW){
      barrier_flag = 1;
      servo_horn.write(servo_init);
    }
    else{
      if(barrier_flag == 0)
        edge_flag = 1;
      servo_horn.write(servo_init);
    }
  }
  else if(right_limit_switch == HIGH && left_limit_switch == HIGH){
    servo_horn.write(servo_init); 
  }
  else
    servo_horn.write(servo_init);

  if(barrier_flag == 1){
      Serial.println("Barrier Detected");
      //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 180);
  }
  else if(edge_flag == 1){
      Serial.println("Edge Detected");
      //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 180);
  }
  else if(barrier_flag == 0 && edge_flag == 0){ 
      Serial.println("All clear to clean");
      //forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 180);
  }
  */
  /*
  if(x_g <= 5000){
  // Output raw
  Serial.print("X_a = ");
  Serial.print(x_g);
  Serial.print(" ");
  Serial.print("Y_a = ");
  Serial.println(y_g);
  //Serial.print(" ");
  Serial.print(" Z_a = ");
  Serial.println(yaw);
  }*/
  
  /*
  Serial.print("X_b = ");
  Serial.print(x_base*2);
  Serial.print(" ");
  Serial.print("Y_b = ");
  Serial.print(y_base*2);
  Serial.print(" ");  
  Serial.print(" Yaw = ");
  Serial.println(yaw,6);
  //Serial.print(" ");
  Serial.print("X_g = ");
  Serial.print(x_g);
  Serial.print(" ");
  Serial.print("Y_g = ");
  Serial.print(y_g);
  Serial.print(" ");
  Serial.print("a_x = ");
  Serial.print(accel.XAxis);
  Serial.print(" ");
  Serial.print("a_y = ");
  Serial.print(accel.YAxis);
  Serial.print(" ");
  Serial.print("a_z = ");
  Serial.println(accel.ZAxis);
  */
  // Wait to full timeStep period
  //waitForNextTimer();
  //delay(20);
  //delay((timeStep*1000) - (millis() - timer));
}
