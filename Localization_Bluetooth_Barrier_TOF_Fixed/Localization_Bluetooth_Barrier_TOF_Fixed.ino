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
#define DEBUG_CROSS 4
#define SECONDSIDE 5
#define SWARM_SIDE 6
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
volatile float roll = 0;
float d2r = PI/180.0;
volatile float alpha = 0.1;

int servo_init = 5;
int servo_final = 140;

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
volatile float sign_change=1;
void setup() 
{
  brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
  Serial.begin(115200);
  Serial.println("Q");
  fan_control.attach(fan_esc_pwm);
  fan_control.writeMicroseconds(1000);
  delay(3000);
  pinMode(13, OUTPUT);
  path_state = SWEEP;
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
  delay(400);
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
  roll=0;
 // byte f_1=0;
 // byte f_2=0;
 /* while (digitalRead(A2))
  {
    if (digitalRead(A0))
  }*/
  digitalWrite(13,LOW);
  while (!digitalRead(A0) && !digitalRead(A1))
  {
  }
  if (digitalRead(A0))
    path_state=SWEEP;
  else if (digitalRead(A1))
    path_state=SWARM_SIDE;
  digitalWrite(13,HIGH);
  delay(400);
  digitalWrite(13,LOW);
  delay(400);
  digitalWrite(13,HIGH);
  delay(400);
  digitalWrite(13,LOW);
  tof_width.startContinuous();
  tof_length.startContinuous();
  *TCNT2 =0x00; //Set Timer Counter to 0 for safety
  *TIFR2=0xff; //Clear interrupt flags for safety
  *TCCR2A=0x02; //Set to Clear on Timer Compare
  *TCCR2B=0x00;
  *OCR2A=249;   
  *TIMSK2 = 0x02; // Enables Interrupts on OCRA = TCNT2
  *TCCR2B = 0x07; // Starts the timer
  
  //delay(500);
  //fan_control.write(50);
  //delay(5000);
  TOFCounter=0;
  delay(500);
}


char a;
int flag = 0;
volatile uint16_t y_t=0;
Vector norm, accel;


float dlpf_freq=1;
float dlpf_freq2=5;
float dlpf_alpha2 = 2*PI*dlpf_freq2*timeStep/(2*PI*timeStep*dlpf_freq2 + 1); float dlpf_alpha_m2=1-dlpf_alpha2;
float dlpf_freq3=30;
float dlpf_alpha3 = 2*PI*dlpf_freq3*timeStep/(2*PI*timeStep*dlpf_freq3 + 1); float dlpf_alpha_m3=1-dlpf_alpha3;

volatile byte TOFCountX=0;
volatile float filtered_accelZ=0; float dlpf_alpha = 2*PI*dlpf_freq*timeStep/(2*PI*timeStep*dlpf_freq + 1); float dlpf_alpha_m=1-dlpf_alpha;
volatile float filtered_accelY=0;
volatile float filtered_accelX=0;
volatile float filtered_xt=0;
volatile uint16_t x_t=0;
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
/*#define XAccel_Offset -1738
#define YAccel_Offset 0
#define ZAccel_Offset -1253*/
//Yellow Boy below
#define XAccel_Offset -1920
#define YAccel_Offset 484
#define ZAccel_Offset 4264
volatile float filtered_gyro=0;
volatile uint16_t fanSpeed=1000;
volatile byte debounce =0;
volatile uint8_t fake_delay=0;
ISR (TIMER2_COMPA_vect)
{
  sei();
  if (I2C_Lock==0)
  {
    fake_delay++;
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
    filtered_gyro   = norm.XAxis*dlpf_alpha2 + filtered_gyro*dlpf_alpha_m2;
    
    if (fabs(filtered_accelX)>14500)
    {
      fan_control.writeMicroseconds(1000);
      debounce=0;
      fake_delay=0;
    }
    else if(debounce==0 && fake_delay>=100)
    {
      fan_control.writeMicroseconds(fanSpeed); 
      debounce=1;
    }
    // Calculate Pitch, Roll and Yaw
     float accelX= atan2((filtered_accelY/16384.0),sqrt(pow((filtered_accelX/16384.0),2) + pow((filtered_accelZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     //float accelY= atan2(-1*(filtered_accelX/16384.0),sqrt(pow((filtered_accelY/16384.0),2) + pow((filtered_accelZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
    //Serial.println(accel.YAxis);
    float gyro_angle_change = sign_change*filtered_gyro * timeStep;
    //Serial.print(accelX);
    //Serial.print("   ");
    //Serial.print(gyro_angle_change);
    //Serial.print("   ");
   // if (!isnan(accelX))
   // pitch = (1-alpha)*(pitch + norm.YAxis * timeStep)+(alpha*accelY);
    if (!isnan(accelX))
    {
      roll =  (1-alpha)*(roll + gyro_angle_change)+(alpha*accelX);
    }
    //Serial.println(roll);
    //Serial.print(norm.XAxis);
    //Serial.print(" ");
    //Serial.print(filtered_accelX);
    //Serial.print(" ");
    //Serial.print(filtered_accelY);
    //Serial.print(" ");
    //Serial.print(filtered_accelZ);
    //Serial.print(" ");
    //Serial.println(roll);
    timerRoll=1;
    rollCounter=0;
  }
  else
  {
    rollCounter++;
  }
  TOFCounter++;
  TOFCountX++;
  byte read_mpu_y=0;
  //must have 8 timer ticks in order to avoid I2C conflict
  if (TOFCounter==6 && TOFCounter>=4)
  {
    //Serial.println("Yeet");
    y_t = tof_length.readRangeContinuousMillimeters();
    TOFCounter=0;
    read_mpu_y=1;
  }
  if (TOFCountX==7 && TOFCountX>=4)
  {
    if (read_mpu_y==0)
    {
      x_t = tof_width.readRangeContinuousMillimeters();
      if (x_t <2000)
      filtered_xt = accel.XAxis*dlpf_alpha3 + filtered_xt*dlpf_alpha_m3;
    }
    TOFCountX=0;
    
  }
  //Serial.print(y_t);
  //Serial.print(" ");
  //Serial.println(x_t);
}
void waitForNextTimer()
{
  timerRoll=0;
  while (timerRoll==0){}

}


volatile int control=0;
byte startflag=0;
//volatile int16_t prev_filteredX=0;
//volatile int16_t prev_filteredY=0;
//volatile int16_t prev_filteredZ=0;
byte down_right=0;
byte down_left=0;
volatile byte path_done=0;
volatile byte second_side=0;
void loop()
{ 
  Serial.println("1");
  alpha=1;
  fanSpeed=2000;
  //fan_control.writeMicroseconds(2000);
  if (path_done==0 && second_side==0) {
    while (fabs(filtered_accelX)>3000)
    {}
    delay(5000);
  }
  //delay(5000);
  //forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 100);
  bottom_flag=BARRIER_DETECTED;

  //ENU frame +90deg x_g to x_base
  //If power is lost to the TOF sesnor. All the arduino bricks.
  //TOF sensors work in the way I expected them to. Continous will instantly read given
  //The time-alotted period has passed. The Timing budget of 37ms therefore means that after
  //37ms since the previous read, the timer will "instantly" read the next call.
  //delay(10000);
  if (path_state==INITIAL)
  {
    //Serial.println("Test");
    alpha=0;
    sign_change=1;
    while (fabs(roll)>6 || filtered_accelZ<0)
    {
      fixed_speed=110;
      control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      //Serial.println(y_t);
      //Serial.println(control);
      //pcontrol=control;
    }
    //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    //Waitfornext timer, read TOF Width and decide which quadrant you are in.
    alpha=1;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
     // pcontrol=control;
      //Serial.println(control);
    } 
    //Serial.println("l");
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(500);
    int horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
    {
      top_flag = BARRIER_DETECTED;
      digitalWrite(13,HIGH);
    }
    else
      top_flag = EDGE_DETECTED;   
    servo_horn.write(servo_init);
    path_state=OUTERPERIM;
  }
  //fan_control.writeMicroseconds(2000);
  if (path_state == OUTERPERIM)
  {
    alpha=1;
    left_kp=3.5;
    maxSpeed=170;
    /*while (y_t <200)
    {
      fixed_speed=-20;
      control =face_up(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }*/
    left_kp=3.5;
    alpha=0;
    maxSpeed=100;
    while (fabs(roll-90)>8)
    {
      fixed_speed=0;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      if (fabs(roll-90)<15)
        left_kp=6;
    }
    alpha=1;
    left_kp=4;
    maxSpeed=250;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      float bump_value =0;
      /*if (fabs(filtered_xt-100)>0)
      {
        bump_value = (filtered_xt-100)*0.3;
      }*/
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control+bump_value);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control-bump_value);
    } 
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(500);
    int horn_limit_switch = digitalRead(A2);
    Serial.println(horn_limit_switch);
    if(horn_limit_switch == LOW)
    {
      left_flag = BARRIER_DETECTED;
      digitalWrite(13,HIGH);
    }
    else
      left_flag = EDGE_DETECTED;
    servo_horn.write(servo_init);
    //It gets this far
    Serial.println("F Down");
    while (y_t <175)
    {
      fixed_speed=-170;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    sign_change=-1;
    alpha=0;
    maxSpeed=100;
    while (fabs(roll)>8)
    {
      fixed_speed=0;
      control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    alpha=1;
    //Serial.println("Moving");
    maxSpeed=150;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=110;
      control =face_down(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(500);
    horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
    {
      bottom_flag = BARRIER_DETECTED;
      digitalWrite(13,HIGH);
    }
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
    alpha=0;
    maxSpeed=150;
    while (fabs(roll+90)>8)
    {
      fixed_speed=0;
      control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      if (fabs(roll+90)<18)
        right_kp=6.5;
    }
    right_kp=3;
    alpha=1;
    maxSpeed=250;
    delay(100);
    float start_distance=x_t;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      float bump_val_left =0;
      float bump_val_right=0;
        if (start_distance <100)
        {
          if (fabs(start_distance - x_t) > 20)
          {
            if (start_distance > x_t)
              bump_val_right=(start_distance - x_t);
            else 
              bump_val_left=(x_t-start_distance);
          }
        }
      control =face_right(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control + bump_val_left);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control+bump_val_right);
    }
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    servo_horn.write(servo_final);
    delay(500);
    horn_limit_switch = digitalRead(A2);
    if(horn_limit_switch == LOW)
    {
      right_flag = BARRIER_DETECTED;
      digitalWrite(13,HIGH);
    }
    else
      right_flag = EDGE_DETECTED;
    servo_horn.write(servo_init);
    start_distance=x_t;
    while (y_t<200)
    {
      float bump_val_left =0;
      float bump_val_right=0;
      if (start_distance <100)
      {
        if (fabs(start_distance - x_t) > 10)
        {
          if (start_distance > x_t)
            bump_val_right=(start_distance - x_t);
          else 
            bump_val_left=(x_t-start_distance);
        }
      }
      fixed_speed=-170;
      control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control-bump_val_left);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control-bump_val_right);
    }
    alpha=0;
    sign_change=1;
    while (fabs(roll)>5 || filtered_accelZ<0)
    {
      fixed_speed=110;
      control =face_up(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    //Maybe do this deteciton by distance
    alpha=1;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=170;
      control =face_up(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    }
    alpha=0;
    left_kp=3.5;
    maxSpeed=100;
    while (fabs(roll-90)>8)
    {
      fixed_speed=0;
      control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      if (fabs(roll-90)<18)
        left_kp=6.5;
    }
    left_kp=4;
    maxSpeed=250;
    path_state=SWEEP;
  }
  
  if (path_state == SWEEP)
  {
    byte side_done_flag =0;
    byte next_pass_done =0;
    //if (top_flag==BARRIERDETECTED || bottom_flag==BARRIERDETECTED)
    sign_change=-1;
    float start_distance=x_t;
    while (!side_done_flag)
    {
      //fan_control.writeMicroseconds(2000);
      //Serial.println("s");
      delay(100);
      start_distance=x_t;
      alpha=1;
      while (y_t>275)
      {
        float bump_val_left =0;
        float bump_val_right=0;
        if (start_distance <100)
        {
          if (fabs(start_distance - x_t) > 10)
          {
            if (start_distance > x_t)
              bump_val_right=(start_distance - x_t);
            else 
              bump_val_left=(x_t-start_distance);
          }
        }
        fixed_speed=170;
        control =face_left(filtered_accelY, filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control+bump_val_left);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control + bump_val_right);
      }
      alpha=0;
      sign_change=-1;
      if (next_pass_done==1)
      {
        side_done_flag=1;
        //if (second_side==1)
        //{
          path_state=SECONDSIDE;
          path_done=1;
        //}
        //else
        //  path_state=BARRIERCROSS;
        break;
      }
      
      while (fabs(roll+90)>5)
      {
        fixed_speed=120;
        control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }
      alpha=1;
      
      delay(100);
      start_distance=x_t;
      while (y_t>250)
      {
        fixed_speed=170;
        float bump_val_left =0;
        float bump_val_right=0;
        if (start_distance <500)
        {
          if (fabs(start_distance - x_t) > 40)
          {
            if (start_distance > x_t)
              bump_val_right=(start_distance - x_t) * 0.5;
            else 
              bump_val_left=(x_t-start_distance) * 0.5;
          }
        }
        control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
        set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
        set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      }
      if (x_t<250)
      {
        //if (second_side==1)
        //{
          path_state=SECONDSIDE;
          path_done=1;
        //}
        //else
        //  path_state=BARRIERCROSS;
        side_done_flag=1;
        break;
      }
      else
      {
        if (x_t<450)
        {
          next_pass_done=1;
        }
        alpha=0;
        sign_change=-1;
        left_kp=3;
        while (fabs(roll-90)>8)
        {
          fixed_speed=80;
          control =face_left(filtered_accelY, filtered_accelZ,roll,RIGHT);
          set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
          set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
        }
      }
    }
  }
  bottom_flag=BARRIER_DETECTED;
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
      float start_distance=0;
      if (filtered_accelY >0)
      {
        alpha=0;
        sign_change=-1;
        down_kp=10;
        while (fabs(roll)>8 || filtered_accelZ>0)
          {
            fixed_speed=0;
            control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
            set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
            set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
            //Serial.println(control);
          }
          down_kp=3.5;
          alpha=1;
          while (!digitalRead(A0) && !digitalRead(A1))
          {
            fixed_speed=40; 
            control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
            set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
            set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
            //Serial.println(control);
          } 
          for(int i = 0; i<300; i++)
          {
            forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 140);
            delay(2); 
          }
          alpha=1;
          brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
          delay(1000);
          alpha=0;
          sign_change=-1;
          while (fabs(roll+90)>5)
          {
            fixed_speed=80;
            control =face_right(filtered_accelY,filtered_accelZ,roll,LEFT);
            set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
            set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
            //Serial.println(control);
          }
          alpha=1;
          start_distance=x_t;
          while (y_t>250)
          {
            fixed_speed=170;
            float bump_val_left =0;
            float bump_val_right=0;
           /* if (start_distance <500)
            {
              if (fabs(start_distance - x_t) > 40)
              {
                if (start_distance > x_t)
                  bump_val_right=(start_distance - x_t) * 0.5;
                else 
                  bump_val_left=(x_t-start_distance) * 0.5;
              }
            }*/
            control =face_right(filtered_accelY,filtered_accelZ,roll,RIGHT);
            set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
            set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
          }
          alpha=0;
          sign_change=-1;
          while (fabs(roll-90)>8)
          {
            /*if (filtered_accelZ<0)
              sign_change=-1;
            else
              sign_change=1;*/
            fixed_speed=80;
            control =face_left(filtered_accelY, filtered_accelZ,roll,RIGHT);
            set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
            set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
          }
          path_state=SWEEP;
      }
      else
      {
        alpha=0;
        sign_change=-1;
        while (fabs(roll)>8 || filtered_accelZ>0 )
        {
          fixed_speed=0;
          control =face_down(filtered_accelY,filtered_accelZ,roll,RIGHT);
          set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
          set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
        }
        for(int i = 0; i<600; i++)
        {
          forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 140);
          delay(2); 
        }
        alpha=0;
        //Move down to the barrier
        while (fabs(roll-90)>8)
        {
          if (filtered_accelZ<0)
            sign_change=-1;
          else
            sign_change=1;
          fixed_speed=80;
          control =face_left(filtered_accelY, filtered_accelZ,roll,RIGHT);
          set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
          set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
          //Serial.println(control);
        }
      }
      path_state=SECONDSIDE;
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
        //Serial.println(control);
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
      for(int i = 0; i<600; i++)
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
        //Serial.println(control);
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
    second_side=1;
    path_state=SWEEP; 
  }
  if (path_state == SWARM_SIDE) {
    alpha=1;
    while (!digitalRead(A0) && !digitalRead(A1))
    {
      fixed_speed=140; 
      control =face_down(filtered_accelY,filtered_accelZ,roll,LEFT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
    } 
    for(int i = 0; i<300; i++)
    {
      forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 140);
      delay(2); 
    }
    alpha=1;
    brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
    delay(500);
    alpha=0;
    sign_change=-1;
    while (fabs(roll-90)>8)
    {
      /*if (filtered_accelZ<0)
        sign_change=-1;
      else
        sign_change=1;*/
      fixed_speed=40;
      control =face_left(filtered_accelY, filtered_accelZ,roll,RIGHT);
      set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, fixed_speed - control);
      set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, fixed_speed + control);
      if (fabs(roll-90)<18)
        left_kp=6;
    }
    left_kp=3.5;
    alpha=1;
    second_side=1;
    path_state=SWEEP;
 }
 if (path_state == DEBUG_CROSS)
 {
  while (1){
  //fan_control.writeMicroseconds(1000);
  //Serial.print(digitalRead(A0));
  //Serial.print(digitalRead(A1));
  //Serial.println(digitalRead(A2));
  //delay(50);
  }
 }
  //brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
  //delay(2000);
  if (path_done==1)
  {
    while(1)
    {
      fanSpeed=1000;
      fan_control.writeMicroseconds(1000);
     // Serial.println(y_t);
    }
  }
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
