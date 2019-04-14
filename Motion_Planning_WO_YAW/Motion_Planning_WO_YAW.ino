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
#define UP 0
#define DOWN 1
#define RIGHT 2
#define LEFT 3

MPU6050 mpu;
VL53L0X tof_width;
VL53L0X tof_length;

Servo servo_horn;
Servo fan_control;

MovingAverageFilter tof_width_filter(5);
MovingAverageFilter tof_length_filter(5);
MovingAverageFilter accel_x(5);

int servo_horn_pin = 13;

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

int fanspeed = 50;

// Timers
unsigned long timer = 0, timer_d = 0;
float timeStep = 0.016;//0.00576;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
volatile float yaw = 0;

int num_iter = 10000;

int servo_init = 120;
int servo_final = 5;

int move_speed = 0;
int fixed_speed = 170;
volatile byte timerRoll=0;
volatile byte I2C_Lock=0;
volatile byte rollCounter=0;

void setup() 
{
  //byte p=*MCUSR;
  Serial.begin(115200);
  //Serial.println(p);
  //Serial.println("Q");
  fan_control.attach(fan_esc_pwm);
  fan_control.write(50);
  
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  while (mpu.getClockSource()!=MPU6050_CLOCK_PLL_XGYRO){mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);}
  while (mpu.getScale()==0){mpu.setScale(3);}
  while (mpu.getRange()!= MPU6050_RANGE_2G) {mpu.setRange(MPU6050_RANGE_2G);}

  delay(500);
  pinMode(A3,OUTPUT);
  digitalWrite(A3,LOW);

  tof_width.init();
  tof_width.setAddress(0x30);
  tof_width.setTimeout(500);

  digitalWrite(A3,HIGH);
  
  tof_length.init();
  tof_length.setAddress(0x31);
  tof_length.setTimeout(500);
  delay(100);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  delay(100);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);


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
  *EIFR=0x03;  //Clear the INT0 and INT1 digitFlags
  *EICRA=0x05; //Set INT0 and INT1 to trigger on any edge
  *EIMSK=0x03; //Enable INT0 and INT1 interrupt vectors
  
  
  /***Configure PCINT pins***/
  //*PCIFR=0x07;  //Clear all PCINT flags
  //*PCMSK2=0x10; //Enable PCINT20 Pin
  //*PCMSK1=0x00; //Ensure all PCINT1 pins are disabled
  //*PCMSK0=0x04; //Enable PCINT2 pin
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
  
  timerRoll=0;
  I2C_Lock=0;
  rollCounter=0;
  tof_width.startContinuous();
  tof_length.startContinuous();
  *TCNT2 =0x00; //Set Timer Counter to 0 for safety
  *TIFR2=0xff; //Clear interrupt flags for safety
  *TCCR2A=0x02; //Set to Clear on Timer Compare
  *TCCR2B=0x00;
  *OCR2A=249;   
  *TIMSK2 = 0x02; // Enables Interrupts on OCRA = TCNT2
 // *TCCR2B = 0x07; // Starts the timer
  
  delay(5000);
}


char a;
int flag = 0;
//float x_base = 0.0, y_base = 0.0, theta = 0.0, d_prev_l = 0.0, d_prev_r = 0.0;
//float d_l, d_r, del_l, del_r, heading, rad_wheel = 2/2.54, width = 15/2.54;
float x_g, y_g, yaw_goal = 0, yaw_error_prev = 0.0;
/***************************************************************************/
ISR (TIMER2_COMPA_vect)
{
  if (I2C_Lock==0)
  {
    sei();
    //should just be hardset to 0.016 s (with a small difference in millis)
    float del_t = timeStep;
    float e_t = timeStep*rollCounter + 0.016;
    Vector accel = mpu.readNormalizeAccel();
    timerRoll=1;
    rollCounter=0;
  }
  else
  {
    rollCounter++;
  }
}
void waitForNextTimer()
{
  timerRoll=0;
  while (timerRoll==0){}

}

void face(int dir, int ax, int ay, int az){
  float kp = .05;
  if (dir == UP){ //Facing up
    int control = kp*((float)(ay));
    Serial.println(control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, control);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, -control);
  }
  else if (dir ==DOWN) {
    int control = -1*kp*((float)(ay));
    Serial.println(control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, control);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, -control);
  }
  else if (dir ==LEFT) {
    int control = -1*kp*((float)(ax));
    Serial.println(control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, control);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, -control);
  }
  else if (dir ==RIGHT) {
    int control = kp*((float)(ax));
    Serial.println(control);
    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, control);
    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, -control);
  }
}
/********************/
byte startflag=0;
volatile int16_t prev_filteredX=0;
volatile int16_t prev_filteredY=0;
volatile int16_t prev_filteredZ=0;
void loop()
{ 
  /***************************************************************************/ 
  //If power is lost to the TOF sesnor. All the arduino bricks.
  //TOF sensors work in the way I expected them to. Continous will instantly read given
  //The time-alotted period has passed. The Timing budget of 37ms therefore means that after
  //37ms since the previous read, the timer will "instantly" read the next call.

  // time_I2C_LOCK ==1 -> min(timing_budget-(time_current_read - time_prev_read)), 0) + time_of_I2C_Read
 /* waitForNextTimer();
  I2C_Lock=1;
  float x_g_raw = tof_width.readRangeContinuousMillimeters();
  I2C_Lock=0;
  waitForNextTimer();
  I2C_Lock=1;
  float y_g_raw = tof_length.readRangeContinuousMillimeters();
  I2C_Lock=0;*/
  /*****************************************************************************/
  /*if(x_g_raw <= 3000)
    x_g = tof_width_filter.process(x_g_raw) - 8;

  if(y_g_raw <= 3000)
    y_g = tof_length_filter.process(y_g_raw);
 */
  if(Serial.available())
  {
    barrier_flag = 0;
    edge_flag = 0;
    a = Serial.read();
//    if(a == '2')
//    {
//      move_speed = -fixed_speed;
//      timer = 0;
//      /*  
//      for(int i = 0; i< num_iter; i++)
//        backward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);      
//      brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
//      */
//    }  
//    if(a == '1')
//    {
//      move_speed = fixed_speed;
//      timer = 0;
//      /*
//      for(int i = 0; i< num_iter; i++)
//        forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);      
//      brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
//      */
//    }      
//    if(a == '3')
//    {
//      move_speed = 0;
//      timer = 0;
//      yaw_goal = yaw_goal + 5;
//      /*
//      for(int i = 0; i< num_iter; i++)
//        left(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);      
//      brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
//      */
//    } 
//    if(a == '4')
//    {
//      move_speed = 0;
//      timer = 0;
//      yaw_goal = yaw_goal - 5;
//      /*
//      for(int i = 0; i< num_iter; i++)
//        right(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);      
//      brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
//      */
//    }  
//    if(a == '5')
//    {
//      fanspeed=50;
//      move_speed = 0;
//      brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
//    }
//    
//    if(a=='6')
//      fanspeed=170;
//    
//    if(a == '7')
//      fanspeed -= 10;
//
//    if(a == '8')
//      fanspeed += 10;
//
//    if(a == '9')
//      startflag = 1;
//     
//    fan_control.write(fanspeed);
  }
  
  Vector accel = mpu.readRawAccel();

  //Serial.println("t");
  fan_control.write(170);
  int16_t filteredValX = (accel.XAxis + prev_filteredX)/2;
  prev_filteredX=filteredValX;
  int16_t filteredValY = (accel.YAxis + prev_filteredY)/2;
  prev_filteredY=filteredValY;
  int16_t filteredValZ = (accel.ZAxis + prev_filteredZ)/2;
  prev_filteredZ=filteredValZ;
  face(UP,filteredValX,filteredValY,filteredValZ);
  for (int i=0; i<500; i++)
  {
    Serial.println("b");
    accel = mpu.readRawAccel();
    Serial.print(accel.XAxis);
    Serial.print(" ");
    Serial.print(accel.YAxis);
    Serial.print(" ");
    Serial.println(accel.ZAxis);
    filteredValY = (accel.YAxis + prev_filteredY)/2;
    prev_filteredY=filteredValY;
    face(UP,filteredValX,filteredValY,filteredValZ); 
    delay(20);
  }
  Serial.println("RIGHT");
  delay(500);
  for (int i=0; i<500; i++)
  {
    
    Serial.print(accel.XAxis);
    Serial.print(" ");
    Serial.print(accel.YAxis);
    Serial.print(" ");
    Serial.println(accel.ZAxis);
    accel = mpu.readRawAccel();
    Serial.println("b");
    filteredValX = (accel.XAxis + prev_filteredX)/2;
    prev_filteredX=filteredValX;
    face(RIGHT,filteredValX,filteredValY,filteredValZ); 
    delay(20);
  }
  Serial.println("DOWN");
  delay(500);
    for (int i=0; i<500; i++)
  {
    Serial.print(accel.XAxis);
    Serial.print(" ");
    Serial.print(accel.YAxis);
    Serial.print(" ");
    Serial.println(accel.ZAxis);
    accel = mpu.readRawAccel();
    Serial.println("b");
    filteredValY = (accel.YAxis + prev_filteredY)/2;
    prev_filteredY=filteredValY;
    face(DOWN,filteredValX,filteredValY,filteredValZ); 
    delay(20);
  }
  Serial.println("LEFT");
  delay(500);
    for (int i=0; i<500; i++)
  {
    Serial.print(accel.XAxis);
    Serial.print(" ");
    Serial.print(accel.YAxis);
    Serial.print(" ");
    Serial.println(accel.ZAxis);
    accel = mpu.readRawAccel();
    Serial.println("b");
    filteredValX = (accel.XAxis + prev_filteredX)/2;
    prev_filteredX=filteredValX;
    face(LEFT,filteredValX,filteredValY,filteredValZ); 
    delay(20);
  }
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
  delay(20);
}
