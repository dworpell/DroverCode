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
#define DOWN 180
#define RIGHT 90
#define LEFT 270

MPU6050 mpu;
VL53L0X tof_width;
VL53L0X tof_length;

Servo servo_horn;
Servo fan_control;

MovingAverageFilter tof_width_filter(5);
MovingAverageFilter tof_length_filter(5);
MovingAverageFilter yaw_filter(5);

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
uint8_t TOFCounter=0;

int fanspeed = 50;

// Timers
unsigned long timer = 0, timer_d = 0;
float timeStep = 0.0064;//0.00576;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
volatile float yaw = 0;
float d2r = PI/180.0;

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
  Serial.println("Q");
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
  mpu.calibrateGyro(100);
  delay(1000);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);


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
  *PCIFR=0x07;  //Clear all PCINT flags
  *PCMSK2=0x10; //Enable PCINT20 Pin
  *PCMSK1=0x00; //Ensure all PCINT1 pins are disabled
  *PCMSK0=0x04; //Enable PCINT2 pin
  *PCICR=0x05; //Enable PCINT2 and PCINT0 banks

  //servo_horn.attach(servo_horn_pin);
  //servo_horn.write(servo_init);
  
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
  yaw=0;
  roll=0;
  pitch=0;
  *TCNT2 =0x00; //Set Timer Counter to 0 for safety
  *TIFR2=0xff; //Clear interrupt flags for safety
  *TCCR2A=0x02; //Set to Clear on Timer Compare
  *TCCR2B=0x00;
  *OCR2A=99;   
  *TIMSK2 = 0x02; // Enables Interrupts on OCRA = TCNT2
  *TCCR2B = 0x07; // Starts the timer
  
  tof_width.startContinuous();
  tof_length.startContinuous();
  delay(5000);
  fan_control.write(108);
  TOFCounter=0;
}


char a;
int flag = 0;
volatile uint16_t y_t=0;

float x_base = 0.0, y_base = 0.0, theta = 0.0, d_prev_l = 0.0, d_prev_r = 0.0;
float d_l, d_r, del_l, del_r, heading, rad_wheel = 2/2.54, width = 15/2.54;
float x_g, y_g, x_new = 400,y_new = 250, yaw_goal = 0, yaw_error_prev = 0.0;
float alpha = 0.1, yaw3 = 0.0;
/***************************************************************************/
ISR (TIMER2_COMPA_vect)
{
  sei();
  if (I2C_Lock==0)
  {
    //Serial.println(yaw);
    //sei();
    //should just be hardset to 0.016 s (with a small difference in millis)
    float del_t = timeStep;
    float e_t = timeStep*rollCounter + timeStep;
    d_l = EdgeCountLeft/2520.0 * 2 * PI * rad_wheel;
    d_r = EdgeCountRight/2520.0 * 2 * PI * rad_wheel;
    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();
    //Vector accel = mpu.readNormalizeAccel();
         
    // Calculate Pitch, Roll and Yaw
    //pitch = pitch + norm.YAxis * timeStep;
    //roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;
    Serial.println(yaw);
    
    float yaw2 = -yaw * 3.1416/180.0f;
    
    del_l = d_l - d_prev_l;
    del_r = d_r - d_prev_r;
    //Serial.print(del_l,6);
    //Serial.print(" ");
    //Serial.print(fabs(del_l - del_r),6);
  
    if(e_t!=0)
    { 
      if(fabs(del_l - del_r)  <= 0.0145){
    
        float del_avg = (del_l + del_r)/2.0f;
    
        x_base += del_avg * cos(yaw2);
        y_base += del_avg * sin(yaw2);
        //Serial.print("ST ");
      }
    
      else{
    
        del_l = del_l/e_t;
        del_r = del_r/e_t;
    
        float R = width * (del_l + del_r) / (2 * (del_r - del_l)), wd = (del_r - del_l) / width; 
        float icc_x = x_base - R*sin(yaw2), icc_y = y_base + R*cos(yaw2);
        float del_x = x_base - icc_x, del_y = y_base - icc_y; 
    
        x_base = del_x * cos(wd * e_t) - del_y * sin(wd * e_t) + icc_x;
        y_base = del_x * sin(wd * e_t) + del_y * cos(wd * e_t) + icc_y;
        heading = (yaw2 + wd * e_t)*180.0/PI;
        //Serial.print("TU ");
      }
    }
  
    d_prev_l = d_l;
    d_prev_r = d_r;
  
    timerRoll=1;
    rollCounter=0;
    if (fabs(yaw)>=360.0f)
      yaw = yaw - 360.0f;
  }
  else
  {
    rollCounter++;
  }
  TOFCounter++;
  //must have 8 timer ticks in order to avoid I2C conflict
  if (TOFCounter==39 && TOFCounter>=8)
  {
    //Serial.println("Yeet");
    y_t = tof_length.readRangeContinuousMillimeters();
    TOFCounter=0;
  }
}
void waitForNextTimer()
{
  timerRoll=0;
  while (timerRoll==0){}

}
/**********************************************************/
void face(int dir){
  float kp = 15;
  
  float yaw_error = dir - yaw;

  float control = kp * yaw_error;// + kd * (yaw_error - yaw_error_prev) + 10;
  
  
  set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed + control);
  set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, move_speed - control);

  //yaw_error_prev = yaw_error;
  
//  if (dir == UP){ //Facing up
//    int control = 0;
//    if (fabs(yaw)>=)
//      control=kp*(360.0f -yaw);
//    else
//      control=kp*(0.0f-yaw);
//    Serial.println(control);
//    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, -control);
//    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, control);
//  }
//  else if (dir ==DOWN) {
//    int control = kp*(180.0f-yaw);
//    Serial.println(control);
//    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, -control);
//    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, control);
//  }
//  else if (dir ==LEFT) {
//    int control = kp*(270.0f-yaw);
//    Serial.println(control);
//    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, -control);
//    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, control);
//  }
//  else if (dir ==RIGHT) {
//    int control = kp*(90.0f-yaw);
//    Serial.println(control);
//    set_speed_right(motor_right_in_1, motor_right_in_2, motor_right_pwm, -control);
//    set_speed_left(motor_left_in_1, motor_left_in_2, motor_left_pwm, control);
//  }
}
byte startflag=0;
volatile int16_t prev_filteredX=0;
volatile int16_t prev_filteredY=0;
volatile int16_t prev_filteredZ=0;

void loop()
{ 
  //forward(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, 200);


  //ENU frame +90deg x_g to x_base
  //int tas= millis();
  
  
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
  I2C_Lock=0;*/
  /*****************************************************************************/
  /*if(x_g_raw <= 3000)
    x_g = tof_width_filter.process(x_g_raw) - 8;

  if(y_g_raw <= 3000)
    y_g = tof_length_filter.process(y_g_raw);*/
//  if(Serial.available())
//  {
//    barrier_flag = 0;
//    edge_flag = 0;
//    a = Serial.read();
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
//  }
  delay(2000);
  while (y_t>200)
  {
    move_speed=150;
    face(UP);
    //Serial.println(y_t);
    delay(20);
  }
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face(RIGHT);
    delay(20);
  }
  while (y_t>200)
  {
    move_speed=150;
    face(RIGHT);
    //Serial.println(y_t);
    delay(20);
  }
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face(DOWN);
    delay(20);
  }
  while (y_t>200)
  {
    move_speed=150;
    face(DOWN);
    //Serial.println(y_t);
    delay(20);
  }
  for (int i=0; i<50; i++)
  {
    move_speed=0;
    face(LEFT);
    delay(20);
  }
  while (y_t>200)
  {
    move_speed=150;
    face(LEFT);
    //Serial.println(y_t);
    delay(20);
  }
  move_speed=0;
  brake_hard(motor_right_in_1, motor_right_in_2, motor_right_pwm, motor_left_in_1, motor_left_in_2, motor_left_pwm, move_speed);
  delay(3000);
  while (1){
    fan_control.write(70);
    delay(200);
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
  waitForNextTimer();
  //delay(20);
  //delay((timeStep*1000) - (millis() - timer));
}
