#include "defines.h"
void forward(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int motor_speed){
  //Motor A
  //Motor A Inputs
  digitalWrite(ain1,LOW);
  digitalWrite(ain2,HIGH);

  //PWM for Motor A
  analogWrite(pwma,motor_speed);

      
  //Motor B
  //Motor B Inputs
  digitalWrite(bin1,HIGH);
  digitalWrite(bin2,LOW);

  //PWM for Motor B
  analogWrite(pwmb,motor_speed);
}


void backward(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int motor_speed){
  //Motor A
  //Motor A Inputs
  digitalWrite(ain1,HIGH);
  digitalWrite(ain2,LOW);

  //PWM for Motor A
  analogWrite(pwma,motor_speed);


  //Motor B
  //Motor B Inputs
  digitalWrite(bin1,LOW);
  digitalWrite(bin2,HIGH);

  //PWM for Motor B
  analogWrite(pwmb,motor_speed);
}


void left(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int motor_speed){
  
  //Motor A
  //Motor A Inputs
  digitalWrite(ain1,LOW);
  digitalWrite(ain2,HIGH);

  //PWM for Motor A
  analogWrite(pwma,motor_speed);


  //Motor B
  //Motor B Inputs
  digitalWrite(bin1,LOW);
  digitalWrite(bin2,LOW);

  //PWM for Motor B
  analogWrite(pwmb,motor_speed/2); 
}


void right(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int motor_speed){
  
  //Motor A
  //Motor A Inputs
  digitalWrite(ain1,LOW);
  digitalWrite(ain2,LOW);

  //PWM for Motor A
  analogWrite(pwma,motor_speed/2);


  //Motor B
  //Motor B Inputs
  digitalWrite(bin1,HIGH);
  digitalWrite(bin2,LOW);

  //PWM for Motor B
  analogWrite(pwmb,motor_speed);
 
}


void brake_hard(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int motor_speed){
  //Motor A
  //Motor A Inputs
  digitalWrite(ain1,LOW);
  digitalWrite(ain2,LOW);

  //PWM for Motor A
  analogWrite(pwma,motor_speed);


  //Motor B
  //Motor B
  digitalWrite(bin1,LOW);
  digitalWrite(bin2,LOW);

  //PWM for Motor B
  analogWrite(pwmb,motor_speed);
}


void set_speed_left(int bin1, int bin2, int pwmb, int motor_speed){
  
  if(motor_speed<0){
    digitalWrite(bin1,LOW);
    digitalWrite(bin2,HIGH);
  }
  
  else{
    digitalWrite(bin1,HIGH);
    digitalWrite(bin2,LOW);
  }

  if(motor_speed>maxSpeed)
    motor_speed = maxSpeed;

  if(motor_speed<-maxSpeed)
    motor_speed = -maxSpeed;

  analogWrite(pwmb,abs(motor_speed));
}


void set_speed_right(int ain1, int ain2, int pwma, int motor_speed){
  
  if(motor_speed<0){
    digitalWrite(ain1,HIGH);
    digitalWrite(ain2,LOW);
  }
  
  else{
    digitalWrite(ain1,LOW);
    digitalWrite(ain2,HIGH);
  }

  if(motor_speed>maxSpeed)
    motor_speed = maxSpeed;

  if(motor_speed<-maxSpeed)
    motor_speed = -maxSpeed;

  analogWrite(pwma,abs(motor_speed));
}


int face_up(float a_y, float a_z, float roll, int dir)
{
  float kp = up_kp;
  float control= 0;
  //Serial.println("u");
  int turn_speed = 170;
  if(a_z < 0){
    control = turn_speed-65;
    if (dir == RIGHT)
      control=-1*control;
    /*
    if(roll>0)
      control = -turn_speed;
      //right to 0
    else
      //left to 0
      control = turn_speed;
      */
  }
  else
    //err = roll
    control = - kp * roll;
  return control; 
}


int face_left(float a_y, float a_z, float roll, int dir)
{
  float kp = left_kp;
  float control=0;
  int turn_speed = 170;
  if(a_y < 0){
    if(dir==RIGHT)
      //right to 90
      control =-turn_speed; //-(kp * (90 - roll));
    else
      //left to 90
      control = turn_speed;//(kp * (90 - roll));
  }
  else
    //err = 90 - sign(z)*roll
    control = ((a_z>0)?1:-1)*(kp * (90 - roll));

  return control;
    
}


int face_right(float a_y, float a_z, float roll, int dir)
{
  float kp = right_kp;
  float control=0;
  int turn_speed = 170;
  if(a_y > 0){
    if(dir==RIGHT)
      //right to 90
      control = -turn_speed;//(kp * (-90 - roll));
    else
      //left to 90
      control = turn_speed;//-(kp * (-90 - roll));
  }
  else
    //err = -90 - sign(z)*roll
    control = ((a_z>0)?1:-1)*(kp * (-90 - roll));

  return control;
}


int face_down(float a_y, float a_z, float roll, int dir)
{
  float kp = down_kp;
  float control= 0;
  //Serial.println("d");
  int turn_speed = 250;
  if(a_z > 0){
    if(dir==RIGHT)
      //right to 0
      control = -turn_speed;
    else
      //left to 0
      control = turn_speed;
  }//Tried to stop motor from over turning. Did not work too well.
  else
    //err = -roll
    control = kp*roll;

  return control;
}
/*int dleft()
{
  while(fabs(roll-90)>4) 
  {
  }
}*/
