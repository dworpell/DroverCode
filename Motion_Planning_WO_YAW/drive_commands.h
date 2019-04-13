
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

  if(motor_speed>170)
    motor_speed = 170;

  if(motor_speed<-170)
    motor_speed = -170;

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

  if(motor_speed>170)
    motor_speed = 170;

  if(motor_speed<-170)
    motor_speed = -170;

  analogWrite(pwma,abs(motor_speed));
}
