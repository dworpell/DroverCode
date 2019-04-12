#define EIMSK  ((volatile byte*)0x3D)
#define EIFR   ((volatile byte*)0x3C)
#define EICRA  ((volatile byte*)0x69)
#define PCICR  ((volatile byte*)0x68)
#define EICRA  ((volatile byte*)0x69)
#define PCMSK2 ((volatile byte*)0x6D) //PCINT 23...16
#define PCMSK1 ((volatile byte*)0x6C) //PCINT 15...8
#define PCMSK0 ((volatile byte*)0x6B) //PCINT  7...0
#define PORTD  ((volatile byte*)0x2A) //Pins D0-D7
#define PORTC  ((volatile byte*)0x28) //Pins A0-A5
#define PORTB  ((volatile byte*)0x25) //Pins D8-D13
#define PCIFR  ((volatile byte*)0x3B)
#define PIND   ((volatile byte*)0x29)
#define PINC   ((volatile byte*)0x26)
#define PINB   ((volatile byte*)0x25)

#define TIMSK2 ((volatile int *)0x70)
#define EICRA  ((volatile int *)0x69)
#define TIFR2  ((volatile byte*)0x37)
#define TCCR2A ((volatile byte *)0xB0)
#define TCCR2B ((volatile byte *)0xB1)
#define TCNT2  ((volatile byte *)0xB2)
#define OCR2A ((volatile byte *)0xB3)
#define OCR2B ((volatile byte *)0xB4)


byte Encoder0B,Encoder0A,Encoder1A,Encoder1B;
int32_t EdgeCountRight=0;
int32_t EdgeCountLeft =0;

ISR(INT0_vect) {
  //Serial.println(EdgeCount);
  if(Encoder0A)
  {
    if (Encoder0B) //Was 11
      EdgeCountLeft++;
    else // Was 10
      EdgeCountLeft--;
  }
  else //Encoder0A
  {
    if (Encoder0B) //was 01
      EdgeCountLeft--;
    else    //was 00
      EdgeCountLeft++;
  }
  Encoder0B=!Encoder0B;
  //Serial.println("B");
}

ISR(INT1_vect) {
  //Serial.println(EdgeCount);
  //Serial.println("1");

  if(Encoder0B)
  {
    if (Encoder0A) //Was 11
      EdgeCountLeft--;
    else // Was 01
      EdgeCountLeft++;
  }
  else 
  {
    if (Encoder0A) //was 10
      EdgeCountLeft++;
    else //was 00
      EdgeCountLeft--;
  }
  //Serial.println("D");
  Encoder0A=!Encoder0A;
}

ISR(PCINT2_vect) {
  //Serial.println(EdgeCount);
  //Serial.println("1");

  if(Encoder1B)
  {
    if (Encoder1A) //Was 11
      EdgeCountRight--;
    else // Was 01
      EdgeCountRight++;
  }
  else 
  {
    if (Encoder1A) //was 10
      EdgeCountRight++;
    else //was 00
      EdgeCountRight--;
  }
  //Serial.println("D");
  Encoder1A=!Encoder1A;
}

ISR(PCINT0_vect) {
  //Serial.println(EdgeCount);
  if(Encoder1A)
  {
    if (Encoder1B) //Was 11
      EdgeCountRight++;
    else // Was 10
      EdgeCountRight--;
  }
  else //Encoder0A
  {
    if (Encoder1B) //was 01
      EdgeCountRight--;
    else    //was 00
      EdgeCountRight++;
  }
  Encoder1B=!Encoder1B;
  //Serial.println("B");
}
