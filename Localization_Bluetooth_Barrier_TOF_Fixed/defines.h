#ifndef DEFINESFILE_H
#define DEFINESFILE_H
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

/*#define MCUSR ((volatile byte *)0x55)
#define WDTCSR ((volatile byte *)0xB4)
//#define MCUSR ((volatile byte *)0xB4)
//#define MCUSR ((volatile byte *)0xB4)
//#define MCUSR ((volatile byte *)0xB4)*/
#define RIGHT 0
#define LEFT 1

#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

byte Encoder0B,Encoder0A,Encoder1A,Encoder1B;
int32_t EdgeCountRight=0;
int32_t EdgeCountLeft =0;
volatile float left_kp=4;
volatile float right_kp=3; 
volatile float up_kp=10;
volatile float down_kp=3.5;
volatile int16_t maxSpeed=250;
#endif
