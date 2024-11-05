#include <stdio.h>
#include <wiringPi.h>
long Cnt;
int A_pin=16;
int B_pin=13;
float Speed=0;
long int Cha;
long now,pre_time;
void Interrupt (void) {
   Cnt++;
}
PI_THREAD (timer)
{
  for (;;)
  {
    now=micros();
    Cha=(now-pre_time);
    Speed=(float)Cnt/Cha;
    printf ("%ld,%ld,%f\n",Cnt,Cha,Speed);
    pre_time=micros();
    Cnt=0;
    delay(100);
  }
}
int main (void)
{
  wiringPiSetup () ;
  wiringPiISR (A_pin, INT_EDGE_BOTH, &Interrupt) ;
  wiringPiISR (B_pin, INT_EDGE_BOTH, &Interrupt) ;
  piThreadCreate (timer) ;
  for(;;){
  }
  return 0 ;
}


