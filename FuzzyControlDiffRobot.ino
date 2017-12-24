//-----------Fuzzy controller for differential robot actuators--
//------------------WORKING!!!----------------------------------
//-----------------17/08/2017-----------------------------------
//-----This code contains a fuzzy proportional controller with derivative and
//-----integral actions to regulate the position of the actuators of a 
//-----differential robot using an optica incremental encoder as feedback
#include <math.h>
#define pi 3.1416
//Fuzzy partitions
float A[] = {-1,pi/4.0,pi/2.0};
float B[] = {0,pi/2.0,pi};
float C[] = {-pi/4.0,pi/2.0,1000.0};
float D[] = {pi/2.0,pi,1001.0};
float Kpf1[] = {5.0,2.0,1.0};
float Kpf2[] = {5.0,2.0,1.0};
//System variables and references
float q1,q2,q1d,q2d;
//Control variables
float KP1,KD1=0.0,KI1=0.0,KP2,KD2=0.0,KI2=0.0;
float e1,e10=0.0,de1,ie1,e2,e20=0.0,de2,ie2,v1,v2;
unsigned int u1,u2,i;

void setup() {
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT);
  pinMode(21,INPUT_PULLUP);
  pinMode(20,INPUT);
  attachInterrupt(digitalPinToInterrupt(2),encoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(21),encoder2,RISING);
  Serial.begin(9600);
}//void setup

void loop() {
  //Angular position references for each motor
  q1d = pi/4; q2d = -pi/4;       
  //Error signals
  e1 = q1d-q1; e2 = q2d-q2;   //Position errors
  de1 = e1-e10; de2 = e2-e20; //Error derivatives approximation
  ie1 = e1+e10; ie2 = e2+e20; //Error integrals
  //Fuzzifying
  KP1 = 0; KP2 = 0;
  for(i=0;i<3;i++)
  {
    KP1 = KP1+mu(abs(e1),A[i],B[i],C[i],D[i]);
    KP2 = KP2+mu(abs(e2),A[i],B[i],C[i],D[i]);
  }//for Fuzzifying
  //Control signals
  v1 = abs(KP1*e1+KD1*de1+KI1*ie1);
  v2 = abs(KP2*e2+KD2*de2+KI2*ie2);
  //Control to saturated PWM outputs
  u1 = sat(int(v1),255);  u2 = sat(int(v2),255);
  analogWrite(1,u1); analogWrite(2,u2);
  //Energy change (motors direction)
  //motor 1
  if(e1>0)
  {
    
  }//if e1>0
  else if(e1<0)
  {
    
  }//else if e1<0
  else
  {
    
  }//else e1
  //motor 2
  if(e2>0)
  {
    
  }//if e2>0
  else if(e2<0)
  {
    
  }//else if e2<0
  else
  {
    
  }//else e2
  //Position errors update
  e10 = e1; e20 = e2;
}//void loop

//Interruptions for incremental encoders reading
void encoder1()
{
  if(digitalRead(3) == LOW)
  {
    q1 = q1+(2*3.1416/100.0);//Conversion from pulses to radians
  }//if digitalRead(3)
  else
  {
    q1 = q1-(2*3.1416/100.0);//onversion from pulses to radians
  }//else digitalRead(3)
}//void encoder1

void encoder2()
{
  if(digitalRead(20) == LOW)
  {
    q2 = q2+(2*3.1416/100.0);//onversion from pulses to radians
  }//if digitalRead(20)
  else
  {
    q2 = q2-(2*3.1416/100.0);//onversion from pulses to radians
  }//else digitalRead(20)
}//void encoder2

//Trapezoidal membership function for position error
float mu(float x,float a,float b,float c,float d)
{
  float mo = max(min(min((x-a)/(b-a),(d-x)/(d-c)),1),0);
  return mo;
}

//Saturation function
unsigned int sat(unsigned int x,unsigned int lim)
{
  if(x>lim)
    x = lim;
  else
    x = x;
  return x;
}

