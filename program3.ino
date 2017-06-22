#include <TimerOne.h>
#define SENSOR 0
 int pinA = 13;  // Connected to CLK on KY-040
 int pinB = 12;  // Connected to DT on KY-040
 int encoderPosCount = 0; 
 int pinALast;  
 int aVal;
 boolean bCW;
const int PWM = 3;
const int POS = 7;
const int NEG = 4;
int rpsTarget;
int rotation;
unsigned int counter;

//PID Kp=12,Kd=0.125,Ki=1
 float Kp=2;
 float Kd=1.2;
 float Ki=1;
 float dt=1;
 float setPoint, PV, Sum;
 float PIDval,x, y;
 float error = 0;
 float errorprev =0;
 float PIDvalmax = 180;
 float PIDvalmin = 5;
void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
} 

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  //Serial.print("\nRPS: "); 
  rotation = (counter / 20);  // divide by number of holes in Disc
  
  inversKinematic();
  //Serial.print("  Setpoint: ");
  Serial.print(rotation,DEC);  
  Serial.print(" ");
  Serial.println(setPoint);
  //Serial.print("  Error: ");
  //Serial.print(errorprev);
  //Serial.print("  Sum: ");
  //Serial.print(Sum);
  //Serial.print("  PID: ");
  //Serial.print(x);
  //Serial.print("  Output: ");
  //Serial.print(y);
  counter=0;  //  reset counter to zero
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}
 void setup() { 
     Serial.begin (9600);
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(POS, OUTPUT);
  pinMode(NEG, OUTPUT);
   /* Read Pin A
   Whatever state it's in will reflect the last position   
   */
   pinALast = digitalRead(pinA); 

   Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(SENSOR, docount, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
 } 

 void loop() { 
   //aVal = digitalRead(pinA);
   //if (aVal != pinALast){ // Means the knob is rotating
     // if the knob is rotating, we need to determine direction
     // We do that by reading pin B.
     //if (digitalRead(pinB) != aVal) {  // Means pin A Changed first - We're Rotating Clockwise
       //encoderPosCount ++;
      // bCW = true;
     //} else {// Otherwise B changed first and we're moving CCW
       //bCW = false;
       //encoderPosCount--;
     //}
   //}
   //pinALast = aVal;
   setPoint = 25;//encoderPosCount;
     PV = rotation;
   
 } 

void inversKinematic()
{
  error = setPoint - PV;
  Sum = Sum + error;
  PIDval = Kp*error + Ki*Sum*dt + ((Kd/dt)*(error - errorprev));
  if(PIDval > PIDvalmax) {PIDval = PIDvalmax;};
  if(PIDval < PIDvalmin) {PIDval = PIDvalmin;};

  x = abs( PIDval);
      errorprev = error;
  y = 4*0.00000000001*x*x*x*x*x*x - 3*0.00000001*x*x*x*x*x + 7*0.000001*x*x*x*x - 0.0007*x*x*x + 0.0306*x*x - 0.5234*x + 1.9572;
  //y = (-3*0.0000001*x*x*x*x) + (0.0001*x*x*x) - (0.0124*x*x) + (0.4232*x) - 3.0298;
  //y = (-5*0.000000001*x*x*x*x*x) + (2*0.000001*x*x*x*x) - (0.0002*x*x*x) + (0.0102*x*x) - (0.1604*x) + 0.4539;
  //y = 7*0.00001*x*x*x - 0.0325*x*x + 5.403*x - 291.49;
  //y = (21.26*x*x*x*x*x)-(92.68*x*x*x*x)+(151.1*x*x*x)-(102.5*x*x)+(49.68*x)+5.047;
  y= abs(y);
  analogWrite(PWM,y);
  digitalWrite(POS,1);
  digitalWrite(NEG,0);
}

