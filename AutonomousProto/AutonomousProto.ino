/*AUTONOMOUS ROBOT
 * ROBOCON 2K18 - NEM'CON
 * by ANARC, NIT AGARTALA
 * Coded by: Steve Durairaj, Shubham Jha
 * Things to do:
 * 1. move movement code into loop instead of setup
 * 2. Find kp for y axis and x axis seperately
 */

#include <Servo.h>

/*Stopper object*/
Servo stopper;
const int stopperpin=5;
const int stopangle=87;
const int openangle=98;

/*Gripper object*/
Servo s;
const int servopin=8;
const int servoopen=170;
const int servoclose=157;
const int fullclose=150;

/*proximity Sensor*/
const int proximitySensor=46;

/*Motor class
 * data members: dir,pwm pins of each motor
 * Function: moveIn() to write the direction and pwm to each motor object
 */
class motor
{
  public:
  int dir;
  int pwm;
  motor(int d=10,int p=10)
  {
    dir=d;
    pwm=p;
  }
  void moveIn(int d,int p);
}xmf(27,9),xmb(25,11),yml(24,12),ymr(26,10),throwmotor(7,6);
void motor::moveIn(int d,int p)
{
  digitalWrite(dir,d);
  analogWrite(pwm,p);
}


/*Initialization of some constant variables*/
const int CLOCK=HIGH,ACLOCK=LOW;
const int FORWARD=1;
const int BACKWARD=2;
const int LEFT=3;
const int RIGHT=4;
int STATUS;

/*Throw motor initializations*/
int encoderA=2,encoderB=3;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int degreeValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;



/*Encoder pins initialization*/
const float kpy=1.82;
const float kpx=1.82;
const int leftEncoderA=21;
const int leftEncoderB=53;
const int rightEncoderA=19;
const int rightEncoderB=49;
const int frontEncoderA=18;
const int frontEncoderB=50;
const int backEncoderA=20;
const int backEncoderB=52;
volatile int ticksRight=0;
volatile int ticksLeft=0;
volatile int ticksFront=0;
volatile int ticksBack=0;
const int master_pwm=150;
int Xerror=0,Xcorrection=0;
int Yerror=0,Ycorrection=0;


/*setup()
 * Initializes all the above mentioned pins appropriately
 * and initial movement to the loading postition
 */
void setup() 
{
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  Serial.begin(9600);


  stopper.attach(stopperpin);
  /*Motor Initializations*/
  pinMode(xmf.pwm,OUTPUT);
  pinMode(xmb.pwm,OUTPUT);
  pinMode(yml.pwm,OUTPUT);
  pinMode(ymr.pwm,OUTPUT);
  pinMode(xmf.dir,OUTPUT);
  pinMode(xmb.dir,OUTPUT);
  pinMode(yml.dir,OUTPUT);
  pinMode(ymr.dir,OUTPUT);
  pinMode(throwmotor.pwm,OUTPUT);
  pinMode(throwmotor.dir,OUTPUT);

  /*Proximity Sensor Initialization*/
  pinMode(proximitySensor,INPUT);

 /*Encoders initialization*/
  pinMode(leftEncoderA,INPUT_PULLUP);
  pinMode(leftEncoderB,INPUT_PULLUP);
  pinMode(rightEncoderA,INPUT_PULLUP);
  pinMode(rightEncoderB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderA),updateLeft,CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA),updateRight,CHANGE);
  pinMode(frontEncoderA,INPUT_PULLUP);
  pinMode(frontEncoderB,INPUT_PULLUP);
  pinMode(backEncoderA,INPUT_PULLUP);
  pinMode(backEncoderB,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(frontEncoderA),updateFront,CHANGE);
  attachInterrupt(digitalPinToInterrupt(backEncoderA),updateBack,CHANGE);

  /*Throw motor initializations*/
   pinMode(encoderA,INPUT_PULLUP);
   pinMode(encoderB,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(encoderA),Update,CHANGE);
   attachInterrupt(digitalPinToInterrupt(encoderB),Update,CHANGE);
  
  /*Initial Movement to get out of redzone*/
  
  openStopper();
  delay(500);
  closeStopper();
  delay(250);
  closeStopper();
  while(ticksLeft<2650)
  {
    straightenX();
    straightenY();
    forward();
  }
  rest();
  //End of initial movement to loading zone
  
  /*
   * Extra code not required
  ticksLeft=0;
  while(ticksLeft>-2560)
  {
    straightenX();
    straightenY();
    backward();
  }
  */
}


/*loop():
 *Contains the loop of getball, movement, throw after arrival at loading point*/
void loop() 
{
  ticksRight=0;
  ticksLeft=0;
  ticksFront=0;
  ticksBack=0;

  getball();    //get the ball
  //go to the throwing zone
  while(ticksFront<1920)
   {
     straightenX();
     straightenY();
     left();
   }
  rest();
  //get ready for throw and wait for the ball come to rest
  openStopper();

  //Main launch routine called
  launch(1616,255);
  /*  TZ1 - 1360 overarm(old gripper)
   *  TZ1 - 1315 underarm(old gripper)
   *  TZ1 - 1616 underarm(new gripper)
   *  TZ2 - 1920 underarm(new gripper)
   *  TZ2 - 1600 underarm(new gripper)
   */
  //Looping close stopper
  closeStopper();
  delay(250);
  closeStopper();

  //Returning for another shuttle
  ticksFront=0;
  while(ticksFront>-1920)
  {
    straightenX();
    straightenY();
    right();
  }
  rest();
  delay(1000);
}
/*closeStopper()
 * Rotate the arm to its original position and turn on the stopper servo
 */
void closeStopper()
{
  stopper.write(stopangle);       //Writing close angle
  throwmotor.moveIn(CLOCK,40);   //Bringing the shaft back to position
  delay(1000);
  throwmotor.moveIn(CLOCK,7);
}
/*openStopper()
 * Turns off the stopper
 */
void openStopper()
{
    throwmotor.moveIn(CLOCK,0);
    stopper.write(openangle);       //Writing open angle      
 }


/*straighten functions:
 * Functions that calculate the error between the master and slave shaft position
 * Calculates the correction as well
 */
void straightenY()
{
  //Left is the master
  Yerror=ticksLeft-ticksRight;
  Ycorrection=Yerror*kpy;
}
void straightenX()
{
  //Front is the master;
  Xerror=ticksFront-ticksBack;
  Xcorrection=Xerror*kpx;
}

/*getball()
 * Function: used during the transfer of shuttle from manual to automatic
 * Works by opening and closing servo at regular intervals and
 * sensing proximity of the manual bot to the automatic bot
 */
void getball()
{
  /*Initial Gripper Attach*/
  s.attach(servopin);

  /*Read sensor (Same repeated in loop)*/
  int proximity=digitalRead(proximitySensor);

  /*Wait for the manual bot to arrive*/
  while(proximity==HIGH)
  {
    Serial.println(proximity);
    proximity=digitalRead(proximitySensor);
  }

  /*Start opening and closing Servo until manual bot leavees*/
  while(proximity==LOW)
  {
    Serial.println(proximity);
    s.write(servoopen);
    delay(3000);
    s.write(servoclose);
    delay(3000);
    proximity=digitalRead(proximitySensor);
  }
  s.write(fullclose);
}

/*launch()
 * Used when throwing the ball
 * Sets pwm and angle for throw
 * Main function call after getball()
 */

void launch(int t,int p)
{
  int i;

  /*Blink light to indicate Throw*/
  for(i=0;i<3;++i)
  {
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
  }
  
  /*Set to high to indicate danger*/
  digitalWrite(13,HIGH);

  /*Setting Throw Status*/
  int throwstatus=1;

  /*Set all encoder variables to initial*/
  lastEncoded = 0;
  encoderValue = 0;
  degreeValue = 0;
  lastencoderValue = 0;
  lastMSB = 0;
  lastLSB = 0;

  /*Main throw loop*/
  while(throwstatus)
  {
    /*Stop Throwing*/
    if(degreeValue>=t)
    {
      throwstatus=0;        //Exit loop
      digitalWrite(13,LOW); //Turn off indicator
      s.write(servoopen);   //Open Servo
      throwmotor.moveIn(CLOCK,0); //Turn off motor
      delay(1000);          //wait for motor to come to rest
    }
    /*Continue throwing*/
    else
    {
      Serial.print("throwing: ");
      Serial.println(degreeValue);
      throwmotor.moveIn(ACLOCK,p);
    }
  }

  /*Move motor in opposite direction  (prototype: not yet tested*/
  encoderValue=0;
  degreeValue=0;
  while(degreeValue>-t)
  {
    throwmotor.moveIn(CLOCK,50);
  }
  throwmotor.moveIn(CLOCK,0);
  delay(2000);
  /*Final detach after throw*/
  s.detach();
}




/*Movement Functions*/
/*Working:
 * Each motor is an object, which has a moveIn member function
 * ACLOCK and CLOCK are constant integers which represent the movement
 * the second argument represents the speed
 */
void clockwise()
{
  xmf.moveIn(ACLOCK,0);
  xmb.moveIn(CLOCK,0);
  yml.moveIn(ACLOCK,master_pwm);
  ymr.moveIn(CLOCK,master_pwm+Ycorrection);
}
void anticlockwise()
{
  xmf.moveIn(CLOCK,0);
  xmb.moveIn(ACLOCK,0);
  yml.moveIn(CLOCK,master_pwm);
  ymr.moveIn(ACLOCK,master_pwm+Ycorrection);
}
void forward()
{
  STATUS=FORWARD;
  Serial.println("Forward");
  xmf.moveIn(CLOCK,0);
  xmb.moveIn(CLOCK,0);
  yml.moveIn(CLOCK,master_pwm);
  ymr.moveIn(CLOCK,master_pwm+Ycorrection);
}
void backward()
{
  STATUS=BACKWARD;
  Serial.println("Backward");
  xmf.moveIn(CLOCK,0);
  xmb.moveIn(CLOCK,0);
  yml.moveIn(ACLOCK,master_pwm+Ycorrection);
  ymr.moveIn(ACLOCK,master_pwm);
}
void left()
{
  STATUS=LEFT;
  Serial.println("Left");
  xmf.moveIn(CLOCK,master_pwm);
  xmb.moveIn(CLOCK,master_pwm+Xcorrection);
  yml.moveIn(CLOCK,0);
  ymr.moveIn(CLOCK,0);
}
void right()
{
  STATUS=RIGHT;
  Serial.println("Right");
  xmf.moveIn(ACLOCK,master_pwm+Xcorrection);
  xmb.moveIn(ACLOCK,master_pwm);
  yml.moveIn(CLOCK,0);
  ymr.moveIn(CLOCK,0);
}
void rest()
{
  STATUS=-1;
  Serial.println("Rest");
  xmf.moveIn(CLOCK,0);
  xmb.moveIn(CLOCK,0);
  yml.moveIn(CLOCK,0);
  ymr.moveIn(CLOCK,0);
}



/*Encoder update functions*/
/*
 * Simple explanation:
 * As encoder A and encoder B are 90 degree out of phase with eachother,
 * B leads A
 * It is the case that during forward motion when A changes to HIGH, B is already HIGH
 * and when A changes to LOW, B is already LOW
 * i.e. The readings match
 * The opposite occurs for backward motion
 * i.e the readings dont match
 */
void updateLeft()
{
  if(digitalRead(leftEncoderA)==digitalRead(leftEncoderB))
    ticksLeft++;
  else
    ticksLeft--;
}
void updateRight()
{
  if(digitalRead(rightEncoderA)==digitalRead(rightEncoderB))
    ticksRight--;
  else
    ticksRight++;
}
void updateFront()
{
  if(digitalRead(frontEncoderA)==digitalRead(frontEncoderB))
    ticksFront--;
  else
    ticksFront++;
}
void updateBack()
{
  if(digitalRead(backEncoderA)==digitalRead(backEncoderB))
    ticksBack++;
  else
    ticksBack--;
}
/*Throw motors interrupt*/
void Update()
{
  
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);
 
  int encoded = (MSB << 1) |LSB;
  int sum  = (lastEncoded << 2) | encoded;
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  degreeValue = (encoderValue*360)/2400;   //Working formula 2400 pulses == 360 degree
  if(degreeValue >= 3600){
    encoderValue = 0;
  }
 
  lastEncoded = encoded; //store this value for next time

}


