/*MANUAL ROBOT
 * ROBOCON 2018 - NEM'CON
 * Coded By: Shubham Jha, Steve Durairaj
 * Things to do:
 * 1. Find the error in servo 3
 */

 
#include <Servo.h>

/*Motor Class:
 * Data members: location of pins of the joystick and two variables to store the value
 * Functions:
 * 1. Constructor to initialise values
 * 2. jyState to print analog values
 * 3. jyMap to map values from 0 -1023 to -100 to 100
 * 4. filter to relax spikes in movement
 */
class joystick
{
  public:
  int switch_pin;
  int x_pin;
  int y_pin;
  int x, y;
  joystick(int s=A5,int x=A7,int y=A6)
  {
    switch_pin=s;
    x_pin=x;
    y_pin=y;
  }
  void jyState()
  {
    Serial.print("Switch:");
    Serial.println(digitalRead(switch_pin));
    Serial.print("X:");
    Serial.println(x);
    Serial.print("Y:");
    Serial.println(y);
  }
  void jyMap()
  {
   x=map(filter(x_pin),0,1023,100,-100);
   //y axis is flipped as joystick is connected upside down
   y=map(filter(y_pin),0,1023,-100,100);
  }
  int filter(int x)
  {
   int i=0,sum=0;
   for(i=0;i<30;i++)
   {
     sum+=analogRead(x);
     delay(1);
    }
    sum/=30;
    return sum;
  }
}jy1,jy2(A2,A0,A1); //jy1 takes default constructor values, jy2 values  initialized

/*Motor speed*/
int velocity;

/*Declaration of Servo objects*/
Servo s1,s2,s3;
const int s1pin=6,s2pin=9,s3pin=7 ;


/*Declaration of Joystick varriables*/
const int CLOCK = 1;
const int REST =-1;
const int ACLOCK =0;

/*Motor pins*/
const int dir2=2,dir3=13,dir1=11,dir4=4;
const int pwm2=3,pwm3=12,pwm1=10,pwm4=5;

/*
 * pwm1 -LMF
 * pwm2 -RMF
 * pwm3 -LMB
 * pwm4 -RMB
 */

/*setup():
 * Initializes all the above declared pins appropriately
 */
void setup() 
{
  
  Serial.begin(9600);

  /*Initialisation of joystick switch*/
  pinMode(jy1.switch_pin,INPUT_PULLUP);
  pinMode(jy2.switch_pin,INPUT_PULLUP); 
  
  /* Servo pin init*/
  s1.attach(s1pin);
  s2.attach(s2pin);
  s3.attach(s3pin);

  /*Initialization of Motor driver pins*/
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(pwm3,OUTPUT);
  pinMode(pwm4,OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(dir4,OUTPUT);
  
}



/*loop():
 * Runs a loop to read values of the joystick and calls 
 * driveRoutine maped to jy1
 * and servoRoutine maped to jy2
 */
void loop() 
{  
  jy1.jyMap();
  jy2.jyMap();
  driverRoutine1();
  servoRoutine();
}



/*monveIn():
 * Inputs: pwm,dir,type
 * Function: Writes the direction and pwm value to the motor drivers
 * based on the value of type
 */
void moveIn(int pwm,int dir,int type,int raftar=255)
{
  if(type == CLOCK){
    analogWrite(pwm,raftar);
    digitalWrite(dir,HIGH);
  }
  else if(type == ACLOCK){
    Serial.println("aclock");
    analogWrite(pwm,raftar);
    digitalWrite(dir,LOW);
  }
  else if(type == REST){
    analogWrite(pwm,0);
  }
}

/*servoRoutine():
 * Function: Contorls the servos of the gripper based on switch values
 */
void servoRoutine()
{ 
  int x=jy2.x,y=jy2.y;
  if(digitalRead(jy2.switch_pin)==HIGH)
  {
    if(x<-50 && y<50 && y>-50)      //WEST ACTIVATES ALL
    {
      Serial.println("ALL");
      s1.write(120);
      s2.write(40);
      s3.write(90);
      delay(500);
    }
    else        
    {
      if(y>50 && x<50 && x>-50)       //NORTH ACTIVATES SERVO 1
      {
        s1.write(120);
        Serial.println("1");
        delay(500);
      }
      else
      {
        s1.write(105);
      }
      if(x>50 && y<50 && y>-50)  //EAST ACTIVATES SERVO 2
      {
        s2.write(40);
        Serial.println("2");
        delay(500);
      }
      else
      {
       s2.write(21);
      }
      if(y<-50 && x<50 && x>-50)  //SOUTH ACTIVATES SERVO 3
      {
        s3.write(90);
        Serial.println("3");
        delay(500);
      }
      else
      {
       s3.write(110);
      }
      
   }
  }
  
}
/*driveRoutine():
 * Function: Main drive Function - 
 * uses value of joystick variables to decide movement
 */
void driverRoutine1()
{
  int x=jy1.x,y=jy1.y,s;
  s=analogRead(jy2.switch_pin);
  velocity=sqrt(x*x+y*y);
  velocity=map(velocity,0,100,0,100);
  if(s<100)
  {
    if(x>20 && y<20 && y>-20){
      clockwise();
      Serial.println("clockwise");
    }
    else if(x<-20 && y<20 && y>-20){
      anticlockwise();
      Serial.println("anticlockwise");
    }
  }
  else
  {
    if(x>-20 && x <20 && y>-20 && y<20){
      rest();
     Serial.println("rest");
    }
    else if(x>20 && y<20 && y>-20){
     east();
     Serial.println("east");
    }
    else if(x<-20 && y<20 && y>-20){
      west();
      Serial.println("west");
    }
    else if(y>20 && x<20 && x>-20){
       north();
      Serial.println("north");
    }
    else if(y<-20 && x<20 && x>-20){
      south();
      Serial.println("south");
    }
    else if(x>20  &&  y>20)
    {
      northEast();
      Serial.println("north east");
    }
    else if(x<-20  &&  y>20)
    {
      northWest();
      Serial.println("north west");
    }
    else if(x<-20  &&  y<-20)
    {
      southWest();
      Serial.println("south west");
    }
    else if(x>20  &&  y<-20)
    {
      southEast();
      Serial.println("south east");
    }
  }
}

/*Direction  Functions */
void clockwise()
{
  moveIn(pwm1,dir1,CLOCK,velocity);
  moveIn(pwm2,dir2,ACLOCK,velocity);
  moveIn(pwm3,dir3,CLOCK,velocity);
  moveIn(pwm4,dir4,ACLOCK,velocity);
}
void anticlockwise()
{
  moveIn(pwm1,dir1,ACLOCK,velocity);
  moveIn(pwm2,dir2,CLOCK,velocity);
  moveIn(pwm3,dir3,ACLOCK,velocity);
  moveIn(pwm4,dir4,CLOCK,velocity);
}
void north()
{
  moveIn(pwm1,dir1,CLOCK,velocity);
  moveIn(pwm2,dir2,CLOCK,velocity);
  moveIn(pwm3,dir3,CLOCK,velocity);
  moveIn(pwm4,dir4,CLOCK,velocity);
  
}
void south()
{
  moveIn(pwm1,dir1,ACLOCK,velocity);
  moveIn(pwm2,dir2,ACLOCK,velocity);
  moveIn(pwm3,dir3,ACLOCK,velocity);
  moveIn(pwm4,dir4,ACLOCK,velocity);
}
void east()
{
  moveIn(pwm1,dir1,CLOCK,velocity);
  moveIn(pwm2,dir2,ACLOCK,velocity);
  moveIn(pwm3,dir3,ACLOCK,velocity);
  moveIn(pwm4,dir4,CLOCK,velocity);
}
void west()
{
  moveIn(pwm1,dir1,ACLOCK,velocity);
  moveIn(pwm2,dir2,CLOCK,velocity);
  moveIn(pwm3,dir3,CLOCK,velocity);
  moveIn(pwm4,dir4,ACLOCK,velocity);
}
void northEast()
{
  moveIn(pwm1,dir1,CLOCK,velocity);
  moveIn(pwm2,dir2,REST,velocity);
  moveIn(pwm3,dir3,REST,velocity);
  moveIn(pwm4,dir4,CLOCK,velocity);
}
void southWest()
{
  moveIn(pwm1,dir1,ACLOCK,velocity);
  moveIn(pwm2,dir2,REST,velocity);
  moveIn(pwm3,dir3,REST,velocity);
  moveIn(pwm4,dir4,ACLOCK,velocity);
}
void southEast()
{
  moveIn(pwm1,dir1,REST,velocity);
  moveIn(pwm2,dir2,ACLOCK,velocity);
  moveIn(pwm3,dir3,ACLOCK,velocity);
  moveIn(pwm4,dir4,REST,velocity);
}
void northWest()
{
  moveIn(pwm1,dir1,REST,velocity);
  moveIn(pwm2,dir2,CLOCK,velocity);
  moveIn(pwm3,dir3,CLOCK,velocity);
  moveIn(pwm4,dir4,REST,velocity);
}

void rest(){
  moveIn(pwm1,dir1,REST,0);
  moveIn(pwm2,dir2,REST,0);
  moveIn(pwm3,dir3,REST,0);
  moveIn(pwm4,dir4,REST,0);
}

