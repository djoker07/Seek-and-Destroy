#include <LiquidCrystal.h>
#include <NewPing.h>
#include <SPI.h>
#include <Pixy.h>

#define Trig 30
#define Echo 32
#define MAX_DISTANCE 200 //max distance in centimeters

//definitions for pixy
#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)

// initialize the LED pins
LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

//initializa the ultrasonic sensor
NewPing sonar(Trig, Echo, MAX_DISTANCE);

//initialize motors
int forwardRight = 2; //blue
int backRight = 3; //green
int forwardLeft = 4; //yellow
int backLeft = 5; //orange
int laser = 34;//laser pin

//pixy class initialization for gain and servo loop
class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(500, 800);
ServoLoop tiltLoop(700, 900); 

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}
//function for the servos position
void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {	
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    m_pos += vel;
    if (m_pos>RCS_MAX_POS) 
      m_pos = RCS_MAX_POS; 
    else if (m_pos<RCS_MIN_POS) 
      m_pos = RCS_MIN_POS;

    //cprintf("%d %d %d\n", m_axis, m_pos, vel);
  }
  m_prevError = error;
}

Pixy pixy;

void setup() {
  //start lcd
  lcd.begin(16, 2);
  
  //start motor pins
  pinMode(forwardRight, OUTPUT); 
  pinMode(backRight, OUTPUT);
  pinMode(forwardLeft, OUTPUT);
  pinMode(backLeft, OUTPUT);
  
  //initialize laser
  pinMode(laser, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("Starting...");
  pixy.init();
  
  
    
}

void loop() {
  
  delay(500);
  unsigned int distance = sonar.ping();
  //Serial.print(distance / US_ROUNDTRIP_CM);
  //Serial.print("cm ");
  
 //LCD setup
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(" Scanning: ");
  printDistance(distance);
  
  //pixy setup
  uint16_t blocks; 
  int32_t panError, tiltError;
  blocks = pixy.getBlocks();
  
  Serial.print("block: ");
  Serial.print(blocks);
  
  if(blocks)
  {
    Serial.print(" stop"); 
    stopRobot();
    align(blocks);
    shoot();
    Serial.print (" delay\n"); 
    //delay(1000);
  } 


   else  if(distance <= 3500)
  {
    Serial.print(" turning ");
    Serial.print(distance / US_ROUNDTRIP_CM);
    Serial.print("   \n");
     turn(); 
  }

   else
 { 
    moveForward();
    Serial.print(" im moving forward\n");
 }

}

void moveForward()
{
  digitalWrite(forwardRight, HIGH);
  digitalWrite(backRight, LOW);
  digitalWrite(forwardLeft, LOW);
  digitalWrite(backLeft, HIGH);   
}

void stopRobot()
{
  digitalWrite(forwardRight, LOW);
 // digitalWrite(backRight, LOW);
 // digitalWrite(forwardLeft, LOW);
  digitalWrite(backLeft, LOW);   
}

void turn()
{
  digitalWrite(forwardRight, HIGH);
  digitalWrite(backRight, LOW);
  digitalWrite(forwardLeft, HIGH);
  digitalWrite(backLeft, LOW );  
}

void printDistance( int distance)
{
 lcd.setCursor(11,1);
    lcd.print(distance / US_ROUNDTRIP_CM);
     
}

void align(uint16_t blocks)
{
  

  int32_t panError, tiltError;

    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
  
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    
   
}

void shoot()
{
    digitalWrite(laser, HIGH);
    delay(1000);
    digitalWrite(laser, LOW);
}
  

