#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"
#include <LSM303.h>

#define Left_Motor_Forward 12
#define Left_Motor_Backward 11
#define Right_Motor_Forward 10
#define Right_Motor_Backward 9
#define Left_IR 6
#define Right_IR 5
#define BASELINE 70
#define WHEELRAD 17.5
#define BUFFSIZE 1000
#define SPEEDKP 4
#define PHIKP 2

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 1, 23);
char receiveBuffer[BUFFSIZE];

LSM303 compass;

// +x leftright; +y updown; +z forward

WiFiUDP udp;
unsigned int localPort = 5005;
float robot_Circumference = 2 * PI * BASELINE;
float wheel_Circumference = 2 * PI * WHEELRAD;
float wheel_Tick_Dist = wheel_Circumference / (75.8 * 2);
float tick_Angle = wheel_Tick_Dist / BASELINE;

float deltaX = ( BASELINE / 2 ) * sin( tick_Angle );
float deltaY = ( BASELINE / 2 ) - ( ( BASELINE / 2 ) * cos( tick_Angle ) );

float xGlobal = 0, yGlobal = 0, phiGlobalRad = 0, phiGlobalDeg = 0;

int Left_Tick = 0, Right_Tick = 0;

float Left_Velocity = 0, Right_Velocity = 0;
float Left_Error = 0, Right_Error = 0, Phi_Error = 0;

float velocityLeft = 0, velocityRight = 0, omega = 0;
float leftMotorSet = 0, rightMotorSet = 0;

bool turnLeft = false;

float startingHeadingDeg, startingHeadingRad;
float relativeHeadingDeg, relativeHeadingRad;
float currentHeadingRad, averageHeadingRad, errorHeadingRad;
float currentHeadingDeg, averageHeadingDeg = 0, errorHeadingDeg;
float verticalAcceleration = 0;

int t1 = 0, t2 = 0;

typedef struct cmdPacket{
  double vel;
  double phi;
  int mode;
} cmdPacket;

/*
 * mode 0 = return x,y,head
 * mode 1 = driving command
 * mode 2 = cardinal command
 * mode 3 = stop
 */

cmdPacket _cmdPacket;

typedef struct rtnPacket{
  double x;
  double y;
  double head;
} rtnPacket;

rtnPacket _rtnPacket;

void setup()
{

  Serial.begin(9600);
  setupIMU();
  setupPins();
  setupWiFi();
  setupPort();

  analogWrite( Left_Motor_Forward, 0 );
  analogWrite( Left_Motor_Backward, 0 );
  analogWrite( Right_Motor_Forward, 0 );
  analogWrite( Right_Motor_Backward, 0 );

  _cmdPacket.vel = 0;
  _cmdPacket.phi = 270;
  _cmdPacket.mode = 2;
  
}

void loop()
{
  updateIMU();
  
  checkUDP();

  rightMotorKP(_cmdPacket.vel);
  leftMotorKP(_cmdPacket.vel);

  phiKP(_cmdPacket.phi);
  timeUpdate();
}

void leftISR()
{
  //Serial.println("leftisr");
  Left_Tick += 1;
  phiGlobalRad -= tick_Angle;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;
  
  xGlobal += ( ( deltaX * cos( phiGlobalRad ) ) + deltaY * sin( phiGlobalRad ) );
  yGlobal -= ( ( deltaX * sin( phiGlobalRad ) ) + deltaY * cos( phiGlobalRad ) );
}

void rightISR()
{
  //Serial.println("rightisr");
  Right_Tick += 1;
  phiGlobalRad += tick_Angle;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;

  xGlobal += ( ( deltaX * cos( phiGlobalRad ) ) + deltaY * sin( phiGlobalRad ) );
  yGlobal += ( ( deltaX * sin( phiGlobalRad ) ) + deltaY * cos( phiGlobalRad ) );
}

void leftMotorKP( double speedLeft )
{
  Left_Velocity = Left_Tick / ( 2 * 75.8 );
  Left_Error = speedLeft - Left_Velocity;

  velocityLeft = SPEEDKP * Left_Error;
  //Serial.println("velocity left");
  //Serial.println(velocityLeft);
}

void rightMotorKP(double speedRight)
{
  Right_Velocity = Right_Tick / ( 2 * 75.8 );
  Right_Error = speedRight - Right_Velocity;

  velocityRight = SPEEDKP * Right_Error;
  //Serial.println("vel right");
  //Serial.println(velocityRight);
}

void phiKP( double phiDesiredDeg )
{
  averageHeadingDeg = currentHeadingDeg;
  if(errorHeadingDeg >5){
  Serial.println("desired:");
  Serial.println(phiDesiredDeg);
  Serial.println("actual:");
  Serial.println(averageHeadingDeg);
  Serial.println("error:");
  Serial.println(errorHeadingDeg);
  Serial.println();
  }
  errorHeadingDeg = phiDesiredDeg - averageHeadingDeg;
  if( 0 == phiDesiredDeg )
  {
    if ( abs( errorHeadingDeg ) < 10 ) errorHeadingDeg = 0;
    if( averageHeadingDeg >= 0 && averageHeadingDeg < 30 )
    {
      turnLeft = true;
    }
    else
    {
      turnLeft = false;
    }
  }
  else
  {
    if( errorHeadingDeg >= 0 )
    {
      turnLeft = true;
    }
    else
    {
      turnLeft = false;
    }
  }
  
  omega = PHIKP * errorHeadingDeg;
}

void timeUpdate()
{
  /*
   * Allows us to update our motor control value every 20 milliseconds (50 Hz)
   * This is done so we 1) don't update uselessly and
   * 2) so we still update even when the wheel is moving slowly
   */
  
  t1 = millis();
  
  if(t1 - t2 >= 20)
  {
    updateMotors();
    t2 = t1;
  }
}

void setupPins()
{
  
  pinMode(Left_Motor_Forward, OUTPUT);
  pinMode(Left_Motor_Backward, OUTPUT);
  pinMode(Right_Motor_Forward, OUTPUT);
  pinMode(Right_Motor_Backward, OUTPUT);

  analogWrite(Left_Motor_Forward, 0);
  analogWrite(Left_Motor_Backward, 0);
  analogWrite(Right_Motor_Forward, 0);
  analogWrite(Right_Motor_Backward, 0);

  pinMode(Left_IR, INPUT_PULLUP);
  pinMode(Right_IR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Left_IR), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(Right_IR), rightISR, RISING);

  pinMode(LED_BUILTIN, OUTPUT); // pickup indicator
  digitalWrite(LED_BUILTIN, LOW);
}

void setupWiFi()
{
  WiFi.setPins(8,7,4,2);

  WiFi.begin(ssid,pass);
  delay(10000);
  Serial.println("connected to wifi");
}

void setupPort()
{
  udp.begin(localPort);
  Serial.println("port setup");
}

void setupIMU()
{
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-5184, -2879, -2911};
  compass.m_max = (LSM303::vector<int16_t>){+1010, +4142, +4145};
  startingHeadingDeg = compass.heading();
  Serial.println("imu setup");
}

void pickup()
{
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("picked up!!!");
  analogWrite( Left_Motor_Forward, 0 );
  analogWrite( Left_Motor_Backward, 0 );
  analogWrite( Right_Motor_Forward, 0 );
  analogWrite( Right_Motor_Backward, 0 );
  while(true);
}

void updateIMU()
{
  //Serial.println("reading IMU");
  compass.read();
  currentHeadingDeg = compass.heading(); // +x axis
  currentHeadingRad = currentHeadingDeg * PI / 180.0;

  relativeHeadingDeg = currentHeadingDeg - startingHeadingDeg;
  relativeHeadingRad = relativeHeadingDeg * PI / 180.0;
  //Serial.print("currentheadingdeg: ");
  //Serial.println(currentHeadingDeg);
  //Serial.print("zeeee: ");
  //Serial.println(compass.a.z);
  if( compass.a.z > 999999 ) // resting is about -16000
    pickup();
  
}

void sendResponse()
{
  _rtnPacket.x = xGlobal;
  _rtnPacket.y = yGlobal;
  _rtnPacket.head = averageHeadingDeg;

  udp.beginPacket( udp.remoteIP(), udp.remotePort() );

  char trasmitBuffer[BUFFSIZE] = { 0 };

  memcpy( trasmitBuffer, &_rtnPacket, sizeof( rtnPacket ) );

  udp.write( trasmitBuffer, sizeof( rtnPacket ) );
  
  udp.endPacket();
}

void readPacket()
{
  int len = udp.read( receiveBuffer, BUFFSIZE );
  if ( len > 0 )
  {
    Serial.println("packet says: ");
    //Serial.print(receiveBuffer);
    memcpy( &_cmdPacket, receiveBuffer, sizeof( cmdPacket ) );
    /*
     * mode 0 = return x,y,head
     * mode 1 = driving command
     * mode 2 = cardinal command
     * mode 3 = stop
     */

    switch ( _cmdPacket.mode )
    {
      case 0:
        Serial.println( _cmdPacket.vel );
        Serial.println( _cmdPacket.phi );
        Serial.println("mode 0 - report");
        
        sendResponse();
        
        break;
      case 1:
        Serial.println( _cmdPacket.vel );
        Serial.println("mode 1 - drive");
        //parse driving command

        sendResponse();
        
        break;
      case 2:
        Serial.println( _cmdPacket.vel );
        Serial.println( _cmdPacket.phi );
        Serial.println("mode 2 - cardinal");
        //parse cardinal command

        sendResponse();
        
        break;
      case 3:
        Serial.println("mode 3");
        //stop
        pickup();
        break;
    }
  }
}

void checkUDP()
{
  int packetSize = udp.parsePacket();
  if(packetSize)
  {
    Serial.print( "Received packet of size " );
    Serial.println( packetSize );
    Serial.print( "From " );
    IPAddress remoteIp = udp.remoteIP();
    Serial.print( remoteIp );
    Serial.print( ", port " );
    Serial.println( udp.remotePort() );

    readPacket();
  }
}

void updateMotors()
{
  //leftMotorSet = ( ( 2 * velocityLeft ) - ( omega * BASELINE ) ) / ( 2 * WHEELRAD);
  //rightMotorSet = ( ( 2 * velocityRight ) + ( omega * BASELINE ) ) / ( 2 * WHEELRAD);

  if( _cmdPacket.mode == 2 )
  {
    if( !turnLeft )
    {
      rightMotorSet = ( abs( omega * BASELINE ) ) / ( 2 * WHEELRAD );
      leftMotorSet = 0;
    }
    else
    {
      rightMotorSet = 0;
      leftMotorSet = ( abs( omega * BASELINE ) ) / ( 2 * WHEELRAD );
    }
  }
  else
  {
    rightMotorSet = 2 * velocityRight;
    leftMotorSet = 2 * velocityLeft;
  }

  if( leftMotorSet > 255) leftMotorSet = 255;
  else if ( leftMotorSet < 0 ) leftMotorSet = 0;

  if( rightMotorSet > 255) rightMotorSet = 255;
  else if ( rightMotorSet < 0 ) rightMotorSet = 0;

  analogWrite( Left_Motor_Forward, leftMotorSet );
  analogWrite( Left_Motor_Backward, 0 );
  analogWrite( Right_Motor_Forward, rightMotorSet );
  analogWrite( Right_Motor_Backward, 0 );
}
