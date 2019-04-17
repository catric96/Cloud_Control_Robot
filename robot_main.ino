#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 
#define LEFT_IR 12
#define RIGHT_IR 11
#define LEFT_MOTOR 10
#define RIGHT_MOTOR 9
#define BASELINE 80
#define RADIUS 25
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 1, 23);

struct velTheta{
  double velocity;
  double theta;
};
velTheta mainData = {0.0 , 0.0};
WiFiClient client;

float circumference = radius * TWO_PI;
float tickDistance = radius * deg2rad( 90 / 75.8 );
float thetaZ = 0, _thetaZ = tickDistance / baseline;
float x = 0, _x = ( BASELINE / 2 ) * sin ( _thetaZ ), _x0;
float y = 0, _y = ( BASELINE / 2 ) * ( 1 - cos ( _thetaZ ) ), _y0;

void setup() {
  // put your setup code here, to run once:
    accessPoint();
    openPort();
    setIR();
    imuSetup();
    motorSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
   * ax, ay, az = getIMU()
   * a = checkUDP() non blocking
   * velocity, theta = parse(a)
   * setSpeed(velocity)
   * setDir(Theta)
   * ML, MR <-- make globals
   * If(pickedUp) ML = MR = 0 ;; check az
   * setMotor(ML, MR)
   */
   
}

void accessPoint(){
  /*
   * setup microcontroller to connect to an AP
   */
     //Initialize serial and wait for port to open:
  WiFi.setPins(8,7,4,2);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
}

void openPort(){
  /*
   * open port of uc for sbc?
   */
}

//void setupDebug(Serial input){
  /*
   * yeet IDK
   */
//}

void imuSetup(){//sensor blah as input
  /*
   * set up imu and create a global for it?
   */
}

void setIR(){
  //set IR as input pullup
  //set interrupt to IR
  pinMode(LEFT_IR, INPUT_PULLUP);
  pinMode(RIGHT_IR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_IR), checkLeft, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RIGHT_IR), checkRight, CHANGE); 
}

void motorSetup(){
  //set pin modes
  //initialize speed to zero
  //set direction, one wheel forward other backwards due to setup
  pinMode(LEFT_MOTOR, OUTPUT); //left motor
  pinMode(RIGHT_MOTOR, OUTPUT); //right motor
}

void initStruct(){
  //motor value
  //udp buffer input
  //odometry
}
 /*
  * no need to do IMU yet, need more info
  */
int checkIMU(){
  /*
   * check imu for updates, return 1 if active 0 if not
   */
}

int[] getIMU(){
  /*
   * return ax, ay, az
   */
}

void checkUDP(){
  /*
   * check if UDP message has been sent
   * make sure non-blocking
   */
}

void checkLeft(){
  thetaZ += _thetaZ;
  _x0 = ( ( _x * cos( _thetaZ ) ) + ( _y * sin( _thetaZ ) ) );
  _y0 = ( ( _x * sin( _thetaZ ) ) + ( _y * cos( _thetaZ ) ) );
  y -= _y0;
  x += _x0;
}

void checkRight(){
  thetaZ -= _thetaZ;
  _x0 = ( ( _x * cos( _thetaZ ) ) + ( _y * sin( _thetaZ ) ) );
  _y0 = ( ( _x * sin( _thetaZ ) ) + ( _y * cos( _thetaZ ) ) );
  y += _y0;
  x += _x0;
}
