---
layout: post
title: ESP32Robot car
author: [Cheng]
category: [Lecture]
tags: [Wifi,BLE,car]
---
使用DRV8833 馬達驅動NodeMCU自走小車
---
## 系統方塊圖
![](https://raw.githubusercontent.com/chengx231/MCU-course/e3b51c2d2f2e180433c092bd1a142c773198a00c/images/PID%E5%B0%8F%E8%BB%8A.jpg)
---
## 硬體元件
* 18650電池x2(3.7V)+雙槽電池盒
* 穩壓IC LV7805
* 直流馬達驅動板 DRV8833
* NodeMCU自走車
* 焊錫、單芯線、開關、小車模組
* MPU6050
---
## 前進
<iframe width="449" height="798" src="https://www.youtube.com/embed/vw0Oo5jidaM" title="forwork" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

---

## 後退
<iframe width="449" height="798" src="https://www.youtube.com/embed/sw4PZcQTnek" title="back" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

---

## 向左
<iframe width="449" height="798" src="https://www.youtube.com/embed/8leGL-gKyak" title="left" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

---

## 向右
<iframe width="449" height="798" src="https://www.youtube.com/embed/fDQ0pliakaQ" title="right" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
---

# RoboCar_WebUI
## 程式碼(Arduino)
For Wifi controll,using DRV8833 + ESP32 +LV7805

```

  // PWM to DRV8833 dual H-bridge motor driver, PWM freq. = 1000 Hz
  // ESP32 Webserver to receive commands to control RoboCar
  
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESP32MotorControl.h>
   
  // DRV8833 pin connection
  #define IN1pin 16   
  #define IN2pin 17 
  #define IN3pin 18 
  #define IN4pin 19
  <br> 
  #define motorR 0
  #define motorL 1
  #define FULLSPEED 100
  #define HALFSPEED 50
  
  ESP32MotorControl motor;
   
  
  /* Set these to your desired credentials. */
  const char *ssid = "Your_SSID";
  const char *password = "Your_Password";<br> 
   
  WebServer server(80); // Set web server port number to 80
  
  const String HTTP_PAGE_HEAD = "<!DOCTYPE html><html lang=\"en\">\
  <head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>{v}</title>";
  const String HTTP_PAGE_STYLE = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;}\
  input{width:90%;}  body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.6rem;\
  background-color:#1fb3ec;color:#fdd;line-height:2.4rem;font-size:1.2rem;width:100%;} .q{float: right;\
  width: 64px;text-align: right;} .button1 {background-color: #4CAF50;} .button2 {background-color: #008CBA;}\
  .button3 {background-color: #f44336;} .button4 {background-color: #e7e7e7; color: black;} .button5 \
  {background-color: #555555;} </style>";
  const String HTTP_PAGE_SCRIPT = "<script>function c(l){document.getElementById('s').value=\
  l.innerText||l.textContent;document.getElementById('p').focus();}</script>"; 
  const String HTTP_PAGE_BODY= "</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>";
  const String HTTP_PAGE_FORM = "<form action=\"/cmd1\" method=\"get\"><button class=\"button1\">Forward</button>\
  </form></br><form action=\"/cmd2\" method=\"get\"><button class=\"button2\">Backward</button></form></br>\
  <form action=\"/cmd3\" method=\"get\"><button class=\"button3\">Right</button></form>\
  </br><form action=\"/cmd4\" method=\"get\"><button class=\"button4\">Left</button></form></br>\
  <form action=\"/cmd5\" method=\"get\"><button class=\"button5\">Stop</button></form></br></div>"; 
  const String HTTP_WEBPAGE = HTTP_PAGE_HEAD + HTTP_PAGE_STYLE + HTTP_PAGE_SCRIPT + HTTP_PAGE_BODY + HTTP_PAGE_FORM;
  const String HTTP_PAGE_END = "</div></body></html>";
  
  // Current time
  unsigned long currentTime = millis();
  // Previous time
  unsigned long previousTime = 0; 
  // Define timeout time in milliseconds (example: 2000ms = 2s)
  const long timeoutTime = 2000;
  int speed = HALFSPEED;
  
  void handleRoot() {
  String s  = HTTP_WEBPAGE;
  s += HTTP_PAGE_END;<br> 
  server.send(200, "text/html", s);
  } 
  
  void cmd1() { 
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s); 
  motor.motorForward(motorR, speed);  
  motor.motorForward(motorL, speed);
  Serial.println("Move Forward"); 
  }
  
  void cmd2() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, speed);
  motor.motorReverse(motorL, speed);
  Serial.println("Move Backward");   
  } 
   
  void cmd3() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, speed);  
  motor.motorForward(motorL, speed);
  Serial.println("Turn Right");
  
  void cmd4() {
  String s  = HTTP_WEBPAGE;
  s += HTTP_PAGE_END;
  server.send(200, "text/html", s);
  motor.motorForward(motorR, speed);
  motor.motorReverse(motorL, speed);
  Serial.println("Turn Left");
  }
  
  void cmd5() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorStop(motorR);
  motor.motorStop(motorL);
  Serial.println("Motor Stop");
  }

  void setup() { 
  Serial.begin(115200);
  Serial.println("Motor Pins assigned...");
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin); 
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to "); 
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  	delay(500);
  	Serial.print(".") 
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
   
  server.on("/", handleRoot);
  server.on("/cmd1", cmd1);
  server.on("/cmd2", cmd2);
  server.on("/cmd3", cmd3);
  server.on("/cmd4", cmd4);
  server.on("/cmd5", cmd5); 
  
  Serial.println("HTTP server started");
  server.begin(); 
  
  motor.motorStop(motorR);
  motor.motorStop(motorL);
  }
 
  void loop() {
  server.handleClient(); 
  }       

```


## 加上PID 控制 和 MPU6050 偵測座標 實現自動修正
### 使用Ziegler–Nichols方法控制參數。
* MPU6050 座標偵測
<iframe width="449" height="799" src="https://www.youtube.com/embed/4tvbFpI4C40" title="2023年6月4日" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

* 成果影片一
<iframe width="449" height="799" src="https://www.youtube.com/embed/9uHduNI7L98" title="2023年6月4日" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

* 成果影片二
<iframe width="449" height="799" src="https://www.youtube.com/embed/b1AzkakRuUk" title="2023年6月4日" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

---

### mine code

```
// PWM to DRV8833 dual H-bridge motor driver, PWM freq. = 1000 Hz
// ESP32 Webserver to receive commands to control RoboCar

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32MotorControl.h> 
#include <Wire.h>
//-------------------------------
//
// RoboCar with MPU6050 using PID control for going straight line
// by Richard Kuo, NTOU/EE
//
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// MPU6050 : Inertial Measurement Unit

//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
// DRV8833 pin connection
#define IN1pin 16  
#define IN2pin 17 
#define IN3pin 18 
#define IN4pin 19

#define motorR 0
#define motorL 1
#define FULLSPEED 100
#define HALFSPEED 50

MPU6050 mpu;
ESP32MotorControl motor;

//-------------------------------
//
// RoboCar with MPU6050 using PID control for going straight line
// by Richard Kuo, NTOU/EE
//
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// MPU6050 : Inertial Measurement Unit

//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

static float  preHeading, Heading, HeadingTgt;

// PID tuning method : Ziegler-Nichols method
const int Ku = 10;
const int Tu = 100;
const int Kp = 0.6 * Ku;
const int Ki = 1.2 * Ku / Tu;
const int Kd = 3 * Ku * Tu /40;

int USR_FullPower;
int USR_MotorPower;
int PID_FullPower;
int PID_MotorPower;

#define CMD_STOP     0
#define CMD_FORWARD  1
#define CMD_BACKWARD 2
#define CMD_RIGHT    3
#define CMD_LEFT     4
int command;
int angle;
    


// value 1 or -1 for motor spining default
const int offsetA = 1;
const int offsetB = 1;


// Interrup Service Routine (ISR)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



/* Set these to your desired credentials. */
const char *ssid = "jk";
const char *password = "12345678";

WebServer server(80); // Set web server port number to 80

const String HTTP_PAGE_HEAD = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>{v}</title>";
const String HTTP_PAGE_STYLE = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;}  input{width:90%;}  body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.6rem;background-color:#1fb3ec;color:#fdd;line-height:2.4rem;font-size:1.2rem;width:100%;} .q{float: right;width: 64px;text-align: right;} .button1 {background-color: #4CAF50;} .button2 {background-color: #008CBA;} .button3 {background-color: #f44336;} .button4 {background-color: #e7e7e7; color: black;} .button5 {background-color: #555555;} </style>";
const String HTTP_PAGE_SCRIPT = "<script>function c(l){document.getElementById('s').value=l.innerText||l.textContent;document.getElementById('p').focus();}</script>";
const String HTTP_PAGE_BODY= "</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>";
const String HTTP_PAGE_FORM = "<form action=\"/cmd1\" method=\"get\"><button class=\"button1\">Forward</button></form></br><form action=\"/cmd2\" method=\"get\"><button class=\"button2\">Backward</button></form></br><form action=\"/cmd3\" method=\"get\"><button class=\"button3\">Right</button></form></br><form action=\"/cmd4\" method=\"get\"><button class=\"button4\">Left</button></form></br><form action=\"/cmd5\" method=\"get\"><button class=\"button5\">Stop</button></form></br></div>";
const String HTTP_WEBPAGE = HTTP_PAGE_HEAD + HTTP_PAGE_STYLE + HTTP_PAGE_SCRIPT + HTTP_PAGE_BODY + HTTP_PAGE_FORM;
const String HTTP_PAGE_END = "</div></body></html>";

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

int speed = HALFSPEED;

void handleRoot() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;
  server.send(200, "text/html", s);
}

void cmd1() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorForward(motorR, speed);  
  motor.motorForward(motorL, speed);
  Serial.println("Move Forward");   
}

void cmd2() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, speed);
  motor.motorReverse(motorL, speed);
  Serial.println("Move Backward");     
}

void cmd3() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, speed);  
  motor.motorForward(motorL, speed);
  Serial.println("Turn Right");    
}

void cmd4() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorForward(motorR, speed);
  motor.motorReverse(motorL, speed);
  Serial.println("Turn Left"); 
}

void cmd5() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorStop(motorR);
  motor.motorStop(motorL);
  Serial.println("Motor Stop");
}


void setup() {
  Wire.begin();
  Wire.setClock(400000);
    
  Serial.begin(115200);
  Serial.println("NodeMCU RoboCar with IMU");
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin);
  motor.motorStop(motorR);
  motor.motorStop(motorL);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  // initialize device
  Serial.println(F("Initializing I2C devices...f="));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  /*while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // Note - use the 'raw' program to get these.  
  // Expect an unreliable or long startup if you don't bother!!! 
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //  read heading till it is stable
  for (int i=0;i<200;i++) {
      GetHeading(&Heading); 
      delay(100);
  }

  // set command & angle for moving RoboCar
  command = CMD_FORWARD; // CMD_RIGHT
  angle = 0;             // +60

  switch(command) {
    case CMD_STOP:
      USR_FullPower = 0;
      PID_FullPower = 0;
      break;    
    case CMD_FORWARD:
      USR_FullPower = FULLSPEED * 3/4;
      PID_FullPower = FULLSPEED- USR_FullPower;
      break;
    case CMD_BACKWARD:
      USR_FullPower = FULLSPEED * 3/4;
      PID_FullPower = FULLSPEED - USR_FullPower;
      break;
    case CMD_RIGHT:
      USR_FullPower = FULLSPEED * 1/4;
      PID_FullPower = FULLSPEED - USR_FullPower;
      break;
    case CMD_LEFT:
      USR_FullPower = FULLSPEED * 1/4;
      PID_FullPower = FULLSPEED - USR_FullPower;
      break;
    default:
      USR_FullPower = 0;
      PID_FullPower = 0;    
      break;
  }


  // set target heading to default heading
  GetHeading(&Heading); 
  HeadingTgt = Heading + angle;
  if (HeadingTgt>=360) HeadingTgt = HeadingTgt - 360;
  else if (HeadingTgt<0) HeadingTgt = HeadingTgt + 360;
  Serial.print("Heading Target = \t");
  Serial.println(HeadingTgt);
  //--------------------------------
  Serial.begin(115200);
  Serial.println("Motor Pins assigned...");
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/cmd1", cmd1);
  server.on("/cmd2", cmd2);
  server.on("/cmd3", cmd3);
  server.on("/cmd4", cmd4);  
  server.on("/cmd5", cmd5);  

  Serial.println("HTTP server started");
  server.begin();

  motor.motorStop(motorR);
  motor.motorStop(motorL);
}

void loop() {
  //-----------------------
  const int Moving = 1; 
  
  if (!dmpReady) return;

  // NOT USING MPU6050 INT pin
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) {
  //} // 100Hz Fast Loop

  GetHeading(&Heading);
  Serial.print("Yaw:\t");
  Serial.print(Heading);
  Serial.print("\t");
  Serial.println(HeadingTgt);
  
  PID(Heading,HeadingTgt,&PID_MotorPower, Kp, Ki , Kd, Moving);

  USR_MotorPower = USR_FullPower; // assign User defined full power 
  Serial.print("Power:\t"); ;  
  Serial.print(USR_MotorPower);
  Serial.print("\t");   
  Serial.println(PID_MotorPower);
  
  if (Heading==HeadingTgt) PID_MotorPower = 0;
  switch (command) {
    case CMD_STOP:
        Serial.println("Move STOP");
        motor.motorStop(motorR);
        motor.motorStop(motorL);
      break;
    case CMD_FORWARD:
        motor.motorForward(motorR, USR_MotorPower - PID_MotorPower);  
        motor.motorForward(motorL, USR_MotorPower + PID_MotorPower);
        Serial.println("Move Forward");   
      break;
    case CMD_BACKWARD:
      motor.motorReverse(motorR,-USR_MotorPower - PID_MotorPower);
      motor.motorReverse(motorL, -USR_MotorPower + PID_MotorPower);
      Serial.println("Move Backward");  
      break;
    case CMD_RIGHT:
      motor.motorReverse(motorR, USR_MotorPower - PID_MotorPower);  
      motor.motorForward(motorL, -USR_MotorPower + PID_MotorPower);
      Serial.println("Turn Right"); 
      break;
    case CMD_LEFT:
      motor.motorForward(motorR, -USR_MotorPower + PID_MotorPower);
      motor.motorReverse(motorL, USR_MotorPower - PID_MotorPower);
       Serial.println("Turn Left"); 
      break;
    default:
      motor.motorForward(motorR, USR_FullPower - PID_MotorPower);
      motor.motorForward(motorL, USR_FullPower + PID_MotorPower);
      break;
  }
  //-----------------------
  server.handleClient();
  
}



void  GetHeading(float *Heading)                                                                                                                                                   
{
  //calc heading from IMU
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
          
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    *Heading = int((ypr[0] * 180/M_PI)) + 180;    
  }//done
}//END GetHeading

void PID(float Heading,float HeadingTarget,int *Power, float kP,float kI,float kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr,error ; 

  // IF not moving then 
  if(!Moving)
  {
        errSum = 0;
        lastErr = 0;
        return;
  }

  //error correction for angular overlap
  error = Heading-HeadingTarget;
  if(error<180)
    error += 360;
  if(error>180)
    error -= 360;
      
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -300, 300);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  *Power = kP * error + kI * errSum + kD * dErr;
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitInt(Power, - PID_FullPower,  PID_FullPower);

}//END getPID

void LimitInt(int *x,int Min, int Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt

//
// Clamp a float between a min and max.  Note doubles are the same 
// as floats on this platform.

void LimitFloat(float *x,float Min, float Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt
```

### 組員一的程式碼(01053122)
```

 RoboCar with MPU6050 using PID control for going straight line
 by Richard Kuo, NTOUEE

#include ESP32MotorControl.h 
#include Wire.h
#include MPU6050_6Axis_MotionApps20.h

 MPU6050  Inertial Measurement Unit
MPU6050 mpu;

#define IN1pin 16  
#define IN2pin 17  
#define IN3pin 18 
#define IN4pin 19

#define motorR 0
#define motorL 1

#define PWM_FULLPOWER  100
ESP32MotorControl motor;
MPU6050 mpu(0x69);  -- use for AD0 high

 MPU controlstatus vars
bool dmpReady = false;   set true if DMP init was successful
uint8_t mpuIntStatus;    holds actual interrupt status byte from MPU
uint8_t devStatus;       return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  FIFO storage buffer

 orientationmotion vars
Quaternion q;            [w, x, y, z]         quaternion container
VectorInt16 aa;          [x, y, z]            accel sensor measurements
VectorInt16 aaReal;      [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;     [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;     [x, y, z]            gravity vector
float euler[3];          [psi, theta, phi]    Euler angle container
float ypr[3];            [yaw, pitch, roll]   yawpitchroll container and gravity vector

 packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, 'r', 'n' };

static float  preHeading, Heading, HeadingTgt;

 PID tuning method  Ziegler-Nichols method
const int Ku = 10;
const int Tu = 100;
const int Kp = 0.6  Ku;
const int Ki = 1.2  Ku  Tu;
const int Kd = 3  Ku  Tu 40;

 PWM freq  NodeMCU = 1KHz, UNO = 500Hz
 PWM duty   NodeMCU = 1023 (10-bit PWM), UNO = 255 (8-bit PWM)

int USR_FullPower;
int USR_MotorPower;
int PID_FullPower;
int PID_MotorPower;

#define CMD_STOP     0
#define CMD_FORWARD  1
#define CMD_BACKWARD 2
#define CMD_RIGHT    3
#define CMD_LEFT     4
int command;
int angle;
    
 TB6612FNG  Full-Bridge DC Motor Driver


 Interrup Service Routine (ISR)
volatile bool mpuInterrupt = false;      indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {  
  Wire.begin();
  Wire.setClock(400000);
    
  Serial.begin(115200);
  Serial.println(NodeMCU RoboCar with IMU);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
   initialize device
  Serial.println(F(Initializing I2C devices...f=));
  mpu.initialize();

   verify connection
  Serial.println(F(Testing device connections...));
  Serial.println(mpu.testConnection()  F(MPU6050 connection successful)  F(MPU6050 connection failed));

   wait for ready
  Serial.println(F(nSend any character to begin DMP programming and demo ));
  while (Serial.available() && Serial.read());  empty buffer
  while (!Serial.available());                  wait for data
  while (Serial.available() && Serial.read());  empty buffer again

   load and configure the DMP
  Serial.println(F(Initializing DMP...));
  devStatus = mpu.dmpInitialize();

   supply your own gyro offsets here, scaled for min sensitivity
   Note - use the 'raw' program to get these.  
   Expect an unreliable or long startup if you don't bother!!! 
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
    
   make sure it worked (returns 0 if so)
  if (devStatus == 0) {
     turn on the DMP, now that it's ready
    Serial.println(F(Enabling DMP...));
    mpu.setDMPEnabled(true);

     enable Arduino interrupt detection
    Serial.println(F(Enabling interrupt detection (Arduino external interrupt 0)...));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

     set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F(DMP ready! Waiting for first interrupt...));
    dmpReady = true;

     get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
     ERROR!
     1 = initial memory load failed
     2 = DMP configuration updates failed
     (if it's going to break, usually the code will be 1)
    Serial.print(F(DMP Initialization failed (code ));
    Serial.print(devStatus);
    Serial.println(F()));
  }

    read heading till it is stable
  for (int i=0;i200;i++) {
      GetHeading(&Heading); 
      delay(100);
  }

   set command & angle for moving RoboCar
  command = CMD_FORWARD;  CMD_RIGHT
  angle = 0;              +60

  switch(command) {
    case CMD_STOP
      USR_FullPower = 0;
      PID_FullPower = 0;
      break;    
    case CMD_FORWARD
      USR_FullPower = PWM_FULLPOWER  34;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      
      break;
    case CMD_BACKWARD
      USR_FullPower = PWM_FULLPOWER  34;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    case CMD_RIGHT
      USR_FullPower = PWM_FULLPOWER  14;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    case CMD_LEFT
      USR_FullPower = PWM_FULLPOWER  14;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    default
      USR_FullPower = 0;
      PID_FullPower = 0;    
      break;
  }


   set target heading to default heading
  GetHeading(&Heading); 
  HeadingTgt = Heading + angle;
  if (HeadingTgt=360) HeadingTgt = HeadingTgt - 360;
  else if (HeadingTgt0) HeadingTgt = HeadingTgt + 360;
  Serial.print(Heading Target = t);
  Serial.println(HeadingTgt);
}

void loop() { 
  const int Moving = 1; 
  
  if (!dmpReady) return;

   NOT USING MPU6050 INT pin
   wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount  packetSize) {
  }  100Hz Fast Loop

  GetHeading(&Heading);
  Serial.print(Yawt);
  Serial.print(Heading);
  Serial.print(t);
  Serial.println(HeadingTgt);
  
  PID(Heading,HeadingTgt,&PID_MotorPower, Kp, Ki , Kd, Moving);

  USR_MotorPower = USR_FullPower;  assign User defined full power 
  Serial.print(Powert); ;  
  Serial.print(USR_MotorPower);
  Serial.print(t);   
  Serial.println(PID_MotorPower);
  
  if (Heading==HeadingTgt) PID_MotorPower = 0;
  switch (command) {
    case CMD_STOP
       motor.motorStop(motorR);  
       motor.motorStop(motorL);
      break;
    case CMD_FORWARD
      motor.motorForward(motorR, USR_MotorPower - PID_MotorPower);  
      motor.motorForward(motorL, USR_MotorPower + PID_MotorPower);
      break;
    case CMD_BACKWARD
      motor.motorForward(motorR, -USR_MotorPower - PID_MotorPower);  
      motor.motorForward(motorL, -USR_MotorPower + PID_MotorPower);
      break;
    case CMD_RIGHT
      motor.motorForward(motorR, USR_MotorPower - PID_MotorPower);  
      motor.motorForward(motorL, -USR_MotorPower + PID_MotorPower);
      break;
    case CMD_LEFT
      motor.motorForward(motorR, -USR_MotorPower + PID_MotorPower);  
      motor.motorForward(motorL, USR_MotorPower - PID_MotorPower);
      break;
    default
      motor.motorForward(motorR, USR_FullPower - PID_MotorPower);  
      motor.motorForward(motorL, USR_FullPower + PID_MotorPower);
      break;
  }
}

void  GetHeading(float Heading)                                                                                                                                                   
{
  calc heading from IMU
   reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

   get current FIFO count
  fifoCount = mpu.getFIFOCount();

   check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10)  fifoCount == 1024) {
     reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F(FIFO overflow!));

     otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
     wait for correct available data length, should be a VERY short wait
    while (fifoCount  packetSize) fifoCount = mpu.getFIFOCount();

     read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
     track FIFO count here in case there is  1 packet available
     (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
          
     display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Heading = int((ypr[0]  180M_PI)) + 180;    
  }done
}END GetHeading

void PID(float Heading,float HeadingTarget,int Power, float kP,float kI,float kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr,error ; 

   IF not moving then 
  if(!Moving)
  {
        errSum = 0;
        lastErr = 0;
        return;
  }

  error correction for angular overlap
  error = Heading-HeadingTarget;
  if(error180)
    error += 360;
  if(error180)
    error -= 360;
      
  httpbrettbeauregard.comblog201104improving-the-beginners-pid-introduction

  How long since we last calculated
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  Compute all the working error variables
  float error = Setpoint - Input;    
  errSum += (error  timeChange);   

  integral windup guard
  LimitFloat(&errSum, -300, 300);

  float dErr = (error - lastErr)  timeChange;       

  Compute PID Output
  Power = kP  error + kI  errSum + kD  dErr;
  Remember some variables for next time
  lastErr = error;    
  lastTime = now; 

  limit demand 
  LimitInt(Power, - PID_FullPower,  PID_FullPower);

}END getPID

void LimitInt(int x,int Min, int Max)
{
  if(x  Max)
    x = Max;
  if(x  Min)
    x = Min;

}END LimitInt


 Clamp a float between a min and max.  Note doubles are the same 
 as floats on this platform.

void LimitFloat(float x,float Min, float Max)
{
  if(x  Max)
    x = Max;
  if(x  Min)
    x = Min;

}END LimitInt
```

### 組員二的程式碼(01053119)

```
// EPS32 PWM to DRV8833 dual H-bridge motor driver, PWM freq. = 1000 Hz
#include <ESP32MotorControl.h> 
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <WiFi.h>
#include <WebServer.h>
// DRV8833 pin connection
MPU6050 mpu;
ESP32MotorControl motor;
#define IN1pin 16  
#define IN2pin 17  
#define IN3pin 18 
#define IN4pin 19

#define motorR 0
#define motorL 1

#define FULLSPEED 100
#define HALFSPEED 50
const char *ssid = "01053119";
const char *password = "s372148295";

WebServer server(80); // Set web server port number to 80
const String HTTP_PAGE_HEAD = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>{v}</title>";
const String HTTP_PAGE_STYLE = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;}  input{width:90%;}  body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.6rem;background-color:#1fb3ec;color:#fdd;line-height:2.4rem;font-size:1.2rem;width:100%;} .q{float: right;width: 64px;text-align: right;} .button1 {background-color: #4CAF50;} .button2 {background-color: #008CBA;} .button3 {background-color: #f44336;} .button4 {background-color: #e7e7e7; color: black;} .button5 {background-color: #555555;} </style>";
const String HTTP_PAGE_SCRIPT = "<script>function c(l){document.getElementById('s').value=l.innerText||l.textContent;document.getElementById('p').focus();}</script>";
const String HTTP_PAGE_BODY= "</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>";
const String HTTP_PAGE_FORM = "<form action=\"/cmd1\" method=\"get\"><button class=\"button1\">Forward</button></form></br><form action=\"/cmd2\" method=\"get\"><button class=\"button2\">Backward</button></form></br><form action=\"/cmd3\" method=\"get\"><button class=\"button3\">Right</button></form></br><form action=\"/cmd4\" method=\"get\"><button class=\"button4\">Left</button></form></br><form action=\"/cmd5\" method=\"get\"><button class=\"button5\">Stop</button></form></br></div>";
const String HTTP_WEBPAGE = HTTP_PAGE_HEAD + HTTP_PAGE_STYLE + HTTP_PAGE_SCRIPT + HTTP_PAGE_BODY + HTTP_PAGE_FORM;
const String HTTP_PAGE_END = "</div></body></html>";

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

static float  preHeading, Heading, HeadingTgt;

// PID tuning method : Ziegler-Nichols method
const int Ku = 10;
const int Tu = 100;
const int Kp = 0.6 * Ku;
const int Ki = 1.2 * Ku / Tu;
const int Kd = 3 * Ku * Tu /40;

#define PWM_FULLPOWER  100
int USR_FullPower;
int USR_MotorPower;
int PID_FullPower;
int PID_MotorPower;

#define CMD_STOP     0
#define CMD_FORWARD  1
#define CMD_BACKWARD 2
#define CMD_RIGHT    3
#define CMD_LEFT     4
int command;
int angle;
    
// TB6612FNG : Full-Bridge DC Motor Driver
#define STBY D0
#define PWMA D3
#define AIN2 D4
#define AIN1 D5
#define BIN1 D6
#define BIN2 D7
#define PWMB D8

// value 1 or -1 for motor spining default
const int offsetA = 1;
const int offsetB = 1;

// Interrup Service Routine (ISR)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int Speed = HALFSPEED;

void handleRoot() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;
  server.send(200, "text/html", s);
}

void cmd1() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorForward(motorR, Speed);  
  motor.motorForward(motorL, Speed);
  Serial.println("Move Forward");   
}

void cmd2() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, Speed);
  motor.motorReverse(motorL, Speed);
  Serial.println("Move Backward");     
}

void cmd3() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorReverse(motorR, Speed);  
  motor.motorForward(motorL, Speed);
  Serial.println("Turn Right");    
}

void cmd4() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorForward(motorR, Speed);
  motor.motorReverse(motorL, Speed);
  Serial.println("Turn Left"); 
}

void cmd5() {
  String s  = HTTP_WEBPAGE; 
  s += HTTP_PAGE_END;  
  server.send(200, "text/html", s);
  motor.motorStop(motorR);
  motor.motorStop(motorL);
  Serial.println("Motor Stop");
}
void setup() {
  Serial.begin(115200);
  Serial.println("Motor Pins assigned...");
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin);

  motor.motorStop(motorR);
  motor.motorStop(motorL);
   Wire.begin();
  Wire.setClock(400000);
    
  Serial.println("NodeMCU RoboCar with IMU");

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  // initialize device
  Serial.println(F("Initializing I2C devices...f="));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  /*while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // Note - use the 'raw' program to get these.  
  // Expect an unreliable or long startup if you don't bother!!! 
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //  read heading till it is stable
  for (int i=0;i<200;i++) {
      GetHeading(&Heading); 
      delay(100);
  }

  // set command & angle for moving RoboCar
  command = CMD_FORWARD; // CMD_RIGHT
  angle = 0;             // +60

  switch(command) {
    case CMD_STOP:
      USR_FullPower = 0;
      PID_FullPower = 0;
      break;    
    case CMD_FORWARD:
      USR_FullPower = PWM_FULLPOWER * 3/4;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    case CMD_BACKWARD:
      USR_FullPower = PWM_FULLPOWER * 3/4;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    case CMD_RIGHT:
      USR_FullPower = PWM_FULLPOWER * 1/4;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    case CMD_LEFT:
      USR_FullPower = PWM_FULLPOWER * 1/4;
      PID_FullPower = PWM_FULLPOWER - USR_FullPower;
      break;
    default:
      USR_FullPower = 0;
      PID_FullPower = 0;    
      break;
  }


  // set target heading to default heading
  GetHeading(&Heading); 
  HeadingTgt = Heading + angle;
  if (HeadingTgt>=360) HeadingTgt = HeadingTgt - 360;
  else if (HeadingTgt<0) HeadingTgt = HeadingTgt + 360;
  Serial.print("Heading Target = \t");
  Serial.println(HeadingTgt);
  Serial.println("Motor Pins assigned...");
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/cmd1", cmd1);
  server.on("/cmd2", cmd2);
  server.on("/cmd3", cmd3);
  server.on("/cmd4", cmd4);  
  server.on("/cmd5", cmd5);  

  Serial.println("HTTP server started");
  server.begin();

  motor.motorStop(motorR);
  motor.motorStop(motorL);
}

void loop() {
  server.handleClient();
  Serial.println("Move Forward");
  motor.motorForward(motorR, Speed);  
  motor.motorForward(motorL, Speed);
  delay(1000); // 1 sec

  Serial.println("Move Stop");  
  motor.motorStop(motorR);  
  motor.motorStop(motorL);
  delay(1000); // 1 sec
   const int Moving = 1; 
  
  if (!dmpReady) return;

  // NOT USING MPU6050 INT pin
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) {
  //} // 100Hz Fast Loop

  GetHeading(&Heading);
  Serial.print("Yaw:\t");
  Serial.print(Heading);
  Serial.print("\t");
  Serial.println(HeadingTgt);
  
  PID(Heading,HeadingTgt,&PID_MotorPower, Kp, Ki , Kd, Moving);

  USR_MotorPower = USR_FullPower; // assign User defined full power 
  Serial.print("Power:\t"); ;  
  Serial.print(USR_MotorPower);
  Serial.print("\t");   
  Serial.println(PID_MotorPower);
  
  if (Heading==HeadingTgt) PID_MotorPower = 0;
  switch (command) {
    case CMD_STOP:
      motor.motorForward(motorR,USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,USR_MotorPower + PID_MotorPower);
      break;
    case CMD_FORWARD:
      motor.motorForward(motorR,USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,USR_MotorPower + PID_MotorPower);
      break;
    case CMD_BACKWARD:
      motor.motorForward(motorR,-USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,-USR_MotorPower + PID_MotorPower);
      break;
    case CMD_RIGHT:
      motor.motorForward(motorR,USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,-USR_MotorPower + PID_MotorPower);
      break;
    case CMD_LEFT:
      motor.motorForward(motorR,-USR_MotorPower + PID_MotorPower);
      motor.motorForward(motorL,USR_MotorPower - PID_MotorPower);
      break;
    default:
      motor.motorForward(motorR,USR_FullPower - PID_MotorPower);
      motor.motorForward(motorL,USR_FullPower + PID_MotorPower);
      break;
  }
}

void  GetHeading(float *Heading)                                                                                                                                                   
{
  //calc heading from IMU
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
          
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    *Heading = int((ypr[0] * 180/M_PI)) + 180;    
  }//done
}//END GetHeading

void PID(float Heading,float HeadingTarget,int *Power, float kP,float kI,float kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr,error ; 

  // IF not moving then 
  if(!Moving)
  {
        errSum = 0;
        lastErr = 0;
        return;
  }

  //error correction for angular overlap
  error = Heading-HeadingTarget;
  if(error<180)
    error += 360;
  if(error>180)
    error -= 360;
      
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -300, 300);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  *Power = kP * error + kI * errSum + kD * dErr;
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitInt(Power, - PID_FullPower,  PID_FullPower);

}//END getPID

void LimitInt(int *x,int Min, int Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt

//
// Clamp a float between a min and max.  Note doubles are the same 
// as floats on this platform.

void LimitFloat(float *x,float Min, float Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}
```

### 組員三的程式碼

```

#include <Wire.h>
#include <ESP32MotorControl.h>
#include <MPU6050_6Axis_MotionApps20.h>

// MPU6050 : Inertial Measurement Unit
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define IN1pin 16  
#define IN2pin 17  
#define IN3pin 18 
#define IN4pin 19
#define FULLSPEED 100
#define motorR 0
#define motorL 1

ESP32MotorControl motor;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

static float  preHeading, Heading, HeadingTgt;

// PID tuning method : Ziegler-Nichols method
const int Ku = 10;
const int Tu = 100;
const int Kp = 0.6 * Ku;
const int Ki = 1.2 * Ku / Tu;
const int Kd = 3 * Ku * Tu /40;

// PWM freq : NodeMCU = 1KHz, UNO = 500Hz
// PWM duty   NodeMCU = 1023 (10-bit PWM), UNO = 255 (8-bit PWM)
#define PWM_FULLPOWER  1023
int USR_FullPower;
int USR_MotorPower;
int PID_FullPower;
int PID_MotorPower;

#define CMD_STOP     0
#define CMD_FORWARD  1
#define CMD_BACKWARD 2
#define CMD_RIGHT    3
#define CMD_LEFT     4
int command;
int angle;
 

// Interrup Service Routine (ISR)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {  
  Wire.begin();
  Wire.setClock(400000);
    
  Serial.begin(115200);
  Serial.println("NodeMCU RoboCar with IMU");

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  // initialize device
  Serial.println(F("Initializing I2C devices...f="));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  /*while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // Note - use the 'raw' program to get these.  
  // Expect an unreliable or long startup if you don't bother!!! 
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //  read heading till it is stable
  for (int i=0;i<200;i++) {
      GetHeading(&Heading); 
      delay(100);
  }

  // set command & angle for moving RoboCar
  command = CMD_FORWARD; // CMD_RIGHT
  angle = 0;             // +60

  switch(command) {
    case CMD_STOP:
      USR_FullPower = 0;
      PID_FullPower = 0;
      break;    
    case CMD_FORWARD:
      USR_FullPower = (PWM_FULLPOWER * 3/4)*100/1024;
      PID_FullPower = (PWM_FULLPOWER - USR_FullPower)*100/1024;
      break;
    case CMD_BACKWARD:
      USR_FullPower = (PWM_FULLPOWER * 3/4)*100/1024;
      PID_FullPower = (PWM_FULLPOWER - USR_FullPower)*100/1024;
      break;
    case CMD_RIGHT:
      USR_FullPower = (PWM_FULLPOWER * 1/4)*100/1024;
      PID_FullPower = (PWM_FULLPOWER - USR_FullPower)*100/1024;
      break;
    case CMD_LEFT:
      USR_FullPower = (PWM_FULLPOWER * 1/4)*100/1024;
      PID_FullPower = (PWM_FULLPOWER - USR_FullPower)*100/1024;
      break;
    default:
      USR_FullPower = 0;
      PID_FullPower = 0;    
      break;
  }
  motor.motorStop(motorR);
  motor.motorStop(motorL);

  // set target heading to default heading
  GetHeading(&Heading); 
  HeadingTgt = Heading + angle;
  if (HeadingTgt>=360) HeadingTgt = HeadingTgt - 360;
  else if (HeadingTgt<0) HeadingTgt = HeadingTgt + 360;
  Serial.print("Heading Target = \t");
  Serial.println(HeadingTgt);
}

void loop() { 
  const int Moving = 1; 
  
  if (!dmpReady) return;

  // NOT USING MPU6050 INT pin
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) {
  //} // 100Hz Fast Loop

  GetHeading(&Heading);
  Serial.print("Yaw:\t");
  Serial.print(Heading);
  Serial.print("\t");
  Serial.println(HeadingTgt);
  
  PID(Heading,HeadingTgt,&PID_MotorPower, Kp, Ki , Kd, Moving);

  USR_MotorPower = USR_FullPower* 100 / 1024; // assign User defined full power 
  Serial.print("Power:\t"); ;  
  Serial.print(USR_MotorPower);
  Serial.print("\t");   
  Serial.println(PID_MotorPower);
  
  if (Heading==HeadingTgt) PID_MotorPower = 0;
  switch (command) {
    case CMD_STOP:
      motor.motorForward(motorR,USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,USR_MotorPower + PID_MotorPower);
      break;
    case CMD_FORWARD:
      motor.motorForward(motorR,USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,USR_MotorPower + PID_MotorPower);
      break;
    case CMD_BACKWARD:
      motor.motorForward(motorR,-USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,-USR_MotorPower + PID_MotorPower);
      break;
    case CMD_RIGHT:
      motor.motorForward(motorR, USR_MotorPower - PID_MotorPower);
      motor.motorForward(motorL,-USR_MotorPower + PID_MotorPower);
      break;
    case CMD_LEFT:
      motor.motorForward(motorR,-USR_MotorPower + PID_MotorPower);
      motor.motorForward(motorL, USR_MotorPower - PID_MotorPower);
      break;
    default:
      motor.motorForward(motorR,USR_FullPower - PID_MotorPower);
      motor.motorForward(motorL,USR_FullPower + PID_MotorPower);
      break;
  }
}

void  GetHeading(float *Heading)                                                                                                                                                   
{
  //calc heading from IMU
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
          
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    *Heading = int((ypr[0] * 180/M_PI)) + 180;    
  }//done
}//END GetHeading

void PID(float Heading,float HeadingTarget,int *Power, float kP,float kI,float kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr,error ; 

  // IF not moving then 
  if(!Moving)
  {
        errSum = 0;
        lastErr = 0;
        return;
  }

  //error correction for angular overlap
  error = Heading-HeadingTarget;
  if(error<180)
    error += 360;
  if(error>180)
    error -= 360;
      
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -300, 300);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  *Power = kP * error + kI * errSum + kD * dErr;
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitInt(Power, - PID_FullPower,  PID_FullPower);

}//END getPID

void LimitInt(int *x,int Min, int Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt

//
// Clamp a float between a min and max.  Note doubles are the same 
// as floats on this platform.

void LimitFloat(float *x,float Min, float Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt
```






