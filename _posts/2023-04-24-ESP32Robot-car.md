---
layout: post
title: ESP32Robot car
author: [Richard Kuo]
category: [Lecture]
tags: [Wifi,BLE,car]
---
# WiFi遙控機器人
---
## 系統方塊圖
![](https://raw.githubusercontent.com/chengx231/MCU-course/1e845e4b44920158de33d51028f84ca12904bd1c/images/Robot_CAR.png)
---
## 硬體元件
* 18650電池x2(3.7V)+雙槽電池盒
* 穩壓IC LV7805
* 直流驅動馬達 DRV8833
* NodeMCU自走車
---
 * 程式碼(Arduino)
```
<p>
  // PWM to DRV8833 dual H-bridge motor driver, PWM freq. = 1000 Hz<br><br> 
  // ESP32 Webserver to receive commands to control RoboCar<br> <br> 
  <br> 
  #include <WiFi.h><br> 
  #include <WebServer.h><br> 
  #include <ESP32MotorControl.h><br>  
  <br> 
  // DRV8833 pin connection<br> 
  #define IN1pin 16  <br> 
  #define IN2pin 17 <br> 
  #define IN3pin 18 <br> 
  #define IN4pin 19<br> 
  <br> 
  #define motorR 0<br> 
  #define motorL 1<br> 
  #define FULLSPEED 100<br> 
  #define HALFSPEED 50<br> 
  <br> 
  ESP32MotorControl motor;<br> 
  <br> 
  
  /* Set these to your desired credentials. */<br> 
  const char *ssid = "Your_SSID";<br> 
  const char *password = "Your_Password";<br> <br> 
  <br> 
  WebServer server(80); // Set web server port number to 80<br> 
  <br> 
  const String HTTP_PAGE_HEAD = "<!DOCTYPE html><html lang=\"en\">\<br> 
  <head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>{v}</title>";<br> 
  const String HTTP_PAGE_STYLE = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;}\<br> 
  input{width:90%;}  body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.6rem;\<br> 
  background-color:#1fb3ec;color:#fdd;line-height:2.4rem;font-size:1.2rem;width:100%;} .q{float: right;\<br> 
  width: 64px;text-align: right;} .button1 {background-color: #4CAF50;} .button2 {background-color: #008CBA;}\<br> 
  .button3 {background-color: #f44336;} .button4 {background-color: #e7e7e7; color: black;} .button5 \<br> 
  {background-color: #555555;} </style>";<br> 
  const String HTTP_PAGE_SCRIPT = "<script>function c(l){document.getElementById('s').value=\<br> 
  l.innerText||l.textContent;document.getElementById('p').focus();}</script>";<br> 
  const String HTTP_PAGE_BODY= "</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>";<br> 
  const String HTTP_PAGE_FORM = "<form action=\"/cmd1\" method=\"get\"><button class=\"button1\">Forward</button>\<br> 
  </form></br><form action=\"/cmd2\" method=\"get\"><button class=\"button2\">Backward</button></form></br>\<br> 
  <form action=\"/cmd3\" method=\"get\"><button class=\"button3\">Right</button></form>\<br> 
  </br><form action=\"/cmd4\" method=\"get\"><button class=\"button4\">Left</button></form></br>\<br> 
  <form action=\"/cmd5\" method=\"get\"><button class=\"button5\">Stop</button></form></br></div>";<br> 
  const String HTTP_WEBPAGE = HTTP_PAGE_HEAD + HTTP_PAGE_STYLE + HTTP_PAGE_SCRIPT + HTTP_PAGE_BODY + HTTP_PAGE_FORM;<br> 
  const String HTTP_PAGE_END = "</div></body></html>";<br> 
  
  // Current time<br> 
  unsigned long currentTime = millis();<br> 
  // Previous time<br> 
  unsigned long previousTime = 0; <br> 
  // Define timeout time in milliseconds (example: 2000ms = 2s)<br> 
  const long timeoutTime = 2000;<br> 
  
  int speed = HALFSPEED;<br> 
  
  void handleRoot() {<br> 
  String s  = HTTP_WEBPAGE;<br>  
  s += HTTP_PAGE_END;<br> 
  server.send(200, "text/html", s);<br> 
  }<br> 
  
  void cmd1() {<br> 
  String s  = HTTP_WEBPAGE; <br> 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s);<br> 
  motor.motorForward(motorR, speed);  <br> 
  motor.motorForward(motorL, speed);<br> 
  Serial.println("Move Forward");<br>    
  }<br> 
  <br> 
  void cmd2() {<br> 
  String s  = HTTP_WEBPAGE; <br> 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s);<br> 
  motor.motorReverse(motorR, speed);<br> 
  motor.motorReverse(motorL, speed);<br> 
  Serial.println("Move Backward");  <br>    
  }<br> 
  <br> 
  void cmd3() {<br> 
  String s  = HTTP_WEBPAGE; <br> 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s);<br> 
  motor.motorReverse(motorR, speed);  <br> 
  motor.motorForward(motorL, speed);<br> 
  Serial.println("Turn Right");<br>     
  }<br> 
  <br> 
  void cmd4() {<br> 
  String s  = HTTP_WEBPAGE;<br>  
  s += HTTP_PAGE_END;<br>   
  server.send(200, "text/html", s);<br> 
  motor.motorForward(motorR, speed);<br> 
  motor.motorReverse(motorL, speed);<br> 
  Serial.println("Turn Left");<br>  
  }<br> 
  
  void cmd5() {<br> 
  String s  = HTTP_WEBPAGE; <br> 
  s += HTTP_PAGE_END;  <br> 
  server.send(200, "text/html", s);<br> 
  motor.motorStop(motorR);<br> 
  motor.motorStop(motorL);<br> 
  Serial.println("Motor Stop");<br> 
  }<br> 
  
  void setup() {<br> 
  Serial.begin(115200);<br> 
  Serial.println("Motor Pins assigned...");<br> 
  motor.attachMotors(IN1pin, IN2pin, IN3pin, IN4pin);<br> 
  
  // Connect to Wi-Fi network with SSID and password<br> 
  Serial.print("Connecting to ");<br> 
  Serial.println(ssid);<br> 
  WiFi.begin(ssid, password);<br> 
  while (WiFi.status() != WL_CONNECTED) {<br> 
  	delay(500);<br> 
  	Serial.print(".");<br> 
  }<br> 
  // Print local IP address and start web server<br> 
  Serial.println("");<br> 
  Serial.println("WiFi connected.");<br> 
  Serial.println("IP address: ");<br> 
  Serial.println(WiFi.localIP());<br> 
  <br> 
  server.on("/", handleRoot);<br> 
  server.on("/cmd1", cmd1);<br> 
  server.on("/cmd2", cmd2);<br> 
  server.on("/cmd3", cmd3);<br> 
  server.on("/cmd4", cmd4);<br> 
  server.on("/cmd5", cmd5);<br>   
  
  Serial.println("HTTP server started");<br> 
  server.begin();<br> 
  
  motor.motorStop(motorR);<br> 
  motor.motorStop(motorL);<br> 
  }<br> 
  <br> 
  void loop() {<br> 
  server.handleClient();<br> 
  }<br>       
 </p>
```
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





