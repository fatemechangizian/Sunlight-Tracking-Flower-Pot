#include <L298N.h>
#include <SparkFunTSL2561.h>
#include <Wire.h>
#include <NewPing.h>
#include <Servo.h> 
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2 // DS18B20 on pin D2 
#define maxdistance 200 
#include <stdlib.h>

const int lightThresh=3000;
const int distanceThresh=20;
int distanceTemp=0;
int lightpercent = 0;

//battery
const float referenceVolts = 5.0; // the default reference on a 5-volt board 
const int batteryPin = 3; // battery is connected to analog pin 3 
float batterycharge = 0;

// Thingspeak  
String statusChWriteKey = "1QQ8D4FPB9DA75PA";  // Status Channel id: 596372
SoftwareSerial EspSerial(9,8); // Rx,  Tx
#define HARDWARE_RESET 10

// Soil humidity
#define soilHumPIN 0
int soilHum = 0;

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
int soilTemp = 0;

//
const int lightThreshold=1000;
int maxlight;
// defines pins numbers
int trigPin=A1;
int echoPin=A2;
// defines variables
long duration;
int distance = 100;
int distanceR;
int distanceL;
//***********************

// Variables to be used with timers
long writeTimingSeconds = 60; // ==> Define Sample time in seconds to send data
long startWriteTiming = 0;
long elapsedWriteTime = 0;

int spare = 0;
boolean error;
//***********************
NewPing sonar(trigPin, echoPin, maxdistance); 
Servo myservo;   

// Create an SFE_TSL2561 object, here called "light":

SFE_TSL2561 light;

// Global variables:

boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds
double lux;    // Resulting lux value
int luxnum;
//motor variables:
const int EnableA = 11;
const int RightMotorForward = 4;
const int RightMotorBackward = 5;
const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int EnableB = 3;

L298N motorA(EnableA, RightMotorForward, RightMotorBackward);
L298N motorB(EnableB, LeftMotorForward, LeftMotorBackward);

//********************************************

void setup() {
    Serial.begin(9600);
    //
    pinMode(HARDWARE_RESET,OUTPUT);
  
  digitalWrite(HARDWARE_RESET, HIGH);
  
  DS18B20.begin();
  
  EspSerial.begin(9600); // Comunicacao com Modulo WiFi
  EspHardwareReset(); //Reset do Modulo WiFi
  startWriteTiming = millis(); // starting the "program clock"
    //
    
    //motor setup
  pinMode (RightMotorForward, OUTPUT);
  pinMode (RightMotorBackward, OUTPUT);
  pinMode (LeftMotorForward, OUTPUT);
  pinMode (LeftMotorBackward, OUTPUT);
  pinMode (EnableA, OUTPUT);
  pinMode (EnableB, OUTPUT);
  motorA.setSpeed(175); // an integer between 0 and 255
  motorB.setSpeed(175); // an integer between 0 and 255
  
  myservo.attach(12);  
  myservo.write(90);
  
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);  
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  Serial.begin(9600);
  //Serial.println("TSL2561 example sketch");
  light.begin();
  gain = 0;
  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration
  unsigned char time = 1;
  //Serial.println("Set timing...");
  light.setTiming(gain,time,ms);
  // To start taking measurements, power up the sensor:
  Serial.println("Powerup...");
  light.setPowerUp();
  //
  //
  turnAround() ;   

}

//********************************************

void loop() {
  start: //label 
  error=0;
  
  elapsedWriteTime = millis()-startWriteTiming; 
  
  if (elapsedWriteTime > (writeTimingSeconds*1000)) 
  {
    readSensors();
    writeThingSpeak();
    startWriteTiming = millis();   
  }
  
  if (error==1) //Resend if transmission is not completed 
  {       
    Serial.println(" <<<< ERROR >>>>");
    delay (2000);  
    goto start; //go to label "start"
  }
 
  //
  avoidObstacle();

  if(luxnum >= lightThresh){
    
    moveStop();
    delay(10000);
    turnAround();
    }
    else{}
   
}
//********************************************
//
/********* Read Sensors value *************/
void readSensors(void)
{          
  lightpercent = map(luxnum, 0, 3000, 0, 100); //LDRDark:0  ==> light 100%
  soilHum = map(analogRead(soilHumPIN), 1023, 0, 0, 100);   
  DS18B20.requestTemperatures(); 
  soilTemp = DS18B20.getTempCByIndex(0); // Sensor 0 will capture Soil Temp in Celcius
  int val = analogRead(batteryPin); // read the value from the sensor 
  float volts = (val / 1023.0) * referenceVolts; // calculate the ratio Serial.
  batterycharge = map(volts, 0, 5, 0, 100);
}

/********* Conexao com TCP com Thingspeak *******/
void writeThingSpeak(void)
{
  startThingSpeakCmd();
  // preparacao da string GET
  String getStr = "GET /update?api_key=";
  getStr += statusChWriteKey;
  getStr +="&field1=";
  getStr += String(lightpercent);
  getStr +="&field2=";
  getStr += String(soilHum);
  getStr +="&field3=";
  getStr += String(soilTemp);
  getStr +="&field4=";
  getStr += String(batterycharge);
  getStr += "\r\n\r\n";
  sendThingSpeakGetCmd(getStr); 
}
/********* Reset ESP *************/
void EspHardwareReset(void)
{
  Serial.println("Reseting......."); 
  digitalWrite(HARDWARE_RESET, LOW); 
  delay(500);
  digitalWrite(HARDWARE_RESET, HIGH);
  delay(8000);//Tempo necessário para começar a ler 
  Serial.println("RESET"); 
}
/********* Start communication with ThingSpeak*************/
void startThingSpeakCmd(void)
{
  EspSerial.flush();//limpa o buffer antes de começar a gravar
  
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += "184.106.153.149"; // Endereco IP de api.thingspeak.com
  cmd += "\",80";
  EspSerial.println(cmd);
  Serial.print("enviado ==> Start cmd: ");
  Serial.println(cmd);
  if(EspSerial.find("Error"))
  {
    Serial.println("AT+CIPSTART error");
    return;
  }
}
/********* send a GET cmd to ThingSpeak *************/
String sendThingSpeakGetCmd(String getStr)
{
  String cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  EspSerial.println(cmd);
  Serial.print("enviado ==> lenght cmd: ");
  Serial.println(cmd);
  if(EspSerial.find((char *)">"))
  {
    EspSerial.print(getStr);
    Serial.print("enviado ==> getStr: ");
    Serial.println(getStr);
    delay(500);//tempo para processar o GET, sem este delay apresenta busy no próximo comando
    String messageBody = "";
    while (EspSerial.available()) 
    {
      String line = EspSerial.readStringUntil('\n');
      if (line.length() == 1) 
      { //actual content starts after empty line (that has length 1)
        messageBody = EspSerial.readStringUntil('\n');
      }
    }
    Serial.print("MessageBody received: ");
    Serial.println(messageBody);
    return messageBody;
  }
  else
  {
    EspSerial.println("AT+CIPCLOSE");     // alert user
    Serial.println("ESP8266 CIPSEND ERROR: RESENDING"); //Resend...
    spare = spare + 1;
    error=1;
    return "error";
  } 
} 
//********************************************

int lookRight()
{
    myservo.write(15); 
    delay(100);
     distanceR = readPing();
    delay(100);
     distanceR = readPing();
    delay(100);
     distanceR = readPing();
    delay(100);
     distanceR = readPing();
    delay(100);
    return distanceR;
    delay(100);
}

//

int lookLeft()
{
    myservo.write(180); 
    delay(100);
     distanceL = readPing();
    delay(100);
     distanceL = readPing();
    delay(100);
     distanceL = readPing();
    delay(100);
     distanceL = readPing();
    delay(100);
    return distanceL;
    delay(100);
}

//

//

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  
}
//
void moveForward(){

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  
}
//
void moveBackward(){


  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
}
//
void turnRight(){

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
}

//
void turnBack(){
  turnRight();
  delay(800);
  }

//

void turnLeft(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

//

void turnAround(){
int LUX1, LUX2, LUX3, LUX4 ;
  
  //First Turn
  motorA.forward();
  motorB.forward();
  delay(1);
  turnRight();
  delay(600);
  moveStop();
  delay(2000);
  getLight();
  LUX1= luxnum;
  Serial.println("LUX1: ");
  Serial.println(LUX1);
  
  //second turn
  turnRight();
  delay(600);
  moveStop();
  delay(2000);
  getLight();
  LUX2=luxnum;
  Serial.println("LUX2: ");
  Serial.println(LUX2);
  
  //third turn
  turnRight();
  delay(600);
  moveStop();
  delay(2000);
  getLight();
  LUX3=luxnum;
  Serial.println("LUX3: ");
  Serial.println(LUX3);  

  
  //fourth Turn
  turnRight();
  delay(600);
  moveStop();
  delay(2000);
  getLight();
  LUX4=luxnum;
  Serial.println("LUX4: ");
  Serial.println(LUX4); 

  int maxlight = max(max(LUX1,LUX2), max(LUX3,LUX4));
  Serial.print("Maxlight is ");
  Serial.println(maxlight);
    //
  
    if (maxlight==LUX1){
    turnRight();
    delay(600);
    moveStop();
    delay(100);
    moveForward();
    }
  //    
    else if (maxlight==LUX2){
    turnRight();
    delay(1200);
    moveStop();
    delay(100);
    moveForward();
    }
    //
    else if (maxlight==LUX3){
    turnRight();
    delay(1800);
    moveStop();
    delay(100);
    moveForward();

    }
    //
    else if (maxlight==LUX4){
    moveStop();
    delay(100);
    moveForward();
    }
}

//************************************************

void getLight(){
  
   delay(ms);
   unsigned int data0, data1;
  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful
    /*
    Serial.print("data0: ");
    Serial.print(data0);
    Serial.print(" data1: ");
    Serial.print(data1);
    */
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);
    luxnum=lux;
    
    // Print out the results:
  
    //Serial.println("lux: ");
    //Serial.println(lux);
    
    /*
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else
  {
    // getData() returned false because of an I2C error, inform the user.

    byte error = light.getError();
    printError(error);
  }
  */
  }
  return luxnum;
  }

//************************************************

  void printError(byte error)
{
  
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  
  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}

//**************************************************

void avoidObstacle(){
  
  distance=readPing();
  Serial.println("distanceForward: ");
  Serial.println(distance);
  if (distance <= distanceThresh){
      moveStop();
      delay(300);
      distanceR=lookRight();
      delay(500);
      Serial.println("distanceRight: ");
      Serial.println(distanceR);
      distanceL=lookLeft();
      delay(500);
      Serial.println("distanceLeft: ");
      Serial.println(distanceL);
  
      if(distanceR > distanceL){
        turnRight();
        delay(600);
        moveStop();
        delay(300);
        distanceTemp=lookLeft();
        delay(500);
        Serial.println("distanceL: ");
        Serial.println(distanceTemp);
        if(distanceTemp <= 30){
          moveForward();
          distanceTemp=lookLeft();
          delay(500);
        }
          if(distanceTemp > 30){
            moveStop();
            delay(300);
            myservo.write(90);
            turnLeft();
            delay(600);
            moveStop();
            delay(300);
            moveForward();
            }
      }
      
    else if (distanceL > distanceR){
        turnLeft();
        delay(600);
        moveStop();
        delay(300);
        distanceTemp=lookRight();
        delay(500);
        Serial.println("distanceR: ");
        Serial.println(distanceTemp);
        if(distanceTemp <= 30){
          moveForward();
          distanceTemp=lookRight();
          delay(200);
        }
          if(distanceTemp > 30){
            moveStop();
            delay(300);
            myservo.write(90);
            turnRight();
            delay(600);
            moveStop();
            delay(300);
            moveForward();
        }
        }
        
        else if(distanceR = distanceL){
        turnLeft();
        delay(600);
        moveStop();
        delay(300);
        distanceTemp=lookRight();
        delay(200);
        Serial.println("distanceR: ");
        Serial.println(distanceTemp);
        if(distanceTemp <= 30){
          moveForward();
          distanceTemp=lookRight();
          delay(200);
        }
        else if(distanceTemp > 30){
          moveStop();
          delay(300);
          myservo.write(90);
          turnRight();
          delay(600);
          moveStop();
          delay(300);
          moveForward();
          }    
         }
    }    
  


else{
  moveForward();
  
  }
}

//**************************************************

int readPing(){ 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}
 
  
