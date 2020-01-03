/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 22
#define RELAY_HEAT_1 32
#define RELAY_HEAT_2 31

//PID Variables
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//call one wire and tempurature functions
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Variables used to set temperature and time
float mashTemp = 78.00; // Degree F
float boilTemp = 80.00; // Degree F
unsigned long mashTime = 1; // Minutes
unsigned long boilTime = 1; // Minutes

/*
the following variables are used as place holders and will be manipulated in the resulting code
 */
unsigned long startTime1=0;
unsigned long startTime2=0;
unsigned long stopTime1=0;
unsigned long stopTime2=0;

int pump = 0;  // variable can be changed to manually operate pump
int valve1 = 0; // variable can be changed to manually open valve
int valve2 = 0; // variable can be changed to manually open valve
int heater1 = 0; // variable can be changed to turn on heating element
int heater2 = 0; // variable can be changed to turn on heating element
int duty = 0;

bool atTemp1=0;
bool atTemp2=0;

//convert time in minutes to time in milliseconds
unsigned long boilTimeMillis = boilTime*60000;
unsigned long mashTimeMillis = mashTime*60000;


void setup()
{
  // set pinmodes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_HEAT_1, OUTPUT);
  pinMode(RELAY_HEAT_2, OUTPUT);
  sensors.begin();//used to begin temerature sensors reading
  Serial.begin(9600);//initilize a serial connection used for debuging

  sensors.requestTemperatures();//request temperture reading
  Input = (sensors.getTempFByIndex(0));//Input to PID function
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Setpoint=boilTemp;//sets the setpoint tempurature for the PID function
  sensors.requestTemperatures();
  Serial.print("Temp ");
  Serial.println(atTemp1);
  Serial.println(boilTemp);
  Serial.println(sensors.getTempFByIndex(0));

  if ((atTemp1==0 )&&( sensors.getTempFByIndex(0)>= boilTemp))
  {
    atTemp1=1;//sets the booleen value atTemp1 to true so that the while statment will run.  upon completion of while
    // statment the atTemp1 value will be set back to zero (0)
    startTime1=millis();
    stopTime1=startTime1 + boilTimeMillis;
    Serial.print("start time:  ");
    Serial.println(startTime1);
    Serial.print("stop time: ");
    Serial.println(stopTime1);
    Serial.println("if");
  }
  else if ((atTemp1 == 1 )&&( sensors.getTempFByIndex(0)>= mashTemp) && (millis()>=stopTime1))
  {
    Serial.println("if else ");
    delay(150);
  }
  else{

    Serial.println("else");
    Serial.print("stop time: ");
    Serial.println(stopTime1);
    Serial.println(millis());
    digitalWrite(LED_BUILTIN, LOW);
  }


//while loop used for the boil process
  while((atTemp1==1) && (millis()<=stopTime1))
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Input = (sensors.getTempFByIndex(0));
    myPID.Compute();
      if(Output > 10)
      {
        digitalWrite(RELAY_HEAT_1, HIGH);
        duty = 50;
      }
      else
      {
        digitalWrite(RELAY_HEAT_1, LOW);
        duty = 0;
      }
    sensors.requestTemperatures();
    Serial.println(millis());
    Serial.println(stopTime1);
    delay(100);
    Serial.println("while 1");
  }

//while loop used for the Mash process

  while((atTemp2==1) && (millis()<=stopTime2))
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Input = (sensors.getTempFByIndex(0));
    myPID.Compute();
      if(Output > 10)
      {
        digitalWrite(RELAY_HEAT_1, HIGH);
        duty = 50;
      }
      else
      {
        digitalWrite(RELAY_HEAT_1, LOW);
        duty = 0;
      }
    sensors.requestTemperatures();
    Serial.println(millis());
    Serial.println(stopTime1);
    delay(100);
    Serial.println("while 1");
  }
  delay(1000);
}
