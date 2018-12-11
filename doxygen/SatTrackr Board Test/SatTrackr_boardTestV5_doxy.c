/*
 *                %%%%%%%%%%%%%%%%%%%%%
 *                %      SatTrackr    %
 *                %    ME507-project  %
 *                %%%%%%%%%%%%%%%%%%%%%
 */
/**
 * \mainpage SatTrackr board-test
 * See \ref appdoc_main "here" for project documentation.
 * \copydetails appdoc_preface
 *
 *
 * \page appdoc_preface Overview
 * This application demonstrates the basic functions of the SatTrackr-board
 */

/**
 * \page appdoc_main SatTrackr board-test
 *
 * Overview:
 * - \ref appdoc_SatTrackr_boardtest_intro
 * - \ref appdoc_SatTrackr_boardtest_usage
 * - \ref appdoc_SatTrackr_boardtest_compinfo
 * - \ref appdoc_SatTrackr_boardtest_contactinfo
 *
 * \section appdoc_SatTrackr_boardtest_intro Introduction
 * This application demonstrates the basic functions of the SatTrackr-board.
 * It is for test and presentation purpose. 
 *
 * \subsection Setup procedure
 * This application runs the following setup procedure:
 * - blinking LED once
 * 
 * - opening Serial connection
 * - sending welcome message
 * - blinking LED twice
 * 
 * - creating servo-object
 * - sending serial message
 * - blinking LED 3-times
 *
 * \section appdoc_SatTrackr_boardtest_usage Usage
 * The application reads in serial input and set servo motor to received position
 * input string has to be terminated with a semicolon ","
 * and hast to be in range between 0 and 180
 *
 * \section appdoc_SatTrackr_boardtest_compinfo Compilation Info
 * This software it can be compiled for arduino nano and the exported 
 * hex-file can be uploaded to the SatTrackr Board rev A
 *
 * \section appdoc_SatTrackr_boardtest_contactinfo Contact Information
 * For further information, visit
 * <a href="https://github.com/alex-w-johnson/SatTrackr">https://github.com/alex-w-johnson/SatTrackr</a>.
 */


 #define F_CPU 8000000UL
//Pin-definition:
#define LEDpin 7
#define SerialTXpin 4   //SoftSerial transmit pin
#define SerialRXpin 3   //SoftSerial receive pin
#define ServoPinPitch 5 //Servo1-Data-Pin S1
#define ServoPinYaw 6   //Servo2-Data-Pin S2
#define ShutterPin 2    //Shutter trigger pin

//include additional libraries
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void LEDblink(uint16_t ONdelay, uint16_t OFFdelay, uint8_t times);
//End of Auto generated function prototypes by Atmel Studio



SoftwareSerial SoftSerial(SerialRXpin, SerialTXpin); // RX, TX
Servo PitchServo;
Servo YawServo;

String receivedString = "";
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//initialisation
void setup() {

  //initialise LED
  pinMode(LEDpin, OUTPUT);
  LEDblink(200,100,1);    //blink once

  delay(1000);
  
  //initialise Serial interface
  SoftSerial.begin(9600);
  SoftSerial.println("serial interface initialised");
  LEDblink(200,100,2);  //blink twice

  delay(1000);

  //initialise Serial interface
  PitchServo.attach(ServoPinPitch);
  PitchServo.write(90);
  YawServo.attach(ServoPinYaw);
  YawServo.write(90);
  LEDblink(200,100,3);  //blink 3 times
  
  SoftSerial.println("servos initialised");  
  delay(1000);
  
  digitalWrite(ShutterPin, HIGH);
  delay(1000);
  digitalWrite(ShutterPin, LOW);
  SoftSerial.println("Angle input");

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//continous actions
void loop() {
  
      if(SoftSerial.available()>0){
      int CharReceived = SoftSerial.read();
      receivedString += (char)CharReceived;
      if(CharReceived==','){
        SoftSerial.print("received: ");
        SoftSerial.println(receivedString);
      int receivedValue = receivedString.toInt();
        SoftSerial.print("value: ");
        SoftSerial.println(receivedValue);
        receivedString = "";
        PitchServo.write(receivedValue);
        YawServo.write(receivedValue);
        SoftSerial.print("Pitch position:");
        SoftSerial.println(receivedValue);
        SoftSerial.print("Yaw position:");
        SoftSerial.println(receivedValue); 
        LEDblink(400,200,1);  //blink twice
        digitalWrite(ShutterPin, HIGH);
        delay(1000);
        digitalWrite(ShutterPin, LOW);
        SoftSerial.println("Photo taken, great!");
        SoftSerial.println("Angle input");
      }
    }
  
  

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//custom function implementations:

void LEDblink(uint16_t ONdelay, uint16_t OFFdelay, uint8_t times){
  for(uint16_t i=0; i<times; i++){
    digitalWrite(LEDpin,HIGH);
    delay(ONdelay);
    digitalWrite(LEDpin,LOW);
    delay(OFFdelay);
  }
  
}
