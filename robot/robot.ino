//Pins:
//D2: camera CS
//D3: Bluetooth RXD
//D4: camera MISO
//D5: camera MOSI
//D6: Motor
//D7: camera SCK
//D8: camera SDA
//D9: Motor
//D10: Motor
//D11: Motor
//D12: camera SCL
//D13: Bluetooth TXD

//A0: Trigger
//A1: Echo
//A2: LED blue
//A3: LED red
//A4: OLED
//A5: OLED
//===============================================================

#include <ros.h> // include ros libraries
#include <std_msgs/Int16.h>

#include <SPI.h>
#include <Wire.h>

#include "SSD1306Ascii.h" //library for OLED
#include "SSD1306AsciiWire.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define RST_PIN -1 // -1 because sharing Arduino reset pin
#define I2C_ADDRESS 0x3C 
SSD1306AsciiWire oled; // introducing the display object

//=========================================================

//initialise variables for the Ultrasonic Sensor
const int trigger = A0;
const int echo = A1;
long duration = 0;
long distance = 0;

//initialise variables for right motor
const int rightMotor1 = 10;
const int rightMotor2 = 11;

//initialise variables for left motor
const int leftMotor1 = 6;
const int leftMotor2 = 9;

//initialise variables for the LEDs
const int ledRight = A3;
const int ledLeft = A2;

//initialise speed, so not too fast
int leftSpeed = 30;
int rightSpeed = 30;

//initialise directions
String currentDirection = "forward";

//create ros node
ros::NodeHandle  nh;
int x;

// callback function for recieving the label
void changeDirection(std_msgs::Int16& msg){

  //get label from publisher
  x = msg.data;

  if (x == 1){

    //if receiving 1, turn right
    turnRight();
    delay(1000);
    stopDriving();
  }

  if (x == 2){

    //if receiving 2, turn left
    turnLeft();
    delay(1000);
    stopDriving();
  }
}


// Declare a Subscriber object
ros::Subscriber<std_msgs::Int16> sub("ros_label", &changeDirection);

// Declare a Publisher object
std_msgs::Int16 reaction;
ros::Publisher pub("arduino_reaction", &reaction);



void setup() {
  // set up robot to run code once:

  // set up serial port
  Serial.begin(9600);

  // set up OLED
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN); // start OLED
  setDisplay();
  oled.println("Hello!"); // text to display
  oled.println("The programme");
  oled.println("will start now.");
  delay(3000);

  //set up pins
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  // set up ros
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

}

void loop() {

  // publish changes to subscriber
  reaction.data = x;
  pub.publish(&reaction);
  nh.spinOnce();
  delay(1);

  // read ultrasonic sensor
  readUSensor();
  delay(1000);

  // update OLED
  setDisplay();
  oled.print("Going "); oled.println(currentDirection);// text to oled
  oled.print("Obstacle in: "); oled.print(distance); oled.println("cm");

}

// function to read the ultrasonic sensor
void readUSensor() {

  //apply pulse to the trigger
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  //prepare echo to read the pulse
  duration = pulseIn(echo, HIGH);

  //formula according to sellers: uS/58 = centimeters
  distance = 0.017 * duration;

  // show data on OLED
  Serial.print("Distance to object = ");
  Serial.print(distance);
  Serial.print("cm \n");
}

// function for driving straight
void moveForward() {
  digitalWrite(rightMotor1, rightSpeed);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, leftSpeed);
  digitalWrite(leftMotor2, LOW);
  
  // signalise on OLED and LEDs
  digitalWrite(ledLeft, HIGH);
  digitalWrite(ledRight, HIGH);
  Serial.print("Direction = Straight Forward \n");
  delay(500);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
}

//function for turning left
void turnLeft() {
  digitalWrite(rightMotor1, rightSpeed);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);

  // signalise on OLED and LEDs
  digitalWrite(ledLeft, HIGH);
  Serial.print("Direction = Left \n");
  currentDirection = "left";
  delay(500);
  digitalWrite(ledLeft, LOW);
}

//function for turning right
void turnRight() {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, leftSpeed);
  digitalWrite(leftMotor2, LOW);

  // signalise on OLED and LEDs
  digitalWrite(ledRight, HIGH);
  Serial.print("Direction = Right \n");
  currentDirection = "right";
  delay(500);
  digitalWrite(ledRight, LOW);
}

//function for stopping
void stopDriving() {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);

  // signalise on OLED
  Serial.print("Direction = Stopping \n");
  currentDirection = "stopping";
}

//function to reset display
void setDisplay() {
  delay(100);         // wait for initializing
  oled.clear(); // clear display

  oled.setFont(System5x7);
}