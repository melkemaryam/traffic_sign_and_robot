// Set up Serial connection with Bluetooth module
#include <SoftwareSerial.h>
 
//(Receiver RX | Trasmitter TX)
SoftwareSerial EEBlue(3, 13); 
 
////////////////////////////////////////////////////////
 
//Module to interface with the ultrasonic sensor
#define TRIGGER_PIN  A0  //Arduino pin connected to TRIG
#define ECHO_PIN     A1  //Arduino pin connected to ECHO
 
////////////////////////////////////////////////////////
 
//Setup Ultrasonic sensor
void Setup_Ultrasonic()
{
 
  // Define each pin as an input or output.
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
   
}
 
////////////////////////////////////////////////////////
 
 /* Motor driver pin definitions and mappings to Arduino
  * 
  *  
  */
 
/* MOTOR PIN DEFINITIONS 
 
ARDUINO DIGITAL PIN ||||  MOTOR DRIVER (L298 PIN)
 
          5               ENA (Enable 1 - Left Motor)
      
          6               IN1
           
          7               IN2
           
          8               ENB (Enable 2 - Right Motor)
           
          10              IN4
           
          9               IN3
           
 
 
*/
 
 
#define enableA 5   // Connected to Left Motor
#define MotorA1 6
#define MotorA2 7
 
#define enableB 8  //Connected to Right Motor
#define MotorB1 9
#define MotorB2 10
 
 
///////////////////////////////////////////////////////
 
//Initializes the motor pins that are defined as MACROS
void Setup_Motors()
{
   
  // Set up left motor
  pinMode(enableA,OUTPUT);
  pinMode(MotorA1,OUTPUT);
  pinMode(MotorA2,OUTPUT);
 
  // Set up right motor
  pinMode(enableB,OUTPUT);
  pinMode(MotorB1,OUTPUT);
  pinMode(MotorB2,OUTPUT);
  
  delay(200);     // Pause 200 milliseconds 
  
  go_forward();  // Move forward
   
}
 
////////////////////////////////////////////////////////
 
//Setup Serial communication
void Setup_Serial(int baud_rate)
{
 
  Serial.begin(9600);  
  EEBlue.begin(9600);  //Default Baud for communications
   
}
 
 
//////////////////////////////////////////////////////
// Returns the distance to the obstacle as an integer
int Update_Ultrasonic()
{
 
  int distance = 0;
  int average = 0;
  
  // Grab four measurements of distance and calculate
  // the average.
  for (int i = 0; i < 4; i++) {
  
    // Make the TRIGGER_PIN LOW (0 volts) 
    // for 2 microseconds
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2); 
      
    // Emit high frequency 40kHz sound pulse
    // (i.e. pull the TRIGGER_PIN) 
    // by making TRIGGER_PIN HIGH (5 volts) 
    // for 10 microseconds
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
       
    // Detect a pulse on the ECHO_PIN pin 8. 
    // pulseIn() measures the time in 
    // microseconds until the sound pulse
    // returns back to the sensor.    
    distance = pulseIn(ECHO_PIN, HIGH);
  
    // Speed of sound is:
    // 13511.811023622 inches per second
    // 13511.811023622/10^6 inches per microsecond
    // 0.013511811 inches per microsecond
    // Taking the reciprocal, we have:
    // 74.00932414 microseconds per inch 
    // Below, we convert microseconds to inches by 
    // dividing by 74 and then dividing by 2
    // to account for the roundtrip time.
    distance = distance / 74 / 2;
  
    // Compute running sum
    average += distance;
  
    // Wait 10 milliseconds between pings
    delay(10);
  }
   
  distance = average / 4;
 
  Serial.print("u ");
  Serial.print(distance);
  Serial.print("\n"); 
 
  int distance_copy = distance;
 
  // Initialize string
  char str[] = "u ";
  char str_dist[10];
 
  // Convert distance integer into a string
  sprintf(str_dist, "%d", distance_copy);
 
  // Add a new line
  char add_new_line[] = "\n";
 
  // Concatenate to produce the new string
  strcat(str_dist, add_new_line);  
  strcat(str, str_dist); 
 
  // Output data to bluetooth
  EEBlue.write(str);
 
  return distance;  
 
}
 
//////////////////////////////////////////////////////////
// The following function controls 
// the motion of the robot
 
void Move_Robot(int distance)
{
   
  // If obstacle <= 2 inches away
  if (distance >= 0 && distance <= 2) {    
    go_backwards();   // Move in reverse for 0.5 seconds
    delay(500);
  
    /* Go left or right to avoid the obstacle*/
    if (random(2) == 0) {  // Generates 0 or 1, randomly        
      go_right();  // Turn right for one second
    }
    else {
      go_left();  // Turn left for one second
    }
    delay(1000);
    go_forward();  // Move forward
  }
  delay(50); // Wait 50 milliseconds before pinging again 
}
 
/*   
 *  Forwards, backwards, right, left, stop.
 */
void go_forward() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Move forward
  digitalWrite (MotorA1, LOW);
  digitalWrite (MotorA2, HIGH);
  digitalWrite (MotorB1, LOW);
  digitalWrite (MotorB2, HIGH);
 
}
void go_backwards() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Go backwards
  digitalWrite (MotorA1,HIGH);
  digitalWrite (MotorA2,LOW);  
  digitalWrite (MotorB1,HIGH);
  digitalWrite (MotorB2,LOW);  
   
}
void go_right() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Turn right
  digitalWrite (MotorA1, LOW);
  digitalWrite (MotorA2, HIGH);
  digitalWrite (MotorB1,HIGH);
  digitalWrite (MotorB2,LOW); 
}
void go_left() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Turn left
  digitalWrite (MotorA1,HIGH);
  digitalWrite (MotorA2,LOW);  
  digitalWrite (MotorB1, LOW);
  digitalWrite (MotorB2, HIGH);
}
void stop_all() {
  digitalWrite (enableA, LOW);
  digitalWrite (enableB, LOW);
}
 
//////////////////////////////////////////////////
//Read from Serial Function
 
void Read_From_Serial()
{
  // Read data from Serial terminal of Arduino IDE
  while(Serial.available() > 0)
    {      
     EEBlue.write(Serial.read());        
    } 
 
   // Read data from Bluetooth module
   //while(EEBlue.available() > 0)
   // {
   //   Serial.write(EEBlue.read());     
   //  int data = Serial.read();       
      
   // } 
 
}
 
////////////////////////////////////////
//Update all
void Update_all()
{
   
  int distance = Update_Ultrasonic();
   
  Read_From_Serial();
   
  Move_Robot(distance);
   
}
 
 
/////////////////////////////////////////
// Setup function for Arduino
void setup() {
 
  // Initializes the pseudo-random number generator
  // Needed for the robot to wander around the room
  randomSeed(analogRead(3));
   
  Setup_Ultrasonic();
 
    Setup_Serial(9600);
     
    Setup_Motors();    
 
}
///////////////////////////////////////
// This part loops over and over again
void loop() {
 
 
  Update_all();
 
     
}