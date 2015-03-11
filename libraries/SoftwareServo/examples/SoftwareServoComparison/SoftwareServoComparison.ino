// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
// Modified to use Software Servo for Intel Edison

#include <SoftwareServo.h> 
#include <Servo.h> 
 
SoftwareServo myservo2;  // create servo object to control a servo 
Servo myservo;

int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);  // attaches the servo on pin 10 to the softservo object 
} 
 
 
void loop() 
{ 
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos); 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos); 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 