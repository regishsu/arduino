

#include "SoftwareServo.h" 
#include "TimerOne.h"

SoftwareServo myservo;  // create servo object to control a servo 

#define pinServo A0
#define T1_CYCLE 200000
#define LED_PIN 13

int blink_led = 0;
int speed = 10;
int limits[2] = {0,180};	// set limitations (min/max: 0->180)
boolean refresh = false;	// toggle refresh on/off

int timer1_sw = 0;

void Timer1_callback(void)
{
  blink_led = !blink_led;
  digitalWrite(LED_PIN, blink_led);
  timer1_sw++;
}

void setup() 
{
  Serial.begin(57600);
  // attaches the servo on pin to the servo object
  myservo.attach(pinServo);  
  //500us for 0 degree
  //myservo.setMinimumPulse(530);
  //2500us for 180 degree
  //myservo.setMinimumPulse(2470);
  
  // init angle of servo inbetween two limitations
  myservo.write(0); 

  Timer1.initialize(T1_CYCLE); //1s
  Timer1.attachInterrupt(Timer1_callback);  // attaches callback() as a timer overflow interrupt
  Serial.println("System ready!");

} 

void loop() 
{ 
  
  static int v = 0;

  if ( Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        Serial.print("Angle: ");
        Serial.println(v);
        break;
      case 'w':
        myservo.write(v);
        v = 0;
        break;
      case 'd':
        myservo.detach();
        break;
      case 'a':
        myservo.attach(pinServo);
        break;
    }
  }

  if (timer1_sw)
  {

  // refresh angle
  int angle = myservo.read();
  Serial.print("Angle: ");
  Serial.println(angle);
  // change direction when limits
  if (angle >= limits[1] || angle <= limits[0])
    speed = -speed;
  
  myservo.write(angle + speed);	
  timer1_sw --;
  }
  
  refresh = !refresh;
  if (refresh) 
    SoftwareServo::refresh();
} 

