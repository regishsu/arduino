// is used in I2Cdev.h
#include "Wire.h"
#include "TimerOne.h"

#define TRIG_PIN  4
#define ECHO_PIN  2
#define LED_PIN  13
#define SONER_CYCLE 100000


int blink_led = 0;
int timer1_sw = 0;

void Timer1_callback(void)
{
  //blink_led = !blink_led;
  //digitalWrite(LED_PIN, blink_led);
  timer1_sw++;
}

unsigned long t_echo_start, t_echo_end, t_echo;
float distance; 
void Echo_time(void)
{
  if (digitalRead(ECHO_PIN))
    t_echo_start = micros();
  else
    t_echo_end = micros();
}

int sw_trig;
void setup() 
{
  pinMode(TRIG_PIN, OUTPUT);         
  pinMode(ECHO_PIN, INPUT);         

  Wire.begin();
  //Initialize serial and wait for port to open:
  Serial.begin(57600);
  Serial.println("System Ready!");
  t_echo_start = t_echo_end = 0;
  sw_trig = 0;
  Timer1.initialize(SONER_CYCLE); //1s
  Timer1.attachInterrupt(Timer1_callback);  // attaches callback() as a timer overflow interrupt
  //important: waiting 1s for Soner device stable.
  delay(1000);
}

//unsigned long time;
void loop() 
{
  if (timer1_sw)
  {
    detachInterrupt(0);
    // Send a trig to Soner
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    t_echo_start = t_echo_end = 0;
    timer1_sw--;
    digitalWrite(TRIG_PIN, LOW);
    attachInterrupt(0, Echo_time, CHANGE );
  }
  
  if (t_echo_end > 0 && t_echo_start > 0)
  {
    detachInterrupt(0);
    distance = float(t_echo_end - t_echo_start) / 58;
    
    Serial.print("D:\t");
    Serial.print(t_echo_start); 
    Serial.print("\t");
    Serial.print(t_echo_end); 
    Serial.print("\t");
    Serial.println(distance,2);
    t_echo_start = t_echo_end = 0;
  }
}
