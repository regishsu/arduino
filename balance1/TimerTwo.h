
#ifndef TIMERTWO_h
#define TIMERTWO_h

#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 256    // Timer2 is 8 bit

class TimerTwo
{
  public:
  
    // properties
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;
    char oldSREG;					// To hold Status Register while ints disabled

    // methods
    void initialize(long microseconds=1000000);
    void start();
    void stop();
    void restart();
    void resume();
    unsigned long read();
    void pwm(char pin, int duty, long microseconds=-1);
    void disablePwm(char pin);
    void attachInterrupt(void (*isr)(), long microseconds=-1);
    void detachInterrupt();
    void setPeriod(long microseconds);
    void setPwmDuty(char pin, int duty);
    void (*isrCallback)();
};

extern TimerTwo Timer2;
#endif
