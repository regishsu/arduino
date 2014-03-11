#ifndef TIMERTWO_cpp
#define TIMERTWO_cpp

#include "TimerTwo.h"

TimerTwo Timer2;              // preinstatiate

ISR(TIMER2_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer2.isrCallback();
}


void TimerTwo::initialize(long microseconds)
{
  TCCR2A = 0;                 // clear control register A 
  TCCR2B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  setPeriod(microseconds);
}


void TimerTwo::setPeriod(long microseconds)		// AR modified for atomic access
{
  
  long cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  
  oldSREG = SREG;				
  cli();							// Disable interrupts for 16 bit register access
  ICR1 = pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
  SREG = oldSREG;
  
  TCCR2B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR2B |= clockSelectBits;                                          // reset clock select register, and starts the clock
}

void TimerTwo::setPwmDuty(char pin, int duty)
{
  unsigned long dutyCycle = pwmPeriod;
  
  dutyCycle *= duty;
  dutyCycle >>= 10;
  
  oldSREG = SREG;
  cli();
  if(pin == 3 || pin == 11)       OCR2A = dutyCycle;
  else if(pin == 4 || pin == 12)  OCR2B = dutyCycle;
  SREG = oldSREG;
}

void TimerTwo::pwm(char pin, int duty, long microseconds)  // expects duty cycle to be 10 bit (1024)
{
  if(microseconds > 0) setPeriod(microseconds);
  if(pin == 3 || pin == 11) {
    DDRB |= _BV(PORTB2);                                   // sets data direction register for pwm output pin
    TCCR2A |= _BV(COM2A1);                                 // activates the output pin
  }
  else if(pin == 4 || pin == 12) {
    DDRB |= _BV(PORTB2);
    TCCR2A |= _BV(COM2B1);
  }
  setPwmDuty(pin, duty);
  resume();			// Lex - make sure the clock is running.  We don't want to restart the count, in case we are starting the second WGM
					// and the first one is in the middle of a cycle
}

void TimerTwo::disablePwm(char pin)
{
  if(pin == 3 || pin == 11)       TCCR2A &= ~_BV(COM2A1);   // clear the bit that enables pwm on PB1
  else if(pin == 4 || pin == 12) TCCR2A &= ~_BV(COM2B1);   // clear the bit that enables pwm on PB2
}

void TimerTwo::attachInterrupt(void (*isr)(), long microseconds)
{
  if(microseconds > 0) setPeriod(microseconds);
  isrCallback = isr;                                       // register the user's callback with the real ISR
  TIMSK2 = _BV(TOIE1);                                     // sets the timer overflow interrupt enable bit
	// might be running with interrupts disabled (eg inside an ISR), so don't touch the global state
//  sei();
  resume();												
}

void TimerTwo::detachInterrupt()
{
  TIMSK2 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit 
															// timer continues to count without calling the isr
}

void TimerTwo::resume()				// AR suggested
{ 
  TCCR2B |= clockSelectBits;
}

void TimerTwo::restart()		// Depricated - Public interface to start at zero - Lex 10/9/2011
{
	start();				
}

void TimerTwo::start()	// AR addition, renamed by Lex to reflect it's actual role
{
  unsigned int tcnt2;
  
  TIMSK2 &= ~_BV(TOIE1);        // AR added 
  GTCCR |= _BV(PSRSYNC);   		// AR added - reset prescaler (NB: shared with all 16 bit timers);

  oldSREG = SREG;				// AR - save status register
  cli();						// AR - Disable interrupts
  TCNT2 = 0;                	
  SREG = oldSREG;          		// AR - Restore status register
  resume();
  do {	// Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt
	oldSREG = SREG;
	cli();
	tcnt2 = TCNT2;
	SREG = oldSREG;
  } while (tcnt2==0); 
 
//  TIFR1 = 0xff;              		// AR - Clear interrupt flags
//  TIMSK1 = _BV(TOIE1);              // sets the timer overflow interrupt enable bit
}

void TimerTwo::stop()
{
  TCCR2B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
}

unsigned long TimerTwo::read()		//returns the value of the timer in microseconds
{									//rember! phase and freq correct mode counts up to then down again
  	unsigned long tmp;				// AR amended to hold more than 65536 (could be nearly double this)
  	unsigned int tcnt1;				// AR added

	oldSREG= SREG;
  	cli();							
  	tmp=TCNT2;    					
	SREG = oldSREG;

	char scale=0;
	switch (clockSelectBits)
	{
	case 1:// no prescalse
		scale=0;
		break;
	case 2:// x8 prescale
		scale=3;
		break;
	case 3:// x64
		scale=6;
		break;
	case 4:// x256
		scale=8;
		break;
	case 5:// x1024
		scale=10;
		break;
	}
	
	do {	// Nothing -- max delay here is ~1023 cycles.  AR modified
		oldSREG = SREG;
		cli();
		tcnt1 = TCNT2;
		SREG = oldSREG;
	} while (tcnt1==tmp); //if the timer has not ticked yet

	//if we are counting down add the top value to how far we have counted down
	tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );		// AR amended to add casts and reuse previous TCNT1
	return ((tmp*1000L)/(F_CPU /1000L))<<scale;
}

#endif
