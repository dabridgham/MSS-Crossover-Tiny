#define F_CPU 1000000UL		/* default internal clock is 1 MHz (8 MHz div 8) */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

 
#define bit(n) (1<<n)
#define bit_set(reg, bit) { reg |= (1<<bit); }
#define bit_clear(reg, bit) { reg &= ~(1<<bit); }

#define LED_ON() { bit_set(PORTB, PORTB2) }
#define LED_OFF() { bit_clear(PORTB, PORTB2) }

/* Control the LED, MSS, and detect outputs */
#define OUTPUT_ON() { PORTB |= bit(PORTB3) | bit(PORTB2) | bit(PORTB0); }
#define OUTPUT_OFF() { PORTB &= ~(bit(PORTB3) | bit(PORTB2) | bit(PORTB0)); }

#define BUTTON_PUSHED() (!bit_is_set(PINB, DDB1))

const int16_t attack = 100;
const int16_t decay = 10;
const int32_t filtern = 980;	/* filter numerator */
const int32_t filterd = 1000;	/* filter denominator */
const unsigned int timeout = 2000; /* 2 seconds */
const unsigned int sample_time = 2000; /* 2 seconds */
#define filter(old, new) (((old * filtern)/filterd) + (((new * (filterd-filtern))/filterd)))

uint16_t *threshold_p = 0; /* EEPROM address for the saved threshold */

/*
 * Timer interrupt code to implement a millis() function.
 */
volatile unsigned long timer0_millis = 0;

// Initialize the timer for 1ms interrupts
void millis_init(void) {
    // Set Timer0 to CTC mode with a prescaler of 64
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64

    // Set compare value for 1ms at 16MHz: (16,000,000 / 64) / 1000 = 250
    // Set compare value for 1ms at 1MHz: (1,000,000 / 64) / 1000 = 16
    OCR0A = 16;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE0A);

    // Enable global interrupts
    sei();
}

// Timer0 compare match interrupt service routine
ISR(TIMER0_COMPA_vect) {
    timer0_millis++;
}

// Return the number of milliseconds since the program started
unsigned long millis(void) {
    unsigned long m;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        m = timer0_millis;
    }
    return m;
}   

/* Setup the ADC */
void adc_init()
{
  ADMUX =		  // Vref = Vcc
    bit(ADLAR) |	  // left shifted
    0x2;		  // input is single-ended PB4 (ADC2)
  DIDR0 = bit(ADC2D);	  // disable the digital inputs on ADC2
  ADCSRB = bit(BIN);	  // bipolar input mode, free running mode
  ADCSRA = bit(ADEN) |	  // enable ADC
    //    bit(ADSC) |		// start conversion
    //    bit(ADATE) |		// auto trigger enable
    bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // scale ADC clock by 128

  /* setup the ADC */
  ADMUX =		  // Vref = Vcc
    bit(ADLAR) |	  // left shifted
    0x2;		  // input is single-ended PB4 (ADC2)
  DIDR0 = bit(ADC2D);	  // disable the digital inputs on ADC2
  ADCSRB = bit(BIN);	  // bipolar input mode, free running mode
  ADCSRA = bit(ADEN) |	  // enable ADC
    //    bit(ADSC) |		// start conversion
    //    bit(ADATE) |		// auto trigger enable
    bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // scale ADC clock by 128
}

/* Read the A/D Converter */
uint16_t read_adc()
{
  uint8_t c_low, c_hi;

  ADCSRA |= bit(ADSC);		   /* start conversion */
  while (bit_is_set(ADCSRA, ADSC)) /* wait for ADC to finish */
    ;				   /* should probably time out */

  /* must read low then high, per the datasheet */
  c_low = ADCL;
  c_hi = ADCH;
  return (c_hi << 8) | c_low;
}

int main()
{
  int16_t amps = 0,
    hi_amps = 0,
    threshold = 1000,
    setting_threshold = 0;
  unsigned long last_hi = 0,
    current_millis = 0;

  threshold = eeprom_read_word(threshold_p); /* read the stored threshold */

  millis_init();		/* enable the millisecond counter */

  // Set pins 5 (PB0), 7 (PB2), and 2 (PB3) as outputs, all others
  // inputs
  DDRB = bit(DDB0) | bit(DDB2) | bit(DDB3); 
  PORTB |= bit(DDB1);		/* enable pullup on pin 6 (PB1) for the pushbutton */

  adc_init();			/* setup the ADC */

  while (1) {
    current_millis = millis();
    if (setting_threshold) {
      if ((current_millis - last_hi) > sample_time) {
	threshold = (hi_amps * 15) / 10; /* set threshold 50% higher */
	eeprom_write_word(threshold_p, threshold);
	setting_threshold = 0;
	last_hi = 0;
	LED_OFF();
      } else {
	/* set hi_amps to the highest reading during the sample period */
	amps = read_adc();
	if (amps > hi_amps)
	  hi_amps = amps;
      }
    } else {
      if (BUTTON_PUSHED()) {
	setting_threshold = 1;
	last_hi = current_millis; /* use last_hi for the sample timeout */
	hi_amps = 0;
	LED_ON();
      } else {
	amps = read_adc();

	/* values greater than the threshold update the time */
	if (amps > threshold)
	  last_hi = current_millis;

	/* if the last high value was more recent than timeout, set detection outputs */
	if ((current_millis - last_hi) < timeout) {
	  OUTPUT_ON();
	} else {
	  OUTPUT_OFF();
	}

	_delay_ms(1);
      }
    }
  }
}
