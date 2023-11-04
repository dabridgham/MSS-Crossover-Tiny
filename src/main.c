#define F_CPU 1000000UL		/* default internal clock is 1 MHz (8 MHz div 8) */
#include <avr/io.h>
#include <util/delay.h>
 
#define bit(n) (1<<n)
#define bit_set(reg, bit) { reg |= (1<<bit); }
#define bit_clear(reg, bit) { reg &= ~(1<<bit); }

#define LED_ON() { bit_set(PORTB, PORTB2) }
#define LED_OFF() { bit_clear(PORTB, PORTB2) }

/* Control both the LED and the MSS output */
#define OUTPUT_ON() { PORTB |= bit(PORTB2) | bit(PORTB0); }
#define OUTPUT_OFF() { PORTB &= ~(bit(PORTB2) | bit(PORTB0)); }

const int16_t attack = 50;
const int16_t decay = 30;
const int32_t filtern = 980;
const int32_t filterd = 1000;
#define filter(old, new) (((old * filtern)/filterd) + (((new * (filterd-filtern))/filterd)))

int main()
{
  int16_t current, hi, low, amplitude, hi_amp, threshold, mean, variance;
  uint8_t c_low, c_hi;

  hi = low = 0;
  amplitude = 10000;
  hi_amp = 0;
  threshold = 0;
  mean = 0;
  variance = 0;

  DDRB = 0;			// set all pin as inputs (is the default anyway)
  PORTB |= bit(DDB1);		// enable pullup on pin 6 (PB1)
  DDRB |= bit(DDB2);		// Make pin 7 (PB2) be an output.  

  /* setup the ADC */
  ADMUX = bit(REFS2) | bit(REFS1) | // Vref = internal 2.56V
    bit(ADLAR) | 0x7;	  // left shifted
				// input is differential PB4/PB3 (ADC2/ADC3)
				// plus 20x gain
  DIDR0 = bit(ADC2D) | bit(ADC3D); // disable the digital inputs on ADC2/ADC3
  ADCSRB = bit(BIN);	      // bipolar input mode, free running mode
  ADCSRA = bit(ADEN) |		// enable ADC
    //    bit(ADSC) |		// start conversion
    //    bit(ADATE) |		// auto trigger enable
    bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // scale ADC clock by 128


  while (1) {

    ADCSRA |= bit(ADSC);	/* start conversion */
    while (bit_is_set(ADCSRA, ADSC))
      ;				/* busy wait (should probably time out) */

    /* must read low then high, per the datasheet */
    c_low = ADCL;
    c_hi = ADCH;
    current = (c_hi << 8) | c_low;

    /* consider some stats (probably to be removed later) */
    mean = filter(mean, current);
    variance = filter(variance, (current - mean) * (current - mean));

    /* track hi and low */
    hi = (current > hi) ? hi + attack : hi - decay;
    low = (current < low) ? low - attack : low + decay;


    /* low-pass filter the amplitude (difference from low to high) */
    amplitude = filter(amplitude, hi - low);

    /* set threshold */
    if (!bit_is_set(PINB, DDB1)) {
      if (amplitude > hi_amp) {
	hi_amp = amplitude;
	threshold = (hi_amp * 3) / 2;
      }
    } else
      hi_amp = 0;


    /* use the LED to show various things */
    if (!bit_is_set(PINB, DDB1) ||
	(amplitude > threshold)) {
      OUTPUT_ON();
    } else {
      OUTPUT_OFF();
    }

    _delay_ms(1);
  }
}
