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

#define BUTTON_PUSHED() !bit_is_set(PINB, DDB1)

const int16_t attack = 50;
const int16_t decay = 30;
const int32_t filtern = 980;	/* filter numerator */
const int32_t filterd = 1000;	/* filter deniminator */
#define filter(old, new) (((old * filtern)/filterd) + (((new * (filterd-filtern))/filterd)))

/* Read the A/D Converter */
uint16_t read_adc()
{
  uint8_t c_low, c_hi;

  ADCSRA |= bit(ADSC);	/* start conversion */
  while (bit_is_set(ADCSRA, ADSC)) /* wait for ADC to finish */
    ;				   /* should probably time out */

  /* must read low then high, per the datasheet */
  c_low = ADCL;
  c_hi = ADCH;
  return (c_hi << 8) | c_low;
}

int main()
{
  int16_t current, hi, low, amplitude, hi_amp, threshold, mean, variance;

  hi = low = 0;
  amplitude = 10000;
  hi_amp = 0;
  threshold = 0;
  mean = 0;
  variance = 0;

  DDRB = bit(DDB0) | bit(DDB2); // Set pin 5 (PB0) and 7 (PB2) be
				// outputs, all others inputs
  PORTB |= bit(DDB1);		// enable pullup on pin 6 (PB1)

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
    current = read_adc();

    /* consider some stats (probably to be removed later) */
    mean = filter(mean, current);
    variance = filter(variance, (current - mean) * (current - mean));

    /* track hi and low */
    hi = (current > hi) ? hi + attack : hi - decay;
    low = (current < low) ? low - attack : low + decay;


    /* low-pass filter the amplitude (difference from low to high) */
    amplitude = filter(amplitude, hi - low);

    /* set threshold */
    if (BUTTON_PUSHED()) {
      if (amplitude > hi_amp) {
	hi_amp = amplitude;
	threshold = (hi_amp * 3) / 2;
      }
    } else
      hi_amp = 0;


    /* use the LED to show various things */
    if (BUTTON_PUSHED() ||
	(amplitude > threshold)) {
      OUTPUT_ON();
    } else {
      OUTPUT_OFF();
    }

    _delay_ms(1);
  }
}
