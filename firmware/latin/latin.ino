// dsp-L8 Drum Chip (c) DSP Synthesizers 2015
// Free for non commercial use

// Modified by ASCII to be triggered by digital inputs (2024)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "drum_samples.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
  (((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
  (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

byte inputs[8] = { 2, 3, 4, 5, 6, 7, 8, 9 };                                           // could add , 8, 9 here..
bool prevInputStates[8] = { false, false, false, false, false, false, false, false };  // and increase this array size to 8

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;

//-----------------------------------------
ISR(TIMER1_COMPA_vect) {

  //-------------------  Ringbuffer handler -------------------------

  if (RingCount) {                     //If entry in FIFO..
    OCR2A = Ringbuffer[(RingRead++)];  //Output LSB of 16-bit DAC
    RingCount--;
  }

  //-----------------------------------------------------------------
}



void setup() {
  OSCCAL = 0xFF;

  for (byte i = 0; i < 6; i++) {
    pinMode(inputs[i], INPUT);
  }

  //8-bit PWM DAC pin
  pinMode(11, OUTPUT);

  // Set up Timer 1 to send a sample every interrupt.
  cli();
  // Set CTC mode
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  // No prescaler
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  //OCR1A = F_CPU / SAMPLE_RATE;
  // Enable interrupt when TCNT1 == OCR1A
  TIMSK1 |= _BV(OCIE1A);
  OCR1A = 400;  //40KHz Samplefreq

  // Set up Timer 2 to do pulse width modulation on D11

  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  // Do non-inverting PWM on pin OC2A (p.155)
  // On the Arduino this is pin 11.
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set initial pulse width to the first sample.
  OCR2A = 128;

  //set timer0 interrupt at 61Hz
  TCCR0A = 0;  // set entire TCCR0A register to 0
  TCCR0B = 0;  // same for TCCR0B
  TCNT0 = 0;   //initialize counter value to 0
  // set compare match register for 62hz increments
  OCR0A = 255;  // = 61Hz
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for prescaler 1024
  TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);  //1024 prescaler

  TIMSK0 = 0;


  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // Choose prescaler PS_128.
  ADCSRA |= PS_128;
  ADMUX = 64;
  sbi(ADCSRA, ADSC);

  Serial.begin(9600);
  Serial.println("Modified [Jan Ostman] dsp-D8:");
  Serial.println("https://janostman.wordpress.com/the-dsp-d8-drum-chip-source-code/");
  Serial.println("Press asdfjkl; to play the drums.");

  sei();
}

uint8_t phaccAG,phaccCA,phaccMA,phaccWH,phaccTI,phaccCH,phaccQU,phaccBO;
uint8_t pitchAG=32;
uint8_t pitchCA=64;
uint8_t pitchMA=64;
uint8_t pitchWH=32;
uint8_t pitchTI=64;
uint8_t pitchCH=16;
uint8_t pitchQU=16;
uint8_t pitchBO=64;
uint16_t samplecntAG,samplecntCA,samplecntMA,samplecntWH,samplecntTI,samplecntCH,samplecntQU,samplecntBO;
uint16_t samplepntAG,samplepntCA,samplepntMA,samplepntWH,samplepntTI,samplepntCH,samplepntQU,samplepntBO;

uint8_t oldPORTB;
uint8_t oldPORTD;

int16_t total;
uint8_t divider;
uint8_t MUX = 0;
char ser = ' ';

void loop() {


  //------ Add current sample word to ringbuffer FIFO --------------------

  if (RingCount < 255) {  //if space in ringbuffer
    total = 0;
    if (samplecntAG) {
			phaccAG+=pitchAG;
			if (phaccAG & 128) {
				phaccAG &= 127;
				total+=(pgm_read_byte_near(AG + samplepntAG)-128);
				samplepntAG++;
				samplecntAG--;
      }
      total += (pgm_read_byte_near(AG + samplepntAG) - 128);
    }
    if (samplecntBO) {
			phaccBO+=pitchBO;
			if (phaccBO & 128) {
				phaccBO &= 127;
				total+=(pgm_read_byte_near(BO + samplepntBO)-128);
				samplepntBO++;
				samplecntBO--;
    }
    total += (pgm_read_byte_near(BO + samplepntBO) - 128);
    }
    if (samplecntMA) {
			phaccMA+=pitchMA;
			if (phaccMA & 128) {
				phaccMA &= 127;
				total+=(pgm_read_byte_near(MA + samplepntMA)-128);
				samplepntMA++;
				samplecntMA--;
      }
      total += (pgm_read_byte_near(MA + samplepntMA) - 128);     //128
    }
    if (samplecntQU) {
			phaccQU+=pitchQU;
			if (phaccQU & 128) {
				phaccQU &= 127;
				total+=(pgm_read_byte_near(QU + samplepntQU)-128);
				samplepntQU++;
				samplecntQU--;
      }
      total += (pgm_read_byte_near(QU + samplepntQU) - 128);
    }
    if (samplecntCA) {
			phaccCA+=pitchCA;
			if (phaccCA & 128) {
				phaccCA &= 127;
				total+=(pgm_read_byte_near(CA + samplepntCA)-128);
				samplepntCA++;
				samplecntCA--;
      }
      total += (pgm_read_byte_near(CA + samplepntCA) - 128);
    }
    if (samplecntTI) {
			phaccTI+=pitchTI;
			if (phaccTI & 128) {
				phaccTI &= 127;
				total+=(pgm_read_byte_near(TI + samplepntTI)-128);
				samplepntTI++;
				samplecntTI--;
      }
      total += (pgm_read_byte_near(TI + samplepntTI) - 128);
    }
    if (samplecntWH) {
			phaccWH+=pitchWH;
			if (phaccWH & 128) {
				phaccWH &= 127;
				total+=(pgm_read_byte_near(WH + samplepntWH)-128);
				samplepntWH++;
				samplecntWH--;
      }
      total += (pgm_read_byte_near(WH + samplepntWH) - 128);
    }
    if (samplecntCH) {
			phaccCH+=pitchCH;
			if (phaccCH & 128) {
				phaccCH &= 127;
				total+=(pgm_read_byte_near(CH + samplepntCH)-128);
				samplepntCH++;
				samplecntCH--;
      }
      total += (pgm_read_byte_near(CH + samplepntCH) - 128);
    }
    total >>= 1;
    if (!(PINB & 4)) total >>= 1;
    total += 128;
    if (total > 255) total = 255;

    cli();
    Ringbuffer[RingWrite] = total;
    RingWrite++;
    RingCount++;
    sei();
  }

  //----------------------------------------------------------------------------

  //----------------- Handle Triggers ------------------------------

  bool triggered[8];
  for (byte i = 0; i < 8; i++) {
    bool currentInputState = opt_read(inputs[i]);
    triggered[i] = (currentInputState && !prevInputStates[i]);
    prevInputStates[i] = currentInputState;
  }

  if (triggered[0]) {
    samplepntBO = 0;
		samplecntBO = 1474;
  }
  if (triggered[1]) {
    samplepntAG = 0;
		samplecntAG = 1720;
  }
  if (triggered[2]) {
    samplepntTI = 0;
		samplecntTI = 3680;
		
  }
  if (triggered[3]) {
    samplepntQU = 0;
		samplecntQU = 6502;
  }
  if (triggered[4]) {
    samplepntMA = 0;
		samplecntMA = 394;
  }
  if (triggered[5]) {
    samplepntCH = 0;
		samplecntCH = 5596;
  }
  if (triggered[6]) {
    samplepntCA = 0;
		samplecntCA = 2174;
  }
  if (triggered[7]) {
    samplepntWH = 0;
		samplecntWH = 2670;
  }

  //-----------------------------------------------------------------
}

bool opt_read(byte pin) {
  if (pin >= 14) {
    pin -= 14;
    return bitRead(PINC, pin);
  } else if (pin >= 8) {
    pin -= 8;
    return bitRead(PINB, pin);
  } else {
    return bitRead(PIND, pin);
  }
}

void opt_write(byte pin, bool val) {
  if (pin >= 14) {
    pin -= 14;
    bitWrite(PORTC, pin, val);
  } else if (pin >= 8) {
    pin -= 8;
    bitWrite(PORTB, pin, val);
  } else {
    bitWrite(PORTD, pin, val);
  }
}

void opt_mode(byte pin, byte val) {
  if (val == 2) {
    opt_write(pin, HIGH);
    val = 0;
  }
  if (pin >= 14) {
    pin -= 14;
    bitWrite(DDRC, pin, val);
  } else if (pin >= 8) {
    pin -= 8;
    bitWrite(DDRB, pin, val);
  } else {
    bitWrite(DDRD, pin, val);
  }
}
