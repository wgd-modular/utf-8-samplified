// dsp-D8 Drum Chip (c) DSP Synthesizers 2015
// Free for non commercial use

// Modified by ASCII to be triggered by digital inputs (2024)

// Drumset modified 3/2K24 by DiFo: 
// Kick, Snare, Cymbal & Toms: Mattel Synsonics Drums
// Clap & HiHat: Fricke 512 / 501
// Outs: 1=Kick / 2=Snare / 3=HH / 4=TomHi / 5=TomMid / 6=Clap / 7=TomLo / 8=Cymbal

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "mattel.h"

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

uint8_t phaccBD, phaccSD, phaccCY, phaccHT, phaccHT3, phaccLT, phaccCL, phaccCH;
uint8_t pitchBD = 65;
uint8_t pitchSD = 75;
uint8_t pitchCY = 90;
uint8_t pitchHT = 80;
uint8_t pitchHT3 =100;
uint8_t pitchLT = 60;
uint8_t pitchCL = 70;
uint8_t pitchCH = 75;
uint16_t samplecntBD, samplecntSD, samplecntCY, samplecntHT, samplecntHT3, samplecntLT, samplecntCL, samplecntCH;
uint16_t samplepntBD, samplepntSD, samplepntCY, samplepntHT, samplepntHT3, samplepntLT, samplepntCL, samplepntCH;

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
    if (samplecntBD) {
      phaccBD += pitchBD;
      if (phaccBD & 128) {
        phaccBD &= 127;
        samplepntBD++;
        samplecntBD--;
      }
      total += (pgm_read_byte_near(BD + samplepntBD) - 128);
    }
    if (samplecntSD) {
      phaccSD += pitchSD;
      if (phaccSD & 128) {
        phaccSD &= 127;
        samplepntSD++;
        samplecntSD--;
      }
      total += (pgm_read_byte_near(SD + samplepntSD) - 128);
    }
    if (samplecntCY) {
      phaccCY &= pitchCY;
      if (phaccCY + 128) {   //128
        phaccCY &= 127;     //127
        samplepntCY++;
        samplecntCY--;
      }
      total += (pgm_read_byte_near(CY + samplepntCY) - 128);     //128
    }
    if (samplecntHT) {
      phaccHT += pitchHT;
      if (phaccHT & 128) {
        phaccHT &= 127;
        samplepntHT++;
        samplecntHT--;
      }
      total += (pgm_read_byte_near(HT + samplepntHT) - 128);
    }
    if (samplecntHT3) {
      phaccHT3 += pitchHT3;
      if (phaccHT3 & 128) {
        phaccHT3 &= 127;
        samplepntHT3++;
        samplecntHT3--;
      }
      total += (pgm_read_byte_near(HT3 + samplepntHT3) - 128);
    }
    if (samplecntLT) {
      phaccLT += pitchLT;
      if (phaccLT & 128) {
        phaccLT &= 127;
        samplepntLT++;
        samplecntLT--;
      }
      total += (pgm_read_byte_near(LT + samplepntLT) - 128);
    }
    if (samplecntCL) {
      phaccCL += pitchCL;
      if (phaccCL & 128) {
        phaccCL &= 127;
        samplepntCL++;
        samplecntCL--;
      }
      total += (pgm_read_byte_near(CL + samplepntCL) - 128);
    }
    if (samplecntCH) {
      phaccCH += pitchCH;
      if (phaccCH & 128) {
        phaccCH &= 127;
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
    samplepntBD = 0;
    samplecntBD = 890;     //orginal length is 923. but theres a cracker at sample end... 
  }
  if (triggered[1]) {
    samplepntSD = 0;
    samplecntSD = 1824;
  }
  if (triggered[7]) {
    samplepntCY = 0;
    samplecntCY = 6770;
  }
  if (triggered[3]) {
    samplepntHT = 0;
    samplecntHT = 2424;
  }
  if (triggered[6]) {
    samplepntHT3 = 0;
    samplecntHT3 = 2885;
  }
  if (triggered[4]) {
    samplepntLT = 0;       
    samplecntLT = 1921;   
  }
  if (triggered[5]) {
    samplepntCL = 0;
    samplecntCL = 1474;
  }
  if (triggered[2]) {
    samplepntCH = 0;
    samplecntCH = 330;
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
