#include <MIDI.h>
#include <SPI.h>
#include <StackArray.h>

//http://www.ripmat.it/mate/d/dc/dcee.html
/*
  NOTES
  void keyword  N/A
  boolean 1 byte
  byte 8 bit               0 to 255
  char  1 byte             -128 to 127
  unsigned char 1 byte
  int 2 byte               -32768 to 32767
  unsigned int  2 byte     0 to 65535
  word  2 byte  0-65535
  long  4 byte             -2,147,483,648 to 2,147,483,647
  unsigned long 4 byte     0 to 4,294,967,295
  float 4 byte             -3.4028235E38 to 3.4028235E38
  double  4 byte
  string  1 byte + x
  array 1 byte + x
  enum  N/A
  struct  N/A
  pointer N/A
*/

const int PIN_GATE = 45;
const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
const unsigned int pitch_max = 4095;
const byte notes_max = 60;
const byte notes_lowest = 24;
// min note 24
// max note 84, 5 oct
// max note 96, 6 oct

unsigned long env_trig_time_msec = 0;
unsigned long note_change_time_msec = 0;
unsigned long note_off_time_msec = 0;
unsigned int last_pitch = 0;
unsigned int curr_pitch = 0;
unsigned int velocity = 0;
unsigned int velocity_scaling = 90; // 0-100

unsigned long now_usec = 0;
unsigned long now_sec = 0;
unsigned long now_msec = 0;
bool legato = true;
bool first_note = true;
bool portamento = false;
unsigned long slide_time_msec = 100;
bool note_on = false;
StackArray <unsigned int> note_stack;

unsigned int vcf = 0;
unsigned long vcf_env_amt = 0;
const unsigned long vcf_env_amt_max = 4095;
unsigned long env1_attack_msec = 0; //milliseconds
unsigned long env1_decay_msec = 0;
unsigned long env1_sustain = 0;
unsigned long env1_release_msec = 0;
unsigned int env_at_note_off = 0;
unsigned int env_at_half_release = 1000; //this will be overwritten
unsigned int env_at_half_decay = 3000; //this will be overwritten
unsigned int env_at_half_attack = 3200; // this is fixed
const unsigned int env1_min = 0;
const unsigned int env1_max = 4095;
unsigned long env1_value = 0;
bool env_trig = false;
const unsigned int filter_max = 4095;
unsigned long curr_filter = 0;
unsigned int bent_pitch = 0;
unsigned int pitch_bend_cv = 0;
unsigned int pitch_bend_max = 8192;
unsigned int pitch_bend_notes = 12;

unsigned int lfo1_type = 0; // 0 = square, 1 = saw, 2 = sine, 3 = random
unsigned long lfo1_period_msec = 1000;

unsigned int lfo1_amount = 0;
unsigned int lfo1_amount_max = 127;

void setup() {
  Serial.begin(9600);
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.setHandleNoteOn(handleNoteOn);
  midiA.setHandleNoteOff(handleNoteOff);
  midiA.setHandleControlChange(handleControlChange);
  midiA.setHandlePitchBend(handlePitchBend);
  pinMode(PIN_CS_PITCH, OUTPUT);
  pinMode(PIN_CS_FILTER, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  digitalWrite(PIN_GATE, HIGH); // gate is always on. it just creates noise
}

void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  note_change_time_msec = now_msec;
  last_pitch = curr_pitch;
  if (note_stack.isEmpty()) {
    env_trig = true;
    env_trig_time_msec = now_msec ;
  }
  else {
    first_note = false;
  }
  unsigned int in_pitch = pitch_max / notes_max * (inNote - notes_lowest) ;
  note_stack.push( in_pitch );
  note_on = true;

}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  note_change_time_msec = now_msec;
  unsigned int in_pitch = pitch_max / notes_max * (inNote - notes_lowest);
  if (!note_stack.isEmpty()) {
    last_pitch = in_pitch;
    note_stack.pop();
  }
  if (note_stack.isEmpty()) {
    note_off_time_msec = now_msec;
    env_at_note_off = env1_value;
    first_note = true;
    note_on = false;
  }
}

void handlePitchBend(byte channel, int bend) {
  pitch_bend_cv =  ((long)bend * ((pitch_max / notes_max) * pitch_bend_notes) ) / pitch_bend_max;
}

void handleControlChange(byte channel, byte number, byte value) {
  switch (number) {
    case 65:

      if (value == 0) {
        portamento = false; //TODO: portamento and legato  booleans should be fixed, they don't behave correctly, when both are 0 then there is a delay between notes
      }
      else {
        portamento = true;
      }
      break;
    case 66:
      if (value == 0) {
        legato = false;
      }
      else {
        legato = true;
      }
      break;
    case 67:
      if (value == 0) {
        lfo1_type = 0;
      }
      else {
        lfo1_type = 1;
      }
      break;
    case 68:
      if (value == 0) {
        lfo1_type = 2;
      }
      else {
        lfo1_type = 3;
      }
      break;
    case 69:
      if (value == 0) {
        lfo1_type = 4;
      }
      else {
        lfo1_type = 4;
      }
      break;

    case 70:
      if (value == 0) {
        pitch_bend_notes = 2;
      }
      else {
        pitch_bend_notes = 12;
      }
      break;

    case 81:
      vcf = (float)value / 127 * 4095;
      break;

    case 83:
      vcf_env_amt = ((unsigned long)value * 4095) / 127 ;
      break;

    case 84:
      lfo1_amount = value;
      break;

    case 89:
      //log scale:
      env1_attack_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      // linear version:
      //env1_attack_msec =  (unsigned long)value * 10000 / 127 ;
      break;

    case 90:
      //log scale:
      env1_decay_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      // linear version:
      //env1_decay_msec = (unsigned long)value * 10000 / 127;
      break;

    case 91:
      env1_sustain = (unsigned long)value * 4095 / 127 ;
      env_at_half_decay =   env1_sustain + ((env1_max - env1_sustain) / 5) ;
      break;

    case 92:
      //log scale:
      env1_release_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      // linear version:
      //env1_release_msec =  (unsigned long)value * 10000 / 127 ;
      env_at_half_release =  ( env1_sustain - env1_min ) / 5 ;
      break;

    case 97:
      slide_time_msec = (unsigned long)value * 1000 / 127;
      break;

    case 98:
      lfo1_period_msec = 1 + (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;
  }
}


//assuming single channel, gain=2
void setOutput(unsigned int val)
{

  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
  digitalWrite(PIN_CS_PITCH, LOW);
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS_PITCH, HIGH);
}

void setOutput_filter(unsigned int val)
{

  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
  digitalWrite(PIN_CS_FILTER, LOW);
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS_FILTER, HIGH);
}

void setOutput(byte channel, byte gain, byte shutdown, unsigned int val)
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | channel << 7 | gain << 5 | shutdown << 4;

  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
}

unsigned long lfo_square(unsigned long period, unsigned long current_time) {
  unsigned long lfo_value = 0;
  if ((period != 0) && (((current_time / (period / 2) ) % 2) == 0)) {
    lfo_value = 4095;
  }
  return lfo_value;
}

unsigned long lfo_saw(unsigned long period, unsigned long current_time) {
  unsigned long lfo_value = 0;
  if (period != 0) {
    unsigned long y = current_time % period;
    lfo_value = (4095 * y) /  (period) ;
  }
  return lfo_value;
}

unsigned long lfo_ramp(unsigned long period, unsigned long current_time) {
  unsigned long lfo_value = 0;
  if (period != 0) {
    unsigned long y = current_time % period;
    lfo_value = 4095 - ((4095 * y) /  (period)) ;
  }
  return lfo_value;
}

unsigned long lfo_triangle(unsigned long period, unsigned long current_time) {
  unsigned long lfo_value = 0;
  if (period != 0) {
    unsigned long y = current_time % period;
    unsigned long value = (4095 * y) /  (period) ;
    if ( ((current_time / period) % 2) == 0) {
      lfo_value = value;
    }
    else {
      lfo_value =  4095 - value;
    }
  }
}

unsigned long lfo_random(unsigned long period, unsigned long current_time) {
  unsigned long lfo_value = 0;
  if ((period != 0) && ((current_time % (period / 2) ) == 0)) {
    lfo_value = random(4095);
  }
  return lfo_value;
}


//NOTE: side effects on env1_value
unsigned long adsr_attack(unsigned long attack_time, unsigned long current_time, unsigned long trig_time, bool trig_state, unsigned long half_attack_value) {
  // attack section
  // first_half
  if ((attack_time / 2) != 0) {
    if (((current_time - trig_time) <= attack_time / 2) && trig_state ) {
      env1_value =  (env1_min + (current_time - trig_time) * ((half_attack_value) - env1_min) / (attack_time / 2) );
    }
    // second_half
    else if (((current_time - trig_time) <= attack_time) && trig_state ) {
      env1_value =  (half_attack_value + (current_time - (trig_time + attack_time / 2)) * (env1_max - half_attack_value) / (attack_time / 2) );
    }
  }

}

//NOTE: side effects on env1_value
void adsr_decay(unsigned long decay_time, unsigned long current_time, unsigned long trig_time, bool trig_state, unsigned long half_decay_value, unsigned long attack_time, unsigned long sustain_value) {

  if ((decay_time / 2) != 0) {
    // decay section
    //first half
    if  (((current_time - trig_time)  > attack_time ) && ((current_time - trig_time) <= (attack_time + decay_time / 2) ) && trig_state)   {
      env1_value = env1_max - (((current_time - (trig_time + attack_time)) * (env1_max - half_decay_value)) / (decay_time / 2))   ;
    }
    //second half
    else if  (((current_time - trig_time)  > attack_time ) && ((current_time - trig_time) <= (attack_time + decay_time) ) && trig_state)   {
      env1_value = half_decay_value - (((current_time - (trig_time + attack_time + decay_time / 2 )) * (half_decay_value - sustain_value)) / (decay_time / 2))   ;
    }
  }
}

//NOTE: side effects on env1_value
void adsr_sustain(unsigned long sustain_value, unsigned long current_time, unsigned long trig_time, bool trig_state, unsigned long attack_time, unsigned long decay_time) {

  if (((current_time - trig_time) > (attack_time + decay_time) ) && trig_state )  { // sustain section
    env1_value =  sustain_value  ;
  }
}

//NOTE: side effects on env1_value
void adsr_release(unsigned long release_time, unsigned long current_time, unsigned long untrig_time, unsigned long half_release_value, unsigned long note_off_value ) {

  if ((release_time / 2) == 0) {
    env1_value = 0;
  }
  else {
    // Release section
    // trick to make it fake exponential... divide the release time by 2, start with a steeper slope
    if ((current_time - untrig_time) <= (release_time / 2)) {
      env1_value = note_off_value - ((current_time - untrig_time) * (note_off_value - half_release_value)) / (release_time / 2)  ;
    }
    else if ((current_time - untrig_time) <= release_time) { // Release section
      env1_value = half_release_value - ((current_time - untrig_time - release_time / 2) * (half_release_value - env1_min)) / (release_time / 2) ;
    }
    else { // note off
      env1_value = 0;
      //digitalWrite(PIN_GATE, HIGH);
    }
  }
}

void adsr() {

  // ADSR
  if (!note_stack.isEmpty()) {
    // watch out these functions have side effects right now, they modify env1_value, to be fixed
    adsr_attack( env1_attack_msec,   now_msec, env_trig_time_msec, env_trig, env_at_half_attack );
    adsr_decay( env1_decay_msec,     now_msec, env_trig_time_msec, env_trig, env_at_half_decay, env1_attack_msec, env1_sustain );
    adsr_sustain( env1_sustain,      now_msec, env_trig_time_msec, env_trig, env1_attack_msec,  env1_decay_msec);
  }
  else {
    adsr_release(env1_release_msec, now_msec, note_off_time_msec, env_at_half_release , env_at_note_off);
  }
}

void pitch_cv() {

  // PITCH
  if (!note_stack.isEmpty()) {
    if ((now_msec > (note_change_time_msec + slide_time_msec ))  || ((legato) && (first_note )) || (slide_time_msec == 0) ) { // normal pitch section
      curr_pitch = note_stack.peek() + pitch_bend_cv; //TODO: change from peek to the highest note
    }
    else if ((portamento) || ((legato) && (note_stack.count() >= 1)) ) { // portamento section, or legato

      if (note_stack.peek() > last_pitch) {
        curr_pitch = last_pitch + ((now_msec - note_change_time_msec) * (note_stack.peek() - last_pitch)) / slide_time_msec + pitch_bend_cv ;
      }
      else {
        curr_pitch = last_pitch - ((now_msec - note_change_time_msec) * (last_pitch - note_stack.peek())) / slide_time_msec + pitch_bend_cv ;
      }
    }
  }
}

void loop() {
  // static
  unsigned long lfo1_value = 0; 

  
  midiA.read();
  now_usec = micros();
  now_msec = now_usec / 1000;
  now_sec = now_usec / 1000000;


  switch (lfo1_type) {
    case 0:
      lfo1_value = lfo_square(lfo1_period_msec, now_msec);
      break;
    case 1:
      lfo1_value = lfo_triangle(lfo1_period_msec, now_msec);
      break;
    case 2:
      lfo1_value = lfo_saw(lfo1_period_msec, now_msec);
      break;
    case 3:
      lfo1_value = lfo_ramp(lfo1_period_msec, now_msec);
      break;
    case 4:
      lfo1_value = lfo_random(lfo1_period_msec, now_msec);
      break;
  }

  pitch_cv();
  adsr();

  curr_filter = (env1_value * vcf_env_amt) /  vcf_env_amt_max + vcf + (lfo1_value * lfo1_amount) / lfo1_amount_max;

  // boundaries for what the dac is capable of doing
  if (curr_pitch > pitch_max) {
    curr_pitch = pitch_max;
  }
  if (curr_filter > filter_max) { // clear up too big numbers
    curr_filter = filter_max;
  }

  // scale based on velocity
  //curr_filter = curr_filter * velocity;
  //Serial.println(curr_filter);
  //curr_filter = 300;
  //Serial.println(curr_filter);
  setOutput(curr_pitch);
  setOutput_filter((unsigned int)curr_filter);

}


