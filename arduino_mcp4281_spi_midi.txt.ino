#include <MIDI.h>
#include <SPI.h>
#include <StackArray.h>

const int PIN_GATE = 45;
const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
const int pitch_max = 4095;
const int notes_max = 60;
const int notes_lowest = 24;
// min note 24
// max note 84, 5 oct
// max note 96, 6 oct

long env_trig_time = 0;
long note_change_time = 0;
long note_off_time = 0;
int last_pitch = 0;
int curr_pitch = 0;
float velocity = 0;
float velocity_scaling = 90; // 0-100


// this is double otherwise some operations overflow... check carefully
double now = 0;
double now_sec = 0;
double now_msec = 0; 
bool legato = true;
bool first_note = true;
bool portamento = false;
long slide_time = 100000;
bool note_on = false;
StackArray <int> note_stack;

int vcf = 0;
int vcf_env_amt = 2048;
const int vcf_env_amt_max = 4095;
long env1_attack = 100000; //milliseconds
long env1_decay = 1000000;
int env1_sustain = 2000;
long env1_release = 2000000;
int env_at_note_off = 0;
int env_at_half_note_off = 0;
const int env1_min = 0;
const int env1_max = 4095;
bool env_trig = false;
const int filter_max = 4095;
float curr_filter = 0;
int bent_pitch = 0;
int pitch_bend_cv = 0;
int pitch_bend_max = 8192;
int pitch_bend_notes = 12;

int lfo1_type = 0; // 0 = square, 1 = saw, 2 = sine, 3 = random
float lfo1_rate = 1;
float lfo1_value = 0;
float lfo1_amount = 0;
float lfo1_amount_max = 127;

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
  note_change_time = micros();
  int note_cv = pitch_max / notes_max * (inNote - notes_lowest);
  last_pitch = curr_pitch;
  if (note_stack.isEmpty()) {
    env_trig = true;
    env_trig_time = micros();
  }
  note_stack.push(note_cv);
  if (note_stack.count() > 1) {
    first_note = false;
  }
  note_on = true;
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  note_change_time = micros();
  if (!note_stack.isEmpty()) {
    last_pitch = note_stack.pop();
  }
  if (note_stack.isEmpty()) {
    note_off_time = micros();
    env_at_note_off = curr_filter;
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
        portamento = false; // portamento and legato  booleans should be fixed, they don't behave correctly
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
        lfo1_type = 2;
      }
      break;


    case 68:
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
      vcf_env_amt = (float)value / 127 * 4095; // to be implemented
      break;
    case 84:
      lfo1_amount = value;
      break;
    case 89:
      env1_attack = (float)value / 127 * 1000000;
      break;
    case 90:
      env1_decay = (float)value / 127 * 1000000;
      break;
    case 91:
      env1_sustain = (float)value / 127 * 4095;
      break;
    case 92:
      env1_release = (float)value / 127 * 10000000;
      break;
    case 97:
      slide_time = (float)value / 127 * 1000000;
      break;
    case 98:
      lfo1_rate = 1 + (float)value;
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

void lfo1_square() {
  if (   ((int)((now_sec) / ((1 / lfo1_rate) / 2) ) % 2) == 0) {
    lfo1_value = 4095;
  }
  else {
    lfo1_value = 0;
  }
}

void lfo1_saw() {

  long period_usec = 1 / (lfo1_rate / 1000000);

  long long y = (long)now % period_usec;
  lfo1_value = ((float) ((4095 * y) /  (period_usec)) );



}

void lfo1_triangle() {

  long period_usec = 1 / (lfo1_rate / 1000000);
  
  long long y = (long long)now % period_usec;
 
  float value =  ((float) ((4095 * y) /  (period_usec)) );

  if ((((int)floor(now * (lfo1_rate / 1000000))) % 2) == 0) {
    lfo1_value = value;
  }
  else {

    lfo1_value =  4095 - value;
  }
}

void lfo1_noise() {
}


void loop() {
  midiA.read();
  //TODO: ADD CHECK FOR OVERFLOW
  now = micros();
  now_msec = now / 1000;
  now_sec = now / 1000000;

  switch (lfo1_type) {
    case 0:
      lfo1_square();
      break;
    case 1:
      lfo1_saw();
      break;
    case 2:
      lfo1_triangle();
      break;
    case 3:
      lfo1_noise();
      break;
  }


  // PITCH
  if (!note_stack.isEmpty()) {
    if ((now > (note_change_time + slide_time ))  || ((legato) && (first_note ))  ) { // normal pitch section
      curr_pitch = note_stack.peek() + pitch_bend_cv; //TODO: change from peek to the highest note
    }
    else if ((portamento) || ((legato) && (note_stack.count() >= 1))  ) { // portamento section, or legato
      curr_pitch = last_pitch + (now - note_change_time) * (note_stack.peek() - last_pitch) / slide_time + pitch_bend_cv ;
    }

  }


  // ADSR
  if (!note_stack.isEmpty()) {

    // attack section
    if ((now <= (env_trig_time + env1_attack )) && env_trig ) {
      curr_filter =  (env1_min + (now - env_trig_time) * (env1_max - env1_min) / (env1_attack) )   ;
    }

    // decay section
    else if  (((now >= (env_trig_time + env1_attack )) && (now <= (env_trig_time + env1_decay )) && env_trig)  ) {
      curr_filter = (env1_max + (now - env_trig_time) * (env1_sustain - env1_max) / env1_decay )  ;
    }
    else if ((now > (env_trig_time + env1_attack + env1_decay )) && env_trig )  { // sustain section
      curr_filter =  env1_sustain  ;

    }
  }
  else {
    // Release section
    // trick to make it fake exponential... divide the release time by 2, start with a steeper slope
    if ((now <= (note_off_time + env1_release / 2))) {
      curr_filter = env_at_note_off + ( now -  note_off_time) * ( env1_min -  env_at_note_off) / (env1_release / 2)  ;
      env_at_half_note_off = curr_filter;
    }
    else if ((now <= (note_off_time + env1_release))) { // Release section
      curr_filter = env_at_half_note_off + ( now -  note_off_time) * ( env1_min -  env_at_half_note_off) / env1_release  ;
    }
    else { // note off
      curr_filter = 0;
      //digitalWrite(PIN_GATE, HIGH);
    }
  }

  curr_filter = curr_filter * ((float)vcf_env_amt / vcf_env_amt_max) + vcf + lfo1_value * (lfo1_amount / lfo1_amount_max);

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
  setOutput((int)curr_pitch);
  setOutput_filter((int)curr_filter);
}
