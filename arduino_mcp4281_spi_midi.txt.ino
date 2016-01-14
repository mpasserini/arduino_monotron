#include <MIDI.h>
#include <SPI.h>
#include <StackArray.h>

const int PIN_GATE = 45;
const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
const int pitch_max = 4096;
const int notes_max = 60;
const int notes_lowest = 24;
// min note 24
// max note 84, 5 oct
// max note 96, 6 oct
long note_on_time = 0;
long note_change_time = 0;
long note_off_time = 0;
//long slide_time = 0;
int last_pitch = 0;
//int desired_pitch = 0;
int curr_pitch = 0;
float velocity = 0;
float velocity_scaling = 90; // 0-100


// this is double otherwise some operations overflow... check carefully
double now = 0;
bool legato = true;
bool first_note = true;
bool portamento = false;
long slide_time = 100000;
bool note_on = false;
StackArray <int> note_stack;

int vcf = 0;
int vcf_env_amt = 2048;
const int vcf_env_amt_max = 4096;
long env1_attack = 200000;
long env1_decay = 300000;
int env1_sustain = 3000;
long env1_release = 2000000;
int env_at_note_off = 0;
const int env1_min = 0;
const int env1_max = 4096;
const int filter_max = 4096;
float curr_filter = 0;
int bent_pitch = 0;
int pitch_bend_cv = 0;
int pitch_bend_max = 8192;
int pitch_bend_notes = 12;

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

  note_on_time = micros();
  note_change_time = note_on_time;
  int note_cv = pitch_max / notes_max * (inNote - notes_lowest);
  last_pitch = curr_pitch;
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

void handlePitchBend(byte channel, int bend){
  pitch_bend_cv =  ((long)bend * ((pitch_max / notes_max) * pitch_bend_notes) ) / pitch_bend_max; 
}

void handleControlChange(byte channel, byte number, byte value){
  
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

void loop() {
  midiA.read();
  //TODO: ADD CHECK FOR OVERFLOW
  now = micros();

  // PITCH
  if (!note_stack.isEmpty()) {
    if ((now > (note_change_time + slide_time ))  || ((legato) && (first_note ))  ) { // normal pitch section
      curr_pitch = note_stack.peek() + pitch_bend_cv;
    }
    else if ((portamento) || ((legato) && (note_stack.count() >= 1))  ) { // portamento section, or legato
      curr_pitch = last_pitch + (now - note_change_time) * (note_stack.peek() - last_pitch) / slide_time + pitch_bend_cv ;
    }

  }


  // ADSR
  if (!note_stack.isEmpty()) {
    if ((now <= (note_on_time + env1_attack )) && (note_stack.count() == 1) ) { // attack section
      curr_filter =  (env1_min + (now - note_on_time) * (env1_max - env1_min) / env1_attack )   ;
    }
    else if  (((now >= (note_on_time + env1_attack )) && (now <= (note_on_time + env1_decay ))) && (note_stack.count() == 1) ) { // decay section
      curr_filter = (env1_max + (now - note_on_time) * (env1_sustain - env1_max) / env1_decay )  ;
    }
    else if (now > (note_on_time + env1_attack + env1_decay ))  { // sustain section
      curr_filter =  env1_sustain  ;
    }
  }
  else {
    if (now <= (note_off_time + env1_release )) { // Release section

      curr_filter = (env_at_note_off + ( now -  note_off_time) * ( env1_min -  env_at_note_off) / env1_release )  ;
    }
    else { // note off
      curr_filter = 0;
      //digitalWrite(PIN_GATE, HIGH);
    }
  }

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
