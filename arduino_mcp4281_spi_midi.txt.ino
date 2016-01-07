#include <MIDI.h>
#include <SPI.h>

const int PIN_GATE = 45;
const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
const int pitch_max = 4096;
const int notes_max = 60;
const int notes_lowest = 24;
int note_on_count = 0;
// min note 24
// max note 84, 5 oct
// max note 96, 6 oct
long note_on_time = 0;
long note_off_time = 0;
//long portamento = 500000;
long portamento = 0;
int last_pitch = 0;
int desired_pitch = 0;
int curr_pitch = 0;
long now = 0;

bool note_on = false;

int vcf = 0;
int vcf_env_amt = 2048;
const int vcf_env_amt_max = 4096;
long env1_attack = 200000;
long env1_decay = 300000;
int env1_sustain = 1200;
long env1_release = 1000000;
int env_at_note_off = 0;
const int env1_min = 0;
const int env1_max = 4096;
const int filter_max = 4096;
float curr_filter;


void setup() {
  Serial.begin(9600);
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.setHandleNoteOn(handleNoteOn);
  midiA.setHandleNoteOff(handleNoteOff);
  pinMode(PIN_CS_PITCH, OUTPUT);
  pinMode(PIN_CS_FILTER, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  note_on = true;
  note_on_count += 1;
  note_on_time = micros();
  int note_cv = pitch_max / notes_max * (inNote - notes_lowest);
  last_pitch = curr_pitch;
  desired_pitch = note_cv;
  note_on_time = micros();
  digitalWrite(PIN_GATE, HIGH);
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  note_on_count -= 1;
  if (note_on_count <= 0) {
    ///////////////////////////// digitalWrite(PIN_GATE, LOW);
    note_on = false;
    note_off_time = micros();
    env_at_note_off = curr_filter;
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

void loop() {
  midiA.read();
  //TODO: ADD CHECK FOR OVERFLOW
  now = micros();
  if (now > (note_on_time + portamento ))  { // normal pitch section
    curr_pitch = desired_pitch;
  }
  else { // portamento section
    curr_pitch = last_pitch + (now - note_on_time) * (desired_pitch - last_pitch) / portamento ;
  }

  if (note_on) {
    if (now <= (note_on_time + env1_attack )) { // attack section
      curr_filter =  (env1_min + (now - note_on_time) * (env1_max - env1_min) / env1_attack )   ;
    }
    else if  ((now >= (note_on_time + env1_attack )) && (now <= (note_on_time + env1_decay ))) { // decay section
      curr_filter = (env1_max + (now - note_on_time) * (env1_sustain - env1_max) / env1_decay )  ;
    }
    else if (now > (note_on_time + env1_attack + env1_decay ))  { // sustain section
      curr_filter =  env1_sustain ;
    }
  }
  else { // note off
    if (now > (note_off_time + env1_release )) {
      curr_filter = 0;
    }
    else { // Release section
      curr_filter = ( env_at_note_off + (now - note_off_time) * (env1_min - env_at_note_off) / env1_release )   ;
    }
  }
  //curr_filter = (int)((float) vcf + (float) curr_filter * ((float)vcf_env_amt / (float) vcf_env_amt_max)) ;
  if (curr_filter > filter_max) { // clear up too big numbers
    curr_filter = filter_max;
  }
  setOutput(curr_pitch);
  setOutput_filter((int)curr_filter);
}
