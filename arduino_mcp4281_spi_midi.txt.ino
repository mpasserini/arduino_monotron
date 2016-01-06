#include <MIDI.h>
#include <SPI.h>

const int PIN_GATE = 49;
const int PIN_CS = 53;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
int note_on_count = 0;
// min note 24
// max note 84, 5 oct
// max note 96, 6 oct
long note_on_time = 0;
long portamento_time = 500000;
int last_cv = 0;
int desired_cv = 0;
int curr_cv = 0;
long now = 0;

void setup() {
  Serial.begin(9600);
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.setHandleNoteOn(handleNoteOn);
  midiA.setHandleNoteOff(handleNoteOff);
  pinMode(PIN_CS, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  note_on_count += 1;
  note_on_time = micros();
  int note_cv = 4096 / 60 * (inNote - 24);
  last_cv = curr_cv;
  desired_cv = note_cv;
  note_on_time = micros();
  digitalWrite(PIN_GATE, HIGH);
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  note_on_count -= 1;
  if (note_on_count <= 0) {
    digitalWrite(PIN_GATE, LOW);
  }
}


//assuming single channel, gain=2
void setOutput(unsigned int val)
{

  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
  digitalWrite(PIN_CS, LOW);
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS, HIGH);
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
  if (now > (note_on_time + portamento_time ))  {
    curr_cv = desired_cv;
  }
  else {
    curr_cv = last_cv + (now - note_on_time) * (desired_cv - last_cv) / portamento_time ;
  }
  setOutput(curr_cv);
}
