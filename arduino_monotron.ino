#include <MIDI.h>
#include <SPI.h>
#include "libraries/StackArray/StackArray.h" 

//http://www.ripmat.it/mate/d/dc/dcee.html
//https://www.arduino.cc/en/Tutorial/DueSimpleWaveformGenerator
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


// TODO:
// fix legato
// check key tracking

const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int PIN_CS_VOLUME = 42;

// digital input
const int LEGATO_PORTAMENTO_PIN = 22;
const int ENV_TRIG_MODE_PIN = 23;
const int FILTER_TRACK_PIN = 24;
const int ADSR_SELECTOR_PIN = 25;
const int ADSR_VELOCITY_PIN = 26;
const int LFO_RETRIG_PIN = 27;
const int LFO_SELECTOR_PIN = 28;

const int GATE_INPUT_PIN = 29;

int gate_state = LOW;
int gate_prev_state = LOW;

// analog input
const int LFO_DELAY_PIN = 0;
const int PORTAMENTO_TIME_PIN = 1;
const int ATTACK_PIN = 2;
const int DECAY_PIN = 3;
const int SUSTAIN_PIN = 4;
const int RELEASE_PIN = 5;
const int ENV_AMT_PIN = 6;
//const int ATTACK_TIME_PIN = 7;
const int LFO_AMT_PIN =7;
const int LFO_PERIOD_PIN =8;
const int LFO_WAVEFORM = 9;

const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);

const unsigned int dac_max = 4095; 
const unsigned int pot_max = 1023;

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
bool adsr_velocity_on_off = false; 
//bool lfo_velocity_on_off = false; 
unsigned int velocity_max = 126;

unsigned long now_usec = 0;
unsigned long now_msec = 0;

bool first_note = true;
// True = legato, False = portamento
bool legato_or_portamento = true;
unsigned long slide_time_msec = 0;
bool note_on = false;
StackArray <unsigned int> note_stack;

bool filter_track = false;

unsigned int vcf = 0;
//unsigned long vcf_env_amt = 4095;
//unsigned long vca_env_amt = 4095;

unsigned long attack_msec = 0;
unsigned long decay_msec = 0;
unsigned long sustain = 0;
unsigned long release_msec = 0;

// current val and prev are stored so that
// the adsr1/2 or lfo_filt/2 switches don't skip
// when changing
unsigned long attack_pot = 0;
unsigned long decay_pot = 0;
unsigned long sustain_pot = 0;
unsigned long release_pot = 0;
unsigned long env_amt_pot = 0;
unsigned long attack_pot_prev = 0;
unsigned long decay_pot_prev = 0;
unsigned long sustain_pot_prev = 0;
unsigned long release_pot_prev = 0;
unsigned long env_amt_pot_prev = 0;

unsigned long lfo_amt_pot = 0;
unsigned long lfo_period_pot = 0;
unsigned long lfo_wave_pot = 0;
unsigned long lfo_amt_pot_prev = 0;
unsigned long lfo_period_pot_prev = 0;
unsigned long lfo_wave_pot_prev = 0;
bool lfo_retrig_switch = 0;

unsigned long filter_env_attack_msec = 000; //milliseconds
unsigned long filter_env_decay_msec = 400;
unsigned long filter_env_sustain = 2000;
unsigned long filter_env_release_msec = 100;
unsigned long filter_env_amt = 4095;

unsigned long vca_env_attack_msec = 0; //milliseconds
unsigned long vca_env_decay_msec = 0;
unsigned long vca_env_sustain = 4095;
unsigned long vca_env_release_msec = 300;
unsigned long vca_env_amt = 4095;

unsigned int filter_env_at_note_off = 0;
unsigned int vca_env_at_note_off = 0;
const unsigned int filter_env_min = 0;
unsigned long filter_env_value = 0;
unsigned long vca_env_value = 0;

// True: Trigger at every note
// False: Trigger at first note only
bool env_trig_mode = false; 

// True: 1, False: 2
bool env_sel = true;

unsigned long curr_filter = 0;
unsigned long curr_vca = 0;
unsigned int bent_pitch = 0;
unsigned int pitch_bend_cv = 0;
unsigned int pitch_bend_max = 8192;
unsigned int pitch_bend_notes = 12;

enum adsr_states {
  Attack,
  Decay,
  Sustain,
  Release
};
adsr_states adsr1_state = Release;
adsr_states adsr2_state = Release;

float lfo_amt = 0.0;
unsigned int lfo_period = 0;
float lfo_amt_max = 1.0;

unsigned long lfo_filt_trig_time_msec = 0;
unsigned int lfo_filt_wave = 0; 
unsigned int lfo_filt_wave_new = 0; 
unsigned long lfo_filt_period_msec = 1000;
unsigned long lfo_filt_period_msec_new = 1000;
unsigned long lfo_filt_value = 0; 
unsigned long lfo_filt_value_prev = 0;
float lfo_filt_amt = 0.0;
bool lfo_filt_retrig = false;
// true 1, false 2
bool lfo_sel = true;

unsigned long lfo_vca_trig_time_msec = 0;
unsigned int lfo_vca_wave = 0; 
unsigned int lfo_vca_wave_new = 0; 
unsigned long lfo_vca_period_msec = 1000;
unsigned long lfo_vca_period_msec_new = 1000;
unsigned long lfo_vca_value = 0; 
unsigned long lfo_vca_value_prev = 0;
float lfo_vca_amt = 0.0;
bool lfo_vca_retrig = false;

unsigned long lfo_delay;


const int max_wave_types=7;
const int  wave_table_resolution=64; //increase for more precision, 
                                     //from 16 it looks ok?
                                     
unsigned int wave_table[max_wave_types][wave_table_resolution];

unsigned int exp_curve = 3;


const float pi = 3.14159;

void setup() {

  Serial.begin(9600);

  // Fill in Sine wavetable
  for (int i=0; i < wave_table_resolution; i++) {
    float rads = ( 2 * pi * i) / wave_table_resolution;
    wave_table[0][i] = (unsigned int) ((2048 * sin(rads)) + 2048);
  }

  // Fill in Square wavetable
  for (int i=0; i < wave_table_resolution; i++) {
    if (i < (wave_table_resolution / 2)){
      wave_table[1][i] = dac_max;
    }
    else {
      wave_table[1][i] = 0;
    }
  }

  // Fill in Ramp up wavetable
  for (int i=0; i < wave_table_resolution; i++) {
    wave_table[2][i] = (unsigned int)(((unsigned long)i * dac_max) / 
                       wave_table_resolution);
  }

  // Fill in Ramp down wavetable
  for (int i=0; i < wave_table_resolution; i++) {
    wave_table[3][i] = (unsigned int)(dac_max - 
                       (((unsigned long)i * dac_max) / 
                       wave_table_resolution));
  }

  // Fill in triangle wavetable
  for (int i=0; i < wave_table_resolution; i++) {
    if (i < (wave_table_resolution / 2)) {
      wave_table[4][i] = (unsigned int)( (((unsigned long)i * dac_max) / 
                         (wave_table_resolution/2)));
    }
    else {
      wave_table[4][i] = (unsigned int)  dac_max - 
                         ((((unsigned long)i - wave_table_resolution/2 ) * 
                         dac_max) / (wave_table_resolution/2))    ;      
    }
  }

/*
Ascending EXP:
=EXP(-$A1/20)

Descending EXP:
=1-(EXP(-$A1/20))

where 100 is the max A1 value
*/
  // descending
  // adding a 0 on the last value as a "trick" to make it scale better
  for (int i=0; i < (wave_table_resolution-1 ); i++) {
    wave_table[5][i] = (unsigned int) (dac_max * 
                        (exp( (double)-i / ((double)(wave_table_resolution-1) /
                                            exp_curve) 
                            )
                        )
                       );
  }
  wave_table[5][wave_table_resolution-1] = (unsigned int)0;
  
  // ascending
  for (int i=0; i < (wave_table_resolution-1); i++) {
    
    wave_table[6][i] = (unsigned int) (dac_max * 
                        ( 1 - exp( (double)-i / 
                                 ((double)(wave_table_resolution-1)/exp_curve)
                                )
                        )
                       );
  }
  wave_table[6][wave_table_resolution-1] = (unsigned int)dac_max;

  // TODO ADD RANDOM
  
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.setHandleNoteOn(handleNoteOn);
  midiA.setHandleNoteOff(handleNoteOff);
  midiA.setHandleControlChange(handleControlChange);
  midiA.setHandlePitchBend(handlePitchBend);
  pinMode(PIN_CS_PITCH, OUTPUT);
  pinMode(PIN_CS_FILTER, OUTPUT);
  pinMode(PIN_CS_VOLUME, OUTPUT);

  pinMode(ENV_TRIG_MODE_PIN, INPUT_PULLUP);
  pinMode(LEGATO_PORTAMENTO_PIN, INPUT_PULLUP);
  pinMode(FILTER_TRACK_PIN, INPUT_PULLUP);
  pinMode(ADSR_VELOCITY_PIN, INPUT_PULLUP);
  pinMode(ADSR_SELECTOR_PIN, INPUT_PULLUP);
  pinMode(LFO_RETRIG_PIN, INPUT_PULLUP);
  pinMode(LFO_SELECTOR_PIN, INPUT_PULLUP);
  pinMode(GATE_INPUT_PIN, INPUT);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

void readInputs()
{
  // TODO: IMPLEMENT MISSING FUNCIONALITY FOR VELOCITY ETC
  env_trig_mode = digitalRead(ENV_TRIG_MODE_PIN); // implement
  legato_or_portamento = digitalRead(LEGATO_PORTAMENTO_PIN);
  filter_track = digitalRead(FILTER_TRACK_PIN);
  adsr_velocity_on_off = digitalRead(ADSR_VELOCITY_PIN); // implement
  env_sel = digitalRead(ADSR_SELECTOR_PIN);
  lfo_retrig_switch = digitalRead(LFO_RETRIG_PIN); // implement
  lfo_sel = digitalRead(LFO_SELECTOR_PIN);

  if (lfo_retrig_switch){
      if (lfo_sel){
          lfo_vca_retrig = true;
      } else {
          lfo_filt_retrig = true;
      }
  } else {
      if (lfo_sel){
          lfo_vca_retrig = false;
      } else {
          lfo_filt_retrig = false;
      }
  }
  
  // todo: scale the correct values
  lfo_delay = analogRead(LFO_DELAY_PIN);
  slide_time_msec = pot_max - analogRead(PORTAMENTO_TIME_PIN);

  
  attack_pot  = analogRead(ATTACK_PIN);
  decay_pot   = analogRead(DECAY_PIN);
  sustain_pot = analogRead(SUSTAIN_PIN);
  release_pot = analogRead(RELEASE_PIN);
  env_amt_pot = analogRead(ENV_AMT_PIN);
  if (attack_pot != attack_pot_prev){
    if (env_sel){
      filter_env_attack_msec = (pot_max - attack_pot) * 4;
    } else {
      vca_env_attack_msec = (pot_max - attack_pot) * 4;
    }
    attack_pot_prev = attack_pot;
  }
  if (decay_pot != decay_pot_prev){
    if (env_sel){
      filter_env_decay_msec = (pot_max - decay_pot) * 4;
    } else {
      vca_env_decay_msec = (pot_max - decay_pot) * 4;
    }
    decay_pot_prev = decay_pot;
  }
  if (sustain_pot != sustain_pot_prev){
    if (env_sel){
      filter_env_sustain = (pot_max - sustain_pot) * 4;
    } else {
      vca_env_sustain = (pot_max - sustain_pot) * 4;
    }
    sustain_pot_prev = sustain_pot;
  }
  if (release_pot != release_pot_prev){
    if (env_sel){
      filter_env_release_msec = (pot_max - release_pot) * 4;
    } else {
      vca_env_release_msec = (pot_max - release_pot) * 4;
    }
    release_pot_prev = release_pot;
  }
  if (env_amt_pot != env_amt_pot_prev){
    if (env_sel){
      filter_env_amt = (pot_max - env_amt_pot) * 4;
    } else {
      vca_env_amt = (pot_max - env_amt_pot) * 4;
    }
    env_amt_pot_prev = env_amt_pot;
  }

  lfo_amt_pot = analogRead(LFO_AMT_PIN);
  lfo_period_pot = analogRead(LFO_PERIOD_PIN);
  lfo_wave_pot = analogRead(LFO_WAVEFORM);
  if (lfo_amt_pot != lfo_amt_pot_prev){
    if (lfo_sel){
      lfo_filt_amt =  (float)(pot_max - lfo_amt_pot)/(float)pot_max ;
    } else {
      lfo_vca_amt =  (float)(pot_max - lfo_amt_pot)/(float)pot_max ;
    } 
    lfo_amt_pot_prev = lfo_amt_pot;
  } 
  if (lfo_period_pot != lfo_period_pot_prev){
    if (lfo_sel){
      lfo_filt_period_msec_new = ((pot_max - lfo_period_pot))*4;
    } else {
      lfo_vca_period_msec_new = ((pot_max - lfo_period_pot))*4;
    }
    lfo_period_pot_prev = lfo_period_pot;    
  }
  if (lfo_wave_pot != lfo_wave_pot_prev){
    if (lfo_sel){
      lfo_filt_wave_new = (pot_max - lfo_wave_pot) / 170;
    } else {
      lfo_vca_wave_new = (pot_max - lfo_wave_pot) / 170;
    }
    lfo_wave_pot_prev = lfo_wave_pot;
  }
}

void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  note_change_time_msec = now_msec;
  last_pitch = curr_pitch;
  if (note_stack.isEmpty()) {
      env_trig_time_msec = now_msec ;

      if (lfo_filt_retrig) {
          lfo_filt_trig_time_msec = now_msec;
      }
      if (lfo_vca_retrig) {
          lfo_vca_trig_time_msec = now_msec;
      }
  }
  else {
    first_note = false;
  }
  unsigned int in_pitch = dac_max / notes_max * (inNote - notes_lowest) ;
  note_stack.push( in_pitch );
  
  note_on = true;
  //velocity = ((unsigned int)inVelocity*dac_max)/velocity_max;
   velocity =  (((long)4096*((long)inVelocity))/126);  
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  note_change_time_msec = now_msec;
  unsigned int in_pitch = dac_max / notes_max * (inNote - notes_lowest);
  if (!note_stack.isEmpty()) {
    last_pitch = in_pitch;
    note_stack.pop();
  }
  if (note_stack.isEmpty()) {
    note_off_time_msec = now_msec;
    filter_env_at_note_off = filter_env_value;
    vca_env_at_note_off = vca_env_value;
    first_note = true;
    note_on = false;
  }
}

void handlePitchBend(byte channel, int bend) {
  pitch_bend_cv =  ((long)bend * ((dac_max / notes_max) 
                   * pitch_bend_notes) ) / pitch_bend_max;
}

void handleControlChange(byte channel, byte number, byte value) {

  switch (number) {


    // FIRST BUTTON ROW 
//    case 65:

//      if (value == 0) {
//        portamento = false; //TODO: portamento and legato  booleans should 
                            //be fixed, they don't behave correctly, when 
                            //both are 0 then there is a delay between notes
//      }
//      else {
//        portamento = true;
//      }
//      break;
    case 66:
      if (value == 0) {
        legato_or_portamento = false;
      }
      else {
        legato_or_portamento = true;
      }
      break;
    case 67:
      if (value == 0) {
        lfo_filt_wave_new = 0;
      }
      else {
        lfo_filt_wave_new = 1;
      }
      break;
    case 68:
      if (value == 0) {
        lfo_filt_wave_new = 2;
      }
      else {
        lfo_filt_wave_new = 3;
      }
      break;
    case 69:
      if (value == 0) {
        lfo_filt_wave_new = 5;
      }
      else {
        lfo_filt_wave_new = 6;
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
    case 71:
      if (value == 0) {
        lfo_filt_retrig = true;
      }
      else {
        lfo_filt_retrig = false;
      }
      break;

    // case 72

    // SECOND BUTTON ROW

    case 73:
      if (value == 0) {
        lfo_vca_wave = 0;
      }
      else {
        lfo_vca_wave = 1;
      }
      break;
    case 74:
      if (value == 0) {
        lfo_vca_wave = 2;
      }
      else {
        lfo_vca_wave = 3;
      }
      break;
    case 75:
      if (value == 0) {
        lfo_vca_wave = 5;
      }
      else {
        lfo_vca_wave = 6;
      }
      break;


    // FIRST KNOB ROW

    case 81:
      vcf = (float)value / 127 * dac_max;
      break;


    case 82:
      vca_env_amt = ((unsigned long)value * dac_max) / 127 ;
      break;

    case 83:
      filter_env_amt = ((unsigned long)value * dac_max) / 127 ;
      break;

    case 84:
      lfo_filt_amt = value;
      break;

    case 85:
      lfo_vca_amt = value;
      break;

    // Use attack current and attack next so it gets changed only at the next env run
    // VCF
    case 89:
      //log scale:
      filter_env_attack_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      // linear version:
      //filter_env_attack_msec =  (unsigned long)value * 10000 / 127 ;
      break;

    case 90:
      //log scale:
      filter_env_decay_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      // linear version:
      //filter_env_decay_msec = (unsigned long)value * 10000 / 127;
      break;

    case 91:
      filter_env_sustain = (unsigned long)value * dac_max / 127 ;
      break;

    case 92:
      //log scale:
      filter_env_release_msec = (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;


    // VCA
    case 93:
      //log scale:
      vca_env_attack_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;

    case 94:
      //log scale:
      vca_env_decay_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;

    case 95:
      vca_env_sustain = (unsigned long)value * dac_max / 127 ;
      break;

    case 96:
      //log scale:
      vca_env_release_msec = (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;


    // THIRD KNOB ROW      

    case 97:
      slide_time_msec = (unsigned long)value * 1000 / 127;
      break;

    case 98:
      lfo_filt_period_msec_new = 1 + (1 / log10(129 - (unsigned long)value)) * 
                             10000 - 4738;
      break;


    case 99:
      lfo_vca_period_msec_new = 1 + (1 / log10(129 - (unsigned long)value)) * 
                             10000 - 4738;
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


void setOutput_volume(unsigned int val)
{

  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
  digitalWrite(PIN_CS_VOLUME, LOW);
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS_VOLUME, HIGH);
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


unsigned int lfo_wavetable(unsigned long period, 
                           unsigned long current_time, 
                           unsigned int wave_table[][wave_table_resolution], 
                           int wave_table_type, 
                           unsigned long trig_time, 
                           bool lfo_retrig) {

  float wave_table_index_float = 0;
  //if (not lfo_retrig ){
  //    trig_time = 0;                                                   
  //}
                         
  // if period != 0
  //float wave_table_index_float;

  // REMOVE <<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //lfo_retrig = 0;
   
    wave_table_index_float = get_wave_table_index_float(wave_table_resolution,
                               ((current_time-trig_time) % period),
                               period);
  
  int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = (wave_table_index + 1) % wave_table_resolution;


  unsigned int interpolated_value = line_by_two_points(wave_table_index_float, 
                                            wave_table_index, 
                                            wave_table_index_next,
                                            (int)wave_table[wave_table_type][wave_table_index], 
                                            (int)wave_table[wave_table_type][wave_table_index_next]);
  
  return interpolated_value;
}


// return next index or stop at max to avoid overflow
int next_index(int wave_table_index, int wave_table_resolution){
  int wave_table_index_next = 0;

  if (wave_table_index >= (wave_table_resolution - 1)){ 
     wave_table_index_next = wave_table_resolution - 1; 
  }
  else wave_table_index_next = wave_table_index + 1;
  return wave_table_index_next;
}



  // line by two points
  // x  - x1   y  - y1
  // ------- = ------
  // x2 - x1   y2 - y1
  //
  // index_float - index   y    - val1
  // ------------------- = -----------
  // index_next  - index   val2 - val1
  //
  // y = val1 + (index_float - index) + (val2 - val1)
  //            -------------------------------------
  //             index_next - index 
unsigned int line_by_two_points(float x, int x1, int x2 ,int y1, int y2){
  if ((x2 - x1) == 0) {
    return y2;
  }
  return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1) ;
}


// index : resolution-1 = (curr-trigtime) : attack_time 
// => index = (resolution-1 * (curr-trigtime)) / attack_time
float get_wave_table_index_float(const int wave_table_resolution,
                                 unsigned long time_pos,
                                 unsigned long time_len
                                ) {
  return (float)((wave_table_resolution-1) * time_pos) / time_len;
}


// remove glitches
// do so that you change only the next phase params via midi
unsigned int adsr_attack(unsigned long attack_time, 
                         unsigned long current_time, 
                         unsigned long trig_time, 
                         unsigned int wave_table[][wave_table_resolution], 
                         int wave_table_type ) {
  if (attack_time < 0 or attack_time > dac_max){
    Serial.println("adsr_attack input out of range");
  }
  if (current_time < trig_time){
    Serial.println("adsr_attack current_time too small");
  }  
                          
  // avoid division by 0
  if (attack_time == 0){
      return 0;
  } else {

      float wave_table_index_float = get_wave_table_index_float(wave_table_resolution,
                                                            (current_time - trig_time),
                                                            attack_time); 
                                                             
      int wave_table_index = (int)wave_table_index_float;
  
      int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);
                                         
      unsigned int interpolated_value = line_by_two_points(wave_table_index_float, 
                                            wave_table_index, 
                                            wave_table_index_next,
                                            (int)wave_table[wave_table_type][wave_table_index], 
                                            (int)wave_table[wave_table_type][wave_table_index_next]);
      return interpolated_value;
  }
}

unsigned int  adsr_decay( unsigned long decay_time, 
                          unsigned long current_time, 
                          unsigned long trig_time, 
                          unsigned long attack_time, 
                          unsigned long sustain_value,  
                          unsigned int wave_table[][wave_table_resolution], 
                          int wave_table_type
                        ) {
  if (decay_time < 0 or decay_time > dac_max){
    Serial.println("adsr_decay input out of range");
  }
  if (current_time < (trig_time + attack_time)){
    Serial.println("adsr_decay current_time too small");
  }
                          
  // avoid division by 0
  if (decay_time == 0){
    return sustain_value;
  }

  float wave_table_index_float = get_wave_table_index_float(wave_table_resolution,
                                                            (current_time - (trig_time+attack_time)),
                                                            decay_time);

  int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);

  unsigned int interpolated_value = line_by_two_points(wave_table_index_float, 
                                            wave_table_index, 
                                            wave_table_index_next,
                                            (int)wave_table[wave_table_type][wave_table_index], 
                                            (int)wave_table[wave_table_type][wave_table_index_next]);

  return sustain_value + ((interpolated_value * 
                            (dac_max - sustain_value)) / dac_max);
}


unsigned int  adsr_sustain( unsigned long sustain_value
                          ){
    if (sustain_value < 0 or sustain_value > dac_max){
      Serial.println("adsr_sustain input out of range");
    }
    return sustain_value  ;
}


unsigned int adsr_release( unsigned long release_time, 
                           unsigned long current_time, 
                           unsigned long untrig_time, 
                           unsigned long note_off_value, 
                           unsigned int wave_table[][wave_table_resolution], 
                           int wave_table_type 
                         ){
  if ((release_time < 0 or release_time > dac_max) and 
      (current_time <  untrig_time)){
      Serial.println("adsr_release wrong input");
  }
  if (release_time == 0) {
    return 0;
  }
  if ((current_time - untrig_time) <= (release_time)) {


    float wave_table_index_float = get_wave_table_index_float(wave_table_resolution,
                                                            (current_time - untrig_time),
                                                            release_time);
    
    int wave_table_index = (int)wave_table_index_float;
    int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);

    unsigned int interpolated_value = line_by_two_points(wave_table_index_float, 
                                            wave_table_index, 
                                            wave_table_index_next,
                                            (int)wave_table[wave_table_type][wave_table_index], 
                                            (int)wave_table[wave_table_type][wave_table_index_next]);
  
    return (interpolated_value * note_off_value) / dac_max ;
  }
  else return wave_table[wave_table_type][wave_table_resolution-1];
}


// this function returns the adsr value at a given time
unsigned long adsr( bool note_stack_is_empty, // or gate state
                    unsigned long now_msec, 
                    unsigned long env_trig_time_msec, 
                    unsigned long env_attack_msec, 
                    bool note_on,
                    adsr_states adsr_state, 
                    unsigned long env_sustain,
                    unsigned long env_decay_msec,
                    unsigned long env_release_msec,
                    unsigned int env_at_note_off
                    //unsigned long env_value
                   ) {
  unsigned long env_value=0;
  
  if (!note_stack_is_empty) {
    
    unsigned long env_trig_time_diff_msec = now_msec - env_trig_time_msec;

    // modify the state according to current time
    if (( env_trig_time_diff_msec <= env_attack_msec) && note_on){
      adsr_state = Attack; // SIDE EFFECTS? CHECK
    }
    else if ((env_trig_time_diff_msec  > env_attack_msec) && 
             (env_trig_time_diff_msec <= (env_attack_msec + env_decay_msec)) &&
              note_on) {
      adsr_state = Decay;
    }
    else if ((env_trig_time_diff_msec > 
              (env_attack_msec + env_decay_msec)) && note_on ) {
       adsr_state = Sustain;
    }    
  }
  else {
    adsr_state = Release;
  }

  // compute envelope
  switch(adsr_state) {
    case Attack: 
      env_value = adsr_attack( env_attack_msec, 
                               now_msec, 
                               env_trig_time_msec, 
                               wave_table, 
                               6 );
      break;
    case Decay: 
      
      env_value = adsr_decay( env_decay_msec,     
                              now_msec, 
                              env_trig_time_msec, 
                              env_attack_msec, 
                              env_sustain, 
                              wave_table, 
                              5 );
      break;
    case Sustain: 
      env_value = adsr_sustain( env_sustain); 
      break;
    case Release: 
      env_value = adsr_release( env_release_msec, 
                                now_msec, 
                                note_off_time_msec , 
                                env_at_note_off, 
                                wave_table, 
                                5); //TODO change hardcoded exponential 
                                    //type on wavetable
      break;
  }

  return env_value;
}


// this function calculates the pitch CV
unsigned int pitch_cv(
    //StackArray <unsigned int> note_stack,
    bool note_stack_is_empty,
    unsigned int note_stack_peek,
    unsigned int note_stack_count,
    unsigned long now_msec,
    unsigned long note_change_time_msec,
    unsigned long slide_time_msec,
    bool legato_or_portamento,
    bool first_note,
    unsigned int pitch_bend_cv,
    unsigned int last_pitch,
    unsigned int dac_max,    
    unsigned int curr_pitch
  ) {
  unsigned int result_pitch = curr_pitch;

  if (!note_stack_is_empty) {
    if ((now_msec > (note_change_time_msec + slide_time_msec ))  || 
       ((legato_or_portamento) && (first_note )) || (slide_time_msec == 0) ) { 
      // normal pitch section
      //TODO: change from peek to the highest note
      result_pitch = note_stack_peek + pitch_bend_cv; 
      
      //Serial.println(note_stack.peek() );
    }
    else if ((not legato_or_portamento) || ((legato_or_portamento) && (note_stack_count >= 1)) ) { 
      // portamento section, or legato
      if (note_stack_peek > last_pitch) {
        result_pitch = last_pitch + ((now_msec - note_change_time_msec) * 
                     (note_stack_peek - last_pitch)) / slide_time_msec + 
                     pitch_bend_cv ;
      }
      else {
        result_pitch = last_pitch - ((now_msec - note_change_time_msec) * 
                     (last_pitch - note_stack_peek)) / slide_time_msec + 
                     pitch_bend_cv ;
      }
    }
  }

  // boundaries for what the dac is capable of doing
  if (result_pitch > dac_max) {
    result_pitch = dac_max;
  }

  return result_pitch;
}







void loop() {

  unsigned int note_stack_peek = 0;
  bool note_stack_is_empty = true;
  // static

  gate_state = digitalRead(GATE_INPUT_PIN); 
  
  midiA.read();
  now_usec = micros();
  now_msec = now_usec / 1000;

  readInputs();

  // gate goes on
  if ((gate_prev_state == LOW) && (gate_state == HIGH)){
    gate_prev_state = HIGH; 
    note_on=1;
    env_trig_time_msec = now_msec ;
    if (lfo_filt_retrig) {
        lfo_filt_trig_time_msec = now_msec;
    }
    if (lfo_vca_retrig) {
        lfo_vca_trig_time_msec = now_msec;
    }
  }
  // gate goes off
  else if ((gate_prev_state == HIGH) && (gate_state == LOW)){
    gate_prev_state = LOW;
    note_on = 0;
    note_off_time_msec = now_msec;
    filter_env_at_note_off = filter_env_value;
    vca_env_at_note_off = vca_env_value;
  }

  // FILTER
  if (lfo_filt_period_msec == 0) {
    lfo_filt_value = dac_max / 2;
  }
  else {
      lfo_filt_value_prev = lfo_filt_value;
      lfo_filt_value = lfo_wavetable(lfo_filt_period_msec, 
                             now_msec, 
                             wave_table, 
                             lfo_filt_wave, 
                             lfo_filt_trig_time_msec, 
                             lfo_filt_retrig);  
  }

  // knob has changed
  if ((lfo_filt_period_msec != lfo_filt_period_msec_new)
      || (lfo_filt_wave != lfo_filt_wave_new)){
      // wait zero crossing
      if ((lfo_filt_value_prev <= (dac_max/2) and lfo_filt_value >= (dac_max/2))){
        // restart wave at zero crossing
        lfo_filt_trig_time_msec = now_msec; 
        lfo_filt_period_msec = lfo_filt_period_msec_new;
        lfo_filt_wave = lfo_filt_wave_new;
      }
  }
  



  // VCA
  if (lfo_vca_period_msec == 0) {
    lfo_vca_value = dac_max / 2;
  }
  else {
      lfo_vca_value_prev = lfo_vca_value;
      lfo_vca_value = lfo_wavetable(lfo_vca_period_msec, 
                             now_msec, 
                             wave_table, 
                             lfo_vca_wave, 
                             lfo_vca_trig_time_msec, 
                             lfo_vca_retrig);  
  }

  // knob has changed
  if ((lfo_vca_period_msec != lfo_vca_period_msec_new)
      || (lfo_vca_wave != lfo_vca_wave_new)){
      // wait zero crossing
      if ((lfo_vca_value_prev <= (dac_max/2) and lfo_vca_value >= (dac_max/2))){
        // restart wave at zero crossing
        lfo_vca_trig_time_msec = now_msec; 
        lfo_vca_period_msec = lfo_vca_period_msec_new;
        lfo_vca_wave = lfo_vca_wave_new;
      }
  }

  
  
  note_stack_is_empty = note_stack.isEmpty();
  if (!note_stack_is_empty) {
    note_stack_peek = note_stack.peek();
  } 

  note_stack_is_empty = not ((not note_stack_is_empty) || gate_state);

  curr_pitch = pitch_cv( 
                         note_stack_is_empty,
                         note_stack_peek,
                         note_stack.count(),
                         now_msec,
                         note_change_time_msec,
                         slide_time_msec,
                         legato_or_portamento,
                         first_note,
                         pitch_bend_cv,
                         last_pitch, 
                         dac_max, 
                         curr_pitch);

  filter_env_value = adsr(note_stack_is_empty, 
                    now_msec, 
                    env_trig_time_msec, 
                    filter_env_attack_msec, 
                    note_on,
                    adsr1_state, 
                    filter_env_sustain,
                    filter_env_decay_msec,
                    filter_env_release_msec,
                    filter_env_at_note_off);

  vca_env_value = adsr(note_stack_is_empty, 
                    now_msec, 
                    env_trig_time_msec, 
                    vca_env_attack_msec, 
                    note_on,
                    adsr2_state, 
                    vca_env_sustain,
                    vca_env_decay_msec,
                    vca_env_release_msec,
                    vca_env_at_note_off);
                    
  float  lfo_filt_value_scaled = 0;
  lfo_filt_value_scaled = ((((float)dac_max/2)-(float)lfo_filt_value) /2)*lfo_filt_amt; 
  float filter_float=0;
  filter_float = (float)(((filter_env_value * filter_env_amt) /  dac_max + vcf))+lfo_filt_value_scaled ;
  if (filter_float < 0){
    curr_filter=0;
  } else {
      curr_filter = filter_float;
  }


  // ADD VELOCITY SENSITIVITY HERE!!!
  float  lfo_vca_value_scaled = 0;
  lfo_vca_value_scaled = ((((float)dac_max/2)-(float)lfo_vca_value) /2)*lfo_vca_amt; 
  float vca_float=0;
  vca_float = (float)(((vca_env_value * vca_env_amt) /  dac_max))+lfo_vca_value_scaled ;
  if (vca_float < 0){
    curr_vca=0;
  } else {
      curr_vca = vca_float;
  }

  if (curr_filter > dac_max) { // clear up too big numbers
    curr_filter = dac_max;
  }
  if (curr_vca > dac_max) { 
    curr_vca = dac_max;
  }
  // scale based on velocity
  //curr_filter = curr_filter * velocity;
  //Serial.println(curr_filter);
  //curr_filter = 300;
  //Serial.println(curr_vca);
  
  setOutput(curr_pitch);
  setOutput_filter((unsigned int)curr_filter);
  setOutput_volume((unsigned int)curr_vca);
}
