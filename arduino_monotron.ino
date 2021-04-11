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
float velocity = 0.0;
bool velocity_on_off = false;
bool velocity_on_off_prev = false;  
bool filt_velocity_on_off = false; 
bool vca_velocity_on_off = false; 
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

unsigned long lfo_delay = 0;

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


  filter_float = ((float)(((filter_env_value * filter_env_amt) /  dac_max + vcf))+lfo_filt_value_scaled);

  
  if (filter_track){
      filter_float = (float)filter_float + (float)curr_pitch;
  }

  if (filt_velocity_on_off){
        filter_float = filter_float*velocity;
  } 

  
  if (filter_float < 0){
    curr_filter=0;
  } else {
      curr_filter = filter_float;
  }

  float  lfo_vca_value_scaled = 0;
  lfo_vca_value_scaled = ((((float)dac_max/2)-(float)lfo_vca_value) /2)*lfo_vca_amt; 
  float vca_float=0;

  if (vca_velocity_on_off){
      vca_float = ((float)(((vca_env_value * vca_env_amt) /  dac_max))+lfo_vca_value_scaled)*velocity  ;
  } else {  
      vca_float = ((float)(((vca_env_value * vca_env_amt) /  dac_max))+lfo_vca_value_scaled) ;
  }
  
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

  //Serial.println(curr_pitch);

  
  setOutput(curr_pitch);
  setOutput_filter((unsigned int)curr_filter);
  setOutput_volume((unsigned int)curr_vca);
}
