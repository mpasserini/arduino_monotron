#include <MIDI.h>
#include <SPI.h>
#include <StackArray.h>

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


const int PIN_GATE = 45;
const int PIN_CS_PITCH = 44;
const int PIN_CS_FILTER = 46;
const int PIN_CS_VOLUME = 42;
const int GAIN_1 = 0x1;
const int GAIN_2 = 0x0;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);

const unsigned int dac_max = 4095;

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
unsigned int velocity_max = 126;

unsigned long now_usec = 0;
//unsigned long now_sec = 0;
unsigned long now_msec = 0;
bool legato = true;
bool first_note = true;
bool portamento = false;
unsigned long slide_time_msec = 100;
bool note_on = false;
StackArray <unsigned int> note_stack;

unsigned int vcf = 0;
unsigned long vcf_env_amt = 0;
unsigned long env1_attack_msec = 0; //milliseconds
unsigned long env1_decay_msec = 0;
unsigned long env1_sustain = 0;
unsigned long env1_release_msec = 0;

unsigned long vca_env_amt = 0;
unsigned long env2_attack_msec = 0; //milliseconds
unsigned long env2_decay_msec = 0;
unsigned long env2_sustain = 0;
unsigned long env2_release_msec = 0;

unsigned int env1_at_note_off = 0;
unsigned int env2_at_note_off = 0;
const unsigned int env1_min = 0;
unsigned long env1_value = 0;
unsigned long env2_value = 0;
bool env_trig = false;
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


unsigned int lfo_amount_max = 127;

unsigned int lfo1_type = 0; //
unsigned long lfo1_period_msec = 1000;
unsigned long lfo1_period_msec_new = 1000;
unsigned long lfo1_value = 0; 
unsigned int lfo1_amount = 0;
bool lfo1_retrig = true;
unsigned long lfo1_trig_time_msec = 0;

unsigned int lfo2_type = 0; //
unsigned long lfo2_period_msec = 1000;
unsigned long lfo2_period_msec_new = 1000;
unsigned long lfo2_value = 0; 
unsigned int lfo2_amount = 0;
bool lfo2_retrig = true;
unsigned long lfo2_trig_time_msec = 0;


const int max_wave_types=7;
const int  wave_table_resolution=16; //increase for more precision, 
                                     //maybe not needed? 
                                     //It's  already interpolated
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
  for (int i=0; i < wave_table_resolution; i++) {
    wave_table[5][i] = (unsigned int) (dac_max * 
                        (exp( (double)-i / ((double)wave_table_resolution /
                                            exp_curve) 
                            )
                        )
                       );
  }

  for (int i=0; i < wave_table_resolution; i++) {
    wave_table[6][i] = (unsigned int) (dac_max * 
                        ( 1 - exp( (double)-i / 
                                 ((double)wave_table_resolution/exp_curve)
                                )
                        )
                       );
  }

  

  
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.setHandleNoteOn(handleNoteOn);
  midiA.setHandleNoteOff(handleNoteOff);
  midiA.setHandleControlChange(handleControlChange);
  midiA.setHandlePitchBend(handlePitchBend);
  pinMode(PIN_CS_PITCH, OUTPUT);
  pinMode(PIN_CS_FILTER, OUTPUT);
  pinMode(PIN_CS_VOLUME, OUTPUT);
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
    lfo1_trig_time_msec = now_msec;
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
    env1_at_note_off = env1_value;
    env2_at_note_off = env2_value;
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
    case 65:

      if (value == 0) {
        portamento = false; //TODO: portamento and legato  booleans should 
                            //be fixed, they don't behave correctly, when 
                            //both are 0 then there is a delay between notes
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
        lfo1_type = 5;
      }
      else {
        lfo1_type = 6;
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
        lfo1_retrig = true;
      }
      else {
        lfo1_retrig = false;
      }
      break;

    // case 72

    // SECOND BUTTON ROW

    case 73:
      if (value == 0) {
        lfo2_type = 0;
      }
      else {
        lfo2_type = 1;
      }
      break;
    case 74:
      if (value == 0) {
        lfo2_type = 2;
      }
      else {
        lfo2_type = 3;
      }
      break;
    case 75:
      if (value == 0) {
        lfo2_type = 5;
      }
      else {
        lfo2_type = 6;
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
      vcf_env_amt = ((unsigned long)value * dac_max) / 127 ;
      break;

    case 84:
      lfo1_amount = value;
      break;

    case 85:
      lfo2_amount = value;
      break;

    // SECOND KNOB ROW
    // VCF
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
      env1_sustain = (unsigned long)value * dac_max / 127 ;
      break;

    case 92:
      //log scale:
      env1_release_msec = (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;


    // VCA
    case 93:
      //log scale:
      env2_attack_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;

    case 94:
      //log scale:
      env2_decay_msec =  (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;

    case 95:
      env2_sustain = (unsigned long)value * dac_max / 127 ;
      break;

    case 96:
      //log scale:
      env2_release_msec = (1 / log10(129 - (unsigned long)value)) * 10000 - 4738;
      break;


    // THIRD KNOB ROW      

    case 97:
      slide_time_msec = (unsigned long)value * 1000 / 127;
      break;

    case 98:
      lfo1_period_msec_new = 1 + (1 / log10(129 - (unsigned long)value)) * 
                             10000 - 4738;
      break;


    case 99:
      lfo2_period_msec_new = 1 + (1 / log10(129 - (unsigned long)value)) * 
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
  // if period != 0
  float wave_table_index_float;
  if (lfo_retrig == true){
    wave_table_index_float = (float)(wave_table_resolution * 
                             ((current_time - trig_time) % period)) / period;
  }
  else {
    wave_table_index_float = (float)(wave_table_resolution * 
                             (current_time % period)) / period;
  }

  int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = (wave_table_index + 1) % wave_table_resolution;
  return wave_table[wave_table_type][wave_table_index] + 
         ((wave_table_index_float - wave_table_index) * 
         ((int)wave_table[wave_table_type][wave_table_index_next] - 
         (int)wave_table[wave_table_type][wave_table_index]));  
}


int next_index(int wave_table_index, int wave_table_resolution){
  int wave_table_index_next = 0;

  if (wave_table_index >= (wave_table_resolution - 1)){ 
     wave_table_index_next = wave_table_resolution - 1; 
  }
  else wave_table_index_next = wave_table_index + 1;
  return wave_table_index_next;
}

//TODO: refactor common parts of functions
// remove glitches
// do so that you change only the next phase params via midi
unsigned int adsr_attack(unsigned long attack_time, 
                         unsigned long current_time, 
                         unsigned long trig_time, 
                         bool trig_state, 
                         unsigned int wave_table[][wave_table_resolution], 
                         int wave_table_type ) {
  float wave_table_index_float = (float)(wave_table_resolution * 
                                 (current_time - trig_time)) / attack_time;
  int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);

  unsigned int interpolated_value = 
                      wave_table[wave_table_type][wave_table_index] + 
                      ((wave_table_index_float - wave_table_index) * 
                      ((int)wave_table[wave_table_type][wave_table_index_next] -
                      (int)wave_table[wave_table_type][wave_table_index]));  
  return interpolated_value;
}

unsigned int  adsr_decay( unsigned long decay_time, 
                          unsigned long current_time, 
                          unsigned long trig_time, 
                          bool trig_state, 
                          unsigned long attack_time, 
                          unsigned long sustain_value,  
                          unsigned int wave_table[][wave_table_resolution], 
                          int wave_table_type
                        ) {
  float wave_table_index_float = (float)(wave_table_resolution * 
                                 (current_time - (trig_time + attack_time))) / 
                                 decay_time;
  int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);

  unsigned int interpolated_value = 
                    wave_table[wave_table_type][wave_table_index] + 
                    ((wave_table_index_float - wave_table_index) * 
                    ((int)wave_table[wave_table_type][wave_table_index_next] - 
                    (int)wave_table[wave_table_type][wave_table_index]));  

  return sustain_value + ((interpolated_value * 
                            (dac_max - sustain_value)) / dac_max);
}


unsigned int  adsr_sustain( unsigned long sustain_value, 
                            unsigned long current_time, 
                            unsigned long trig_time, 
                            bool trig_state, 
                            unsigned long attack_time, 
                            unsigned long decay_time
                          ){
    return sustain_value  ;
}


unsigned int adsr_release( unsigned long release_time, 
                           unsigned long current_time, 
                           unsigned long untrig_time, 
                           unsigned long note_off_value, 
                           unsigned int wave_table[][wave_table_resolution], 
                           int wave_table_type 
                         ){
  if ((current_time - untrig_time) <= (release_time)) {
    float wave_table_index_float = (float)(wave_table_resolution * 
                                   (current_time - untrig_time)) 
                                   / release_time;
    int wave_table_index = (int)wave_table_index_float;
  int wave_table_index_next = next_index(wave_table_index, 
                                         wave_table_resolution);

    unsigned int interpolated_value = 
                   wave_table[wave_table_type][wave_table_index] + 
                   ( (wave_table_index_float - wave_table_index) * 
                     (
                       (int)wave_table[wave_table_type][wave_table_index_next] - 
                       (int)wave_table[wave_table_type][wave_table_index]
                     )
                   );  
    return (interpolated_value * note_off_value) / dac_max ;
  }
  else 
    return wave_table[wave_table_type][wave_table_resolution];
}


// this function returns the adsr value at a given time
unsigned long adsr( bool note_stack_is_empty, 
                    unsigned long now_msec, 
                    unsigned long env_trig_time_msec, 
                    unsigned long env_attack_msec, 
                    bool env_trig, 
                    adsr_states adsr_state, 
                    unsigned long env_sustain,
                    unsigned long env_decay_msec,
                    unsigned long env_release_msec,
                    unsigned int env_at_note_off
                   ) {
  unsigned long env_value=0;
  
  if (!note_stack_is_empty) {
    
    unsigned long env_trig_time_diff_msec = now_msec - env_trig_time_msec;

    // modify the state according to current time
    if (( env_trig_time_diff_msec <= env_attack_msec) && env_trig){
      adsr_state = Attack; // SIDE EFFECTS? CHECK
    }
    else if ((env_trig_time_diff_msec  > env_attack_msec) && 
             (env_trig_time_diff_msec <= (env_attack_msec + env_decay_msec)) &&
              env_trig) {
      adsr_state = Decay;
    }
    else if ((env_trig_time_diff_msec > 
              (env_attack_msec + env_decay_msec)) && env_trig ) {
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
                               env_trig, 
                               wave_table, 
                               6 );
      break;
    case Decay: 
      env_value = adsr_decay( env_decay_msec,     
                              now_msec, 
                              env_trig_time_msec, 
                              env_trig, 
                              env_attack_msec, 
                              env_sustain, 
                              wave_table, 
                              5 );
      break;
    case Sustain: 
      env_value = adsr_sustain( env_sustain,
                                now_msec, 
                                env_trig_time_msec, 
                                env_trig, 
                                env_attack_msec,  
                                env_decay_msec); 
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
    bool legato,
    bool first_note,
    unsigned int pitch_bend_cv,
    bool portamento,
    unsigned int last_pitch,
    unsigned int dac_max,    
    unsigned int curr_pitch
  ) {
  unsigned int result_pitch = curr_pitch;


  
  if (!note_stack_is_empty) {
    if ((now_msec > (note_change_time_msec + slide_time_msec ))  || 
       ((legato) && (first_note )) || (slide_time_msec == 0) ) { 
      // normal pitch section
      //TODO: change from peek to the highest note
      result_pitch = note_stack_peek + pitch_bend_cv; 
      
      //Serial.println(note_stack.peek() );
    }
    else if ((portamento) || ((legato) && (note_stack_count >= 1)) ) { 
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






 // if (!note_stack_is_empty) {
 //   if (now_msec > note_change_time_msec ) { 
 //     result_pitch = note_stack_peek; 
 //   }
 //   }

  


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
  
  midiA.read();
  now_usec = micros();
  now_msec = now_usec / 1000;
 // now_sec = now_usec / 1000000;


  // todo: 
  // add random
  // fix attack / lfo interaction
  // lfo envelope?
  // multiple lfos
  
  unsigned int lfo1_value_prev = lfo1_value;
  lfo1_value = lfo_wavetable(lfo1_period_msec, 
                             now_msec, 
                             wave_table, 
                             lfo1_type, 
                             lfo1_trig_time_msec, 
                             lfo1_retrig);    
  if (lfo1_period_msec != lfo1_period_msec_new){
    if (lfo1_value >= 2048 && lfo1_value_prev <= 2048){
       lfo1_period_msec = lfo1_period_msec_new;
       lfo1_trig_time_msec = now_msec;
       lfo1_value = lfo_wavetable(lfo1_period_msec, 
                                  now_msec, 
                                  wave_table, 
                                  lfo1_type, 
                                  lfo1_trig_time_msec, 
                                  lfo1_retrig);
    }
  }





  unsigned int lfo2_value_prev = lfo2_value;
  lfo2_value = lfo_wavetable(lfo2_period_msec, 
                             now_msec, 
                             wave_table, 
                             lfo2_type, 
                             lfo2_trig_time_msec, 
                             lfo2_retrig);    
  if (lfo2_period_msec != lfo2_period_msec_new){
    if (lfo2_value >= 2048 && lfo2_value_prev <= 2048){
       lfo2_period_msec = lfo2_period_msec_new;
       lfo2_trig_time_msec = now_msec;
       lfo2_value = lfo_wavetable(lfo2_period_msec, 
                                  now_msec, 
                                  wave_table, 
                                  lfo2_type, 
                                  lfo2_trig_time_msec, 
                                  lfo2_retrig);
    }
  }


  


  note_stack_is_empty = note_stack.isEmpty();
  if (!note_stack_is_empty) {
    note_stack_peek = note_stack.peek();
  } 
  curr_pitch = pitch_cv( 
                         note_stack_is_empty,
                         note_stack_peek,
                         note_stack.count(),
                         now_msec,
                         note_change_time_msec,
                         slide_time_msec,
                         legato,
                         first_note,
                         pitch_bend_cv,
                         portamento,
                         last_pitch, 
                         dac_max, 
                         curr_pitch);

  env1_value = adsr(note_stack_is_empty, 
                    now_msec, 
                    env_trig_time_msec, 
                    env1_attack_msec, 
                    env_trig, 
                    adsr1_state, 
                    env1_sustain,
                    env1_decay_msec,
                    env1_release_msec,
                    env1_at_note_off);


  env2_value = adsr(note_stack_is_empty, 
                    now_msec, 
                    env_trig_time_msec, 
                    env2_attack_msec, 
                    env_trig, 
                    adsr2_state, 
                    env2_sustain,
                    env2_decay_msec,
                    env2_release_msec,
                    env2_at_note_off);
                    
  
  curr_filter = (env1_value * vcf_env_amt) /  dac_max + vcf + 
                ((lfo1_value * lfo1_amount) / lfo_amount_max) ;

  curr_vca = (env2_value * vca_env_amt) /  dac_max  + 
                ((lfo2_value * lfo2_amount) / lfo_amount_max) ; // ADD VELOCITY SENSITIVITY HERE!!!

  
  if (curr_filter > dac_max) { // clear up too big numbers
    curr_filter = dac_max;
  }

  // scale based on velocity
  //curr_filter = curr_filter * velocity;
  //Serial.println(curr_filter);
  //curr_filter = 300;
  //Serial.println(curr_filter);
  setOutput(curr_pitch);
  setOutput_filter((unsigned int)curr_filter);
  setOutput_volume((unsigned int)curr_vca);
}


