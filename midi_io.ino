
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
