
// remove glitches
// do so that you change only the next phase params via midi
unsigned int adsr_attack(unsigned long attack_time, 
                         unsigned long current_time, 
                         unsigned long trig_time, 
                         unsigned int wave_table[][wave_table_resolution], 
                         int wave_table_type ) {
  if (attack_time < 0){
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
  if (decay_time < 0 ){
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
  if ((release_time < 0 ) and 
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
