

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
