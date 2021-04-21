
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

    if (period == 0 ){
      Serial.println("Period is 0, but % 0 is undefined, illegal value");
      return 0;
    }
   
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
