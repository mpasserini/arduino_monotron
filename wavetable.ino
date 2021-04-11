
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
