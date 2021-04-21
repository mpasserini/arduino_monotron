
void readInputs()
{
  env_trig_mode = digitalRead(ENV_TRIG_MODE_PIN); 
  legato_or_portamento = digitalRead(LEGATO_PORTAMENTO_PIN);
  filter_track = digitalRead(FILTER_TRACK_PIN);
  
  // turn velcity on based on filt/vca selection
  velocity_on_off = digitalRead(ADSR_VELOCITY_PIN); 
  if (velocity_on_off != velocity_on_off_prev){
    if (env_sel){
      filt_velocity_on_off = velocity_on_off;
    } else {
      vca_velocity_on_off = velocity_on_off;
    }
    velocity_on_off_prev = velocity_on_off;
  }

  
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
  

  lfo_delay = 1/log10((float)analogRead(LFO_DELAY_PIN)+2) * 10000 - 3321;
  slide_time_msec = pot_max - analogRead(PORTAMENTO_TIME_PIN);

  
  attack_pot  = analogRead(ATTACK_PIN);
  decay_pot   = analogRead(DECAY_PIN);
  sustain_pot = analogRead(SUSTAIN_PIN);
  release_pot = analogRead(RELEASE_PIN);
  env_amt_pot = analogRead(ENV_AMT_PIN);
  if (attack_pot != attack_pot_prev){
    if (env_sel){
      //log10(0) = not a number
      //1/log10(1) = 1/0 = not a number
      filter_env_attack_msec = 1/log10((float)(attack_pot+2)) * 10000 - 3321;
    } else {
      vca_env_attack_msec = 1/log10((float)attack_pot+2) * 10000 - 3321;
    }
    attack_pot_prev = attack_pot;
  }
  if (decay_pot != decay_pot_prev){
    if (env_sel){
      filter_env_decay_msec = 1/log10((float)decay_pot+2) * 10000 - 3321;
    } else {
      vca_env_decay_msec = 1/log10((float)decay_pot+2) * 10000 - 3321;
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
      filter_env_release_msec = 1/log10((float)release_pot+2) * 10000 - 3321;
    } else {
      vca_env_release_msec = 1/log10((float)release_pot+2) * 10000 - 3321;
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
      // don't let the value reach 0 , but a little higher, 1
      lfo_filt_period_msec_new = 1/log10((float)lfo_period_pot+2) * 10000 - 3318;

    } else {
      // don't let the value reach 0 , but a little higher, 1
      lfo_vca_period_msec_new =  1/log10((float)lfo_period_pot+2) * 10000 - 3318;
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
