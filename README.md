# arduino_monotron

The code is for Arduino and is supposed to control a Korg Monotron via MIDI


Done:
* CV pitch tracking,
* Filter control,
* pseudo exponential ADSR,
* Legato,  Portamento and normal mode,
* pitch bend,
* logarithmic scaling for adsr pots,
* LFO saw, ramp, square,  triangle
* wavetable based LFO, with also waves for exp env to be implemented
* VCA,
* Different envelopes for VCA and VCF
* Different LFOs for VCA and VCF
* exp envelopes based on wavetable
* trigger lfo at gate on or continuous
* change lfo at zero crossing
* adsr changes at new stage to avoid glitches

Todo:
* add several modes for legato,
* lfo pitch tracking,
* Velocity,
* Add LFO and Env for pitch
* Upload schematics,
* code with functions written correctly,
* When in Legato, let the decay continue or it creates artifacts with short notes
* add a proper modulation matrix with several lfos and env going to different destinations
* random lfo
* make lfo affect + and - the wave and not only +
* save settings to EPROM and recall
