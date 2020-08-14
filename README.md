# arduino_monotron

This is a new version of the code, it is for Arduino and is supposed to output 2 envelopes and 1 LFO. It's for modular by in my case is for a DIY Monotron-based modular synth.


Done:
* Filter control,
* pseudo exponential ADSR,
* LFO saw, ramp, square,  triangle
* wavetable based LFO, with also waves for exp env to be implemented
* Different envelopes for VCA and VCF
* Different LFOs for VCA and VCF
* exp envelopes based on wavetable
* trigger lfo at gate on or continuous
* change lfo at zero crossing
* adsr changes at new stage to avoid glitches

Todo:
* Velocity,
* Add LFO and Env for pitch
* Upload schematics,
* code with functions written correctly,
* add a proper modulation matrix with several lfos and env going to different destinations
* random lfo
* make lfo affect + and - the wave and not only +
* save settings to EPROM and recall
* fix sound glitches, in some ENV settings
* build a proper case
* add an output buffer opamp
