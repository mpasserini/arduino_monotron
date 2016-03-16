# arduino_monotron
The code is for Arduino and is supposed to control a Korg Monotron via MIDI


Done:
CV pitch tracking,
Filter control,
pseudo exponential ADSR,
Legato,  Portamento and normal mode,
pitch bend,
logarithmic scaling for adsr pots,
LFO saw, ramp, square,  triangle
wavetable based LFO, with also waves for exp env to be implemented

Todo:
add several modes for legato,
lfo pitch tracking,
Velocity,
VCA,
Upload schematics,
code with functions written correctly,
When in Legato, let the decay continue or it creates artifacts with short notes
add a proper modulation matrix with several lfos and env going to different destinations
random lfo
exp envelopes based on wavetable
trigger lfo at gate on
change lfo at zero crossing
make lfo affect + and - the wave and not only +
make the adsr changes at new stage to avoid glitches

