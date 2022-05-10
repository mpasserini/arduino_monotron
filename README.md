# arduino_monotron

The project contains code and schematics which explains how to control a Korg Monotron with Arduino Mega.
I put this into a 1U rackmount unit with some switches and pots. I should soon post some photos and a video.

It consists of a modified Korg Monotron board controller by and Arduino Mega board. The Arduino board received MIDI Input, and outputs control signals via three DACs. The DACs have the following uses:

* control the pitch of the VCO
* control the cutoff of the VCF
* control how much the the VCA is open

The schematics folder show how to modify the Monotron board and where to connect the control connectors. It also shows simple diagrams showing how to  conenct the DAC, MIDI,  etc. The VCA is not present on the Montron, so I added a very simple VCA using a vactrol (photoresistor). I had to leave out the gate control, the LFO from the original board, the touch keyboard and the internal speaker.

The software contains MIDI handling functions, and is able to generate envelopes and LFOs which can be used to alter the VCO, VCF and VCA in real time. It also handles the inputs from the various knobs.

It's possible to control the synth with CV Gate as well, bypassing MIDI control.
