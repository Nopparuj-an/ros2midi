# MIDI protocol https://www.songstuff.com/recording/article/midi-message-format/

#### NOTE ON / NOTE OFF ####
#
# 1XXXCCCC 0PPPPPPP 0VVVVVVV
# XXX     - Message (0-OFF, 1-ON)
# CCCC    - Channel 0-15
# PPPPPPP - Pitch 0-127
# VVVVVVV - Velocity 0-127

#### SUSTAIN PEDAL ####
#
# 1XXXCCCC 0PPPPPPP 0VVVVVVV
# XXX     - Message (011-CC)
# CCCC    - Channel 0-15
# PPPPPPP - Controller 64
# VVVVVVV - Value 0-127

# uint8 status
# uint8 channel
# uint8 pitch
# uint8 velocity

# Python mido Message
string type
uint8 channel
uint8 note
uint8 velocity
uint8 control
uint8 value