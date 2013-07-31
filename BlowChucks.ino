/**
 * Copyright (c) 2012-2013, Gordon S. Good (velo27 [at] yahoo [dot] com)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * The author's name may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL GORDON S. GOOD BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <i2c_t3.h>
#include "WiiChuckTeensy3.h"

// The MIDI channel on which we send note data from
// breath actions, as well as continuous controller
// data.
#define MIDI_CHANNEL 1
// The MIDI channel we use for Ableton Live scene control
#define SCENE_MGMT_MIDI_CHANNEL 2
// The MIDI channel we use for Ableton Live track control
#define TRACK_MGMT_MIDI_CHANNEL 2
// The MIDI note for Live scene launch
#define SCENE_LAUNCH_MIDI_NOTE 0
// The MIDI note for the Live previous scene action
#define SCENE_PREV_MIDI_NOTE 1
// The MIDI note for the Live next scene action
#define SCENE_NEXT_MIDI_NOTE 2
// Suppress multiple scene lanches if < 100 ms apart
#define SCENE_LAUNCH_DELAY 100
// The MIDI note for the Live previous track action
#define TRACK_PREV_MIDI_NOTE 3
// The MIDI note for the Live next track action
#define TRACK_NEXT_MIDI_NOTE 4

// The threshold level for sending a note on event. If the
// sensor is producing a level above this, we should be sounding
// a note.
#define NOTE_ON_THRESHOLD 100
// The maximum raw pressure value you can generate by
// blowing into the tube.
#define MAX_PRESSURE 1023

// MIDI breath controller number
#define BREATH_CONTROLLER 2
// If true, then send breath values as MIDI aftertouch values,
// which is necessary for some Synths, e.g. Korg M1
#define SEND_MIDI_AT true
// MIDI controller number for nunchuck roll sensor
#define CHUCK_R_ROLL_CONTROLLER 17  // 17 = General Purpose Slider 2
// TODO(ggood) - the Teensy doesn't want to seem to send any
// messages on GP Slider 1 (CC 16) - huh?
// MIDI controller number for nunchuck pitch sensor
#define CHUCK_R_PITCH_CONTROLLER 18  // 18 = General Purpose Slider 3

// The three states of our state machine
// No note is sounding
#define NOTE_OFF 1
// We've observed a transition from below to above the
// threshold value. We wait a while to see how fast the
// breath velocity is increasing
#define RISE_TIME 0
// A note is sounding
#define NOTE_ON 3

// The maximum number of simultaneous notes (polyphony)
#define MAX_NOTES 9

// Send breath controller data no more than every BC_INTERVAL
// milliseconds
#define BC_INTERVAL 10
// Send continuous controller data no more than every CC_INTERVAL
// milliseconds
#define CC_INTERVAL 10

// When interpreting roll of each nunchuck, rotated -90 to +90
// produces these values
#define ROLL_MIN -65
#define ROLL_MAX 65

// The nine notes that the player selects using the joystick
#define HOME_NOTE 62
byte pentatonic_notes[9] = {
  HOME_NOTE + 0,
  HOME_NOTE + 2,
  HOME_NOTE + 5,
  HOME_NOTE + 7,
  HOME_NOTE + 9,
  HOME_NOTE + 12,
  HOME_NOTE + 14,
  HOME_NOTE + 17,
  HOME_NOTE + 19
};

byte diminished_notes[9] = {
  HOME_NOTE + 0,
  HOME_NOTE + 0,
  HOME_NOTE + 1,
  HOME_NOTE + 3,
  HOME_NOTE + 4,
  HOME_NOTE + 6,
  HOME_NOTE + 7,
  HOME_NOTE + 9,
  HOME_NOTE + 10
};

byte dorian_notes[9] = {
  HOME_NOTE + 0,
  HOME_NOTE + 0,
  HOME_NOTE + 2,
  HOME_NOTE + 3,
  HOME_NOTE + 5,
  HOME_NOTE + 7,
  HOME_NOTE + 9,
  HOME_NOTE + 10,
  HOME_NOTE + 12
};

byte phrygian_notes[9] = {
  HOME_NOTE + 0,
  HOME_NOTE + 0,
  HOME_NOTE + 1,
  HOME_NOTE + 3,
  HOME_NOTE + 5,
  HOME_NOTE + 7,
  HOME_NOTE + 8,
  HOME_NOTE + 10,
  HOME_NOTE + 12
};

byte *notes = dorian_notes;

// Mappings from joystick regions
#define CENTER 0
#define NORTH 1
#define NORTHEAST 2
#define EAST 3
#define SOUTHEAST 4
#define SOUTH 5
#define SOUTHWEST 6
#define WEST 7
#define NORTHWEST 8

// The base note - used to detect note changes
int base_note;
// We keep track of which notes are sounding, so we know
// which note to turn off when breath stops.
int sounding_notes[MAX_NOTES];
// This is the selected note, which may be different than
// the base note
int selected_note;
// The value read from the breath sensor
int breath_sensor_value;
// The current state of our state machine
int state;
// The time that we noticed the breath off -> on transition
unsigned long breath_on_time = 0L;
// The breath value at the time we observed the transition
int initial_breath_value;
// The breath controller value we will send
int bc_val;
// The last time we sent a breath controller value
unsigned long bc_send_time = 0L;
// The last time we sent a MIDI CC value
unsigned long cc_send_time = 0L;

// Array of pointers to functions for various harmonizations
#define NUM_HARMONIZATIONS 6
void (*harmonizations[NUM_HARMONIZATIONS])(byte) = {stacked_fourths, hassell, sharp9,
                                                    randomRootslash, random64slash, schwantner};
// The index of the harmonization currently being used
byte current_harmonization = 0;

// The nunchuck objects (2)
WiiChuckTeensy3 chuck_left = WiiChuckTeensy3(1); // The nunchuck controller - Left Hand
WiiChuckTeensy3 chuck_right = WiiChuckTeensy3(0); // The nunchuck controller - Right Hand

// Current button state from nunchucks
byte buttonRState = 0;
byte prevButtonRState = 0;
byte buttonLState = 0;
byte prevButtonLState = 0;
// True when both right buttons pressed
boolean sceneLaunchArmed;
// Accelerometer history
int xVal, yVal, zVal, zSum, zAvg;
int zValues[10] = {
  0};
unsigned long noteOnTime;
int i;


void setup() {
  state = NOTE_OFF;  // initialize state machine
  // Initialize the nunchuck-related things
  prevButtonRState = buttonRState = 0;
  delay(100);
  chuck_left.begin();
  chuck_right.begin();
}


// ================
// Note selection
// ================
byte get_direction(int x_dir, int y_dir) {
  if (x_dir < 0) {
    if (y_dir < 0) {
      return SOUTHWEST;
    } 
    else if (y_dir > 0) {
      return NORTHWEST;
    } 
    else {
      return WEST;
    }
  } 
  else if (x_dir > 0) {
    if (y_dir < 0) {
      return SOUTHEAST;
    } 
    else if (y_dir > 0) {
      return NORTHEAST; // OK
    } 
    else {
      return EAST;  // OK
    }
  } 
  else {
    if (y_dir < 0) {
      return SOUTH;
    } 
    else if (y_dir > 0) {
      return NORTH;  // OK
    } 
    else {
      return CENTER;  // OK
    }
  }
}

// Examine the joystick positions and compute a new
// base_note
byte get_base_note() {
#define ZERO_TOLERANCE 10
  byte new_note;

  // Read joystick
  int x = chuck_right.readJoyX();
  int y = chuck_right.readJoyY();
  int x_dir, y_dir = 0;

  // Map x and y to a direction (-1, 0, 1)
  if (abs(x) < ZERO_TOLERANCE) {
    x_dir = 0;
  } 
  else {
    x_dir = x > 0 ? 1 : -1;
  }
  if (abs(y) < ZERO_TOLERANCE) {
    y_dir = 0;
  } 
  else {
    y_dir = y > 0 ? 1 : -1;
  }
  // Now map pairs of x, y directions to a note
  byte direction = get_direction(x_dir, y_dir);
  new_note = notes[direction];

  // Now look at the left joystick to select an octave, based
  // on X axis centered, or left/right of center.
  int lx = chuck_left.readJoyX();
  if (lx < -ZERO_TOLERANCE) {
    new_note -= 12;
  } 
  else if (lx > ZERO_TOLERANCE) {
    new_note += 12;
  }
  return new_note;
}

byte get_num_notes() {
  // Return the number of harmonization notes that should
  // be played.
  int roll = chuck_left.readRoll();
  int roll_c = constrain(roll, ROLL_MIN, ROLL_MAX);
  int n = map(roll_c, ROLL_MIN, ROLL_MAX, 1, MAX_NOTES);
  return n;
}

// ========= Harmonizatons ============
void stacked_fourths(byte note) {
  // Base + 4th above + 4th above that
  sounding_notes[0] = note;
  sounding_notes[1] = note + 5;
  sounding_notes[2] = note + 10;
  for (byte i = 3; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}

void hassell(byte note) {
  sounding_notes[0] = note;
  sounding_notes[1] = note + 2;
  sounding_notes[2] = note + 5;
  for (byte i = 3; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}

void sharp9(byte note) {
  // Base + tritone + 4th above that
  sounding_notes[0] = note;
  sounding_notes[1] = note + 6;
  sounding_notes[2] = note + 10;
  for (byte i = 3; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}

void randomRootslash(byte note) {
  // Major chord in root position, and a bass note
  // randomly selected from one of 4 notes.
  int bass_notes[4] = {0, -12, -7, -5};
  sounding_notes[0] = note;
  sounding_notes[1] = note + 4;
  sounding_notes[2] = note + 7;
  sounding_notes[3] = note + bass_notes[random(0, 3)]; 
  for (byte i = 4; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}

void random64slash(byte note) {
  // Major chord in second inversion, and a bass note
  // randomly selected from one of 4 notes.
  int bass_notes[4] = {0, -12, -7, -5};
  sounding_notes[0] = note;
  sounding_notes[1] = note + 5;
  sounding_notes[2] = note + 9;
  sounding_notes[3] = note + bass_notes[random(0, 3)]; 
  for (byte i = 4; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}

void schwantner(byte note) {
  // A chord from Joseph Schwantner's "And the Mountains
  // Rising Nowhere" that I've always really liked. A
  // revoicing of a minor 11th chord.
  sounding_notes[0] = note;
  sounding_notes[1] = note + 7;
  sounding_notes[2] = note + 14;
  sounding_notes[3] = note + 15;
  sounding_notes[4] = note + 22;
  sounding_notes[5] = note + 29;
  for (byte i = 6; i < get_num_notes(); i++) {
    sounding_notes[i] = -1;
  }
}
// ========= End Harmonizatons ============

void notes_on() {
  // Turn on all currently selected notes
  harmonizations[current_harmonization](base_note = get_base_note());
  int velocity = get_velocity(initial_breath_value, breath_sensor_value, RISE_TIME);
  for (byte i = 0; i < get_num_notes(); i++) {
    if (sounding_notes[i] != -1) {
      usbMIDI.sendNoteOn(sounding_notes[i], velocity, MIDI_CHANNEL);
    }
  }
}

void notes_off() {
  // Turn off all currently sounding notes
  for (byte i = 0; i < MAX_NOTES; i++) {
    if (sounding_notes[i] != -1) {
      usbMIDI.sendNoteOff(sounding_notes[i], 127, MIDI_CHANNEL);
    }
  }
}

// ================
// Breath sensor, note on/off, MIDI breath controller routines
// ================

int get_velocity(int initial, int final, unsigned long time_delta) {
  // Just return a fixed velocity value, which works well for wind-controller-aware
  // synth patches. For playing other patches, you might try the commented-out
  // line, which sends a variable velocity.
  return 127;
  //return map(constrain(final, NOTE_ON_THRESHOLD, MAX_PRESSURE), NOTE_ON_THRESHOLD, MAX_PRESSURE, 0, 127);
}

void handle_breath_sensor() {
  // Process pressure sensor data
  // read the input on analog pin 0
  breath_sensor_value = analogRead(A0);
  if (state == NOTE_OFF) {
    if (breath_sensor_value > NOTE_ON_THRESHOLD) {
      // Value has risen above threshold. Move to the RISE_TIME
      // state. Record time and initial breath value.
      breath_on_time = millis();
      initial_breath_value = breath_sensor_value;
      state = RISE_TIME;  // Go to next state
    }
  } 
  else if (state == RISE_TIME) {
    if (breath_sensor_value > NOTE_ON_THRESHOLD) {
      // Has enough time passed for us to collect our second
      // sample?
      if (millis() - breath_on_time > RISE_TIME) {
        // Yes, so calculate MIDI notes and velocity, then send note on events
        notes_on();
        state = NOTE_ON;
      }
    } 
    else {
      // Value fell below threshold before RISE_TIME passed. Return to
      // NOTE_OFF state (e.g. we're ignoring a short blip of breath)
      state = NOTE_OFF;
    }
  } 
  else if (state == NOTE_ON) {
    byte new_base_note = get_base_note();
    if (new_base_note != base_note) {
      notes_off();
      state = NOTE_OFF;
      handle_breath_sensor();
    }
    if (breath_sensor_value < NOTE_ON_THRESHOLD) {
      // Value has fallen below threshold - turn the note off
      notes_off(); 
      state = NOTE_OFF;
    } 
    else {
      // Is it time to send more breath controller data?
      if (millis() - bc_send_time > BC_INTERVAL) {
        // Map the sensor value to the breath controller range 0-127
        unsigned int mval = constrain(breath_sensor_value, NOTE_ON_THRESHOLD, MAX_PRESSURE);
        bc_val = map(mval, NOTE_ON_THRESHOLD, MAX_PRESSURE, 0, 127);
        usbMIDI.sendControlChange(BREATH_CONTROLLER, bc_val, MIDI_CHANNEL);
        if (SEND_MIDI_AT) {
          usbMIDI.sendAfterTouch(bc_val, MIDI_CHANNEL);
        }
        bc_send_time = millis();
      }
    }
  }
}

// ================
// Button/Ableton Scene Management Routines
// ================

byte get_button_r_state() {
  return (chuck_right.buttonC ? 0x2 : 0x0) | (chuck_right.buttonZ ? 0x1 : 0x0);
}

byte get_button_l_state() {
  return (chuck_left.buttonC ? 0x02 : 0x0) | (chuck_left.buttonZ ? 0x1 : 0x0);
}

void scene_next() {
  usbMIDI.sendNoteOn(SCENE_NEXT_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(SCENE_NEXT_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
}

void scene_prev() {
  usbMIDI.sendNoteOn(SCENE_PREV_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(SCENE_PREV_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL); 
}

void scene_launch() {
  usbMIDI.sendNoteOn(SCENE_LAUNCH_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(SCENE_LAUNCH_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
}

void track_next() {
  usbMIDI.sendNoteOn(TRACK_NEXT_MIDI_NOTE, 100, TRACK_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(TRACK_NEXT_MIDI_NOTE, 100, TRACK_MGMT_MIDI_CHANNEL);
}

void track_prev() {
  usbMIDI.sendNoteOn(TRACK_PREV_MIDI_NOTE, 100, TRACK_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(TRACK_PREV_MIDI_NOTE, 100, TRACK_MGMT_MIDI_CHANNEL); 
}

void handle_joystick() {
  selected_note = get_base_note();
  if (state == NOTE_ON && selected_note != base_note) {
    // player moved joystick to a new position. Turn off
    // current note and turn on new one.
    handle_breath_sensor();
  }
}

void handle_scene_navigation() {
  // Check for scene up/down and launch. All actions occur on
  // button release.
  prevButtonRState = buttonRState;
  buttonRState = get_button_r_state();
  sceneLaunchArmed = (buttonRState == 0x03) || sceneLaunchArmed;
  if (buttonRState == 0x0) {
    if (sceneLaunchArmed) {
      scene_launch();
      sceneLaunchArmed = false;
    } 
    else if (prevButtonRState != buttonRState) {
      // Button state transitioned
      switch (prevButtonRState) {
      case 0x01:
        scene_next();
        break;
      case 0x02:
        scene_prev();
        break;
      case 0x03:
        //scene_launch();
        break;
      }
    }
  }
}

void handle_track_navigation() {
  // Check for track prev/next. All actions occur on
  // button release.
  prevButtonLState = buttonLState;
  buttonLState = get_button_l_state();
  if (prevButtonLState != buttonLState) {
    // Button state transitioned
    switch (prevButtonLState) {
    case 0x01:
      track_next();
      break;
    case 0x02:
      track_prev();
      break;
    }
  }
}

// ================
// Pitch/Roll mapping to MIDI continuous controllers
// ================

void handle_pitch_roll() {
  byte ccValRoll;
  byte ccValPitch;
  // Is it time to send more CC data?
  if (state != NOTE_OFF && millis() - cc_send_time > CC_INTERVAL) {
    // Map the CC values from the nunchuck
    ccValRoll = map(chuck_right.readRoll(), -180, 180, 0, 127);
    usbMIDI.sendControlChange(CHUCK_R_ROLL_CONTROLLER, ccValRoll, MIDI_CHANNEL);
    chuck_right.update();  // XXX(ggood) is this needed?
    delay(1);  // XXX(ggood) and this?
    ccValPitch = map(chuck_right.readPitch(), 0, 140, 0, 127);
    usbMIDI.sendControlChange(CHUCK_R_PITCH_CONTROLLER, ccValPitch, MIDI_CHANNEL);
    cc_send_time = millis();
  }
}

void handle_harmonization_change() {
  if (chuck_left.cPressed()) {
    current_harmonization = (current_harmonization + 1) % NUM_HARMONIZATIONS;
  } else if (chuck_left.zPressed()) {
    current_harmonization = (current_harmonization == 0 ? NUM_HARMONIZATIONS -1: current_harmonization - 1);
  }
}

void loop() {
  // Process nunchuck data
  chuck_left.update(); 
  chuck_right.update();


  handle_breath_sensor();
  handle_joystick();
  handle_pitch_roll();
  handle_scene_navigation();
  handle_track_navigation();
  handle_harmonization_change();
}



