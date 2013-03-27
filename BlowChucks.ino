/**
 * Copyright (c) 2012, Gordon S. Good (velo27 [at] yahoo [dot] com)
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
// The MIDI channel we use for Ableton Live control
#define SCENE_MGMT_MIDI_CHANNEL 2
// The MIDI note for Live scene launch
#define SCENE_LAUNCH_MIDI_NOTE 0
// The MIDI note for the Live previous scene action
#define SCENE_PREV_MIDI_NOTE 1
// The MIDI note for the Live next scene action
#define SCENE_NEXT_MIDI_NOTE 2
// Suppress multiple scene lanches if < 100 ms apart
#define SCENE_LAUNCH_DELAY 100

// The threshold level for sending a note on event. If the
// sensor is producing a level above this, we should be sounding
// a note.
#define NOTE_ON_THRESHOLD 80
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
#define RISE_TIME 2
// A note is sounding
#define NOTE_ON 3

// The maximum number of simultaneous notes (polyphony)
#define MAX_NOTES 4

// Send breath controller data no more than every AT_INTERVAL
// milliseconds
#define BC_INTERVAL 10
// Send continuous controller data no more than every CC_INTERVAL
// milliseconds
#define CC_INTERVAL 10

// The nine notes that the player selects using the joystick
unsigned int notes[9] = {
  60, 62, 65, 67, 69, 72, 74, 77, 79};

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
// The value read from the sensor
int sensor_value;
// The state of our state machine
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

// The nunchuck objects (2)
WiiChuckTeensy3 chuck_left = WiiChuckTeensy3(0); // The nunchuck controller - Left Hand
WiiChuckTeensy3 chuck_right = WiiChuckTeensy3(1); // The nunchuck controller - Right Hand

// Current button state from nunchuck
byte buttonState = 0;
byte prevButtonState = 0;
// True when both buttons pressed
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
  prevButtonState = buttonState = 0;
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

// Examine the joystick positions and computer a new
// base_note
byte get_base_note() {
#define ZERO_TOLERANCE 10
  byte new_note;

  // Read joystick
  int x = chuck_left.readJoyX();
  int y = chuck_left.readJoyY();
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
  int lx = chuck_right.readJoyX();
  if (lx < -ZERO_TOLERANCE) {
    new_note -= 12;
  } 
  else if (lx > ZERO_TOLERANCE) {
    new_note += 12;
  }
  return new_note;
}

void update_sounding_notes(byte note) {
  sounding_notes[0] = note;
  sounding_notes[1] = note + 5;
  sounding_notes[2] = note + 10;
  for (byte i = 3; i < MAX_NOTES; i++) {
    sounding_notes[i] = -1;  // XXX(ggood) change for more polyphony
  }
}

void notes_on() {
  update_sounding_notes(base_note = get_base_note());
  int velocity = get_velocity(initial_breath_value, sensor_value, RISE_TIME);
  for (byte i = 0; i < MAX_NOTES; i++) {
    if (sounding_notes[i] != -1) {
      usbMIDI.sendNoteOn(sounding_notes[i], velocity, MIDI_CHANNEL);
    }
  }
}

void notes_off() {
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
  sensor_value = analogRead(A0);
  if (state == NOTE_OFF) {
    if (sensor_value > NOTE_ON_THRESHOLD) {
      // Value has risen above threshold. Move to the RISE_TIME
      // state. Record time and initial breath value.
      breath_on_time = millis();
      initial_breath_value = sensor_value;
      state = RISE_TIME;  // Go to next state
    }
  } 
  else if (state == RISE_TIME) {
    if (sensor_value > NOTE_ON_THRESHOLD) {
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
    if (sensor_value < NOTE_ON_THRESHOLD) {
      // Value has fallen below threshold - turn the note off
      notes_off(); 
      state = NOTE_OFF;
    } 
    else {
      // Is it time to send more breath controller data?
      if (millis() - bc_send_time > BC_INTERVAL) {
        // Map the sensor value to the breath controller range 0-127
        unsigned int mval = constrain(sensor_value, NOTE_ON_THRESHOLD, MAX_PRESSURE);
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

// TODO(ggood) rewrite in terms of bit shift operations
byte get_button_l_state() {
  if (chuck_left.buttonC) {
    if (chuck_left.buttonZ) {
      return 0x03;
    } 
    else {
      return 0x02;
    }
  } 
  else {
    if (chuck_left.buttonZ) {
      return 0x01;
    } 
    else {
      return 0x0;
    }
  }
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
  usbMIDI.sendNoteOn(SCENE_NEXT_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(SCENE_NEXT_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
}

void track_prev() {
  usbMIDI.sendNoteOn(SCENE_PREV_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL);
  usbMIDI.sendNoteOff(SCENE_PREV_MIDI_NOTE, 100, SCENE_MGMT_MIDI_CHANNEL); 
}

void handle_joystick() {
  selected_note = get_base_note();
  if (state == NOTE_ON && selected_note != base_note) {
    // player moved joystick to a new position. Turn off
    // current note and turn on new one.
    handle_breath_sensor();
  }
}

void handle_scene_launch() {
  // Check for scene up/down and launch. All actions occur on
  // button release.
  prevButtonState = buttonState;
  buttonState = get_button_l_state();
  sceneLaunchArmed = (buttonState == 0x03) || sceneLaunchArmed;
  if (buttonState == 0x0) {
    if (sceneLaunchArmed) {
      scene_launch();
      sceneLaunchArmed = false;
    } 
    else if (prevButtonState != buttonState) {
      // Button state transitioned
      switch (prevButtonState) {
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

// ================
// Pitch/Roll mapping to MIDI continuous controllers
// ================

void handle_pitch_roll() {
  byte ccValRoll;
  byte ccValPitch;
  // Is it time to send more CC data?
  if (state != NOTE_OFF && millis() - cc_send_time > CC_INTERVAL) {
    // Map the CC values from the nunchuck
    ccValRoll = map(chuck_left.readRoll(), -180, 180, 0, 127);
    usbMIDI.sendControlChange(CHUCK_R_ROLL_CONTROLLER, ccValRoll, MIDI_CHANNEL);
    chuck_left.update();  // XXX(ggood) is this needed?
    delay(1);  // XXX(ggood) and this?
    ccValPitch = map(chuck_left.readPitch(), 0, 140, 0, 127);
    usbMIDI.sendControlChange(CHUCK_R_PITCH_CONTROLLER, ccValPitch, MIDI_CHANNEL);
    cc_send_time = millis();
  }
}

void handle_track_change() {
  if (chuck_right.cPressed()) {
    track_prev();
  }
  if (chuck_right.zPressed()) {
    track_next();
  }
}

void loop() {
  // Process nunchuck data
  chuck_left.update(); 
  chuck_right.update();


  handle_breath_sensor();
  handle_joystick();
  handle_pitch_roll();
  handle_scene_launch();
  handle_track_change();
}



