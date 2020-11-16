/*   
    MPE-DIY-Device using a Teensy LC, 4 finger-joysticks and a Led&Key-panel
    Copyright (C) 2019  Mathias Brüssel

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
// ! Important ! Please comment out the next line, if you want to build your MPE-DIY-Device exactly as described in the documentation ---
#define PROTOTYP    1         // This is used for wiring with short cables

//  ! Important ! Please comment out this line if your joysticks do not respond to being pressed as expected ---
#define KEYES_JOY   1         // Reason for this: There are joysticks where the switch operates as a closing and others as an opening one

// --- Microbuttons arrays as "bass-fret-board" + PS2-Joysticks as "Benders" to send MPE compatible information per MIDI-Channel for each "string" ---
// --- Read MIDI via USB to change various settings, Write MDII via USB to send MPE-data ---

// Pedal Blackbox: 0.99 - 9.55 kOhm min to max Ring to Tip (Second Ring for Stero == Ring)
// Setting on Black Box Pedal (back switch): "Other" => ADC-Input Voltage VCC on Tip, Measurement Vdd on Ring, Ground on Sleeve

/* -----------------------------
MPE (Multidimensional Polyphonic Expression) DIY Controller brief manual:
"Strike" == Velocity / "Press" == Aftertouch / "Slide" == CC74 (vertically) / "Glide" = Pitchbend (horizontally) / "Lift" == Release-Velocity are according to Roli the "5 Dimensions of Touch"

- Buttons:
01,02,03,04: Use Pitch on (green) / off (blue) // Details see below
05,06,07,08: Use MPE on/off yellow if Pitsh is green and purple if pitch is blue // Details see below
09,10,11,12: Standard-Bass-Tuning (polyphonic), Open-D-Tuning (polyphonic), Standard-Bass-Tuning+Keys from Launchpad are exclusive (monophonic), Open-D-Tuning Keys from Launchpad are exclusive (monophonic) 
13:14,15,16: Octaves: -1 Octave, Standard Octave, +1 Octaves, +2 Octaves (Notes starting with 28 [E] or 26[D]

- whenever Modes (buttons) 01-08 are selected, all controllers used are reset
- whenever Modes (buttons) 09-12 are selected, "all notes off" is send, to prevent hanging notes when octaves and so on are changed and noteoffs get differnt pitch...

- Default:
Use Pitch on, Use MPE off, Standard Bass-Tuning (polyphonic),+1 Octave

MPE Joysticks 4 Controller-modes per Joystick availabe:


- 'E' Pitch / MPE
        CC 74
pitch down - pitch up
       Aftertouch

- 'N' non Pitch / MPE
       CC 74
Mod     -  CC 74
       Aftertouch

- 'P' Pitch / Non MPE
        Modulation
pitch down - pitch up
       Aftertouch

- 'C' Controller mode: non Pitch / non MPE
       CC 74
Breath  -  Aftertouch
       Aftertouch

"Incoming Channels Logic:" 
- On Midi-channels 1-10 we will use a regular keyboard as input
- On Midi-channel 11 information that also would be applied via attached Footswitches and Pedals can be applied
- Midi-channel 16 will be associated with a Launchpad and it's possible Bass-strings logic!

----------------------- */ 
#include "TM1638_MB.h"
#define trace(on)  on       // -> on <- to display / -> ; <- to do nothing! 
// ### #define KEYES_JOY build_in  // Decide if we have a closing or opening switch on the Joysticks

typedef int ChannelNote[3];  // Array of channel + note + note_id (relative value 0-15)

class MPE_CTRL_SETTINGS;    // Forward declariation needed, we define it right below this class

class MPE_CTRL
{
public:
  MPE_CTRL(byte channel, byte joy_vertical, byte joy_horizontal, byte button_pin, byte key_pin=0) 
  {
    // --- Initialize inital states / private variables ---
    this->channel = channel;  // Set and memorize Midichannel 1-16 for this instance
    this->joy_vertical = joy_vertical;  // Set and memorize Teensy pin for vertical joystick of this instance for 4 joysticks
    this->joy_horizontal = joy_horizontal;  // Set and memorize Teensy pin for horizontal joystick of this instance for 4 joysticks
    this->button_pin = button_pin;        // Button on Joystick to freeze stuff and so on...
    this->key_pin = key_pin;              // memorize pin of external keypad attached per instance (if any) 0 if not attached
  }
  void freeze_joy()       { m_joy_freeze=true; }
  void melt_joy()         { m_joy_freeze=false; }
  boolean joy_is_frozen() { return m_joy_freeze; }
  
  void adjust_mode(char mpe_mode);    // // set intanal flags accorging to MPE mode - pitch-bend allowed y/n, use which CCs?
  
  void handleJoyStick(MPE_CTRL_SETTINGS* mpe_ptr);
  boolean handleJoyStickButton(MPE_CTRL_SETTINGS* mpe_ptr);
  void joyStickUp(boolean is_frozen, int joy_value, MPE_CTRL_SETTINGS* mpe_ptr);
  void joyStickDown(int joy_value, MPE_CTRL_SETTINGS* mpe_ptr);
  void joyStickRight(int joy_value, MPE_CTRL_SETTINGS* mpe_ptr);
  void joyStickLeft(int joy_value, MPE_CTRL_SETTINGS* mpe_ptr);
  
  void reset_device(MPE_CTRL** mpes, int button_idx);
  char next_mpe_mode();
  char previous_mpe_mode();

private:
  char m_mpe_mode='E';        // remember which MPE mode we have per instance, to select different expression-actions depending on this later 
  byte channel=0;             // initialized externally via constructor
  byte joy_vertical=0;        // initialized externally via constructor
  byte joy_horizontal=0;      // initialized externally via constructor
  byte button_pin;            // pin-Id for freeze on/off from Joystick...
  byte key_pin=0;      // initialized externally via constructor

  bool m_pitch_off = false;    // disable pitchbend on an individual channel
  bool m_mpe_active = true;     // Use controllers associated with MPE-mode or apropriate CCs instead of pitch if needed
  bool m_mpe_relative = false;     // Use CC74 and Aftertouch beginning at 64 / 63 for Lion synth, Seaboardlike relative
  boolean m_joy_freeze = false;  // Joystick-parameters will be remembered if clicked!
    
  int last_vertical_val_up = 0;  // memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int last_vertical_val_down = 0;  // memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int last_horizontal_val_left = 0;// memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int last_horizontal_val_right = 0;// memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"

  int frozen_vertical_val_up = 0;  // memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int frozen_vertical_val_down = 0;  // memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int frozen_horizontal_val_left = 0;// memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"
  int frozen_horizontal_val_right = 0;// memorize values per Joystick axis to compare to the analogue values coming in constantly / "anti-jitter"

  
  int last_key = 0;                 // memorize key from key_pad to generate midi-note from, if provided at all...
  byte m_last_key_note = 0;         // Last note pressed on keypad of MPE-controller device per channel
  unsigned int bouncer = 0;         // debounce by checking keys on keypad not too often...

  char mpe_modes[5] = {'E', 'N', 'L', 'P', 'C'};
  int current_mpe_mode = 0;

int last_val_button = 127;          // Make sure, that the first value we detect from the joystick-button will be updated on restart

protected:
  void all_notes_off();
  // --- Class variables ---
  static int m_octave_val;            // 0-4 octave as basis...
  static int latch_channel; // latch it to foot-controller...
  static boolean ableton_pedal_mode;  // Use MPE-Mapping or in special case Macro-Binding for Ableton for Pedal...
  static int bass_notes_set;        // We have 4 sets to select from 

  static constexpr int bass_notes[3][4] = {{ 16, 21, 26, 31 }, { 14, 21, 26, 30 }, { 12, 24, 36, 48 }}; // Different tunings: E,A,D,G / D,A,D,F# / C,C1,C2,C3
  static byte last_mono_note[17];    // Memorize last note per channel for note-on and note-off including the id
  static bool channel_active[17];     // Memorize Active Channel for note-on and note-off
  static bool sustain_active;          // True if sustain-pedal pressed
  static int channel_offset;          // We use midichannels 1-4 internally, but 11-14 would be sent out, normally...
  static bool play_monophonic;        // Only one note "per string" / per channel allowed at a time 
  static int m_current_channel;         // Channel selected with the last note played, to be assigned for pedal or latch Joystick to current value by footswitch 1 or 2
}; 
// --- Define and initialize all our variable class-variables (default init is 0) ---
constexpr int MPE_CTRL::bass_notes[3][4]; // already initialized in Class, see above...
int MPE_CTRL::latch_channel = 0;
boolean MPE_CTRL::ableton_pedal_mode = false;
int MPE_CTRL::bass_notes_set = 0;       // We have 4 sets to select fro
int MPE_CTRL::m_octave_val = 2;           // 0-4 octave as basis...
byte MPE_CTRL::last_mono_note[17];      // Memorize last note per channel for note-on and note-off including the id
bool MPE_CTRL::channel_active[17];      // Memorize Active Channel for note-on and note-off
bool MPE_CTRL::sustain_active;          // True if sustain-pedal pressed
int MPE_CTRL::channel_offset = 10;      // We use midichannels 1-4 internally, but 11-14 would be sent out, normally...
bool MPE_CTRL::play_monophonic = true;  // Only one note "per string" / per channel allowed at a time 
int MPE_CTRL::m_current_channel = 0; 


class MPE_CTRL_SETTINGS : public MPE_CTRL
{
public:
  MPE_CTRL_SETTINGS() : MPE_CTRL(0,0,0,0,0) // Ignore instance variables, do default init - We use this class (with mostly static content) to modify class-variable in the MPE_CTRL-class!
  {
  }
  static int m_next_octave_val();        // Class function: We have octaves 0 to 4 to select from
  static int m_previous_octave_val();        // Class function: We have octaves 0 to 4 to select from
  static char m_next_bass_notes_set();   // Class function:  We have four different "scales" to select from
  static char m_previous_bass_notes_set();   // Class function:  We have four different "scales" to select from

  static ChannelNote* launch_to_bass( int note_in );      // Convert Launchpad notes as two rows of 4 strings for a bass
  static ChannelNote* keyboard_to_bass( int note_in );    // Convert regular keyboard notes as 4 octaves to 4 strings for a bass
  static void handleNoteOn(byte inChannel, byte inNumber, byte inVelocity); // convert incoming Launchpad note to "fret on bass" to be send out along with the controllers
  static void handleNoteOff(byte inChannel, byte inNumber, byte inVelocity);
  static void handleControlChange(byte inChannel, byte inCC, byte inValue);
  static void set_poly_note( int channel, unsigned int note_id );    // used to decide if any notes played (still)
  static void unset_poly_note( int channel, unsigned int note_id );  // used to decide if any notes played (still)
  static boolean check_channel_active(int id) {return channel_active[id];}
  static boolean check_sustain_active() {return sustain_active;}  // Check if sustain-pedal is currently pressed
  static int get_latch_channel() {return latch_channel;}      // 0 if not latched, else channel of pedal to be connected
  static unsigned int get_keys_id( int channel );    // get Id of highest note if several notes pressed
  static boolean allow_jitter() {return m_jitter;}    // allow jitter j/n?
  void handleMatrixButtons(MPE_CTRL** mpes);

#ifdef PROTOTYP
  TM1638_MB module = {"EEEEem 2", 8,9,10};    // DIO, CLK, STB == Digital I/O, Clock, Strobe  // LED&KEY module for buttons, display and leds
#else
  TM1638_MB module = {"EEEEem 2", 13,14,15};    // DIO, CLK, STB == Digital I/O, Clock, Strobe  // LED&KEY module for buttons, display and leds
#endif                                              // initialisation list for constructor: or TM1638_MB(8,9,10) /  https://stackoverflow.com/questions/11490988/c-compile-time-error-expected-identifier-before-numeric-constant
private:
  static unsigned int m_poly_active[17];    // For each channel a bit will be set, so we can ask "if zero", no notes are played
  static boolean m_jitter;              // Allow jittering yes / no - if off min and max values will be stablized and identical values not resend...
};
unsigned int MPE_CTRL_SETTINGS::m_poly_active[17];    // For each channel a bit will be set for each note possible (max 16), so we can ask "if zero", no notes are played
boolean MPE_CTRL_SETTINGS::m_jitter = false;               // Allow jittering yes / no - if off min and max values will be stablized and identical values not resend...

// --- Decide if to use PitchBend y/n and per Channel Sustain... ---
void MPE_CTRL_SETTINGS::handleMatrixButtons(MPE_CTRL** mpes)
{
int button_out = 0;               // Screwed on device
int button_idx = 0;
static int button_last = 0;
static unsigned int count;
char mpe_mode = ' ';

  count++;  
  if( count%10 != 0)      // ### static "debounce" counter for static function - change back to 100 if only one of two button-arrays is used?
    return;

  button_out = module.get_button(); 
  if( button_out != button_last )
  {  
    switch(button_out)
    {
      case 0:                                   // Mode buttons, change between mod up, aftertouch down, pitch left-right and
      case 1:                                   // up: mod, right aftertouch, down aftertouch, left breath
      case 2:
      case 3:
        if(mpes[button_out]->joy_is_frozen())            // if Joystick-setting is frozen, we can't switch modes here!
          return;

        mpe_mode = mpes[button_out]->next_mpe_mode();
        module.display(mpe_mode, button_out); // return 'E', 'N', 'P', 'C' modes from a "ring"
        mpes[button_out]->adjust_mode(mpe_mode);    // set intanal flags accorging to MPE mode - pitch-bend allowed y/n, use which CCs?
        reset_device(mpes, button_out); // We change modes here, so settings for pitchbend, aftertouch, controllers and their rememered values get reset!
        trace(Serial.printf("Button: %x selected for next_mpe_mode \n", button_out ));
        break;
      
      case 4:
        module.display(m_next_bass_notes_set(), 4);
        all_notes_off();          // Prevent potenially hanging notes on scale-change
        break;

      case 5:
      case 13:            // Backwards select, but identical to forward in this case, because we only have two values here ;-)
        play_monophonic = !play_monophonic;
        if( play_monophonic )
          module.display('m', 5);
        else
          module.display('p', 5);
        all_notes_off();          // Prevent potenially hanging notes on scale-change
        break;

      case 6:
        trace(Serial.printf("Shift button 7 pressed alone - ignore\n" ));
        break;
        
      case 7:
        module.display('0'+m_next_octave_val(), 7);
        all_notes_off();        // Prevent potenially hanging notes on octave-change
        break;

      case 8:                                           // Mode buttons 1-4, but MPE-mode to be selected backwards...
      case 9:                                             
      case 10:
      case 11:
        button_idx = button_out - 8;
        if(mpes[button_idx]->joy_is_frozen())            // if Joystick-setting is frozen, we can't switch modes here!
          return;

        mpe_mode = mpes[button_idx]->previous_mpe_mode();
        module.display(mpe_mode, button_idx); // return 'E', 'N', 'P', 'C' modes from a "ring"
        mpes[button_idx]->adjust_mode(mpe_mode);    // set intanal flags accorging to MPE mode - pitch-bend allowed y/n, use which CCs?
        reset_device(mpes, button_idx); // We change modes here, so settings for pitchbend, aftertouch, controllers and their rememered values get reset!
        trace(Serial.printf("Button: %x selected for previous_mpe_mode \n", button_idx ));
        break;

      case 12:
        module.display(m_previous_bass_notes_set(), 4);
        all_notes_off();          // Prevent potenially hanging notes on scale-change
        break;
        
      case 14:                                // Shift (Button 7) + Button 8 backwards octaves select...
        module.display('0'+m_previous_octave_val(), 7);
        all_notes_off();        // Prevent potenially hanging notes on octave-change
        break;

      case 15:                                // left and right button pressed at once, allow or disable "Jitter"
        m_jitter = !m_jitter;
        m_jitter ? module.display('J', 6) : module.display(' ', 6);
        break;

      case 16:                  // Button released, nothing to do exept remember we had a button pressed! (button_last = button_out; )
        break;

      case 17:                            // Unrecognized button pressed, nothing to do here
        trace(Serial.printf("Button 7 released \n" ));
        module.display('0'+m_previous_octave_val(), 7);
        all_notes_off();        // Prevent potenially hanging notes on octave-change
        break;
             
      default:                            // Unrecognized button pressed, nothing to do here
        trace(Serial.printf("Unrecognized button: %x pressed \n", button_out ));
        break;
    }
    // trace(Serial.printf("bass_notes_set: %d octave_val: %d play_monophonic: %d \n", bass_notes_set, m_octave_val, play_monophonic  ));
    button_last = button_out; 
  } 
}

int MPE_CTRL_SETTINGS::m_next_octave_val()        // Class function: We have octaves 0 to 4 to select from
{
  m_octave_val++;                      // Class variable in parent-class!
  if( m_octave_val > 4 )
    m_octave_val = 0;
  return(m_octave_val);
}

int MPE_CTRL_SETTINGS::m_previous_octave_val()        // Class function: We have octaves 0 to 4 to select from
{
  m_octave_val--;                      // Class variable in parent-class!
  if( m_octave_val < 0 )
    m_octave_val = 4;
  return(m_octave_val);
}

char MPE_CTRL_SETTINGS::m_next_bass_notes_set()   // Class function:  We have four different "scales" to select from
{
static char tuning[4] = {'e', 'd', 'c'};
  bass_notes_set++;                 // Class variable in parent-class!
  if( bass_notes_set > 2 )
    bass_notes_set = 0;
  return(tuning[bass_notes_set]);
}

char MPE_CTRL_SETTINGS::m_previous_bass_notes_set()   // Class function:  We have four different "scales" to select from
{
static char tuning[4] = {'e', 'd', 'c'};
  bass_notes_set--;                 // Class variable in parent-class!
  if( bass_notes_set < 0 )
    bass_notes_set = 2;
  return(tuning[bass_notes_set]);
}

void MPE_CTRL::adjust_mode(char mpe_mode)
{
  m_mpe_mode = mpe_mode;    // Remember which mode we have selected per channel?
  switch(mpe_mode)
  {
    case 'E':
      m_mpe_active = true;
      m_pitch_off = false;
      m_mpe_relative = false;
      break;

    case 'N':
      m_pitch_off = true;
      m_mpe_active = true;
      m_mpe_relative = false;
      break;  

    case 'L':
      m_pitch_off = true;
      m_mpe_active = true;
      m_mpe_relative = true;
      break;  

    case 'P':
      m_pitch_off = false;
      m_mpe_active = false;
      m_mpe_relative = false;
      break;

    case 'C':
      m_pitch_off = true;
      m_mpe_active = false;
      m_mpe_relative = false;
      break;
      
    default:
      trace(Serial.printf("Error: Unknown adjust mode called for Mode '%c'\n", mpe_mode));
  }
}

void MPE_CTRL::reset_device(MPE_CTRL** mpes, int button_idx)
{
  usbMIDI.sendPitchBend(0, button_idx+channel_offset+1); // Reset Pitchbend to middle-position 
  usbMIDI.sendAfterTouch(0, button_idx+channel_offset+1); // Reset Aftertouch for this channel
  usbMIDI.sendControlChange(1, 0, button_idx+channel_offset+1);  // Reset Modwheel
  usbMIDI.sendControlChange(73, 0, button_idx+channel_offset+1);  // Reset "Soundcontroller 3" - X with Aalto/Kaivo
  usbMIDI.sendControlChange(74, 0, button_idx+channel_offset+1);  // Reset "MPE upwards"/brightness ("Soundcontroller 4") - Y with Aalto/Kaivo 
  usbMIDI.sendControlChange(2, 0, button_idx+channel_offset+1);  // Reset Breath

  // reset values per Joystick axis used to compare to the analogue values coming in constantly / "anti-jitter"
  mpes[button_idx]->last_horizontal_val_left = 0;
  mpes[button_idx]->last_horizontal_val_right = 0;
  // ??? ### mpes[button_idx]->last_vertical_val_up = 0;
  mpes[button_idx]->last_vertical_val_down = 0;
}

char MPE_CTRL::next_mpe_mode()            // Returns the modes 'E', 'N', 'P', 'C' from a "ring"...
{
  current_mpe_mode++;
  if( current_mpe_mode > (int)sizeof(mpe_modes)-1 )
    current_mpe_mode = 0; 

  return(mpe_modes[current_mpe_mode]);
}

char MPE_CTRL::previous_mpe_mode()            // Returns the modes 'E', 'N', 'P', 'C' from a "ring" in reversed order...
{
  current_mpe_mode--;
  if( current_mpe_mode < 0 )
    current_mpe_mode = (int)sizeof(mpe_modes)-1; 

  return(mpe_modes[current_mpe_mode]);
}


boolean MPE_CTRL::handleJoyStickButton(MPE_CTRL_SETTINGS* mpe_ptr)
{
int val_in = 0; 
int val_out = 0; 

  val_in = digitalRead(button_pin);           // If possible for cabeling - convention: use same ID for digital button pin as midi-Channel!
  if(val_in)
    #ifdef KEYES_JOY                          // Some joysticks use "closing buttons", others use "opening buttons" - no auto-detect so far, here... 
      val_out = 127;
    #else
      val_out = 0;
    #endif
  else
    #ifdef KEYES_JOY
      val_out = 0;
    #else
      val_out = 127;
    #endif
  if( val_out != last_val_button ) 
  {
    if( val_out )
      joy_is_frozen() ? melt_joy() : freeze_joy(); // Button pressed again, freeze or unfreeze operation of joystick for channel
    
    trace(Serial.printf("Channel %d joy_button press is: %d / %d button_pin %d JoyFreeze: %d\n", channel, val_in, val_out, button_pin, m_joy_freeze));
    if( joy_is_frozen() )
    {
      mpe_ptr->module.led_on(channel-1);
      frozen_vertical_val_up = last_vertical_val_up;
      frozen_vertical_val_down = last_vertical_val_down;
      frozen_horizontal_val_right = last_horizontal_val_right;
      frozen_horizontal_val_left = last_horizontal_val_left;
    }
    else
    {
      mpe_ptr->module.led_off(channel-1);
      frozen_vertical_val_up = 0;
      frozen_vertical_val_down = 0;
      frozen_horizontal_val_right = 0;
      frozen_horizontal_val_left = 0;
    }
    last_val_button = val_out;
  }
  return( joy_is_frozen() );           // If Joystick is latched with current values process nothing new!
}

void MPE_CTRL::joyStickUp(boolean frozen, int val_in, MPE_CTRL_SETTINGS* mpe_ptr)
{
int val_out = 0; 
  // ### -----------------------------------
  if( mpe_ptr->allow_jitter() )
    ;
  switch(m_mpe_mode)
  {
    case 'E':
      break;
    case 'N':
      break;
    case 'L':
      break;
    case 'P':
      break;
    case 'C':
      break;
    default:
      trace(Serial.printf("joyStickUp: Unexpected mpe_mode '%c' \n", m_mpe_mode));
  }
  // ### -----------------------------------

  if( !mpe_ptr->allow_jitter() )
  {
    if(val_in >= 487) // Formerly 496       // allow ca. 5% inaccuracy  - changed from 507 to 497? because of too much "jitter" on Channel 4 - 20190519
       val_out = 0;
    else
    {
      if( val_in <= 8 )
        val_out = 127;
      else
        val_out = min(127, max(0,map(val_in, 490, 1, 0, 127))); // changed? from 513 because of too much "jitter" on Channel 4 - 20190519
    }
  }
  else
    val_out = min(127, max(0,map(val_in, 493, 1, 0, 127)));
    
  if( mpe_ptr->allow_jitter() || val_out != last_vertical_val_up )
  { 
    if( m_mpe_active )
    {
      if( m_mpe_relative )
      {
        val_out = min(val_out/2+64, 127);
        trace(Serial.printf("Channel %d joy_vertical !relative! %d up (CC 74) is %d / %d \n", channel, joy_vertical, val_in, val_out ));
        usbMIDI.sendControlChange(74, val_out, channel+channel_offset);   // Modwheel or "MPE up", CC1 or CC74
      }
      else
      {
        if( frozen )
          val_out = val_in;
        trace(Serial.printf("Channel %d joy_vertical %d up (CC 74) is %d / %d \n", channel, joy_vertical, val_in, val_out ));
        usbMIDI.sendControlChange(74, val_out, channel+channel_offset);   // Modwheel or "MPE up", CC1 or CC74
      }
    }
    else
    {
      trace(Serial.printf("Channel %d joy_vertical %d up (CC 1) is %d / %d \n", channel, joy_vertical, val_in, val_out ));
      usbMIDI.sendControlChange(1, val_out, channel+channel_offset);   // Modwheel or "MPE up", CC1 or CC74
    }
    last_vertical_val_up = val_out;

    if(last_vertical_val_down != 0)                // reset down when up in case if needed
    {
      if(! m_mpe_relative )
      {
        last_vertical_val_down = 0;
        trace(Serial.printf("Channel %d joy_vertical down reset \n", channel ));
        val_out = min(val_out/2+64, 127);
        usbMIDI.sendAfterTouch(0, channel+channel_offset);
      }
    }
  }
}

void MPE_CTRL::joyStickDown(int val_in, MPE_CTRL_SETTINGS* mpe_ptr)
{
int val_out = 0; 
  
  if(val_in <= 515)       // allow ca. 5% inaccuracy 
    val_out = 0;
  else
    if( val_in >= 1019 )
      val_out = 127;
    else
      val_out = max(0,map(val_in, 522, 1023, 0, 127)); // changed from 513 because of too much "jitter" on Channel 4 - 20190519
  if( val_out != last_vertical_val_down ) 
  {
    if( m_mpe_relative )
    {
      val_out = max(0,63-val_out);
      trace(Serial.printf("Channel %d joy_vertical %d down !relative CC74 is %d / %d \n", channel, joy_vertical, val_in, val_out ));
      usbMIDI.sendControlChange(74, val_out, channel+channel_offset);
    }
    else
    {
      trace(Serial.printf("Channel %d joy_vertical %d down AfterTouch is %d / %d \n", channel, joy_vertical, val_in, val_out ));
      usbMIDI.sendAfterTouch(val_out, channel+channel_offset);
    }
    last_vertical_val_down = val_out;
    
    if(last_vertical_val_up != 0)                // reset up when down in case if needed
    {
      if( !m_mpe_relative )
      {
        // ??? last_vertical_val_up = 0;
        trace(Serial.printf("Channel %d joy_vertical up reset \n", channel ));
        if( m_mpe_active )
          usbMIDI.sendControlChange(74, 0, channel+channel_offset);   // Modwheel or "MPE up", CC1 or CC74
        else
          usbMIDI.sendControlChange(1, 0, channel+channel_offset);   // Modwheel or "MPE up", CC1 or CC74
      }
    }
  }
}


void MPE_CTRL::joyStickRight(int val_in, MPE_CTRL_SETTINGS* mpe_ptr)
{
int val_out = 0; 
  
  if( !m_pitch_off ) // pitch bend left/right
  {
    if(val_in <= 10)
      val_out = 8191;
    else
      if(val_in > 490)    // formerly 508, 500
        val_out = 0;
      else
        val_out = map(val_in, 513, 1, 0, 8191);
  }
  else                      // breath / aftertouch left/right
  {
    if(val_in >= 492)       // allow ca. 10% inaccuracy 
      val_out = 0;
    else
      if(val_in <= 10)
        val_out = 127;
      else
        val_out = map(val_in, 513, 1, 0, 127);
  }
  if( val_out != last_horizontal_val_left ) 
  {
    if( !m_pitch_off )
    {
      trace(Serial.printf("Channel %d joy_horizontal %d pitch right is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
      usbMIDI.sendPitchBend(val_out, channel+channel_offset);      // Pitchbend-Range is 14bit => 0 - 16383, No bend == 8192, Teensy Lib -8192, 0, 8191
    }
    else
    {
      if( !m_mpe_active ) 
      {
        trace(Serial.printf("Channel %d joy_horizontal %d  is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
        usbMIDI.sendAfterTouch(val_out, channel+channel_offset);
      }
      else
      {
        trace(Serial.printf("Channel %d joy_horizontal %d MPE right (73) is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
        usbMIDI.sendControlChange(73, val_out, channel+channel_offset);   // X-Controller for Aalto/Kaivo
      }
    }
    last_horizontal_val_left = val_out;
  }
}

void MPE_CTRL::joyStickLeft(int val_in, MPE_CTRL_SETTINGS* mpe_ptr)
{
int val_out = 0; 
  
  if( !m_pitch_off ) // pitch bend left/right
  {
    if(val_in >= 1017)
      val_out = -8192;          // Correct scaling inaccuracy, so that we get max. bend-range to the left
    else
      if(val_in <= 530)         // new Joysticks 20190812 !!! Changed from 523
        val_out = 0;
      else
        val_out = map(val_in, 530, 1023, 0, -8192);  // new Joysticks 20190812 !!! Changed from 513
  }
  else                // breath / aftertouch left/right
  {
    if(val_in <= 529)       // allow ca. 10% inaccuracy 
      val_out = 0;
    else
    {
      if(val_in >= 1017)
        val_out = 127;          // Correct scaling inaccuracy, so that we get max. breath to the left
      else
        val_out = map(val_in, 513, 1023, 0, 127);
    }
  }
  if( val_out != last_horizontal_val_right ) 
  {
    if( !m_pitch_off )
    {
      trace(Serial.printf("Channel %d joy_horizontal %d left is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
      usbMIDI.sendPitchBend(val_out, channel+channel_offset);     // Pitchbend-Range is 14bit => 0 - 16383, No bend == 8192
    }
    else
    {
      if( !m_mpe_active )
      {
        trace(Serial.printf("Channel %d joy_horizontal %d breath left is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
        usbMIDI.sendControlChange(2, val_out, channel+channel_offset);   // Breath, CC2
      }
      else
      {
        trace(Serial.printf("Channel %d joy_horizontal %d MPE left (CC 1) is %d / %d \n", channel, joy_horizontal, val_in, val_out ));
        usbMIDI.sendControlChange(1, val_out, channel+channel_offset);   // Use Modwheel in MPE-Mode for Aalto/Kaivo
      }
    }
    last_horizontal_val_right = val_out;
  }
}

void MPE_CTRL::handleJoyStick(MPE_CTRL_SETTINGS* mpe_ptr) 
{
int val_in = 0; 

  boolean is_frozen = handleJoyStickButton(mpe_ptr);         // Find out if Joystick is frozen or to be unfrozen and so on...
  if(is_frozen)                                              // ### we may optimize freezing process later, there are synths who demand [re]sending of expression-data for every key pressed...
  {
    if( !mpe_ptr->allow_jitter() )
      return;
    else
    {
      joyStickUp(true, frozen_vertical_val_up, mpe_ptr);    // Joystick is frozen, repeat jitter-values
      // joyStickDown() usw. ... j  
      return;  
    }    
  }
 
  val_in = analogRead(joy_vertical);           // Joystick 1 up/down 1-1023 middle == 513 -> map(value, fromLow, fromHigh, toLow, toHigh)
  // ### trace(Serial.printf("analogRead vertical: %d \n",val_in  ));
  if( val_in <= 513 )                           // Vertically up - formerly 513
  {
    joyStickUp(false, val_in, mpe_ptr);     // Joystick not frozen!
  }
  else                      // vertically down
  {
    joyStickDown(val_in, mpe_ptr);
  }
    
  val_in = analogRead(joy_horizontal);           // Joystick 1 left/right 1023-1, middle == 513
  if( val_in <= 513 )       
  {
    joyStickRight(val_in, mpe_ptr);
  }
  else                      // > 513
  {
    joyStickLeft(val_in, mpe_ptr);
  }  
}

void MPE_CTRL::all_notes_off()
{
  for(int ch=0; ch<16; ch++)
  {
    usbMIDI.sendControlChange(123, 0, ch);  // All notes off on all used channels to avoid hanging notes, if tuning / voicing is changed!
    channel_active[ch] = false;    // no more acivity to display for this channel, too  
  }
  trace(Serial.printf("all notes off sent! \n"  ));
}

/*
4 right columns, 8 rows:
60 61 62 63
..
48 49 4a 4b
44 45 46 47
4 left columns, 8 rows on Lauchpad (hex values), Launchpad95 User 2 pressed 3 times until green
40 41 42 43
...
28 29 2a 2b 
24 25 26 27 // Decimal: 36, 37, 38, 39 -> 40, 45, 50, 55  E, A, D, G
both rows on the Launchpad will be interpreted like a fretboard of a bass, low to up / one row on the left side, one on the right side...
*/


ChannelNote* MPE_CTRL_SETTINGS::launch_to_bass( int note_in )                // Take note from Launchpad and convert to bass-frets channel/note "tuple" 
{
static ChannelNote result;                                // Array with two integers (channel+note)
  
  int note_relative = note_in - 36;                       // relative zero of incoming note from Launchpad is 36, details see above ---
  int channel = note_relative % 4;                        // "index" 0-4, meaning "string one to four (E,A,D,G) on "bass"
  
  result[0] = channel+1;                                  // channel_out for midi-send     
  result[1] = min(max( 0, (note_relative / 4) + bass_notes[bass_notes_set][channel]+m_octave_val*12), 127); // note_out for midi-send
  result[2] = note_relative / 4;                          // Will be values 0 to 15 for all 4 culoms (twice)
  return &result;                                         // Static array, so content (two integers: channel and note) will be accassable by receiver
}

ChannelNote* MPE_CTRL_SETTINGS::keyboard_to_bass( int note_in )   // Take note from regular keyboard and convert per octave to bass-frets channel/note "tuple" 
{
static ChannelNote result;                                // Array with two integers (channel+note)
static int note_range[5] = {48,60,72,84,96};
int note_relative = 0;
int channel = 1;
int i = 0;
  
  note_relative = note_in%12;               // relative zero of incoming note from Keyboard is 48 (no transpose), prevent negative numbers...

  for( i=0; i < 4; i++)
  {
    if( (note_in >= note_range[i]) && (note_in < note_range[i+1])  )
      break;
  }
  channel = i;                                  // "index" 0-4, meaning "string one to four (E,A,D,G) on "bass"
  
  result[0] = channel+1;                                  // channel_out for midi-send      
  result[1] = min(max(0, note_relative + bass_notes[bass_notes_set][channel]+m_octave_val*12),127);  // note_out for midi-send
  result[2] = note_relative;                              // Will be values 0 to 11 for all 4 octaves
  return &result;                                         // Static array, so content (two integers: channel and note) will be accassable by receiver
}

void MPE_CTRL_SETTINGS::set_poly_note( int channel, unsigned int note_id )
{
  if( note_id > 15 )
  {
    trace(Serial.printf("Illigal Index %d to set_poly_note\n", note_id));
    return;
  }
  m_poly_active[channel] |= (1 << note_id);
  trace(Serial.printf("set id %d m_poly_active[%d]: %x \n", note_id, channel, m_poly_active[channel]));
}

unsigned int MPE_CTRL_SETTINGS::get_keys_id( int channel )    // get Id of highest note if several notes pressed
{
int id = -1;

  if( m_poly_active[channel] )    // if no keys pressed for channel selected, id will be 0
  {
    for(id=15; id > 0; id--)
    {
      if( m_poly_active[channel] & (1 << id) )
        break;                                  // We found the highest note still pressed, return that id now
    }
  }
  trace(Serial.printf("get_keys_id(): %d for m_poly_active[%d]: %x \n", id, channel, m_poly_active[channel]));
  return id;                                    // below Zero if no note set currently
}

void MPE_CTRL_SETTINGS::unset_poly_note( int channel, unsigned int note_id )
{
unsigned int pattern = 0xFFFF;

  if( note_id > 15 )
  {
    trace(Serial.printf("Illigal Index %d to set_poly_note\n", note_id));
    return;
  }
  pattern ^= (1 << note_id);              // create pattern with all bits set, exept the one we want to unset
  m_poly_active[channel] &= pattern;      // unset requred bit
  
  trace(Serial.printf("unset id %d m_poly_active[%d]: %x \n", note_id, channel, m_poly_active[channel]));
}

void MPE_CTRL_SETTINGS::handleControlChange(byte inChannel, byte inCC, byte inValue)    // Check is Sustain pedal used for instance
{
  trace(Serial.printf("-> Control %d on Channel %d value: %d \n", inCC, inChannel, inValue));

  if( inChannel <= 10 )
  {
    if(inCC == 64 )
    {
      if( inValue > 63 )
      {
        sustain_active = true;
        trace(Serial.printf("Sustain on detected\n"));
      }
      else
      {
        sustain_active = false;
        trace(Serial.printf("Sustain off detected\n"));
      }
    }
    usbMIDI.sendControlChange(inCC, inValue, 1);          // concentrate incoming controller-data to be send out via Channel 1, to send it "as global data" to a MPE synth
    trace(Serial.printf("<- Control %d on Channel 1 value: %d \n", inCC, inValue));
  }
  else        // Channel >= 11, we will listen to any controller coming in as a potential Pedal-information to be send out like "Joystick up"
  {
    if( latch_channel ) // The channel on which "Joystick up" as "MPE-Timbre" should be send was negotiated via latch-switch on last-note pressed before
    {
      usbMIDI.sendControlChange(74, inValue, latch_channel+channel_offset);   // Latched MPE signal?
      trace(Serial.printf("MPE - Channel %d Pedal (CC 74) is %d \n", latch_channel, inValue ));
    }
  }
}

void MPE_CTRL_SETTINGS::handleNoteOn(byte inChannel, byte inNumber, byte inVelocity) // convert incoming Launchpad note to "fret on bass" to be send out along with the controllers
{
int out_channel = 0;
int out_note = 0;
int note_id = 0;

  trace(Serial.printf("-> Note on ch: %d nt: %d vl: %d \n", inChannel, inNumber, inVelocity));

  if( inChannel == 11 )       // We receive a setup-information via note-on to latch the channel associated with the last note played to a footpedal
  {
    latch_channel = m_current_channel;
    trace(Serial.printf("!!! Channel %d latched to footpedal via noteon over Channel 11 \n", latch_channel));
    return;                   // Caution: Nothing else to do here, we processed information from the setup-channel instead of notes played
  }
  if( inChannel == 16 ) // use Launchpad?
  {
    out_channel = (*launch_to_bass(inNumber))[0];       // Channel
    out_note = (*launch_to_bass(inNumber))[1];          // note
    note_id = (*launch_to_bass(inNumber))[2];          // note id (0-15)
  }
  else      // convert notes from a regular keyboard per octave to 4 strings of a bass
  {
    out_channel = (*keyboard_to_bass(inNumber))[0];       // Channel
    out_note = (*keyboard_to_bass(inNumber))[1];          // note
    note_id = (*keyboard_to_bass(inNumber))[2];          // note id (0-11)
  }
  trace(Serial.printf("<- Note on (X) id: %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
  if( play_monophonic )
  {
    if( m_poly_active[out_channel]==0 || (note_id > (int)get_keys_id( out_channel )) ) // first note on channel or high note priority, only play if new note encountered
    {
      usbMIDI.sendControlChange(123, 0, out_channel+channel_offset);   // All notes off on out-channel, just to make sure we generate no hanging notes in mono-mode
      usbMIDI.sendNoteOff(last_mono_note[out_channel], inVelocity, out_channel+channel_offset);  // Note off for last note, in case CC123 would not be recognized by some DAWs?
      usbMIDI.sendNoteOn(out_note, inVelocity, out_channel+channel_offset);
      last_mono_note[out_channel] = out_note;                             // Remember this one, so we can turn it off when next mono-note comes in
      trace(Serial.printf("<- Note on (A) mono id: %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
    }
  }
  else          // We are in polyphonic mode!
  {
    usbMIDI.sendNoteOn(out_note, inVelocity, out_channel+channel_offset);
    trace(Serial.printf("<- Note on (B) poly id: %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
  }
  m_current_channel = out_channel;        // To possibly select active channel for pedal via foot-switch!
  set_poly_note( out_channel, note_id );  // We remember key pressed last, regardless if poly or mono, even relevant if not played (high key prio) for mono
  
  if( m_poly_active[out_channel] )         // Check if any bit from note_ids on channel is still set?
    channel_active[out_channel] = true;
}

void MPE_CTRL_SETTINGS::handleNoteOff(byte inChannel, byte inNumber, byte inVelocity) // convert incoming Launchpad note to "fret on bass" to be send out along with the controllers
{
int out_channel = 0;
int out_note = 0;  
int note_id = 0;
int play_trill = 0;
int count = 0;

  trace(Serial.printf("-> Note off ch: %d nt: %d vl: %d \n", inChannel, inNumber, inVelocity));

  if( inChannel == 16 ) // use Launchpad?
  {
    out_channel = (*launch_to_bass(inNumber))[0];       // Channel
    out_note = (*launch_to_bass(inNumber))[1];          // note
    note_id = (*launch_to_bass(inNumber))[2];          // note id (0-15)
  }
  else      // convert notes from a regular keyboard per octave to 4 strings of a bass
  {
    out_channel = (*keyboard_to_bass(inNumber))[0];       // Channel
    out_note = (*keyboard_to_bass(inNumber))[1];          // note
    note_id = (*keyboard_to_bass(inNumber))[2];          // note id (0-11)
  }
  // --- Check for channel to associate with footpedal ---
  for(unsigned int i=0; i<sizeof(channel_active); i++)    
    if( !channel_active[i] )
      count++;
  if( count == sizeof(channel_active) || !channel_active[m_current_channel] )   // all elements are "false" or "current channel" not playing anymore
    m_current_channel = 0;

  trace(Serial.printf("<- Note off id: %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
  if( !play_monophonic ) // we had no "auto note off" for this note yet - otherwise we send no note of, to avoid notes cut of, because monophonic on the same channel...
  {
    trace(Serial.printf("<- Note off id (P): %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
    unset_poly_note( out_channel, note_id );
    usbMIDI.sendNoteOff(out_note, inVelocity, out_channel+channel_offset);
  }
  else    // special handling for monophonic playing needed
  {
    if( note_id == (int)get_keys_id( out_channel ) )   // check if the released note is the highest note, so that we have to send a note-off
    {
      trace(Serial.printf("<- Note off id (M): %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));

      usbMIDI.sendNoteOff(out_note, inVelocity, out_channel+channel_offset);
      unset_poly_note( out_channel, note_id ); 
      if( (play_trill=get_keys_id( out_channel )) >= 0 ) // check if any other notes are still active and we have to send a note-on for the highest of those
      {
        out_note = min(max(0, play_trill + bass_notes[bass_notes_set][max(0,out_channel-1)]+(m_octave_val*12)),127);   
        usbMIDI.sendNoteOn(out_note, inVelocity, out_channel+channel_offset);
        trace(Serial.printf("<- Note on now! id (M): %d ch: %d nt: %d vl: %d \n", play_trill, out_channel, out_note, inVelocity));
      }
    }
    unset_poly_note( out_channel, note_id );  // (maybe unset already, yet) a key was released, so delete it from bit-list, regardless if we had a noteoff for it
  }
  trace(Serial.printf("<- Note off id (B): %d ch: %d nt: %d vl: %d \n", note_id, out_channel, out_note, inVelocity));
  
  if( !m_poly_active[out_channel] )         // Check if any bit from note_ids on channel is still set?
  {
    channel_active[out_channel] = false;
    usbMIDI.sendControlChange(123, 0, out_channel+channel_offset);   // All notes off on out-channel
  }
}

void setup() 
{
  Serial.begin(115200);     // USB serial, same Baud rate as MIDI
  usbMIDI.setHandleNoteOn(MPE_CTRL_SETTINGS::handleNoteOn);
  usbMIDI.setHandleNoteOff(MPE_CTRL_SETTINGS::handleNoteOff);
  usbMIDI.setHandleControlChange(MPE_CTRL_SETTINGS::handleControlChange); 
  
  pinMode(1, INPUT_PULLUP);    // digital input for channel 1 on Pin 1 == Switch on/off
  pinMode(2, INPUT_PULLUP);    // digital input for channel 2 on Pin 2
  pinMode(3, INPUT_PULLUP);    // digital input for channel 3 on Pin 3
  pinMode(4, INPUT_PULLUP);    // digital input for channel 4 on Pin 4
                               // analog input is default on PINs 14-23 / A0 to A9!
}

void loop() 
{
static MPE_CTRL_SETTINGS mpe_ctrl;   // We use this class to modify class-variable in the MPE_CTRL-class!
#ifdef PROTOTYP
static MPE_CTRL mpe_1(1, A9, A8, 1);       //  byte channel, byte joy_vertical, byte joy_horizontal, button-pin (joystick freeze), optional pin for keypad
static MPE_CTRL mpe_2(2, A3, A2, 4); 
static MPE_CTRL mpe_3(3, A7, A6, 2); 
static MPE_CTRL mpe_4(4, A5, A4, 3); 
#else
static MPE_CTRL mpe_1(1, A9, A8, 1);       //  byte channel, byte joy_vertical, byte joy_horizontal, button-pin (joystick freeze), optional pin for keypad
static MPE_CTRL mpe_2(2, A7, A6, 2); 
static MPE_CTRL mpe_3(3, A5, A4, 3); 
static MPE_CTRL mpe_4(4, A3, A2, 4); 
#endif
static MPE_CTRL* mpes[4] = {&mpe_1, &mpe_2, &mpe_3, &mpe_4}; 
static unsigned int loop_counter = 0;
static boolean sustain_active = false;
static int last_latched_channel = 0;

  loop_counter++;
  usbMIDI.read();             // Get notes from Launchpad, if any...

  mpe_ctrl.handleMatrixButtons(mpes);   // Decide if to use PitchBend y/n and per Channel Sustain and all other settings...
  
  for(int i=0; i<4; i++)
  {
    mpes[i]->handleJoyStick(&mpe_ctrl);      // handleJoyStick // byte channel, byte joy_vertical, byte joy_horizontal set via Constructor of instance
    
    if( loop_counter%50 == 0)                // avoid setting the LEDs or 7-segment dots too often
    {
      if( mpe_ctrl.check_sustain_active() )                  // Show dots if sustain-pedal is pressed
      {
        if( !sustain_active )               // prevent switching on/off the dots on the display too often...
        {
          mpe_ctrl.module.display_dots("    ++++");
          sustain_active = true;            // prevent switching on/off the dots on the display too often...
        }
      }
      else
      {
        if( sustain_active )
        {
          mpe_ctrl.module.display_dots("    ----");
          sustain_active = false;            // prevent switching on/off the dots on the display too often...
        }
      }
      if( mpe_ctrl.get_latch_channel() && mpe_ctrl.get_latch_channel()!= last_latched_channel)   // A new channel has been latched for foot-controller "MPE up"
      {
        last_latched_channel = mpe_ctrl.get_latch_channel();              // Remember for next time, so that we don't check for latched channel for MPE-pedal too often...   
        switch( last_latched_channel ) 
        {
          case 0:
            mpe_ctrl.module.display_dots("----    ");
            break;
          case 1:
            mpe_ctrl.module.display_dots("+---    ");
            break;
          case 2:
            mpe_ctrl.module.display_dots("-+---   ");
            break;
          case 3:
            mpe_ctrl.module.display_dots("--+-    ");
            break;
          case 4:
            mpe_ctrl.module.display_dots("---+    ");
            break;
          default:
            trace(Serial.printf("Unexpected latch_channel %d \n", last_latched_channel));
        }
      }
      if( mpe_ctrl.check_channel_active(i+1) )                  // Show leds for outgoing midi-data per MPE-channel
        mpe_ctrl.module.led_on(i+4);                            // The TM1638_MB-method will take care to prevent calling setting leds to often!
      else
        mpe_ctrl.module.led_off(i+4);                           // The TM1638_MB-method will take care to prevent calling unset leds to often!
    }
  }
}
