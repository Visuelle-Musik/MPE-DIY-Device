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

#include "TM1638_MB.h"

#include <ctype.h>
#include <math.h>

#define trace(on)  on       // -> on <- to display / -> ; <- to do nothing! 

TM1638_MB::TM1638_MB(byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay, byte intensity)
  : TM1638(dataPin, clockPin, strobePin, activateDisplay, intensity)
{
  strcpy(m_module_text, "12345678");
  m_dots_bits = 0;
  
  // nothing else to do - calling super is enough
}

TM1638_MB::TM1638_MB(const char* display_text, byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay, byte intensity)
  : TM1638(dataPin, clockPin, strobePin, activateDisplay, intensity)
{
  strcpy(m_module_text, display_text);
  m_dots_bits = 0;
  memset(m_led_active, 0, sizeof(m_led_active) );
  this->setDisplayToString(m_module_text, m_dots_bits);
}

void TM1638_MB::display(char letter, int pos )
{
  if( pos > 7 )
    return;
  m_module_text[pos] = letter;
  trace(Serial.printf("Text - Display one Character modification '%s', dots_bits %x \n", m_module_text, m_dots_bits ));
  setDisplayToString(m_module_text, m_dots_bits);     // If no error always refresh display, if no dots needed all dots will be deleted (dots_bits == 0)
}

void TM1638_MB::display(const char* letters, int pos, int num_of_chars )
{
  if( pos>=0 && pos<=7 )
  {
    for(int i=0; i < num_of_chars; i++)
    {
      if( pos > 7 )
      {
        trace(Serial.printf("Index %d out of bounds to update module-text within substitution!\n", pos));
        return;
      }  
      m_module_text[pos] = *(letters+i);
      pos++;
    }
  }
  else
  {
    trace(Serial.printf("Index %d out of bounds to update module-text! Substitution ignored\n", pos));
    return;
  }
  trace(Serial.printf("Text - Display '%s', dots_bits %x \n", m_module_text, m_dots_bits ));
  setDisplayToString(m_module_text, m_dots_bits);     // If no error always refresh display, if no dots needed all dots will be deleted (dots_bits == 0)
}


void TM1638_MB::display_dots(const char* dots)
{
static const char* no_dots = "--------";
char current_char = 0;

  if( dots == NULL )
    dots = no_dots;

  for(int i=0; i < min(8,(int)strlen(dots)); i++)
  {
    // trace(Serial.printf("dot %d '%c'\n", i, *(dots+i) ));
    if( !isspace(current_char = *(dots+i)) )
    {
      if( current_char == '-' )
      {
        if( m_dots_bits & (1 << (7-i)) )       // check if current bit is set, else we are done already
          m_dots_bits ^= (1 << (7-i));    // Unset bit in pattern for string given, if 'minus'-character...
      }
      else
        m_dots_bits |= (1 << (7-i));      // set bit in pattern for string given, if any none-space character (including '+' or '.') 
    }
  }
  trace(Serial.printf("Dots - Display '%s', dots_bits %x \n", m_module_text, m_dots_bits ));
  setDisplayToString(m_module_text, m_dots_bits);     // If no error always refresh display, if no dots needed all dots will be deleted (dots_bits == 0)
}

void TM1638_MB::led_on(int idx) 
{ 
  if( !m_led_active[idx] )  // avoid sending too many requests from the main loop to the tm1638
  {
    setLED(TM1638_COLOR_RED, idx); 
    m_led_active[idx] = true;
  }
}

void TM1638_MB::led_off(int idx) 
{ 
  setLED(0, idx); 
  m_led_active[idx] = false;
}

// --- Return Buttons 1 to 16 from QYF-Matrix Buttons as Index 0-15, no selection or multiple select will be returned as 16 - similar to WaveShare Button Array
int TM1638_MB::get_button()
{
static word last_button = -1;
static word buttons;
static word last_key = 16;
static boolean remember_shift_happened = false;

boolean shift_happened = false;
int key_found = 16;

  buttons = getButtons();
  if( last_button != buttons )    // we have a new button pressed - prevent from processing to often via the main loop this way...
  {
    if( buttons == 0 )           // Button released
    {
      if( last_key == 6 && !remember_shift_happened)   // Maybe button 7 up, but was used to shift other function?
        key_found = 17;         // Key up, but use special case as octave down
      else
        key_found = 16;         // Key up, nothing to do if not Key 7 anyhow
      remember_shift_happened = false;      // Reset shift-mode if it has been set before: key-up of any button resets it
    }
    else
    {
      switch( buttons )
      {
        case 1:
          key_found = 0;
          break;
        case 2:
          key_found = 1;
          break;
        case 4:
          key_found = 2;
          break;
        case 8:
          key_found = 3;
          break;
        case 16:
          key_found = 4;
          break;
        case 32:
          key_found = 5;
          break;
        case 64:
          key_found = 6;
          break;
        case 128:
          key_found = 7;
          break;
    
        case 0x41:                                // Mode buttons 1-4, but MPE-mode to be selected backwards... Shift (Button 7)+Button 1-4
          shift_happened = true;
          key_found = 8;
          break;
        case 0x42:
          shift_happened = true;
          key_found = 9;
          break;
        case 0x44:
          shift_happened = true;
          key_found = 10;
          break;
        case 0x48:
          shift_happened = true;
          key_found = 11;
          break;
    
        case 0x50:                                // Shift (Button 7)+Button5 - tunings
          shift_happened = true;
          key_found = 12;
          break;
        
        case 0x60:                                // Shift (Button 7)+Button6 - mono/poly
          shift_happened = true;
          key_found = 13;
          break;
    
        case 0xc0:                                // Shift (Button 7)+Button7 - octaves
          shift_happened = true;
          key_found = 14;
          break;
    
        case 0x81:                                // left and right button pressed - turn "jitter" on/off
          key_found = 15;
          break;
          
        default:
          trace(Serial.printf("supposedly several buttons pressed at once: %4.4x \n", buttons ) );
          key_found = last_key;       // Ignore completely!
          break;
      }
    }
    last_key = key_found;
    if( shift_happened )
      remember_shift_happened = true;
      
    trace(Serial.printf("remember-shift-happened: %d shift_happend: %d\n", remember_shift_happened, shift_happened ) );
  }
  last_button = buttons;
  return(last_key);
}
