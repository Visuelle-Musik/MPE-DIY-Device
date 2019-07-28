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

#include <TM1638.h>

class TM1638_MB:public TM1638
{
public:
    /** Instantiate a tm1638 module specifying the display state, the starting intensity (0-7) data, clock and stobe pins. */
    TM1638_MB(byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay = true, byte intensity = 1);
    TM1638_MB(const char* text_to_display, byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay = true, byte intensity = 1);
    
    int get_button();
    
    void display(const char* letters, int pos, int num_of_chars=1);
    void display(char letter, int pos );
    
    void display_dots(const char* dots=NULL); 

    void led_on(int idx);
    void led_off(int idx);
    
private:
    byte m_dots_bits;             // Display up to 8 dots on the display
    char m_module_text[9];       // Textbuffer to memorize setting for QYF-TM1638
    boolean m_led_active[9];      // Remember if a led is lit, so to avoid sending too many requests to the TM1638
};  
