
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//    Arduino demo/driver for HT1622-based 16 segment LCDs
/*
    Copyright (c) 2015-2021 Martin F. Falatic
    Updated by Happymacer 2021

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


  Chipset: HT1622

  LCD: 15 digit 14 seg + 15 dp + backlight. first difit has a different segment order to the rest

  Wiring:
  VDD  = 5VDC
  GND  = GND
  DATA = 5V logic
  RW   = 5V logic
  RD   = -not used-
  CS   = 5V logic
  Backlight = drive the base of the transistor with 5V for backlight

  Arduino is little-endian
  H162x commands and addresses are MSB-first (3 bit mode + 9 bit command code)
  Note that the very last bit is X (don't care)
  Data is LSB-first, in address-sequential 4-bit nibbles as desired.

  Something to observe is that data and adresses are directly mapped to the LCD by the chip.  Hence its of no consequence if the bits are
  written in little or big endian.  The only time it matters is for the commands, and then only if teh command is non symetrical - so WR
  command is 101 (symetrical) while setup commands start with 100 (not symetrical)  In this case all teh setup commands are defaults so a
  setup command failure is inconsequential in most cases.

  HOWEVER: to be compatible with other systems, if thats needed, then the code must send the adresses and the data in teh correct bit order.
  This has been attempted here.

  Addressing (each address hold 4 bits):
     Digit   :   #1    #2    #3    #4.........................................................#14   #15
     Address :  0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3c

  Typical names for segments:

    /-----------\   /-----------\
   ||    'a'    || ||    'b'    ||
    \-----------/   \-----------/
   /-\ /--\      /-\      /--\ /-\
  |   |\   \    |   |    /   /|   |
  |   | \   \   |   |   /   / |   |
  |'h'|  \'k'\  |'m'|  /'n'/  |'c'|
  |   |   \   \ |   | /   /   |   |
  |   |    \   \|   |/   /    |   |
   \-/      \--/ \-/ \--/      \-/
    /-----------\   /-----------\
   ||    'u'    || ||    'p'    ||
    \-----------/   \-----------/
   /-\      /--\ /-\ /--\      /-\
  |   |    /   /|   |\   \    |   |
  |   |   /   / |   | \   \   |   |
  |'g'|  /'t'/  |'s'|  \'r'\  |'d'|
  |   | /   /   |   |   \   \ |   |
  |   |/   /    |   |    \   \|   |
   \-/ \--/      \-/      \--/ \-/
    /-----------\   /-----------\
   ||    'f'    || ||    'e'    ||
    \-----------/   \-----------/

  Bit numbering via a little-endian uint16_t mapped to 4 addresses:

    /-----------\   /-----------\
   ||     F     || ||     7     ||
    \-----------/   \-----------/
   /-\ /--\      /-\      /--\ /-\
  |   |\   \    |   |    /   /|   |
  |   | \   \   |   |   /   / |   |
  | E |  \ D \  | 4 |  / 5 /  | 6 |
  |   |   \   \ |   | /   /   |   |
  |   |    \   \|   |/   /    |   |
   \-/      \--/ \-/ \--/      \-/
    /-----------\   /-----------\
   ||     C     || ||     0     ||
    \-----------/   \-----------/
   /-\      /--\ /-\ /--\      /-\
  |   |    /   /|   |\   \    |   |
  |   |   /   / |   | \   \   |   |
  | A |  / 9 /  | 8 |  \ 1 \  | 2 |
  |   | /   /   |   |   \   \ |   |
  |   |/   /    |   |    \   \|   |
   \-/ \--/      \-/      \--/ \-/
    /-----------\   /-----------\
   ||     B     || ||     3     ||
    \-----------/   \-----------/

  Bit numbering over sequential addresses: (below is not correct - need to fix)

    /-----------\   /-----------\
   ||    3-3    || ||    1-3    ||
    \-----------/   \-----------/
   /-\ /--\      /-\      /--\ /-\
  |   |\   \    |   |    /   /|   |
  |   | \   \   |   |   /   / |   |
  |3-2|  \3-1\  |1-0|  /1-1/  |1-2|
  |   |   \   \ |   | /   /   |   |
  |   |    \   \|   |/   /    |   |
   \-/      \--/ \-/ \--/      \-/
    /-----------\   /-----------\
   ||    3-9    || ||    0-0    ||
    \-----------/   \-----------/
   /-\      /--\ /-\ /--\      /-\
  |   |    /   /|   |\   \    |   |
  |   |   /   / |   | \   \   |   |
  |2-2|  /2-1/  |2-0|  \0-1\  |0-2|
  |   | /   /   |   |   \   \ |   |
  |   |/   /    |   |    \   \|   |
   \-/ \--/      \-/      \--/ \-/
    /-----------\   /-----------\
   ||    2-3    || ||    0-3    ||
    \-----------/   \-----------/

*/

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Helper functions and variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// HT162x commands (Start with 0b100, then 8 bits then ends with one "don't care" ("X") bit)
#define  CMD_SYS_DIS  0x00  // SYS DIS    (0000-0000-X) Turn off system oscillator, LCD bias gen [Default]
#define  CMD_SYS_EN   0x01  // SYS EN     (0000-0001-X) Turn on  system oscillator
#define  CMD_LCD_OFF  0x02  // LCD OFF    (0000-0010-X) Turn off LCD display [Default]
#define  CMD_LCD_ON   0x03  // LCD ON     (0000-0011-X) Turn on  LCD display
#define  CMD_RC_INT   0x18  // RC INT     (0001-10XX-X) System clock source, on-chip RC oscillator
#define  CMD_WDT_OFF  0x05  // WDT disable     (0000-0101-X) 
#define  CMD_TMR_OFF  0x04  // TIMER disable     (0000-0100-X)


#define  CMD_4K_TONE_ON  0x60 // turn on 4k buzzer (010X-XXXX-X)
#define  CMD_TONE_OFF 0x8     // turn tone off

#define CS   11 // Active low
#define WR   12 // Active low
#define DATA 13

#define ON  0x1
#define OFF 0x0

#define SETUP_DELAY_USECS 1
#define HOLD_DELAY_USECS  1
#define WRITE_DELAY_USECS 2     // Tclk min. on data sheet - overhead is more than this at low clock speeds
#define RESET_DELAY_USECS 1000  // Not strictly necessary

#define NUM_DIGITS 15

//these are the digit adresses using the address lines
//                digit       #1    #2    #3    #4.........................................................#14   #15
const uint8_t digitAddr[] = {0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3c};

// Based on a font presented at https://github.com/dmadison/LED-Segment-ASCII
const uint16_t SegCharDataLSBfirst[] = {
  0xea45, 0x45, 0xd20c, 0x824c, 0x184c, 0x9a10, 0xda48, 0x0244,   // 01234567
  0xda4c, 0x9a4c, 0x0000, 0x8000, 0x1008, 0x1128,                 // 89 _-+
  0x5a4c, 0x836c, 0xca00, 0x8364, 0xda00, 0x5a00, 0xca48,         // ABCDEFG
  0x584c, 0x8320, 0xc044, 0x5811, 0xc800, 0x4c45, 0x4c54, 0xca44, // HIJKLMNO
  0x5a0c, 0xca54, 0x5a1c, 0x9a48, 0x0320, 0xc844, 0x6801, 0x6854, // PQRSTUVW
  0x2411, 0x984c, 0xa201                                          // XYZ
};


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Send up to 16 bits, MSB (default) or LSB
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void HT162x_SendBits(uint16_t data, uint8_t bits)
{
  // Data is shifted out bitwise, MSB-first.  Data, adresses etc must be provided to the routine in Dn...D3 D2 D1 order,
  // that is already the Arduino bit order (little endian).  This is set by the manufacturer and shown in their data sheets.
  // Dn is then sent out first, followed by the rest of the bits.  Since the HT1622 expects adresses as A5...A0 followed by
  // data D0...Dn then to maintain the convention the data bits must be reversed and then provided to this
  // subroutine so they end up coming out correctly.
  //
  // its important to set up timing correctly too, so this subroutine is time critical
  //
  // The mask is used to isolate the bit being transmitted.

  uint16_t mask = 1 << bits - 1; // bit numbering starts at 0 while counting typically starts at 1

  for (uint8_t i = bits; i > 0; i--)
  {
    delayMicroseconds(WRITE_DELAY_USECS);
    digitalWrite(WR, LOW);
    data & mask ? digitalWrite(DATA, HIGH) : digitalWrite(DATA, LOW);
    delayMicroseconds(WRITE_DELAY_USECS);
    digitalWrite(WR, HIGH);
    delayMicroseconds(HOLD_DELAY_USECS);
    data <<= 1;
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void HT162x_Command(uint8_t cmd)
{
  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, LOW);
  delayMicroseconds(SETUP_DELAY_USECS);
  HT162x_SendBits(0b100, 3);
  HT162x_SendBits(cmd, 8);
  HT162x_SendBits(1, 1);
  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, HIGH);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void HT162x_WriteData(uint8_t addr, uint16_t sdata, uint8_t bits = 4)
{
  // note that the data bits are in the wrong order and must be reversed to be correct, ie
  // Arduino order Dn...D0, and Sendbits needs D0...Dn
  // also as the address auto increments send all 16 bits at once.  In fact if you want to write to
  // multiple digits sequentially then send them all sequentially and increase the size of sdata
  // variable accordingly

  // one way to invert the bits - https://www.youtube.com/watch?v=-5z9dimxxmI
  uint16_t atads = 0; // sdata reversed is atads :-)
  for (int i = 0; i < bits; i++) {
    atads <<= 1; // shift the result left 1 bit
    if ((sdata & 1) == 1) atads = atads | (sdata & 1); // if the last bit in sdata is 1, then change the last bit in atads to 1
    sdata = sdata >> 1; // right shift sdata to be able to access the next bit
  }

  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, LOW);
  delayMicroseconds(SETUP_DELAY_USECS);
  HT162x_SendBits(0b101, 3);
  HT162x_SendBits(addr, 6);
  HT162x_SendBits(atads, bits);
  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, HIGH);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void AllElements(uint8_t state)
{
  for (uint8_t addr = 0; addr <= 0x3f; addr++)
  {
    HT162x_WriteData(addr, (state ? 0xf : 0x0), 4);
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void AllSegments(uint8_t state)
{
  for (uint8_t pos = 0; pos <= NUM_DIGITS; pos++)
  {
    HT162x_WriteData(digitAddr[pos], (state ? 0xffff : 0x0000), 16);
  }
}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void TestSegments(int segDelay = 200, boolean clear_as = true, int cycles = 1)   // clear_as clears the segment before writing the next segment
{

//  while (1) {
//    //HT162x_Command(CMD_WDT_OFF);
//    HT162x_WriteData(0b001000, 0b0000111000100001, 16);
//    delay(10);
//  }

  for (int cnt = 0; cnt < cycles; cnt++) {
    for (uint8_t pos = 0; pos < NUM_DIGITS; pos++) {
        // Test cycle (single uint16_t writes)
        uint16_t sdata = 0;
        for (uint8_t i = 0; i < 16; i++) {
          sdata = clear_as ? 1 << i: sdata | (1 << i);
          HT162x_WriteData(0x1C, sdata, 16);
          delay(segDelay);
        }
        if (clear_as) {
          HT162x_WriteData(0x1C, 0, 16);
        }
    }
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void TestChars(int CharDelay = 150, int cycles = 1)
{
  int aSize = ARRAY_SIZE(SegCharDataLSBfirst);

  for (int cnt = 0; cnt < cycles; cnt++)
  {
    int charData = 0;
    for (int n = 0; n <= aSize ; n++)
    {
      //      for (uint8_t pos = 0; pos < NUM_DIGITS; pos++)
      //      {
      HT162x_WriteData(0x1C, SegCharDataLSBfirst[charData], 16);
      delay(CharDelay);
      charData = (charData < aSize - 1 ? charData + 1 : 0);
      //}
    }
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Message()
{
  
HT162x_WriteData(0x3C, 0x584c, 16); //H
HT162x_WriteData(0x38, 0x5a4c, 16); //a
HT162x_WriteData(0x34, 0x5a0c, 16); //p
HT162x_WriteData(0x30, 0x5a0c, 16); //p
HT162x_WriteData(0x2C, 0x984c, 16); //y
HT162x_WriteData(0x28, 0x4c45, 16); //m
HT162x_WriteData(0x24, 0x5a4c, 16); //a
HT162x_WriteData(0x20, 0xca00, 16); //c
HT162x_WriteData(0x1C, 0xda00, 16); //e
HT162x_WriteData(0x18, 0x5a1c, 16); //r
delay(1000);
}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//#############################################################################
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void setup()
{
  //Serial.begin(115200);
  delayMicroseconds(RESET_DELAY_USECS);

  // Set up I/O and display
  pinMode(CS, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(DATA, OUTPUT);
  delay(50);
  HT162x_Command(CMD_RC_INT);
  HT162x_Command(CMD_WDT_OFF);
  HT162x_Command(CMD_TMR_OFF);
  HT162x_Command(CMD_SYS_EN);

  HT162x_Command(CMD_LCD_OFF);
  AllElements(0);
  HT162x_Command(CMD_LCD_ON); // Should turn it back on
  Message();
delay(1000);
//  AllElements(1);
//  delay(1000);
//  AllElements(0);
//  delay(1000);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//#############################################################################
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void loop()
{
  //  AllSegments(1);
  //  delay(1000);
  //  AllElements(0);
  //
  //TestSegments(1000, true, 1);
  // AllElements(0);

  //TestSegments(1500, true, 1);
  //AllElements(0);

  // TestChars(800);
  //  
  //AllElements(0);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
