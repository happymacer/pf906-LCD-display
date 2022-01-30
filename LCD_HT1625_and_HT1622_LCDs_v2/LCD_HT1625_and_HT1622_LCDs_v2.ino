
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//    Arduino demo/driver for both the HT1622 and HT1625 based LCDs of the PF906 treadmill 
/*
    original Copyright (c) 2015-2021 Martin F. Falatic
    Updated by Happymacer 2021 Copyright (c) 2021-

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Digit font based on images presented at https://github.com/dmadison/LED-Segment-ASCII

  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


  Chipset: HT1625

  LCD: 285 elements in total, comprised of:
      10 x 20 squares (200 elements)
      HEART (1 element)
      PULSE (1 element)
      3 off 7 segment digits  + dec pt (3x8 elements)
      TIME (1 element)
      4 off 7 segment digits  + dec pt + colon in middle (33 segments clock display)
      SPEED (1 element)
      3 off 7 segment digits  + dec pt (3x8 elements)

  Im going to treat the display as just "elements" on the LCD that can be turned on and off... no concept of ä "digit".  
  It takes 1 SEG and 1 COM line to light an element.

  Wiring:
  VDD  = 5VDC
  GND  = GND
  DATA = 5V logic
  RW   = 5V logic
  RD   = -not used-
  CS   = 5V logic
  Backlight = drive the base of the transistor with 5V for backlight

  The LCD is adressed as follows -
  All 8 comm lines connected (com0-com7)
  Adddress lines used: Seg0(0x0)- seg 39(0x27) seg 18 (0x12) is NC

  Arduino is little-endian
  H162x commands and addresses are MSB-first (3 bit mode + 9 bit command code)
  Note that the very last bit is X (don't care)
  Data is LSB-first, in address-sequential 4-bit nibbles as desired.

  Something to observe is that data and adresses are directly mapped to the LCD by the chip.  Hence its of no consequence if the bits are
  written in little or big endian.  The only time it matters is for the commands, and then only if the command is non symetrical - so WR
  command is 101 (symetrical) while setup commands start with 100 (not symetrical)  In this case all the setup commands are defaults so a
  setup command failure is inconsequential in most cases.

  HOWEVER: to be compatible with other systems, if thats needed, then the code must send the adresses and the data in the correct bit order.
  This has been done here.




*/

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Helper functions and variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Function that printf and related will use to print
int serial_putchar(char c, FILE* f) {
  if (c == '\n') serial_putchar('\r', f);
  return Serial.write(c) == 1 ? 0 : 1;
}
FILE serial_stdout;
int k=0;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// HT162x commands (Start with 0b100, then 8 bits then ends with one "don't care" ("X") bit)
// see the HT162x_Command(uint8_t cmd) subroutine - note that the command is written, then the 8 bits below,
// then a "1" in the last position, hence we only need to specify the first 8 bits of the control code
#define  CMD_SYS_DIS  0x00  // SYS DIS    (0000-0000) Turn off system oscillator, LCD bias gen [Default]
#define  CMD_SYS_EN   0x01  // SYS EN     (0000-0001) Turn on  system oscillator
#define  CMD_LCD_OFF  0x02  // LCD OFF    (0000-0010) Turn off LCD display [Default]
#define  CMD_LCD_ON   0x03  // LCD ON     (0000-0011) Turn on  LCD display
#define  CMD_TMR_OFF  0x04  // TIMER disable     (0000-0100)
#define  CMD_WDT_OFF  0x05  // WDT disable     (0000-0101) 
#define  CMD_RC_INT   0x18  // RC INT     (0001-10XX) System clock source, on-chip RC oscillator
#define  CMD_IRQ_DIS  0x80  // IRQ Disable(100X-0XXX) System clock source
#define  CMD_IRQ_EN   0x88  // IRQ Enable(100X-1XXX) System clock source

#define  CMD_TONE_OFF    0x08 // turn tone off
#define  CMD_4K_TONE_ON  0x40 // turn on 4k buzzer (010X-XXXX-X)
#define  CMD_2K_TONE_ON  0x60 // turn on 4k buzzer (0110-XXXX-X)


#define CS1625 10 // Active low
#define CS1622 11 // Active low
#define WR   12 // Active low
#define DATA 13

#define TOP_ADDRESS 81 // The highest segment line used of the HT1625 on this LCD is SEG40.  The decimal address of seg 40 is 80 and 81 
#define BL 9 //Arduino PWM output for backlight LEDs
#define brightness 0.5 // where full backlight brightness = 1
#define ON  0x1
#define OFF 0x0

#define SETUP_DELAY_USECS 1
#define HOLD_DELAY_USECS  1
#define WRITE_DELAY_USECS 2     // Tclk min. on data sheet - overhead is more than this at low clock speeds
#define RESET_DELAY_USECS 1000  // Not strictly necessary



//for the smaller display using the HT1622 chip:
#define NUM_DIGITS 15

//these are the digit adresses using the address lines
//                digit       #1    #2    #3    #4.........................................................#14   #15
const uint8_t digitAddr[] = {0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3c};

// Based on a font presented at https://github.com/dmadison/LED-Segment-ASCII
const uint16_t SegCharDataLSBfirst[] = {
  0xea45, 0x45, 0xd20c, 0x824c, 0x184c, 0x9a10, 0xda48, 0x0244,   // 01234567
  0xda4c, 0x9a4c, 0x0000, 0x8000, 0x1008, 0x1128, 0x00c4, 0x0080, // 89 _-+!.
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
void HT162x_Command(uint8_t CS, uint8_t cmd)
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
void HT162x_WriteData(uint8_t CS, uint16_t addr, uint16_t sdata, uint8_t bits = 4)
{
  // note that the data bits are in the wrong order and must be reversed to be correct, ie
  // Arduino order Dn...D0, and Sendbits needs D0...Dn
  // also as the address auto increments send all 16 bits at once.  In fact if you want to write to
  // multiple elements sequentially then send them all sequentially and increase the size of sdata
  // variable accordingly

  // one way to invert the bits - https://www.youtube.com/watch?v=-5z9dimxxmI
  uint16_t atads = 0; // sdata reversed is atads :-)
  for (int i = 0; i < bits; i++) 
  {
    atads <<= 1; // shift the result left 1 bit
    if ((sdata & 1) == 1) atads = atads | (sdata & 1); // if the last bit in sdata is 1, then change the last bit in atads to 1
    sdata = sdata >> 1; // right shift sdata to be able to access the next bit
  }

  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, LOW);
  delayMicroseconds(SETUP_DELAY_USECS);
  HT162x_SendBits(0b101, 3);
  if (CS == CS1625) 
  {
    HT162x_SendBits(addr, 7); // need an extra bit relative to HT1622- ie 7 bits instead of 6
  } else 
  { 
    HT162x_SendBits(addr, 6); // HT1622 is only 6 bits
  }
  HT162x_SendBits(atads, bits);
  delayMicroseconds(SETUP_DELAY_USECS);
  digitalWrite(CS, HIGH);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void digitElements(uint8_t num,uint8_t addrL, uint8_t addrH) 
// addrL and addrH is the address of the digit, and num is the number to be written.

{
  uint8_t partL[] = {0x7, 0x6, 0x3, 0x7, 0x6, 0x5, 0x5, 0x7, 0x7, 0x7}; // see spreadsheet for explanation
  uint8_t partH[] = {0xD, 0x0, 0xE, 0xA, 0x3, 0xB, 0xF, 0x0, 0xF, 0xB};
  HT162x_WriteData(CS1625, addrL, partL[num], 4);
  HT162x_WriteData(CS1625, addrH, partH[num], 4); 
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void heart_(uint8_t state) 
{
  state ? HT162x_WriteData(CS1625, 0b1001100, 0b0100, 4): HT162x_WriteData(CS1625, 0b1001100, 0b0000, 4); 
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void pulse_(uint8_t state) 
{
  state ? HT162x_WriteData(CS1625, 0b1001010, 0b0100, 4): HT162x_WriteData(CS1625, 0b1001010, 0b0000, 4); 
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void time_(uint8_t state) 
{
  state ? HT162x_WriteData(CS1625, 0b0111000, 0b0100, 4): HT162x_WriteData(CS1625, 0b0111000, 0b0000, 4);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void speed_(uint8_t state) 
{
  state ? HT162x_WriteData(CS1625, 0b1001110, 0b0100, 4): HT162x_WriteData(CS1625, 0b1001110, 0b0000, 4);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void colon_(uint8_t state) 
{
  state ? HT162x_WriteData(CS1625, 0b0111010, 0b0100, 4): HT162x_WriteData(CS1625, 0b0111010, 0b0000, 4);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void AllElements(uint8_t CS, uint8_t state)
{
  if (CS == CS1625) 
  {
    if (state == 0)
    {
      for (uint8_t addr = 0; addr <= TOP_ADDRESS; addr++)
      {
        HT162x_WriteData(CS, addr, 0b0000, 4);
      }
    } else 
    {
      for (uint8_t addr = 0; addr <= TOP_ADDRESS; addr++)
      {
        HT162x_WriteData(CS, addr, 0b1111, 4);
      }
    }
  } else
  {
    for (uint8_t addr = 0; addr <= 0x3f; addr++)
    {
      HT162x_WriteData(CS, addr, (state ? 0xf : 0x0), 4);
    }  
  }

}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void TestElements(int segDelay = 200, boolean clear_as = true, int cycles = 1)   // clear_as clears the segment before writing the next segment
{

  for (uint8_t addr = 0; addr <= TOP_ADDRESS; addr++)
  {
//    for (uint8_t dat = 0; dat <= 15; dat++)
//    {
//     HT162x_WriteData(CS1625, addr, dat , 4);
//    }

//    while (Serial.available()==0) {}
//    int inbyte = Serial.read();
    HT162x_WriteData(CS1625, addr-1, 0b0000 , 4);
    HT162x_WriteData(CS1625, addr, 0b0001 , 4);
//    Serial.print("Address: ");
//    Serial.print(addr,BIN);
//    Serial.print("   Data: ");
//    Serial.println("0001");
    delay(segDelay);
    
    HT162x_WriteData(CS1625, addr, 0b0010 , 4);
//    Serial.print("Address: ");
//    Serial.print(addr,BIN);
//    Serial.print("   Data: ");
//    Serial.println("0010");
    delay(segDelay);
    
    HT162x_WriteData(CS1625, addr, 0b0100 , 4);
//    Serial.print("Address: ");
//    Serial.print(addr,BIN);
//    Serial.print("   Data: ");
//    Serial.println("0100");
    delay(segDelay);
    
    HT162x_WriteData(CS1625, addr, 0b1000 , 4);
//    Serial.print("Address: ");
//    Serial.print(addr,BIN);
//    Serial.print("   Data: ");
//    Serial.println("1000");
    delay(segDelay);
    



  }


}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Message() //on small LCD HT1622
{
  
HT162x_WriteData(CS1622, 0x3C, 0x584c, 16); //H
HT162x_WriteData(CS1622, 0x38, 0x5a4c, 16); //a
HT162x_WriteData(CS1622, 0x34, 0x5a0c, 16); //p
HT162x_WriteData(CS1622, 0x30, 0x5a0c, 16); //p
HT162x_WriteData(CS1622, 0x2C, 0x984c, 16); //y
HT162x_WriteData(CS1622, 0x28, 0x4c45, 16); //m
HT162x_WriteData(CS1622, 0x24, 0x5a4c, 16); //a
HT162x_WriteData(CS1622, 0x20, 0xca00, 16); //c
HT162x_WriteData(CS1622, 0x1C, 0xda00, 16); //e
HT162x_WriteData(CS1622, 0x18, 0x5a1c, 16); //r
//HT162x_WriteData(CS1622, 0x14, 0x00c4, 16); //!
}



//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//#############################################################################
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void setup()
{
  // Set up stdout for printf() use
  Serial.begin(115200);
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;
  //printf("My favorite number is %6d!\n", 42);


  
  delayMicroseconds(RESET_DELAY_USECS);
  delay(1000);

  // Set up I/O and display
  pinMode(BL, OUTPUT);
  pinMode(CS1625, OUTPUT);
  pinMode(CS1622, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(DATA, OUTPUT);
  delay(50);
  HT162x_Command(CS1625, CMD_RC_INT);
  HT162x_Command(CS1622, CMD_RC_INT);
  HT162x_Command(CS1625, CMD_WDT_OFF);
  HT162x_Command(CS1622, CMD_WDT_OFF);
  HT162x_Command(CS1625, CMD_TMR_OFF);
  HT162x_Command(CS1622, CMD_TMR_OFF);
  HT162x_Command(CS1625, CMD_SYS_EN);  
  HT162x_Command(CS1622, CMD_SYS_EN);
  HT162x_Command(CS1625, CMD_LCD_OFF);  
  HT162x_Command(CS1622, CMD_LCD_OFF);
  AllElements(CS1625,0);
  AllElements(CS1622,0);
  delay(1000);
  //fade the backlight on to set brightness
  for (int i=0; i<=brightness*255; i++)
  {
    analogWrite(BL, i);
    delay(5);
  }
  HT162x_Command(CS1625, CMD_LCD_ON); // Should turn it back on
  HT162x_Command(CS1622, CMD_LCD_ON); // Should turn it back on
  //beep the buzzer
  HT162x_Command(CS1625, CMD_4K_TONE_ON);
  delay(5);
  HT162x_Command(CS1625, CMD_TONE_OFF);   
  AllElements(CS1625,0); 
  pulse_(1);
  time_(1);
  speed_(1);
  heart_(1);
  colon_(1);
  Message(); 
  int i=9;
  for (int j=0x2B; j<=0x4F; j=j+4)
    {
      printf("number %d Address: %6X \n", i, j);
      digitElements(i,j,j+2);
      i--;
    }
 
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//#############################################################################
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void loop()
{
  //TestElements(50, true, 1);
  //delay(2000);

//  for (int i=0; i<10; i++)
//  {
//    for (int j=0x2B; j<=0x4F; j=j+4)
//    {
//      digitElements(i,j,j+2);
//
//    }  
//    delay(300);
//    heart_(0);
//  }

heart_(ON);
colon_(ON);
delay(500);
heart_(OFF);
colon_(OFF);
delay(500);

  int i=k;
  for (int j=0x4F; j>=0x2B; j=j-4) // lets increment in the other direction relative to similar code above
    {
      //printf("number %d Address: %6X k= %d \n", i, j, k);
      digitElements(i,j,j+2);
      i++;
      if (i>9) i=0;
    }
    
k++;
if (k>9) k=0;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
