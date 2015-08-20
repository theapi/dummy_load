/**
 Rotary encoder
 */

#include <LiquidCrystal.h>

// LCD connections
#define PIN_LCD_RS 12
#define PIN_LCD_E 8
#define PIN_LCD_D4 4
#define PIN_LCD_D5 5
#define PIN_LCD_D6 6
#define PIN_LCD_D7 7

// The port the encoder is on for port manipulations
#define ENC_PORT PIND


volatile int encoder_counter = 0; // changed by encoder input
volatile byte encoder_ab = 0; // The previous & current reading

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);


void setup() 
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  
  attachInterrupt(0, encoder_ISR, CHANGE);
  attachInterrupt(1, encoder_ISR, CHANGE);
}

void loop() 
{
  
  lcd.setCursor(0, 0);
  lcd.print(encoder_counter / 4);
lcd.print("    ");

}

// Pin interrupt
void encoder_ISR()
{
  char tmpdata;
  tmpdata = encoder_read();
  if (tmpdata) {
    encoder_counter += tmpdata;
  }
}

/**
 * returns change in encoder state (-1,0,1) 
 */
int8_t encoder_read()
{
  
  
  
  // enc_states[] array is a look-up table; 
  // it is pre-filled with encoder states, 
  // with “-1″ or “1″ being valid states and “0″ being invalid. 
  // We know that there can be only two valid combination of previous and current readings of the encoder 
  // – one for the step in a clockwise direction, 
  // another one for counterclockwise. 
  // Anything else, whether it's encoder that didn't move between reads 
  // or an incorrect combination due to bouncing, is reported as zero.
  static int8_t enc_states[] = {
    0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0
  };
  
  //static unsigned long last = 0;
  // Debounce
  //unsigned long now = millis();
  //if (now - last > 100) {
   // last = now;
  
  /*
   The lookup table of the binary values represented by enc_states 
     ___     ___     __
   A    |   |   |   |
        |___|   |___|
      1 0 0 1 1 0 0 1 1
      1 1 0 0 1 1 0 0 1
     _____     ___     __
   B      |   |   |   |
          |___|   |___|
   
   A is represented by bit 0 and bit 2
   B is represented by bit 1 and bit 3
   With previous and current values stored in 4 bit data there can be
   16 possible combinations.
   The enc_states lookup table represents each one and what it means:
   
   [0] = 0000; A & B both low as before: no change : 0
   [1] = 0001; A just became high while B is low: reverse : -1
   [2] = 0010; B just became high while A is low: forward : +1
   [3] = 0011; B & A are both high after both low: invalid : 0
   [4] = 0100; A just became low while B is low: forward : +1
   [5] = 0101; A just became high after already being high: invalid : 0
   [6] = 0110; B just became high while A became low: invalid : 0
   [7] = 0111; A just became high while B was already high: reverse : -1
   [8] = 1000; B just became low while A was already low: reverse : -1
   etc...
   
   Forward: 1101 (13) - 0100 (4) - 0010 (2) - 1011 (11)
   Reverse: 1110 (14) - 1000 (8) - 0001 (1) - 0111 (7)
   
  */

  // ab gets shifted left two times 
  // saving previous reading and setting two lower bits to “0″ 
  // so the current reading can be correctly ORed.
  encoder_ab <<= 2;
  
  // ENC_PORT reads the port to which encoder is connected 
  byte port = ENC_PORT >> 2; // shift right so pins 2 & 3 are set right most
  
  // and set all but two lower bits to zero 
  // so when you OR it with ab bits 2-7 would stay intact. 
  // Then it gets ORed with ab. 
  encoder_ab |= ( port & 0x03 );  //add current state
  // At this point, we have previous reading of encoder pins in bits 2,3 of ab, 
  // current readings in bits 0,1, and together they form index of (AKA pointer to) enc_states[]  
  // array element containing current state.
  // The index being the the lowest nibble of ab (ab & 0x0f)
 // }
  
  
 
  
  return ( enc_states[( encoder_ab & 0x0f )]);
}

