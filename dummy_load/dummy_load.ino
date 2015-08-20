/**
 *
 * Electronic dummy load.
 *
 *
 * https://github.com/theapi/dummy_load
 *
 */


#include <LiquidCrystal.h>


// LCD connections
#define PIN_LCD_RS 12
#define PIN_LCD_E 8
#define PIN_LCD_D4 4
#define PIN_LCD_D5 5
#define PIN_LCD_D6 6
#define PIN_LCD_D7 7

#define PIN_THERMISTOR_MOSFET   A2
#define PIN_THERMISTOR_RESISTOR A3

#define VREF      4096
#define PIN_VOLTS A0
#define PIN_AMPS  A1

#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

volatile int encoder_counter = 0; // changed by encoder input
volatile byte encoder_ab = 0; // The previous & current reading

void setup() 
{
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  
  attachInterrupt(0, encoder_ISR, CHANGE);
  attachInterrupt(1, encoder_ISR, CHANGE);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
}

void loop() 
{
  static unsigned long lcd_update_last = 0;
  
  //readEncoder();
  
  int temperature_mosfet = readThermistor(PIN_THERMISTOR_MOSFET);
  int temperature_resistor = readThermistor(PIN_THERMISTOR_RESISTOR);
  
  int millivolts = readVolts();
  int milliamps = readAmps();
  
    
  unsigned long now = millis();
  if (now - lcd_update_last > 10) {
    lcd_update_last = now;
    
    lcd.setCursor(0, 0);
    lcd.print(encoder_counter / 4 * -1);
    lcd.print("  ");
    lcd.print(millivolts);
    lcd.print("V  ");
    //lcd.setCursor(8, 0);
    lcd.print(milliamps);
    lcd.print("A  ");
    
  
    lcd.setCursor(0, 1);
    lcd.print("m:");
    lcd.print(temperature_mosfet);
    lcd.print("C  ");
    
    lcd.setCursor(8, 1);
    lcd.print("r:");
    lcd.print(temperature_resistor);
    lcd.print("C  ");
  }
  
}

/*
int readEncoder()
{
  static byte encoder_last = HIGH;
  static unsigned long last = 0;
  
  // Debounce
  unsigned long now = millis();
  if (now - last > 2) {
    last = now;
  
  byte n = digitalRead(PIN_ENCODER_A);
    if ((encoder_last == LOW) && (n == HIGH)) {
      if (digitalRead(PIN_ENCODER_B) == LOW) {
        encoder_counter--;
      } else {
        encoder_counter++;
      }
    } 
  encoder_last = n;
  }
}
*/

/**
 * Read the volts on the power supply input & convert to millivolts.
 */
int readVolts()
{
  int val = analogRead(PIN_VOLTS);
  // measured on a voltage divider 330K ---|--- 100K
  // so getting about a quarter of the actual voltage.
  return val * (VREF / 1023) * 4;
}
 
/**
 * Read the volts across the load resistor & convert to milliamps.
 */
int readAmps()
{
  int val = analogRead(PIN_AMPS);
  // 2v = 1A
  // x2 gain from the op amp on a 1ohm resistor
  return val * (VREF / 1023) * 2;
}
 
/**
  * Read the thermistor & translate to degrees Celcius.
  *
  * @see http://playground.arduino.cc/ComponentLib/Thermistor2
 */
int readThermistor(byte pin) 
{   
  int raw_adc = analogRead(pin);

  float temp;
  temp = log(10000.0 * ((1024.0/raw_adc-1))); 
  temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp );
  temp = temp - 273.15; // Convert Kelvin to Celcius
 
  return (int) temp;
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
  
  // read the PORT D pin values to which encoder is connected 
  byte port = PIND >> 2; // shift right so pins 2 & 3 are set right most
  
  // and set all but two lower bits to zero 
  // so when you OR it with ab bits 2-7 would stay intact. 
  // Then it gets ORed with ab. 
  encoder_ab |= ( port & 0x03 );  //add current state
  // At this point, we have previous reading of encoder pins in bits 2,3 of ab, 
  // current readings in bits 0,1, and together they form index of (AKA pointer to) enc_states[]  
  // array element containing current state.
  // The index being the the lowest nibble of ab (ab & 0x0f)
  return ( enc_states[( encoder_ab & 0x0f )]);
}


