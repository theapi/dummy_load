/**
 *
 * Electronic dummy load.
 *
 * Uses the MCPDAC library from http://hacking.majenko.co.uk/MCPDAC
 *
 *
 * https://github.com/theapi/dummy_load
 *
 * Processing graphing from https://github.com/sebnil/RealtimePlotter
 */


#include <LiquidCrystal.h>

// MCPDAC relies on SPI.
#include <SPI.h>
#include <MCPDAC.h>

// LCD connections
#define PIN_LCD_RS 9
#define PIN_LCD_E 8
#define PIN_LCD_D4 4
#define PIN_LCD_D5 5
#define PIN_LCD_D6 6
#define PIN_LCD_D7 7

#define PIN_THERMISTOR_MOSFET   A2
#define PIN_THERMISTOR_RESISTOR A3

#define VREF  4000 //  4V reference from DAC
#define PIN_DAC_CS 10
#define PIN_VOLTS  A0
#define PIN_AMPS   A1

#define PIN_ENCODER_A      2
#define PIN_ENCODER_B      3


// Two switches on analog pins.
#define PIN_SWITCHES_A A5
#define PIN_SWITCHES_B A4

// Thermistor datasheet http://uk.farnell.com/vishay-bc-components/ntcle100e3103jb0/thermistor-10k-5-ntc-rad/dp/1187031
// resistance at 25 degrees C
#define THERMISTOR_NOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define THERMISTER_TEMPERATURE_NOMINAL 25  
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define THERMISTER_NUMSAMPLES 3
// The beta coefficient of the thermistor (usually 3000-4000)
#define THERMISTOR_BCOEFFICIENT 3977
// the value of the 'other' resistor
#define THERMISTOR_SERIES_RESISTOR 9850   

#define SWITCHES_BIT_FINE   0
#define SWITCHES_BIT_LOAD   1
#define SWITCHES_BIT_TARGET 2
#define SWITCHES_BIT_SHOW   3

#define SERIAL_UPDATE_INTEVAL 500 // How often to send data through serial

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);



/**
 * switch 0 : SWITCHES_BIT_FINE  : bit 0 = coarse (0) / fine (1)
 * switch 1 : SWITCHES_BIT_LOAD  : bit 1 = load disable (0) / enable (1)
 * switch 2 : SWITCHES_BIT_TARGET: bit 2 = set target load (0) / minimum volts (1)
 * switch 3 : SWITCHES_BIT_SHOW  : bit 3 = show minimum volts (0) / watts (1)
 */
byte switches_register = 0b00000010; // load enabled, encoder sets target load & show volts

int target_load = 0;
int mosfet_gate_mv = 0;
float min_volts = 2.7;

// Whether the encoder setting should be coarse or fine.
byte encoder_fine = 0;
volatile int encoder_counter = 0; // changed by encoder input
volatile byte encoder_ab = 0; // The previous & current reading

void setup() 
{
  
  // Set to use the external DAC reference. MUST be called before an analogRead().
  analogReference(EXTERNAL);
  
  // CS on pin 10, no LDAC pin (tie it to ground).
  MCPDAC.begin(PIN_DAC_CS);
  
  // Set the gain to "HIGH" mode - 0 to 4096mV.
  MCPDAC.setGain(CHANNEL_A,GAIN_HIGH);
  
  // Set the gain to "HIGH" mode - 0 to 4096mV.
  MCPDAC.setGain(CHANNEL_B,GAIN_HIGH);
  
  // Do not shut down channels
  MCPDAC.shutdown(CHANNEL_A,false);
  MCPDAC.shutdown(CHANNEL_B,false);
  
  // Set the voltage of channel A.
  MCPDAC.setVoltage(CHANNEL_A, 3990 & 0x0fff); // 4000 mV
  MCPDAC.setVoltage(CHANNEL_B, 0 & 0x0fff);
  
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  
  
  
  attachInterrupt(0, encoder_ISR, CHANGE);
  attachInterrupt(1, encoder_ISR, CHANGE);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  
  Serial.begin(115200);
}

void loop() 
{
  static unsigned long lcd_update_last = 0;
  static unsigned long min_volts_blink = 0;
  static unsigned long serial_update_last = 0;
  
  readSwitches();
  
  if (bitRead(switches_register, SWITCHES_BIT_TARGET)) {
    min_volts = getMinimumMilliVolts() / 1000;
  } else {
    target_load = getTargetLoad();
  }
  
  float temperature_mosfet = readTemperature(PIN_THERMISTOR_MOSFET);
  float temperature_resistor = readTemperature(PIN_THERMISTOR_RESISTOR);
  
  float volts = readVolts();
  int milliamps = readAmps();
  
  // Turn off the load if the volts dropped below minimum.
  if (volts <= min_volts) {
    bitWrite(switches_register, SWITCHES_BIT_LOAD, 0);
  }
  
  // Set the DAC output if the required value has changed.
  if (bitRead(switches_register, SWITCHES_BIT_LOAD)) {
    // Load enabled
    if (mosfet_gate_mv != target_load) {
      mosfet_gate_mv = target_load;
      MCPDAC.setVoltage(CHANNEL_B, mosfet_gate_mv & 0x0fff);
    }
  } else {
    // Load disabled
    if (mosfet_gate_mv != 0) {
      mosfet_gate_mv = 0; 
      MCPDAC.setVoltage(CHANNEL_B, mosfet_gate_mv & 0x0fff);
    }
  }
  

    
  unsigned long now = millis();
  if (now - lcd_update_last > 10) {
    lcd_update_last = now;
    
    lcd.setCursor(0, 0);
    
    lcd.print(target_load);
    lcd.print("mA");
    if (encoder_fine) {
      lcd.print("|");
    } else {
      lcd.print(":");
    }
    
    // Not setting minimum volts & wanting to see the watts.
    if (!bitRead(switches_register, SWITCHES_BIT_TARGET) && bitRead(switches_register, SWITCHES_BIT_SHOW)) {
      lcd.print(volts * milliamps / 1000, 1);
      lcd.print("W  ");
    } else {
      lcd.print(min_volts, 1);
      // if setting minimum voltage, "blink" the V
      if (bitRead(switches_register, SWITCHES_BIT_TARGET) && (now - min_volts_blink > 500)) {
        min_volts_blink = now;
        lcd.print("  ");
      } else {
        lcd.print("V ");
      }
    }
    
    lcd.setCursor(13, 0);
    lcd.print(temperature_mosfet, 0);
    lcd.print("C");


    // Second row
    lcd.setCursor(0, 1);
    lcd.print(milliamps);
    lcd.print("mA ");
    
  
    lcd.print(volts, 1);
    lcd.print("V ");
    //

    lcd.setCursor(13, 1);
    lcd.print(temperature_resistor, 0);
    lcd.print("C");
    
  }
  
  // CSV for Processing
  if (now - serial_update_last > SERIAL_UPDATE_INTEVAL) {
    serial_update_last = now;
    Serial.print(target_load); 
    Serial.print(" ");
    Serial.print(milliamps); 
    Serial.print(" ");
    Serial.print(min_volts); 
    Serial.print(" ");
    Serial.print(volts); 
    Serial.print(" ");
    Serial.print(temperature_mosfet); 
    Serial.print(" ");
    Serial.print(temperature_resistor); 
    Serial.print(" ");
    Serial.println();
  }

}

/**
 * switch 0 : SWITCHES_BIT_FINE  : bit 0 = coarse (0) / fine (1)
 * switch 1 : SWITCHES_BIT_LOAD  : bit 1 = load disable (0) / enable (1)
 * switch 2 : SWITCHES_BIT_TARGET: bit 2 = set target load (0) / minimum volts (1)
 * switch 3 : SWITCHES_BIT_SHOW  : bit 3 = show minimum volts (0) / watts (1)
 */
void readSwitches()
{
  byte val = 0;
  
  analogRead(PIN_SWITCHES_A); // Junk the first reading as the mux just changed
  val = (analogRead(PIN_SWITCHES_A) + analogRead(PIN_SWITCHES_A)) / 2;
  //Serial.print(val); Serial.print(" - ");
  
  if (val > 45 && val < 65) {
    bitWrite(switches_register, SWITCHES_BIT_FINE, 1);
  } else {
    bitWrite(switches_register, SWITCHES_BIT_FINE, 0);
  }
  
  if (val > 750 && val < 950) {
    bitWrite(switches_register, SWITCHES_BIT_LOAD, 1);
  } else {
    bitWrite(switches_register, SWITCHES_BIT_LOAD, 0);
  }
  
  analogRead(PIN_SWITCHES_B); // Junk the first reading as the mux just changed
  val = (analogRead(PIN_SWITCHES_B) + analogRead(PIN_SWITCHES_B)) / 2;
  //Serial.println(val);
  
  if (val > 45 && val < 65) {
    bitWrite(switches_register, SWITCHES_BIT_TARGET, 1);
  } else {
    bitWrite(switches_register, SWITCHES_BIT_TARGET, 0);
  }
  
  if (val > 750 && val < 950) {
    bitWrite(switches_register, SWITCHES_BIT_SHOW, 1);
  } else {
    bitWrite(switches_register, SWITCHES_BIT_SHOW, 0);
  }
}  

int getEncoderVal()
{
  static byte down = 0;

  if (bitRead(switches_register, SWITCHES_BIT_FINE)) {
    if (down == 0) {
      // Button has just been pressed.
      down = 1;
      // toggle coarse/fine
      if (encoder_fine == 1) {
        encoder_fine = 0;
      } else {
        encoder_fine = 1;
      }
    } 
  } else {
    // Register the release of the button.
    down = 0; 
  }
  
  // Read the encoder counter but do not change it as it changed by the interrupt.
  return (encoder_counter / 4) * -1;
}

/**
 * get the required load.
 */
int getTargetLoad() 
{
  static int last = 0;
  static int mv = 0;

  // Read the encoder counter.
  int val = getEncoderVal();

  if (val != last) {
    
    if (encoder_fine) {
      mv += val - last;
    } else {
      // coarse setting
      mv += (val - last) * 100;
    }
    
    if (mv < 0) {
      mv = 0; 
    } else if (mv > 4000) {
      mv = 4000; 
    }
    
    last = val;
  }
  
  return mv;
}

/**
 * @todo minimum volts
 */
int getMinimumMilliVolts()
{
  static int last = 0;
  static int mv = 0;

  // Read the encoder counter.
  int val = getEncoderVal();

  if (val != last) {
    
    if (encoder_fine) {
      mv += val - last;
    } else {
      // coarse setting
      mv += (val - last) * 100;
    }
    
    if (mv < 0) {
      mv = 0; 
    } else if (mv > 20000) {
      mv = 20000; 
    }
    
    last = val;
  }
  
  return mv;
}

/**
 * Read the volts on the power supply input & convert to millivolts.
 */
float readVolts()
{
  // Running average
  // @see http://playground.arduino.cc/Main/RunningAverage
  static float value = 0;
  float alpha = 0.7; // factor to tune
  
  analogRead(PIN_VOLTS); // allow multiplexer to settle on this channel
  int measurement = (analogRead(PIN_VOLTS) + analogRead(PIN_VOLTS)) / 2;
  
  value = alpha * measurement + (1-alpha) * value;

  // measured on a voltage divider 400K ---|--- 100K
  // so getting a fith of the actual voltage.
  return value * (VREF / 1024.0) * 5 / 1000;
}
 
/**
 * Read the volts across the load resistor & convert to milliamps.
 */
int readAmps()
{
  // Running average
  static int value = 0;
  float alpha = 0.8; // factor to tune
  
  int measurement = analogRead(PIN_AMPS);
  value = alpha * measurement + (1-alpha) * value;
  // 2v = 1A
  // x2 gain from the op amp on a 1ohm resistor
  return value * (VREF / 1024.0) * 2;
}
 
/**
 * Get the temperature from the thermistor
 */
float readTemperature(byte pin)
{
  uint8_t i;
  float average = 0;
  int samples[THERMISTER_NUMSAMPLES];
  
  analogRead(pin); // Junk the first reading as the mux just changed
  
  // take N samples in a row
  for (i=0; i< THERMISTER_NUMSAMPLES; i++) {
   samples[i] = analogRead(pin);
  }
  
  for (i=0; i< THERMISTER_NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= THERMISTER_NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = THERMISTOR_SERIES_RESISTOR / average;


  float steinhart;
  steinhart = average / THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= THERMISTOR_BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (THERMISTER_TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;   
  
  return steinhart;
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


