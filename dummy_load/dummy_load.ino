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

#define PIN_VOLTS A0
#define PIN_AMPS  A1

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

void setup() 
{
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
}

void loop() 
{

  int temperature_mosfet = readThermistor(PIN_THERMISTOR_MOSFET);
  int temperature_resistor = readThermistor(PIN_THERMISTOR_RESISTOR);
  
  int millivolts = readVolts();
  
  lcd.setCursor(0, 0);
  lcd.print(millivolts);
  lcd.print("V  ");

  lcd.setCursor(0, 1);
  lcd.print("m:");
  lcd.print(temperature_mosfet);
  lcd.print("C  ");
  
  lcd.setCursor(8, 1);
  lcd.print("r:");
  lcd.print(temperature_resistor);
  lcd.print("C  ");
  
  delay(1000);
}

int readVolts()
{
  int val = analogRead(PIN_VOLTS);
  return val * (5000 / 1023);
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

