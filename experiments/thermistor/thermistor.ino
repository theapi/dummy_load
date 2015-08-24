/**
  LiquidCrystal Library to show the temperature measured by a thermistor.
 
 Uses the use a 16x2 LCD display.
 
 @see http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */


#include <LiquidCrystal.h>
#include <math.h>

// LCD connections
#define PIN_LCD_RS 9
#define PIN_LCD_E 8
#define PIN_LCD_D4 4
#define PIN_LCD_D5 5
#define PIN_LCD_D6 6
#define PIN_LCD_D7 7

#define PIN_THERMISTOR_MOSFET   A2
#define PIN_THERMISTOR_RESISTOR A3

// which analog pin to connect
#define THERMISTORPIN A2         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3977
// the value of the 'other' resistor
#define SERIESRESISTOR 10000   



// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
}

void loop() {
  
  //int thermistor_mosfet = analogRead(PIN_THERMISTOR_MOSFET);
  //double temperature_mosfet = thermistor(thermistor_mosfet);
  
  //int thermistor_resistor = analogRead(PIN_THERMISTOR_RESISTOR);
  //double temperature_resistor = thermistor(thermistor_resistor);
  
  float temperature_mosfet = readTemperature(PIN_THERMISTOR_MOSFET);
  float temperature_resistor = readTemperature(PIN_THERMISTOR_RESISTOR);

  lcd.setCursor(0, 1);
  lcd.print("m");
  lcd.print(temperature_mosfet, 0);
  lcd.print("C  ");
  
  
  lcd.setCursor(8, 1);
  lcd.print("r");
  lcd.print(temperature_resistor, 0);
  lcd.print("C  ");
  
  delay(1000);
}

float readTemperature(byte pin)
{
  uint8_t i;
  float average = 0;
  int samples[NUMSAMPLES];
  // take N samples in a row
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(pin);
  }
  
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;     
  
  return steinhart;
}

/**
 Vishay thermistot datasheet http://www.farnell.com/datasheets/1784420.pdf
 Ones I'm using: http://uk.farnell.com/vishay-bc-components/ntcle100e3103jb0/thermistor-10k-5-ntc-rad/dp/1187031
 So in the datasheet table "PARAMETER FOR DETERMINING NOMINAL RESISTANCE VALUES" row number 9 Beta(K): 3977K
 
 Steinhart calculator to test the constants: http://www.daycounter.com/Calculators/Steinhart-Hart-Thermistor-Calculator.phtml
 Explanation: https://www.insidegadgets.com/2011/12/18/verifying-the-thermistor-function-for-temperature-reading/
 constants:
 Coefficient A (K-0):  3.354016E-03     = 0.003354016
 Coefficient B (K-1):  2.569850E-04     = 0.0002569850
 Coefficient C (K-2):  0 //2.620131E-06 = 0 (0.000002620131)
 Coefficient D (K-3):  6.383091E-08     = 0.00000006383091
 
 Commonly used Steinhart equation: 1/T= A + B*ln(R/Rt) + D*ln(R/Rt)3
 
 */
double thermistorFull (int RawADC) {
  double Temp;
  //float val = (10240000 / RawADC) - 10000;
  //float val = ((long)10240000/RawADC) – 10000);
  Temp = log((double)((10240000 / RawADC) - 10000) / 10000); // We divide by our thermistor's resistance at 25C, in this case 10K
  Temp = 1 / (0.003354016 + (0.0002569850 * Temp) + (0.000002620131 * Temp * Temp) +  (0.00000006383091 * Temp * Temp * Temp) );
  Temp = Temp - 273.15; // Convert Kelvin to Celsius
  return Temp;
}
 
/**
  * @see http://playground.arduino.cc/ComponentLib/Thermistor2
 */
double thermistor(int raw_adc) {   
 double temp;
 temp = log(10000.0 * ((1024.0/raw_adc-1))); 
 temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp );
 temp = temp - 273.15;            // Convert Kelvin to Celcius
 
 return temp;
}

