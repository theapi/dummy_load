/*
  LiquidCrystal Library to show the temperature measured by a thermistor.
 
 Uses the use a 16x2 LCD display.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 
 @see http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

// include the library code:
#include <LiquidCrystal.h>
#include <math.h>

//#define THERMISTOR_DIVIDER 9860.0 // The value of the "10k" voltage divider resistor (10000.0)
#define PIN_SENSOR_THERMISTOR A0 // PC0

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  

  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

}

void loop() {
  
  int thermistor_read = analogRead(PIN_SENSOR_THERMISTOR);
  double temperature = thermistor(thermistor_read);
  
  /*
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
  */
  
  lcd.setCursor(0, 0);
  lcd.print(temperature);
  lcd.print("C  ");
  
  lcd.setCursor(0, 1);
  lcd.print(thermistor_read);
  lcd.print("  ");
  
  delay(1000);
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

