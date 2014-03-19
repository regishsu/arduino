
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init(); // initialize the lcd 

}

void loop()
{
 // Print a message to the LCD.
  lcd.print("Hello, Regis  ");
  lcd.backlight();
  delay(3000);
  lcd.print("Let's Go!   ");
  lcd.noBacklight();
  delay(3000);
}


