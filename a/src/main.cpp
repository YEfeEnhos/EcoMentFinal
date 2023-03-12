#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PIDController.h>
#include "max6675.h"
// Define Rotary Encoder Pins
#define CLK_PIN 21
#define DATA_PIN 22
#define SW_PIN 20
// MAX6675 Pins
#define thermoDO  26
#define thermoCS  27
#define thermoCLK  10
// Mosfet Pin
#define mosfet_pin 29
// Serial Enable
#define __DEBUG__
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define __Kp 30 // Proportional constant /*
#define __Ki 0.7 // Integral Constant       Matematiğini çözemedim BCciler çözer
#define __Kd 200 // Derivative Constant  */
int clockPin; 
int clockPinState; 
int set_temperature = 1; 
float temperature_value_c = 0.0; 
long debounce = 0; 
int encoder_btn_count = 0; 
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO); // Create an instance for the MAX6675 Sensor Called "thermocouple"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);// Create an instance for the SSD1306 128X64 OLED "display"
PIDController pid; // Create an instance of the PID controller class, called "pid"


void setup() {
#ifdef __DEBUG__
  Serial.begin(9600);
  Serial.println("A");
#endif
  Serial.println("B");
  pinMode(mosfet_pin, OUTPUT); // MOSFET output PIN
  pinMode(CLK_PIN, INPUT); // Encoer Clock Pin
  pinMode(DATA_PIN, INPUT); //Encoder Data Pin
  pinMode(SW_PIN, INPUT_PULLUP);// Encoder SW Pin
  pid.begin();          // initialize the PID instance
  pid.setpoint(150);    // The "goal" the PID controller tries to "reach"
  pid.tune(__Kp, __Ki,__Kd);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {

#ifdef __DEBUG__
    Serial.println("C");
    Serial.println(F("SSD1306 allocation failed"));
#endif
    Serial.println("D");  
    for (;;); //handbooktan aldım tam bu kısım ne işe yarıyor emin değilim 
  }

  Serial.println("E");  
  display.setRotation(2);
  display.display(); 
  display.clearDisplay(); 
  display.setTextSize(2); 
  display.setTextColor(WHITE); 
  display.setCursor(48, 0); 
  display.println("PID"); 
  display.setCursor(0, 20); 
  display.println("Temperature"); 
  display.setCursor(22, 40); 
  display.println("Control");
  display.display(); 
  delay(2000); 
}

void loop()
{
  clockPin = digitalRead(CLK_PIN); // we read the clock pin of the rotary encoder
  if (clockPin != clockPinState  && clockPin == 1) { // if this condition is true then the encoder is rotaing counter clockwise and we decremetn the counter
    if (digitalRead(DATA_PIN) == clockPin) set_temperature = set_temperature - 3;  // decrmetn the counter.
    else  set_temperature = set_temperature + 3; // Encoder is rotating CW so increment
    if (set_temperature < 1 )set_temperature = 1; // if the counter value is less than 1 the set it back to 1
    if (set_temperature > 150 ) set_temperature = 150; //if the counter value is grater than 150 then set it back to 150 
#ifdef __DEBUG__
    Serial.println(set_temperature); // print the set temperature value on the serial monitor window
#endif
  }
  clockPinState = clockPin; // Remember last CLK_PIN state
  
  if ( digitalRead(SW_PIN) == LOW)   //If we detect LOW signal, button is pressed
  {
    if ( millis() - debounce > 80) { //debounce delay
      encoder_btn_count++; // Increment the values 
      if (encoder_btn_count > 2) encoder_btn_count = 1;
#ifdef __DEBUG__
      Serial.println(encoder_btn_count);
#endif
    }
    debounce = millis(); // update the time variable
  } //Call The Read Encoder Function

  if (encoder_btn_count == 2) // temperature set mode'a getirmek için iki kere basmak lazım
  {
    display.clearDisplay(); 
    display.setTextSize(2); 
    display.setCursor(16, 0); 
    display.print("Set Temperature"); 
    display.setCursor(45, 25); 
    display.print(set_temperature);
    display.display(); 
  } // Call the Set Temperature Function
  if (encoder_btn_count == 1) // check if the button is pressed and its in Free Running Mode -- in this mode the arduino continiously updates the screen and adjusts the PWM output according to the temperature.
  {
    temperature_value_c = thermocouple.readCelsius(); // Read the Temperature using the readCelsius methode from MAX6675 Library.
    int output = pid.compute(temperature_value_c);    // Let the PID compute the value, returns the optimal output
    analogWrite(mosfet_pin, output);           // Write the output to the output pin
    pid.setpoint(set_temperature); // Use the setpoint methode of the PID library to
    display.clearDisplay(); 
    display.setTextSize(2); 
    display.setCursor(16, 0); 
    display.print("Current Temperature"); 
    display.setCursor(45, 25);
    display.print(temperature_value_c); 
    display.display(); 


#ifdef __DEBUG__
    Serial.print(temperature_value_c); // Print the Temperature value in *C on serial monitor
    Serial.print("   "); // Print an Empty Space
    Serial.println(output); // Print the Calculate Output value in the serial monitor.
#endif
    delay(200); //OLED dispaly.
  }
  
}


/*
Arduino Nano - 1
128 X 64 OLED Display - 1
Generic Rotary Encoder - 1
MAX6675 Module - 1
K-type Thermocouple - 1
Breadboard - 1
Jumper Wires - 1
*/