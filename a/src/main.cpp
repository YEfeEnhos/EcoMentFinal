/*    Max6675 Module  
 *    CS              ==>     D10
 *    SO              ==>     D12
 *    SCK             ==>     D13
 *    Vcc             ==>     Vcc (5v)
 *    Gnd             ==>     Gnd      */
#include <SPI.h>

//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f,20,4);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

/*    i2c LCD Module  
 *    SCL             ==>     A5
 *    SDA             ==>     A4
 *    Vcc             ==>     Vcc (5v)
 *    Gnd             ==>     Gnd      */

//I/O
int PWM_pin = 3;  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int clk = 8;      //Pin 1 rotary encoder
int data = 9;     //Pin 2 rotary encoder


//Değişkenler 
float set_temperature = 0; //encoderla setlenecek değer 

float temperature_read = 0.0; //anlık değer 
float PID_error = 0; 
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;

int clk_State;
int Last_State;  
bool dt_State;  

//PID değerleri 
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;

//MAX6675
#define MAX6675_CS   10
#define MAX6675_SO   12
#define MAX6675_SCK  13

void setup() {
  pinMode(PWM_pin,OUTPUT);
  TCCR2B = TCCR2B & B11111000 | 0x03;    
  Time = millis();
  
  Last_State = (PINB & B00000001);      

  PCICR |= (1 << PCIE0);                                       
  PCMSK0 |= (1 << PCINT0);  
  PCMSK0 |= (1 << PCINT1);  
  PCMSK0 |= (1 << PCINT3);  //Set pin D11 trigger an interrupt on state change.   
                           
  pinMode(11,INPUT);
  pinMode(9,INPUT);
  pinMode(8,INPUT);

  lcd.init();
  lcd.backlight();
}



//SPI MAX6675
double readThermocouple() {

  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

 
  
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    
    return NAN;     
  }

 
  v >>= 3;

  
  return v*0.25;
}










//interruption vector 
ISR(PCINT0_vect){
if(menu_activated==1)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     
     if (dt_State != clk_State) { 
       set_temperature = set_temperature+0.5 ;
     }
     else {
       set_temperature = set_temperature-0.5;
     } 
  }
  Last_State = clk_State;
} 

if(menu_activated==2)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
   
     if (dt_State != clk_State) { 
       kp = kp+1 ;
     }
     else {
       kp = kp-1;
     } 
  }
  Last_State = clk_State;
} 


if(menu_activated==3)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     
     if (dt_State != clk_State) { 
       ki = ki+1 ;
     }
     else {
       ki = ki-1;
     } 
  }
  Last_State = clk_State;
}

 if(menu_activated==4)
   {
  clk_State =   (PINB & B00000001); 
  dt_State  =   (PINB & B00000010); 
  if (clk_State != Last_State){     
     
     if (dt_State != clk_State) { 
       kd = kd+1 ;
     }
     else {
       kd = kd-1;
     } 
  }
  Last_State = clk_State;
}
   



  if (PINB & B00001000) 
  {       
    button_pressed = 1;
  } 
  //navigate through the 4 menus with each button pressed
  else if(button_pressed == 1)
  {
   
   if(menu_activated==4)
   {
    menu_activated = 0;  
    PID_values_fixed=1;
    button_pressed=0; 
    delay(1000);
   }

   if(menu_activated==3)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    kd = kd + 1; 
    delay(1000);
   }

   if(menu_activated==2)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    ki = ki + 1; 
    delay(1000);
   }

   if(menu_activated==1)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0; 
    kp = kp + 1; 
    delay(1000);
   }


   if(menu_activated==0 && PID_values_fixed != 1)
   {
    menu_activated = menu_activated + 1;  
    button_pressed=0;
    set_temperature = set_temperature+1;   
    delay(1000);
   }
   PID_values_fixed = 0;
   
  }  
}

void loop() {

if(menu_activated==0)
{
  //eal value of temperature
  temperature_read = readThermocouple();
  //error between the setpoint and the real value
  PID_error = set_temperature - temperature_read + 3;
  //P value
  PID_p = 0.01*kp * PID_error;
  // I value in a range on +-3
  PID_i = 0.01*PID_i + (ki * PID_error);
  

  //derivative zamanla temp artış oranı 
  timePrev = Time;                            
  Time = millis();                            
  elapsedTime = (Time - timePrev) / 1000; 
  //D value
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  // P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //PWM  0  255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  
  
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;    

  delay(250); 
  //lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("PID TEMP control");
  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.setCursor(2,1);
  lcd.print(set_temperature,1);
  lcd.setCursor(9,1);
  lcd.print("R:");
  lcd.setCursor(11,1);
  lcd.print(temperature_read,1);
}//PID control




//temp setpoint
if(menu_activated == 1)
{
   analogWrite(PWM_pin,255);
  if(set_temperature != last_set_temperature)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set  temperature");    
  lcd.setCursor(0,1);
  lcd.print(set_temperature);  
  }
  last_set_temperature = set_temperature;
  
 
}





//P set
if(menu_activated == 2)
{
  
  if(kp != last_kp)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   P  value  ");    
  lcd.setCursor(0,1);
  lcd.print(kp);  
  }
  last_kp = kp;
  
 
}


//I set
if(menu_activated == 3)
{
  
  if(ki != last_ki)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   I  value  ");    
  lcd.setCursor(0,1);
  lcd.print(ki);  
  }
  last_ki = ki;
  
 
}




//D set
if(menu_activated == 4)
{
  
  if(kd != last_kd)
  {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set   D  value  ");    
  lcd.setCursor(0,1);
  lcd.print(kd);  
  }
  last_kd = kd;
}
  
}//Loop end
