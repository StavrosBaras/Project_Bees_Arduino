#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

#include <virtuabotixRTC.h>

#include "HX711.h"
#include "dht.h"
#include <SoftwareSerial.h>
#define dht_apin A1 // Analog Pin DHT11 sensor is connected to

//Or CLK -> 6 , DAT -> 7, Reset -> 8

virtuabotixRTC myRTC(6, 7, 8); //If you change the wiring change the pins here also

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

//KALWDIA TIS SIM 900A :KOKKINO STO 11, KITRINO STO 10  

HX711 scale;
SoftwareSerial SIM900A(10,11);
dht DHT;

float hum,temp,weight;




// how many times remain to sleep before wake up
// volatile to be modified in interrupt function
volatile int nbr_remaining; 


// interrupt raised by the watchdog firing
// when the watchdog fires, this function will be executed
// remember that interrupts are disabled in ISR functions
// now we must check if the board is sleeping or if this is a genuine
// interrupt (hanging)
ISR(WDT_vect)
{
    // Check if we are in sleep mode or it is a genuine WDR.
    if(nbr_remaining > 0)
    {
        // not hang out, just waiting
        // decrease the number of remaining avail loops
        nbr_remaining = nbr_remaining - 1;
        wdt_reset();
    }
    else
    {
        // must be rebooted
        // configure
        MCUSR = 0;                          // reset flags
       
                                            // Put timer in reset-only mode:
        WDTCSR |= 0b00011000;               // Enter config mode.
        WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                            // set WDE (reset enable...4th from left), and set delay interval
                                            // reset system in 16 ms...
                                            // unless wdt_disable() in loop() is reached first
                                       
        // reboot
        while(1);
    }
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution by default
// hangs will correspond to watchdog fired when nbr_remaining <= 0 and will
// be determined in the ISR function
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts 
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
 
  }
 
  // put everything on again
  power_all_enable();
 
}

void setup(){
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  
  delay(1000);
  
  // configure the watchdog
  configure_wdt();

    Serial.begin(9600);
  SIM900A.begin(9600);   // Setting the baud rate of GSM Module  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(1000);

}

long val=0;

void loop(){

  // sleep for a given number of cycles (here, 5 * 8 seconds) in lowest power mode
  sleep(2);
  
   myRTC.updateTime();
   if(myRTC.minutes%2==0){

   digitalWrite(13,HIGH);
   //digitalWrite(5,LOW);

  // usefull stuff

  LoadCell();

  DHT11();

  if (SIM900A.available()>0){
   Serial.write(SIM900A.read()); 
 }
  //if (Serial.available()>0)//Serial.read()
   switch('s')
  {
    case 's':
      SendMessage();
      break;
  }
 if (SIM900A.available()>0){
   Serial.write(SIM900A.read()); 
 }
   }

  // now a real hang is happening: this will rebood the board
  // after 8 seconds
  // (at the end of the sleep, nrb_remaining = 0)
  while (1);

}

void LoadCell()
{
  
  //count=count+1;
  //val=((count-1)/count)*val+(1/count)*scale.read();
  val=scale.read();
  //val=0.5*val+0.5*scale.read();

  //Serial.print("Weight: ");
  //Serial.println(abs(val-175765)/43726.f);
  weight=abs(val-175765)/43726.f;
  //Serial.println(val);
  
    //long reading = scale.read();
    //Serial.print("HX711 reading: ");
    //Serial.println(reading);

  //delay(100);
  
}

void DHT11()
{

    DHT.read11(dht_apin);
    hum=DHT.humidity;
    temp=DHT.temperature;
    
    //Serial.print("Current humidity = ");
    //Serial.print(DHT.humidity);
    //Serial.print("%  ");
    //Serial.print("temperature = ");
    //Serial.print(DHT.temperature); 
    //Serial.println("C  ");
    
    //delay(2000);//Wait 5 seconds before accessing sensor again.
  
}

 void SendMessage()
{
  Serial.println ("Sending Message");
  SIM900A.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);
  Serial.println ("Set SMS Number");
  SIM900A.println("AT+CMGS=\"+306980378343\"\r"); //Mobile phone number to send message
  delay(1000);
  Serial.println ("Set SMS Content");
  //while(Serial.available()==0){}
  //message=Serial.readString();
  SIM900A.print(temp);
  SIM900A.print(" C ");
  SIM900A.print(hum);
  SIM900A.print(" % ");
  SIM900A.print(weight);
  SIM900A.print(" kg ");
  // Messsage content
  delay(100);
  Serial.println ("Finish");
  SIM900A.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
  Serial.println ("Message has been sent");
}
