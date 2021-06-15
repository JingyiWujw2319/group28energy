#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.

String dataString1,dataString2 ;
//mppt
float vb,voltage_pre,power_now, power_pre;
float Vin_pv = 0, Iin_pv = 0;
float current_measure; // Current Control
float pwm_out = 0.05;

//I pid
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float current_ref = 0, error_amps;
float Ts = 0.001; //1 kHz control frequency.


void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("BatCycle.csv")) { // Wipe the datalog when starting
    SD.remove("BatCycle.csv");
  }


  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
//  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
//  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
//  pinMode(7, OUTPUT);
//  pinMode(8, OUTPUT);
//  pinMode(5, OUTPUT);


  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);

  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, (int)(255 - pwm_out * 255)); //just a default state to start with

}

void loop() {
  if (loop_trigger == 1) { // FAST LOOP (1kHZ)

   vb = analogRead(A0) * 4.096 / 1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
   current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
          
   Iin_pv = current_measure* pwm_out ;
   Vin_pv = vb / pwm_out;
   power_now = (Iin_pv/1000) * (Vin_pv/1000);      
   if(current_measure > 250){
        current_ref = 250;
        error_amps = (current_ref - current_measure) / 1000; //PID error calculation
        pwm_out = pidi(error_amps); //Perform the PID controller calculation      
   }else{
        //mppt using Perturb&Observe
        if (power_now > power_pre){ 
                if (Vin_pv > voltage_pre){
                  //pwm_out =  pwm_out*0.9;
                  pwm_out =  pwm_out-0.015;
                }else{
                  //pwm_out = pwm_out*1.3;
                  pwm_out =  pwm_out+0.015;}
              }else{
                  if (Vin_pv > voltage_pre){
                    //pwm_out = pwm_out*1.3;
                    pwm_out =  pwm_out + 0.015;
                  } else{
                    //pwm_out = pwm_out*0.9;
                    pwm_out =  pwm_out - 0.015;}
              }
          }
          power_pre = power_now;
          voltage_pre = Vin_pv;
          pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
          analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)

  }

  dataString1 =  "duty=" +  String(pwm_out)+ "," +"vb=" + String(vb) +  "," + "Ipv=" + String(Iin_pv)+ ","+ "Vpv="  + String(Vin_pv)+ "," +  "power=" +  String(power_now); //build a datastring for the CSV file
  dataString2 = String(pwm_out)+ "," +String(vb)+  "," +String(Iin_pv)+  "," +String(Vin_pv)+  "," +String(power_now);
  Serial.println(dataString1);// send it to serial as well in case a computer is connected
  File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
  if (dataFile) { //If we succeeded (usually this fails if the SD card is out)
    dataFile.println(dataString2); // print the data
  } else {
    Serial.println("File not open"); //otherwise print an error
  }
  dataFile.close(); // close the file
  int_count = 0; // reset the interrupt count so we dont come back here for 1000ms

}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}


float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

float pidi(float pid_input) { // discrete PID function
  float e_integration;
  e0i = pid_input;
  e_integration = e0i;

  //anti-windup
  if (u1i >= ui_max) {
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }

  delta_ui = kpi * (e0i - e1i) + kii * Ts * e_integration + kdi / Ts * (e0i - 2 * e1i + e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i, ui_max, ui_min);

  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}
