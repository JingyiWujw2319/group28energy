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
unsigned int rest_timer, charge_timer, fast_timer;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_measure, current_ref = 0, error_amps; // Current Control
float pwm_out = 0.05, pwm_out_v;
float V_Bat1, V_Bat2, V_Bat;
boolean input_switch;
int state_num = 0, next_state;
String dataString1, dataString2;
float soc = 0, soc1, soc2;

//soh
float cap_delta, soh, soh_fin=0;
float t_remain;

String charge_state;

boolean aged;

//mppt
float vb, voltage_pre, power_now, power_pre;
float Vin_pv = 0, Iin_pv = 0;

void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card----------------------------------------------------------
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
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  //pinMode(5, OUTPUT);


  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);//Vbat

  //bat1 control signals
  pinMode(4, OUTPUT);//Rly1
  pinMode(A1, INPUT);//Mea1
  pinMode(5, OUTPUT);//Dis1

  //bat2 control signals
  pinMode(9, OUTPUT);//Rly2
  pinMode(A2, INPUT);//Mea2
  pinMode(3, OUTPUT);//Dis2, mismarked as D10


  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

}

void loop() {
  if (loop_trigger == 1) { // FAST LOOP (1kHZ)
    state_num = next_state; //state transition
    if ((state_num == 0) || (state_num == 2) || (state_num == 4) || (state_num == 6) || (state_num == 9) || (state_num == 8) || (state_num == 7)|| (state_num == 10)|| (state_num == 11)) { //when resting
      digitalWrite(4, true);//turn on Rly   4
      digitalWrite(9, true);//9
      V_Bat1 = analogRead(A1) * 4.096 / 1.03;//measure from Mea port
      V_Bat2 = analogRead(A2) * 4.096 / 1.03;

    } else {
      digitalWrite(9, false);
      digitalWrite(4, false);
      V_Bat = analogRead(A0) * 4.096 / 1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
    }

    //    if ((V_Bat > 3700 || V_Bat < 2400)) { //Checking for Error states (just battery voltage for now)
    //      state_num = 5; //go directly to jail
    //      next_state = 5; // stay in jail
    //      digitalWrite(7, true); //turn on the red LED
    //      current_ref = 0; // no current
    //    }
    current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)


    if ((state_num == 0) || (state_num == 2) || (state_num == 5) || (state_num == 7) || (state_num == 8) || (state_num == 9)) { //when resting
      current_ref = 0;
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
    } else {
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
    }
    power_pre = power_now;
    voltage_pre = Vin_pv;

    pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
    analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)

    int_count++; //count how many interrupts since this was last reset to zero
    loop_trigger = 0; //reset the trigger and move on with life


  }


  if (int_count == 1000) { // SLOW LOOP (1Hz)

    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE (see diagram)
      case 0: { // Start state (no current, no LEDs)
          current_ref = 0;
          //voltage LUT
          //bat1
          if(V_Bat1 >= 3340){
            soc1 = 100;
          }else if((3340 > V_Bat1) && (V_Bat1 >= 3324)){
            soc1 = 95+(V_Bat1-3324)/(3340-3324)*5;
          }else if((3324 > V_Bat1) && (V_Bat1 >= 3320)){
            soc1 = 85+(V_Bat1-3320)/(3324-3320)*10;
          }else if((3320 > V_Bat1) && (V_Bat1 >= 3316.6)){
            soc1 = 75+(V_Bat1-3316.6)/(3320-3316.6)*10;
          }else if((3316.6 > V_Bat1) && (V_Bat1 >= 3288.7)){
            soc1 = 65+(V_Bat1-3288.7)/(3316.6-3288.7)*10;
          }else if((3288.7 > V_Bat1) && (V_Bat1 >= 3284.8)){
            soc1 = 55+(V_Bat1-3284.8)/(3288.7-3284.8)*10;
          }else if((3284.8 > V_Bat1) && (V_Bat1 >= 3284.7)){
            soc1 = 45+(V_Bat1-3284.7)/(3284.8-3284.7)*10;
          }else if((3284.7 > V_Bat1) && (V_Bat1 >= 3272.8)){
            soc1 = 35+(V_Bat1-3272.8)/(3284.7-3272.8)*10;
          }else if((3272.8 > V_Bat1) && (V_Bat1 >= 3249)){
            soc1 = 25+(V_Bat1-3249)/(3272.8-3249)*10;
          }else if((3249 > V_Bat1) && (V_Bat1 >= 3205.2)){
            soc1 = 15+(V_Bat1-3205.2)/(3249-3205.2)*10;
          }else if((3205.2 > V_Bat1) && (V_Bat1 >= 3113.8)){
            soc1 = 5+(V_Bat1-3113.8)/(3205.2-3113.8)*10;
          }else if((3113.8 > V_Bat1) && (V_Bat1 >= 2847.3)){//2847.3:0%; 
            soc1 = 0+(V_Bat1-2847.3)/(3113.8-2847.3)*5;
          }else{
            soc1 = 0;
          }

          //for Bat2
          if(V_Bat2 >= 3340){
            soc2 = 100;
          }else if((3340 > V_Bat2) && (V_Bat2 >= 3324)){
            soc2 = 95+(V_Bat2-3324)/(3340-3324)*5;
          }else if((3324 > V_Bat2) && (V_Bat2 >= 3320)){
            soc2 = 85+(V_Bat2-3320)/(3324-3320)*10;
          }else if((3320 > V_Bat2) && (V_Bat2 >= 3316.6)){
            soc2 = 75+(V_Bat2-3316.6)/(3320-3316.6)*10;
          }else if((3316.6 > V_Bat2) && (V_Bat2 >= 3288.7)){
            soc2 = 65+(V_Bat2-3288.7)/(3316.6-3288.7)*10;
          }else if((3288.7 > V_Bat2) && (V_Bat2 >= 3284.8)){
            soc2 = 55+(V_Bat2-3284.8)/(3288.7-3284.8)*10;
          }else if((3284.8 > V_Bat2) && (V_Bat2 >= 3284.7)){
            soc2 = 45+(V_Bat2-3284.7)/(3284.8-3284.7)*10;
          }else if((3284.7 > V_Bat2) && (V_Bat2 >= 3272.8)){
            soc2 = 35+(V_Bat2-3272.8)/(3284.7-3272.8)*10;
          }else if((3272.8 > V_Bat2) && (V_Bat2 >= 3249)){
            soc2 = 25+(V_Bat2-3249)/(3272.8-3249)*10;
          }else if((3249 > V_Bat2) && (V_Bat2 >= 3205.2)){
            soc2 = 15+(V_Bat2-3205.2)/(3249-3205.2)*10;
          }else if((3205.2 > V_Bat2) && (V_Bat2 >= 3113.8)){
            soc2 = 5+(V_Bat2-3113.8)/(3205.2-3113.8)*10;
          }else if((3113.8 > V_Bat2) && (V_Bat2 >= 2847.3)){//2847.3:0%; 
            soc2 = 0+(V_Bat2-2847.3)/(3113.8-2847.3)*5;
          }else{
            soc2 = 0;
          }
          
          if(V_Bat1 < V_Bat2){//reverse of charging
            soc = soc1;
          }else{
            soc = soc2;
          }
          
          if (input_switch == 1) { // if switch, move to charge
            next_state = 8;
            digitalWrite(8, true);
          } else { // otherwise stay put
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }

            case 3: { //Discharge state (-250mA and yellow LED)
                current_ref = -250;
                soc = soc + (current_measure/3808800)*100;//2 bats total cap=561+497=1058mAh,total Q = 1058*3600 mAs
                cap_delta = cap_delta - current_measure/3600;
                soh = (cap_delta/3808800)*100;//2 bats total cap=561+497=1058mAh,total Q = 1058*3600 mAs

                t_remain = soc*2.688;//in min
               if (V_Bat > 2500) { // While not at minimum volts, stay here
//                  if (charge_timer < 1200){ // discharging for 20min
                    next_state = 3;
                    digitalWrite(8, true);
//                    charge_timer++;
//               } else { // Or go to rest for 1min
//                    next_state = 9;
//                    digitalWrite(8, false);
//                    charge_timer = 0;
//                  }
               } else { // If we reach full discharged, move to rest
                 next_state = 4;
                 digitalWrite(8,false);
               }
               
                if (input_switch == 0) { //UNLESS the switch = 0, then go back to start
                next_state = 0;
                digitalWrite(8, false);
                }
                break;
              }
            case 4:{ // final Discharge rest, no LEDs no current
              current_ref = 0;
              soh_fin = soh;
              if(soh < 80){
                aged = 1;
              }else{
                aged = 0;
              }
              if (rest_timer < 60) { // Rest here for 1min
                next_state = 4;
                digitalWrite(8,false);
                rest_timer++;
              } else { // When thats done, move back to charging (and light the green LED)
                next_state = 0;
                digitalWrite(8,true);
                rest_timer = 0;
              }
      
              if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
                next_state = 0;
                digitalWrite(8,false);
              }
              break;
            }
      case 5: { // ERROR state RED led and no current
          current_ref = 0;
          next_state = 5; // Always stay here
          digitalWrite(7, true);
          digitalWrite(8, false);
          if (input_switch == 0) { //UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(7, false);
          }
          break;
        }

      case 7: { //soc lut
          digitalWrite(4, true);//Rly1 on
          digitalWrite(9, true);//Rly2 on
          V_Bat1 = analogRead(A1) * 4.096 / 1.03; //Mea1
          V_Bat2 = analogRead(A2) * 4.096 / 1.03; //Mea2
          current_ref = 0;
          
          if(V_Bat1 >= 3340){
            soc1 = 100;
          }else if((3340 > V_Bat1) && (V_Bat1 >= 3324)){
            soc1 = 95+(V_Bat1-3324)/(3340-3324)*5;
          }else if((3324 > V_Bat1) && (V_Bat1 >= 3320)){
            soc1 = 85+(V_Bat1-3320)/(3324-3320)*10;
          }else if((3320 > V_Bat1) && (V_Bat1 >= 3316.6)){
            soc1 = 75+(V_Bat1-3316.6)/(3320-3316.6)*10;
          }else if((3316.6 > V_Bat1) && (V_Bat1 >= 3288.7)){
            soc1 = 65+(V_Bat1-3288.7)/(3316.6-3288.7)*10;
          }else if((3288.7 > V_Bat1) && (V_Bat1 >= 3284.8)){
            soc1 = 55+(V_Bat1-3284.8)/(3288.7-3284.8)*10;
          }else if((3284.8 > V_Bat1) && (V_Bat1 >= 3284.7)){
            soc1 = 45+(V_Bat1-3284.7)/(3284.8-3284.7)*10;
          }else if((3284.7 > V_Bat1) && (V_Bat1 >= 3272.8)){
            soc1 = 35+(V_Bat1-3272.8)/(3284.7-3272.8)*10;
          }else if((3272.8 > V_Bat1) && (V_Bat1 >= 3249)){
            soc1 = 25+(V_Bat1-3249)/(3272.8-3249)*10;
          }else if((3249 > V_Bat1) && (V_Bat1 >= 3205.2)){
            soc1 = 15+(V_Bat1-3205.2)/(3249-3205.2)*10;
          }else if((3205.2 > V_Bat1) && (V_Bat1 >= 3113.8)){
            soc1 = 5+(V_Bat1-3113.8)/(3205.2-3113.8)*10;
          }else{//2847.3:0%; 
            soc1 = 0+(V_Bat1-2847.3)/(3113.8-2847.3)*5;
          }

          //for Bat2
          if(V_Bat2 >= 3340){
            soc2 = 100;
          }else if((3340 > V_Bat2) && (V_Bat2 >= 3324)){
            soc2 = 95+(V_Bat2-3324)/(3340-3324)*5;
          }else if((3324 > V_Bat2) && (V_Bat2 >= 3320)){
            soc2 = 85+(V_Bat2-3320)/(3324-3320)*10;
          }else if((3320 > V_Bat2) && (V_Bat2 >= 3316.6)){
            soc2 = 75+(V_Bat2-3316.6)/(3320-3316.6)*10;
          }else if((3316.6 > V_Bat2) && (V_Bat2 >= 3288.7)){
            soc2 = 65+(V_Bat2-3288.7)/(3316.6-3288.7)*10;
          }else if((3288.7 > V_Bat2) && (V_Bat2 >= 3284.8)){
            soc2 = 55+(V_Bat2-3284.8)/(3288.7-3284.8)*10;
          }else if((3284.8 > V_Bat2) && (V_Bat2 >= 3284.7)){
            soc2 = 45+(V_Bat2-3284.7)/(3284.8-3284.7)*10;
          }else if((3284.7 > V_Bat2) && (V_Bat2 >= 3272.8)){
            soc2 = 35+(V_Bat2-3272.8)/(3284.7-3272.8)*10;
          }else if((3272.8 > V_Bat2) && (V_Bat2 >= 3249)){
            soc2 = 25+(V_Bat2-3249)/(3272.8-3249)*10;
          }else if((3249 > V_Bat2) && (V_Bat2 >= 3205.2)){
            soc2 = 15+(V_Bat2-3205.2)/(3249-3205.2)*10;
          }else if((3205.2 > V_Bat2) && (V_Bat2 >= 3113.8)){
            soc2 = 5+(V_Bat2-3113.8)/(3205.2-3113.8)*10;
          }else{//2847.3:0%; 
            soc2 = 0+(V_Bat2-2847.3)/(3113.8-2847.3)*5;
          }
          
          if(V_Bat1 < V_Bat2){//reverse of charging
            soc = soc1;
          }else{
            soc = soc2;
          }

          next_state = 3;


          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }

      case 8: { // measure initial ocv of each cell and average to fine initial soc
          digitalWrite(4, true);//Rly1 on
          digitalWrite(9, true);//Rly2 on
          digitalWrite(3, false);//dis2 off
          digitalWrite(5, false);//dis1 off
          V_Bat1 = analogRead(A1) * 4.096 / 1.03; //Mea1
          V_Bat2 = analogRead(A2) * 4.096 / 1.03; //Mea2

          if (abs(V_Bat1-V_Bat2) > 10) { //if V_Bat1 != V_Bat2  
            if (V_Bat1 > V_Bat2) { //discharge bat1
              next_state = 10;
            } else { //discharge bat2
              next_state = 11;
            }
          } else { // otherwise go to charge
            digitalWrite(5, false);
            digitalWrite(4, false);
            digitalWrite(9, false);
            digitalWrite(3, false);
            next_state = 7;
          }

          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }
      case 9: { //rest and balance during charge
          current_ref = 0;
          digitalWrite(4, true);//Rly1 on
          digitalWrite(9, true);//Rly2 on
          digitalWrite(3, false);//dis2 off
          digitalWrite(5, false);//dis1 offs
          V_Bat1 = analogRead(A1) * 4.096 / 1.03; //Mea1
          V_Bat2 = analogRead(A2) * 4.096 / 1.03; //Mea2
          if (rest_timer < 120) { //rest for 2min
            next_state = 9;
            rest_timer ++;
          } else {
            next_state = 8;
            rest_timer = 0;
          }
          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }
      case 10: { // discharge bat1
          digitalWrite(9, true);//rly2 on
          digitalWrite(3, false);//dis2 off
          digitalWrite(4, true);//rly1 on
          digitalWrite(5, true);//dis1 on

          V_Bat1 = analogRead(A1) * 4.096 / 1.03; //Mea1
          V_Bat2 = analogRead(A2) * 4.096 / 1.03;//Mea2

          if (abs(V_Bat1 - V_Bat2)>10) { //discharge bat1 with whole current for 3min
            next_state = 10;
            charge_timer++;
          } else { // Or go to rest&compare ocv again
            next_state = 9;
            digitalWrite(5, false);//dis1 off

            charge_timer = 0;
          }

          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }
      case 11: { // discharge bat2
          digitalWrite(9, true);//rly2 on
          digitalWrite(3, true);//dis2 on
          
          digitalWrite(4, true);//rly1 on
          digitalWrite(5, false);//dis1 off
          V_Bat2 = analogRead(A2) * 4.096 / 1.03; //Mea2
          V_Bat1 = analogRead(A1) * 4.096 / 1.03; //Mea1

          if (abs(V_Bat2 - V_Bat1) > 10) { // discharge Bat2 with whole current for 3min
            next_state = 11;
            charge_timer ++;
          } else { // Or go to rest&ocv compare again
            next_state = 9;
            digitalWrite(3, false);//dis2 off
            charge_timer = 0;
          }

          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }



      default : { // Should not end up here ....
          Serial.println("Boop");
          current_ref = 0;
          next_state = 5; // So if we are here, we go to error
          digitalWrite(7, true);
        }

    }


//    dataString1 = String(state_num) + "," + "V_Bat=" + String(V_Bat) + "," + "Iref=" + String(current_ref) + "," + " Imeasure=" + String(current_measure) + "," + " soc=" + String(soc) + "," + " soc1=" + String(soc1)+ "," + " soc2=" + String(soc2)+ "," + "V_Bat1=" + String(V_Bat1) + "," + "V_Bat2=" + String(V_Bat2)+ "," + "soh=" + String(soh)+ "," + "Cap=" + String(cap_delta); //build a datastring for the CSV file
    dataString1 = "<<," +  String(soc) + "%," + String(soh_fin) + "%," + String(aged) + ",Discharging" + "," + String(t_remain)+ "min" ;
    dataString2 = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(soc) + "," + String(V_Bat1) + "," + String(V_Bat2) + "," + String(soh)+ "," + String(cap_delta);
    Serial.println(dataString1);// send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile) { //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString2); // print the data
    } else {
      //----------------------------------------------------------------------------------------
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
  }
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
