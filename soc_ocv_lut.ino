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
unsigned int rest_timer;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_measure, current_ref = 0, error_amps; // Current Control
float pwm_out, pwm_out_v;
float V_Bat;
boolean input_switch;
int state_num = 0, next_state;
String dataString;
float soc = 0;

//voltage pid
float ev=0,cv=0,ei=0,oc=0; //internal signals
float volt_ref = 0, error_volt;
float kpv=0.05024,kiv=15.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float uv_max=4, uv_min=0; //anti-windup limitation
float current_limit;

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
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(5, OUTPUT);
  


  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);//Vbat
  pinMode(4, OUTPUT);//Rly
  pinMode(A1, INPUT);//Mea

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
    if ((state_num ==2) || (state_num ==4) || (state_num ==6)){//when resting
        digitalWrite(4, true);//turn on Rly
        V_Bat = analogRead(A1) * 4.096 / 1.03;//measurem from Mea port
    } else {
      //digitalWrite(9, false);
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
    
    
    //voltage pid
//    if (state_num == 7){
//      current_ref = 0;
//      volt_ref = 3600;
//      current_limit = 3;
//      error_volt = (volt_ref - V_Bat)/1000;
//      cv = pidv(error_volt);
//      cv = saturation(cv,current_limit , 0);
//      ei= cv - (current_measure/1000); //current error
//      pwm_out=pidi(ei);  //current pid
//    } else {
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      pwm_out = pidi(error_amps); //Perform the PID controller calculation
      
//    }
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
          soc = 0;
          if (input_switch == 1) { // if switch, move to charge
            next_state = 1;
            digitalWrite(8, true);
          } else { // otherwise stay put
            next_state = 0;
            digitalWrite(8, false);
          }
          break;
        }
      case 1: { // cc Charge state (250mA and a green LED)
          current_ref = 250;
          soc = 0;
          if (V_Bat < 3600) { // if not charged, stay put
            next_state = 1;
            digitalWrite(8, true);
//            digitalWrite(5, false);
          } else { // otherwise go to charge rest
            next_state = 2;//when reach 3.6v, change to cv charging-------------7
            digitalWrite(8, false);
            digitalWrite(5, false);
          }
          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
            next_state = 0;
//            digitalWrite(8, false);
          }
          break;
        }
      case 2:{ // Charge Rest, green LED is off and no current
        current_ref = 0;        
        //analogWrite(6, (int)(255 - pwm_out * 255));
        soc = 100;
        if (rest_timer < 1800) { // Stay here if timer < 15min
          next_state = 2;
          digitalWrite(8,false);
          rest_timer++;
        } else { // Or move to discharge (and reset the timer)
          next_state = 3;
          digitalWrite(8,false);
          rest_timer = 0;
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;        
      }
      case 3: { //Discharge state (-250mA and yellow LED)
          current_ref = -250;
          soc = soc + (current_measure/2019600)*100;//nominal capacity = 561mAh, total Q = 561*3600 = 2019600mAs
         if ((V_Bat > 2500) && (int(soc*10) != 951) && (int(soc*10) != 851)&& (int(soc*10) != 751) && (int(soc*10) != 651)&&(int(soc*10) != 551) && (int(soc*10) != 451)&&(int(soc*10) != 351) &&(int(soc*10) != 251)&& (int(soc*10) != 151)&&(int(soc*10) != 51)&&(int(soc*10) != 1)){ // While not at wanted soc, stay discharging
           next_state = 3;
           digitalWrite(8,false);
         } else if ((int(soc*10) == 951) || (int(soc*10) == 851)|| (int(soc*10) == 751) || (int(soc*10) == 651) || (int(soc*10) == 551) || (int(soc*10) == 451)||(int(soc*10) == 351) ||(int(soc*10) == 251)|| (int(soc*10) == 151)||(int(soc*10) == 51)||(int(soc*10) == 1)){//move to rest of 15min if soc values reached
           next_state = 6;
           digitalWrite(8,false);
         } else { // If we reach full discharged soc=0 , move to final rest
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
        if (rest_timer < 1800) { // Rest here for 15min
          next_state = 4;
          digitalWrite(8,false);
          rest_timer++;
        } else { // When thats done, move back to charging (and light the green LED)
          next_state = 1;
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

  case 6:{ // Discharge rest, no LEDs no current
        current_ref = 0;
        if (rest_timer < 1800) { // Rest here for 15min
          next_state = 6;
          digitalWrite(8,false);
          rest_timer++;
        } else { // When thats done, move back to discharging (and light the yellow LED)
          next_state = 3;
          digitalWrite(8,true);
          rest_timer = 0;
          soc = soc - 0.1;
        }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
//    case 7: { // cv Charge state (3.6v and green+yellow LED), cv for 30min
//          //volt_ref = 3600;
//          current_ref = 0;
//          //analogWrite(6, (int)(255 - pwm_out_v * 255));
//          soc = 100;
//          if (rest_timer < 1800) {
//            next_state = 7;
//            digitalWrite(8, true);
//            digitalWrite(5, true);
//            rest_timer++;//
//          } else { // otherwise go to charge rest
//            next_state = 2;
//            digitalWrite(8, false);
//            digitalWrite(5, false);
//            rest_timer = 0;//
//            
//          }
//          if (input_switch == 0) { // UNLESS the switch = 0, then go back to start
//            next_state = 0;
//            digitalWrite(8, false);
//          }
//          break;
//        }

  default : { // Should not end up here ....
      Serial.println("Boop");
      current_ref = 0;
      next_state = 5; // So if we are here, we go to error
      digitalWrite(7, true);
    }

  }

  dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure)+ "," + String(soc); //build a datastring for the CSV file
  Serial.println(dataString);// send it to serial as well in case a computer is connected
  File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
  if (dataFile) { //If we succeeded (usually this fails if the SD card is out)
    dataFile.println(dataString); // print the data
  } else {
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

// This is a PID controller for the voltage
float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}
