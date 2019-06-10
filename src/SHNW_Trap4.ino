



/*
Workplan; 29/5/19
Aim placing function calls into state machine, and state transitions into functions,
            incomplete function can be commented back out when incomplete
test state machine with debug function
complete and test all functions individually


FileName--SHNWtrapv1  plan; this project saved at home/vere/Documents/Platformio/projects/SHNWtrapv1
   Starting with an empty state machine, name states from diagram, borrow useful code from previous
   projects to manage traps in a mesh with master trap communicating to opperator
   with SMS
fnxns
DeBug    Done  used before, adapt
DOORs    Done  untested
Battery  done untested
Assemble_A_Report_Message  ?
send text    New GMS board, need to start over
receive text   New GMS board, need to start over
Prox Sensors  ****started switches but incomplete.
incorporate GitHub and learn to use it..just made a meaningful change since master comit
PINMAP;;
A0  dio?                      //voltge ADC from voltage splitter
A1  IR distance sensor1
A2  IR distance sensor2
A3
A4  dio?
A5  dio?
Rx, Rx
Tx, Tx
3
4   =rst OUTPUT
5   =BTN INPUT PULLUP
6   =ECHO input
7   =INT
8     =CS (=nss )
9     servo
10
11  = enable output
12  = trig output
13  =LED  OUTPUT


#define RFM95_CS 17  ???
#define RFM95_RST 21  ???

 */

#include <Arduino.h>    // so can use platformio, Arduino IDE doesn't mind
//include stuff for GMS  //New GMS board, so need new includes and declares
 //#include <GSM.h>       // include the GSM library
//#define PINNUMBER ""        // PIN Number for the SIM
//GSM gsmAccess;              // initialize the library instances
//GSM_SMS sms;

//additional includes for SMS receive
// Array to hold the number a SMS is retreived from
char senderNumber[20];                                 // Array to hold the number a SMS is retreived from

//includes for radios
#define RH_MESH_MAX_MESSAGE_LEN 50
#include <RHMesh.h>                                                 //Problematic on Arduino OK on Platformio
#include <RH_RF22.h>                                                //Problematic on Arduino OK on Platformio
#include <SPI.h>

//include for servo
#include <ServoTimer2.h>


///Traps in the mesh
// In this instance a small network of 6 nodes,
#define CLIENT_ADDRESS 1
#define SERVER1_ADDRESS 2
#define SERVER2_ADDRESS 3
//#define SERVER3_ADDRESS 4                                       //uncomment when more traps in mesh
//#define SERVER4_ADDRESS 5
//#define SERVER5_ADDRESS 6
int I_am_trap;   //Identify individual radio/trap here for reporting it's own ID ****Probably redundant due to server/client addresses above
int Is_Trap_Armed_Disarmed_Or_Tripped;
// Singleton instance of the radio driver
RH_RF22 driver;                               // Singleton instance of the radio driver
// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, CLIENT_ADDRESS);

//States
#define TRAP_is_ARMED 0
#define CALLED_by_SMS 1
#define BATTERY_is_LOW 2
#define TRAP_is_SPRUNG 3
#define REPORT_to_MESH  4
#define CALLED_by_MESH  5
#define REPORTING_by_SMS 6            //only on master=client
#define TRAP_is_DISARMED 7
int STATE_current = TRAP_is_ARMED;                    //basic state
char Trap_is;                                         //there will be 3 options, Armed, Sprung or Disarmed

//battery test states
int Battery_Test_Cycle ;                           //     no names defined, just use 0 and 1
unsigned long Battery_Time;                       // hourly in ms

#define DEBUG //comment out to stop debug messages and save space in Sketch File
              /* use in conjunction with structures like:
              #ifdef DEBUG
              Serial.print(F("pressCounter = "));
              Serial.print(pressCounter, DEC);
              #endif
              OR

              #ifdef DEBUG
                     Serial.println(F("I just typed 'z' to stop the serial"));
                     Serial.end();
              #endif
              */

//Output integers from DeBug serial
int deBugByte1;
int deBugByte2;
int deBugByte3;
int deBugByte4;
int deBugByte5;
int debugByte;


//defines and includes, DOOR_states
/* Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin */
#define DOOR_STATE_WAITING 0
#define DOOR_STATE_INITIALISE 1
#define DOOR_STATE_DROPPING    2
#define DOOR_STATE_RESETTING    3
int DOOR_state = DOOR_STATE_WAITING;
ServoTimer2  DOOR_servo1;                          //declare the servo that will open the door when "attached " to pin9
int DOOR_Drop_Angle = 950;                     //local variable for servo angle
int DOOR_Reset_Angle = 1600;


//Voltage measurements
int analogVolt ;                                           //analog response from pinA0

float Volts = 0.0;                   // multiply analog by voltage split ratio, then multiply by 5/1023 for volts

//SMS stuff
char Make_trap;                                         // there will be  3(4) options, Armed, Disarmed, Report_all (and maybe report[I_am_trap

#define Proximity_Detection  10                          //two cases of sub switch inside "trap is armed" to cycle routines while trap armed
#define SMS_Monitoring      11                          //two cases of sub switch inside "trap is armed" to cycle routines while trap armed
#define Power_Check         12
int Cycle_Though_Sensor_Checks = Proximity_Detection;      //default case of sub switch inside "trap is armed"

//Sensor_Stuff
#define CHECK_FIRST_SENSOR 0                            //checks sensor 1 every loop till finds, only then checks second
#define CHECK_SECOND_SENSOR 1
int Prox_Sensor_Watch = CHECK_FIRST_SENSOR ;                                //switch to cycle through proximity sensor


//Have to reserve the fillowing pins as per libraries;
//

const int Volt_Pin = A0;
const int Sensor_One_Pin = 2;
const int Sensor_Two_Pin = 3;
const int DOOR_servoPin = 9;                          //something else wants 8

// End include / defines
//***********************Setup****setup****Setup****setup****Setup****setup****Setup****setup

void setup() {

//pinmodes
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(8, OUTPUT);
        //Setup for GSM_SMS send
        // initialize serial communications and wait for port to open:
        Serial.begin(9600);
        while (!Serial) {
                ; // wait for serial port to connect. Needed for native USB port only
        }



//Setup for door/servo
        Serial.println(F("setup() - Starting DOOR servo."));
        DOOR_servo1.attach(DOOR_servoPin);
        DOOR_servo1.write(DOOR_Drop_Angle);
        delay(500);
        DOOR_servo1.write(DOOR_Reset_Angle);
        delay(500);
        DOOR_servo1.detach();

//Setup for Voltage
analogVolt = analogRead(Volt_Pin);
Battery_Time = millis ();
int Battery_Test_Cycle = 0;

// setup for state machine
STATE_current = TRAP_is_ARMED;
                           //armed

}                                                                 //End setup

/*
   Debug fnxn
   Switch statement with serial input

   Demonstrates the use of a switch statement. The switch statement allows you
   to choose from among a set of discrete values of a variable. It's like a
   series of if statements.

   To see this sketch in action, open the Serial monitor and send any character.
   The characters a, b, c, d, and e, will turn on LEDs. Any other character will
   turn the LEDs off.

   Adapted from Tom Igoe 1 Jul 2009

   This example code is in the public domain.

   http://www.arduino.cc/en/Tutorial/SwitchCase2
   //use serial for debugging inputs.
   // do something different depending on the character received.
   // The switch statement expects single number values for each case; in this
   // example, though, you're using single quotes to tell the controller to get
   // the ASCII value for the character. For example 'a' = 97, 'b' = 98,
   // and so forth:
 */
void debug () {                       //type in debug inputs via serial monitor

        if (Serial.available() > 0) {
                int debugByte = Serial.read(); //what you type is the case
        }

        switch (debugByte) {
        case 'a':                             //97
                deBugByte1 = LOW;         //mimic inputs from ? sensor ?
                break;
        case 'b':                         //98
                deBugByte1 = HIGH;         //? restore i...e. undo
                break;
        case 'c':
                deBugByte2 = LOW;         //mimic
                break;
        case 'd':                           //
                deBugByte2 = HIGH;          //restore
                break;
      /*  case 'e':
                deBugByte3 = LOW;             //mimic ?

                break;
        case 'f':
                deBugByte4 = LOW;               //mimic
                break;

        case 'g':
                deBugByte5 = LOW;                   // use to mimic
                break;

        case 'h':
                deBugByte5 = HIGH;                   // restores door to high
                break;    */

        case 'z':
                Serial.println(F("I just typed 'z' to stop the serial"));
                Serial.end();
                break;
        }                                                       //end Switch
}                                                               // end fnxn


//a function to drop doors when required
void DOOR_trigger() {                                 //this function drops doors, called from state
        int DOOR_Reset_Angle = 1600;                 //local variable for servo angle
        int DOOR_Drop_Angle = 900;                  //local variable for other servo angle
        unsigned long DOOR_state_time;                  //local time variable for door servo

        switch (DOOR_state) {

        case DOOR_STATE_WAITING:                              //label 0
                Serial.println(F("Switch DOOR_state least active in case DOOR_STATE_WAITING"));           //debug
                DOOR_servo1.detach();                         //Stops servo chatter & power loss
                STATE_current = REPORT_to_MESH;           // state transition

                break;

        case DOOR_STATE_INITIALISE: //label 1
                //**** need a separate state "detach"
                Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_INITIALISE"));             //debug
                DOOR_state = DOOR_STATE_DROPPING; //commit to  next loop state so all sttements now run only once before exit
                DOOR_servo1.attach(DOOR_servoPin);                    // door servo has been attached,
                DOOR_state_time = millis();     // reinitialises timer after commited to case DOOR_state-DOOR_STATE_OPENING

                break;

        case DOOR_STATE_DROPPING:                                  //label 2
                Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_OPENING"));             //debug

                DOOR_servo1.write(DOOR_Reset_Angle); //
                DOOR_state = DOOR_STATE_RESETTING;
                DOOR_state_time = millis();       // reinitialises timer after commited to case transition



                break;

        case DOOR_STATE_RESETTING:                                 //label 3
                Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_CLOSING"));             //debug
                if ( millis() - DOOR_state_time > 1000) {   // allow 1 sec for servo sweep
                        DOOR_servo1.write(DOOR_Reset_Angle);
                }
                if ( millis() - DOOR_state_time > 2000) {
                        DOOR_state = DOOR_STATE_WAITING;   //case transition

                }

                break;

        }                                                     //end Switch
}                                                             // end fnxn DOOR_trigger


/*
void Receive_SMS(){      //New GMS board, needs new function
        char c;                                              //create a variable of type char to temporarily hold characters from any SMS received.

        // If there are any SMSs available()
        if (sms.available()) {                              //
                Serial.println("Message received from:");

                // Get remote number
                sms.remoteNumber(senderNumber, 20);         //enters phone number
                Serial.println(senderNumber);               //prints on monitor

                // An example of message disposal
                // Any messages starting with # should be discarded
                if (sms.peek() == '#') {           //Using sms.peek() it's possible to identify the message index number, which could be helpful for removal
                                                   //Returns the next byte (character) of an incoming SMS without removing it from the message.
                                                   //That is, successive calls to peek() will return the same character,
                        Serial.println("Discarded SMS"); //PRINTS THIS TO monitor
                        sms.flush();                //Dclears the modem memory of any sent messages once all outgoing characters have been sent.
                }

                // Read message bytes and print them
                while (c = sms.read()) {
                        Serial.print(c);
                }

                Serial.println("\nEND OF MESSAGE");

                // Delete message from modem memory
                sms.flush();
                Serial.println("MESSAGE DELETED");
        }

        delay(1000);

}  */



void Watch_Two_proximity_Sensors(){                           // first of 3 fnxns called by state
                                                              // two ways out, state TRAP_is_SPRUNG or sub text
Is_Trap_Armed_Disarmed_Or_Tripped = 0 ;
    int Sensor_One_Val;
    int Sensor_Two_Val;
  int Threshhold_Proximity = 500;  //Local var for proximity threshhold. Start point 500, find best impirical value


  switch (Prox_Sensor_Watch) {                      // a switch to cycle through reading the proximity sensors

    case  CHECK_FIRST_SENSOR:       //label 0:
Sensor_One_Val = analogRead (Sensor_One_Pin);
if (Sensor_One_Val > Threshhold_Proximity) {
  Prox_Sensor_Watch = CHECK_SECOND_SENSOR;                         // only check second sensor if first finds something
}
else{
  Prox_Sensor_Watch = CHECK_FIRST_SENSOR;                           //

}

    break;


    case CHECK_SECOND_SENSOR:       //label 1
Sensor_Two_Val = analogRead(Sensor_Two_Pin);
if (Sensor_Two_Val > Threshhold_Proximity) {
STATE_current = TRAP_is_SPRUNG ;
}
else{
  Prox_Sensor_Watch = CHECK_FIRST_SENSOR;                           //
  //Receive_SMS();
}

  break;

}
}


void Battery_Check(){
        unsigned long Battery_Time;                             //      local variable to time hourly voltage checks

        switch (Battery_Test_Cycle) {
          case 0:
          if (millis() - Battery_Time  > 3600000) {                //check if the hour is up
          Battery_Test_Cycle = 1;
        }
              break;;
          case 1:
        analogVolt = analogRead(Volt_Pin);                       //updates the voltage
        Battery_Test_Cycle = 0;                         //having done reading, return to case 0 wich counts down another hour
        Battery_Time = millis ();               //reset each hour one commited to case transition
        break;

}                                                   //end switch
}                                                    //end fnxn Battery_Check


void Concatenate_Report(){
int Message_Array[5];
Message_Array[0] = I_am_trap;
Message_Array[1] = Is_Trap_Armed_Disarmed_Or_Tripped;      //variable for /armed 0 /Disarmed 1 /tripped 2
Message_Array[2] = analogVolt;

//Message_Array[3] = ;
//Message_Array[4] = ;

}

  /*//This is wrong, wont work,,do Array  or parse a string volts as analog simplest                           //Assembles a comma delineated char for sending report . maybe incorporate into another fnxn later
  To assign a value to an array:
  mySensVals[0] = 10;

        Serial.write( "I_am_trap ");                      // optional headings for readability, remove if problematic
        Serial.write(",");
        Serial.write(I_am_trap);
        Serial.write(",");
        Serial.write("Trap_is");                     //
        Serial.write(",");
        Serial.write(Trap_is);
        Serial.write(",");
        Serial.write("volts");                     //
        Serial.write(",");
        //Serial.write(volts);                              //error: call of overloaded 'write(float&)' is ambiguous
        /*  Serial.write("")
           Serial.write(,)
           Serial.write()
           Serial.write("")
           Serial.write(,)
           Serial.write()
}
*/

/*
void SEND_Text(){                     //New GMS board, need new receive fnxn

        Serial.print("Enter a mobile number: ");
        char remoteNum[20]; // telephone number to send sms
        //****readSerial(remoteNum);                        //error: 'readSerial' was not declared in this scope
        Serial.println(remoteNum);

        // sms text
        Serial.print("Now, enter SMS content: ");
        char txtMsg[200];
        //****readSerial(txtMsg);
        Serial.println("SENDING");
        Serial.println();
        Serial.println("Message:");
        Serial.println(txtMsg);

        // send the message
        sms.beginSMS(remoteNum);                // tells board to start sending
        sms.print(txtMsg);                      //this is where char txtMsg gets sent
        sms.endSMS();                           //tells board to stop
        Serial.println("\nCOMPLETE!\n");
}
/*
   Read input serial


int readSerial(char result[]) {               //a function-definition is not allowed here before '{' token
                                            // but this is straight from the arduino example and compiled on arduino

   int i = 0;
   while (1) {
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      if (inChar == '\n') {
        result[i] = '\0';
        Serial.flush();
        return 0;
      }
      if (inChar != '\r') {
        result[i] = inChar;
        i++;
    }
   }
   }
   }
 */




void loop() {

        //if (Serial.available() > 0) {  //do something****

//}

        switch (STATE_current) {

        case TRAP_is_ARMED:                             // label 0
        Is_Trap_Armed_Disarmed_Or_Tripped = 0;

        switch (Cycle_Though_Sensor_Checks) {              // sub switch to cycle routines while trap armed

break;
        case Proximity_Detection:     //label 10           //check IR distance sensors
         Watch_Two_proximity_Sensors();
Cycle_Though_Sensor_Checks = 11;
break;
         case SMS_Monitoring:         //label 11          //check for new Mesh messages
                                                          //check for new SMS message
  Cycle_Though_Sensor_Checks = 12;
break;
          case Power_Check :                 //label 12
        Battery_Check();
  Cycle_Though_Sensor_Checks = 10;
break;


         }
// call fnxn Proximity_Detection   Activation transitions to TRAP_is_SPRUNG
//call fnxn monitor sms           Activation transitions to CALLED_by_SMS
//call fnxn monitor LoRa          Activation transitions to CALLED_by_MESH:
//Call fnxn hourly battery check  Activation transitions to BATTERY_is_LOW :



        case CALLED_by_SMS:                                                           //label 1
                // if master trap/client, Parse message and call appropriate functions

                break;
        case BATTERY_is_LOW:                                                          //label 2
                //Activation transitions to REPORT_to_MESH
                //decide if default action is to disable trap or just alert by SMS

                break;
        case TRAP_is_SPRUNG:                                                          //label 3
                DOOR_trigger();
                Is_Trap_Armed_Disarmed_Or_Tripped = 2;                          //tripped

                //then transitions to REPORT_to_MESH:


                break;

        case REPORT_to_MESH:                                                        //label 4
// call reporting function "Concatenate_Report()"
// if is master trap/client then transitions to ?? if server
//then transitions to TRAP_is_DISARMED:
                break;
        case CALLED_by_MESH:                                                              // label 5
        //Parse message and call appropriate functions

//then transitions to
                break;
        case REPORTING_by_SMS:                                                         //label 6
                if (I_am_trap = 1) {                                          // if master trap/client, forward mesh message to phone
                        // call SMS function to forward message payload to user phone by text
                }

// send each message as separate text? aggregate for ?30 sec and send all in 1 text,
//? storing incoming LoRa till ready to text if overlap??

                break;
        case TRAP_is_DISARMED:                                                              // label 7
                //call fnxn monitor sms
                //call fnxn monitor LoRa
                //Call hourly battery check
                //transitions to TRAP_is_ARMED only on CALLED_by_SMS > CALLED_by_MESH
                Is_Trap_Armed_Disarmed_Or_Tripped = 1;                            //disarmed

                break;




        }                                                 //end switch
}                                                         //end loop
