// Modified by Sven Rosvall (M3777)
/**************************************************************************************
  Version 1c
  Simplified setup.
  Broke out functions from setup() and loop() functions to make them small.
  The new functions are concerned with only one thing.
*************************************************************************************/

// Modified by Martin Da Costa (M6223)
/**************************************************************************************
  Version 1b
  Allows configuration for either 8MHz or 16MHz CANBUS module crystal
  Replaces defs.h with pre-processing constants
  Corrects initial name printout on serial initialisation
  Correects error in if statement in loop that prevented module switch from working
*************************************************************************************/

/*
  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)
  Copyright (C) Martin Da Costa 2020
  
  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map UNO:
// Digital pin 2 ()       Interupt CAN
// Digital pin 3 (PWM)    Green SLiM LED
// Digital pin 4 ()       Yellow FLiM LED
// Digital pin 5 (PWM)    CBUS Switch
// Digital pin 6 (PWM)    Not Used
// Digital pin 7 ()       Module LED
// Digital pin 8 ()       Module Switch
// Digital pin 9 (PWM)    Not Used
// Digital pin 10 (PWM,SS) CS   CAN
// Digital pin 11 (PWM,MOSI) SI CAN   ICSP-4
// Digital pin 12 (MISO)  SO    CAN   ICSP-1
// Digital pin 13 (SCK)   Sck   CAN   ICSP-3

// Digital / Analog pin 0     Not Used
// Digital / Analog pin 1     Not Used
// Digital / Analog pin 2     Not Used
// Digital / Analog pin 3     Not Used
// Digital / Analog pin 4     Not Used
// Digital / Analog pin 5     Not Used
//////////////////////////////////////////////////////////////////////////


// 3rd party libraries
#include <Streaming.h>

// CBUS library header files
#include <CBUS2515.h>               // CAN controller and CBUS class
#include <CBUSswitch.h>             // pushbutton switch
#include <CBUSLED.h>                // CBUS LEDs
#include <CBUSconfig.h>             // module configuration
#include <CBUSParams.h>             // CBUS parameters
#include <cbusdefs.h>               // MERG CBUS constants

// constants
const byte VER_MAJ = 1;                  // code major version
const char VER_MIN = 'c';                // code minor version
const byte VER_BETA = 0;                 // code beta sub-version
const byte MODULE_ID = 99;               // CBUS module type

const byte CAN_OSC_FREQ = 8000000uL;     // Oscillator frequency on the CAN2515 board

const byte LED_GRN = 3;                  // CBUS green SLiM LED pin
const byte LED_YLW = 4;                  // CBUS yellow FLiM LED pin
const byte SWITCH0 = 5;                  // CBUS push button switch pin

const byte CAN_INT_PIN = 2;  // Only pin 2 and 3 support interrupts
const byte CAN_CS_PIN = 10;
const byte CAN_SI_PIN = 11;  // Cannot be changed
const byte CAN_SO_PIN = 12;  // Cannot be changed
const byte CAN_SCK_PIN = 13;  // Cannot be changed

// 1IN1OUT module pins
const byte MODULE_LED_PIN = 7;
const byte MODULE_SWITCH_PIN = 8;

// CBUS objects
CBUS2515 CBUS;                      // CBUS object
CBUSConfig config;                  // configuration object
CBUSLED ledGrn, ledYlw;             // LED objects
CBUSSwitch pb_switch;               // switch object

// module objects
CBUSSwitch moduleSwitch;            // an example switch as input
CBUSLED moduleLED;                  // an example LED as output

// module name, max 7 characters.
unsigned char mname[] = "1IN1OUT";

// forward function declarations
void eventhandler(byte index, byte opc);

//
/// setup CBUS - runs once at power on from setup()
//

void setupCBUS()
{
  // set config layout parameters
  config.EE_NVS_START = 10;
  config.EE_NUM_NVS = 10;
  config.EE_EVENTS_START = 50;
  config.EE_MAX_EVENTS = 64;
  config.EE_NUM_EVS = 1;
  config.EE_BYTES_PER_EVENT = (config.EE_NUM_EVS + 4);

  // initialise and load configuration
  config.setEEPROMtype(EEPROM_INTERNAL);
  config.begin();

  Serial << F("> mode = ") << ((config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID;
  Serial << F(", NN = ") << config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);
  params.setProcessor(CPUM_ATMEL, 0x32, "328P");

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // initialise CBUS switch and assign to CBUS
  pb_switch.setPin(SWITCH0, LOW);
  pb_switch.run();
  CBUS.setSwitch(pb_switch);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  if (pb_switch.isPressed() && !config.FLiM) {
    Serial << F("> switch was pressed at startup in SLiM mode") << endl;
    config.resetModule(ledGrn, ledYlw, pb_switch);
  }

  // set CBUS LED pins and assign to CBUS
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);
  CBUS.setLEDs(ledGrn, ledYlw);

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);

  // set CBUS LEDs to indicate mode
  CBUS.indicateMode(config.FLiM);

  // configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(2);         // more buffers = more memory used, fewer = less
  CBUS.setOscFreq(CAN_OSC_FREQ);   // select the crystal frequency of the CAN module
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN); // select pins for CAN bus CE and interrupt connections
  CBUS.begin();
}

//
/// setup - runs once at power on
//

void setup()
{
  Serial.begin (115200);
  Serial << endl << endl << F("> ** CBUS 1 in 1 out v1 ** ") << __FILE__ << endl;

  setupCBUS();

  // configure the module switch, active low
  moduleSwitch.setPin(MODULE_SWITCH_PIN, LOW);

  // configure the module LED
  moduleLED.setPin(MODULE_LED_PIN);

  // end of setup
  Serial << F("> ready") << endl << endl;
}

//
/// loop - runs forever
//

void loop() 
{
  // do CBUS message, switch and LED processing
  CBUS.process();

  // process console commands
  processSerialInput();

  // give the switch and LED code some time to run
  moduleSwitch.run();
  moduleLED.run();

  // Check if smich changed and do any processing for this change.
  processModuleSwitchChange();

  // bottom of loop()
}

//
/// test for switch input
/// as an example, it must be have been pressed or released for at least half a second
/// then send a long CBUS event with opcode ACON for on and ACOF for off
/// event number (EN) is 1
//
/// you can just watch for this event in FCU or JMRI, or teach it to another CBUS consumer module
//
void processModuleSwitchChange()
{
  if (moduleSwitch.stateChanged()) 
  {
    Serial << F(">Button state change detected - button ") 
           << (moduleSwitch.isPressed() ? F("pressed") : F("released")) << endl;
    CANFrame msg;
    msg.id = config.CANID;
    msg.len = 5;
    msg.data[0] = (moduleSwitch.isPressed() ? OPC_ACON : OPC_ACOF);
    msg.data[1] = highByte(config.nodeNum);
    msg.data[2] = lowByte(config.nodeNum);
    msg.data[3] = 0;
    msg.data[4] = 1;            // event number (EN) = 1

    if (CBUS.sendMessage(&msg)) {
      Serial << F("> sent CBUS message") << endl;
    } else {
      Serial << F("> error sending CBUS message") << endl;
    }
  }
}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte index, CANFrame *msg)
{
  // as an example, control an LED

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;

  // read the value of the first event variable (EV) associated with this learned event

  byte ev = 1;
  int eeaddress = config.EE_EVENTS_START + (index * config.EE_BYTES_PER_EVENT) + 4 + (ev - 1);
  byte evval = config.readEEPROM(eeaddress);
  Serial << F("> EV1 = ") << evval << endl;

  // set the LED according to the opcode of the received event and its EV
  if (msg->data[0] == OPC_ACON)
  {
    if (evval == 1)
    {
      Serial << F("> switching the LED to blink") << endl;
      moduleLED.blink();
    }
    else if (evval == 0)
    {      
      Serial << F("> switching the LED on") << endl;
      moduleLED.on();
    }
  }
  else if (msg->data[0] == OPC_ACOF)
  {
    Serial << F("> switching the LED off") << endl;
    moduleLED.off();
  }

  return;
}

//
/// print code version config details and copyright notice
//
void printConfig(void) 
{
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2019") << endl;
  Serial << F("> © Martin Da Costa (MERG M6223) 2020") << endl;
  return;
}

//
/// command interpreter for serial console input
//

void processSerialInput(void) 
{
  byte uev = 0;
  char msgstr[32], dstr[32];

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID << F(", node number = ") << config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;

        sprintf(msgstr, "  max events = %d, EVs per event = %d, bytes per event = %d", config.EE_MAX_EVENTS, config.EE_NUM_EVS, config.EE_BYTES_PER_EVENT);
        Serial << msgstr << endl;

        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {
          if (config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * config.EE_BYTES_PER_EVENT) << F(" of ") << (config.EE_MAX_EVENTS * config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (config.EE_NUM_EVS); j++) {
          sprintf(dstr, "EV%03d | ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {

          if (config.getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d  | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (config.EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", config.readEEPROM(config.EE_EVENTS_START + (j * config.EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, "%4d |", config.getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= config.EE_NUM_NVS; j++) {
          byte v = config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':

        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << config.freeSRAM() << F(" bytes") << endl;
        break;

      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
