////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Firmware:  Arduino Compatible Firmware
//
// File:      uno.ino
//
/*! \file
 *
 * $LastChangedDate: 2014-02-08 18:17:50 -0700 (Sat, 08 Feb 2014) $
 * $Rev: 3516 $
 *
 * \brief Hekateros Arduino compatible firmware.
 *
 * For the original version of the Hekateros, an Arduino Uno was used.
 * The current hardware releases use the Arduino Mini Pro.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// FIRMWARE VERSION
//
const int  VERSION = 3;

//
// DIGITAL PINS
//
const int PIN_MAIN_PROC_HALT    =  2; ///<  D2: request main processor to halt 
const int PIN_MAIN_PROC_HALTED  =  3; ///<  D3: main processor halted
const int PIN_LIM_WRIST_ROT_0   =  4; ///<  D4: wrist 0 deg optical limit
const int PIN_LIM_WRIST_PITCH   =  5; ///<  D5: wrist pitch optical limit
const int PIN_LIM_ELBOW         =  6; ///<  D6: elbow optical limit
const int PIN_LIM_SHOULDER      =  7; ///<  D7: shoulder optical limit
const int PIN_LIM_BASE_ROT_180  =  8; ///<  D8: base 180 deg optical limit
const int PIN_LIM_BASE_ROT_0    =  9; ///<  D9: base 0 deg optical limit
const int PIN_EE_GPIO_2         = 10; ///< D10: end effector gpio 2
const int PIN_EE_GPIO_1         = 11; ///< D11: end effector gpio 1
const int PIN_ALARM_LED         = 12; ///< D12: equipement deck alarm LED
const int PIN_STATUS_LED        = 13; ///< D13: base status LED

//
// ANALOG PINS
//
const int PIN_PM_SHUTDOWN       = A1; ///< shutdown request from power module
const int PIN_PM_KILL           = A2; ///< signal power module to kill power
const int PIN_PM_WD_STROBE      = A3; ///< power module watch dog strobe

//
// SERIAL INTERFACE
//

// Command packet
const int  MAX_CMD_LEN      = 16;   ///< maximum command length
const byte CMD_START_CHAR   = '!';  ///< command start character
const byte EOC              = '\r'; ///< end of command character sequence

// Commands
const byte CMD_NONE         = 0;    ///< no command
const byte CMD_CONFIG_PIN   = 'c';  ///< configure pin command
const byte CMD_READ_EE      = 'e';  ///< read end effector i/o command
const byte CMD_PROC_HALTED  = 'h';  ///< main processor halted command
const byte CMD_SET_LED      = 'l';  ///< set LED on/off state
const byte CMD_READ_OPTICAL = 'o';  ///< read optical limits command
const byte CMD_READ         = 'r';  ///< read pin state command
const byte CMD_WRITE        = 'w';  ///< write pin state command
const byte CMD_TEST         = 't';  ///< test main processor - arduino comm
const byte CMD_READ_VERSION = 'v';  ///< read firmware version command

// Fixed command arguments values
const int CMD_ARG_LED_ALARM  =  1;  ///< alarm LED
const int CMD_ARG_LED_STATUS =  2;  ///< status LED

// Response packet
const byte RSP_OK           = '@';  ///< ok response start character
const byte RSP_ERR          = '#';  ///< error response start character
const char EOR[]            = "\n\r"; ///< end of response character sequence

// Response error codes
const byte ERR_CMD_TOO_LONG = '1';  ///< command too long
const byte ERR_NO_CMD_START = '2';  ///< no command start byte
const byte ERR_CMD_UNKNOWN  = '3';  ///< unknown/unsupported command
const byte ERR_CMD_NO_ARG   = '4';  ///< invalid number command arguments
const byte ERR_BAD_ARG      = '5';  ///< bad argument value
const byte ERR_WRONG_STATE  = '6';  ///< cannot execute in current fw state

// Command receive buffer
byte buf[MAX_CMD_LEN+1];            ///< command buffer
int buflen;                         ///< command length
int bufpos;                         ///< command current parse position

//
// TIMES 
//
const unsigned long T_LED_FAST_FLASH  = 100;    ///< LED fast flash period
const unsigned long T_LED_SLOW_FLASH  = 500;    ///< LED slow flash period
const unsigned long T_PM_WD_STROBE    = 100;    ///< powermodule watchdog period
const unsigned long T_FW_WD_MAX       = 5000;   ///< firmware watchdog threshold
const unsigned long T_MAIN_HALTED_MAX = 3000;   ///< main halted signal hold
const unsigned long T_FW_HALT_MAX     = 30000;  ///< main halt timeout thrshhold

unsigned long t_cur;          ///< current up time (msec)
unsigned long t_led_flash;    ///< last time of LED flash
unsigned long t_pm_wd;        ///< last time of power module watchdog strobe
unsigned long t_fw_wd;        ///< last time of firmware watchdog reset
unsigned long t_fw_halt;      ///< time waiting for main processor to halt
unsigned long t_main_halted;  ///< time since main sent halted signal

//
// STATE
//
const byte STATE_POWER_ON   = 0;  ///< power on firmware state
const byte STATE_INIT       = 1;  ///< initialized, no service firmware state
const byte STATE_RUNNING    = 2;  ///< running firmware state
const byte STATE_HALTING    = 3;  ///< halting firmware state
const byte STATE_POWER_OFF  = 4;  ///< power off firmware state

byte  OpState;                    ///< current firmware operation state
byte  oldState;                   ///< debug variable for detecting state change
int   pm_wd_line_state;           ///< current power module strobe line state
int   main_halted_state;          ///< current main processor halted line state
int   alarm_led_state;            ///< current alarm led state
int   status_led_state;           ///< current status led state
int   SeqNum;                     ///< response sequence number (for some cmds)


//------------------------------------------------------------------------------
// Arduino Hook Functions
//------------------------------------------------------------------------------

/*!
 * \brief Arduino Sketch setup() hook.
 *
 * Called once on power-up.
 */
void setup()
{
  // local state 
  OpState           = STATE_POWER_ON;
  oldState          = -1;
  pm_wd_line_state  = LOW;
  main_halted_state = HIGH;
  alarm_led_state   = LOW;
  status_led_state  = LOW;
  SeqNum            = 0;

  // Power Module I/F
  pinMode(PIN_PM_SHUTDOWN, INPUT);
  pinMode(PIN_PM_KILL, OUTPUT);
  digitalWrite(PIN_PM_KILL, HIGH);
  pinMode(PIN_PM_WD_STROBE, OUTPUT);
  digitalWrite(PIN_PM_WD_STROBE, pm_wd_line_state);

  // Main Processor I/F
  pinMode(PIN_MAIN_PROC_HALT, OUTPUT);
  digitalWrite(PIN_MAIN_PROC_HALT, main_halted_state);
  pinMode(PIN_MAIN_PROC_HALTED, INPUT);

  // LEDs
  pinMode(PIN_ALARM_LED, OUTPUT);
  set_alarm_led(alarm_led_state);
  pinMode(PIN_STATUS_LED, OUTPUT);
  set_status_led(status_led_state);

  // End Effector I/F
  pinMode(PIN_EE_GPIO_1, INPUT);
  pinMode(PIN_EE_GPIO_2, INPUT);

  // Joint Limit Switches I/F
  pinMode(PIN_LIM_BASE_ROT_0, INPUT);
  pinMode(PIN_LIM_BASE_ROT_180, INPUT);
  pinMode(PIN_LIM_SHOULDER, INPUT);
  pinMode(PIN_LIM_ELBOW, INPUT);
  pinMode(PIN_LIM_WRIST_PITCH, INPUT);
  pinMode(PIN_LIM_WRIST_ROT_0, INPUT);

  t_cur = millis();

  //strobe_pm_watchdog();

  Serial.begin(115200);
  Serial.setTimeout(250);

  buflen = 0;
  bufpos = 0;

  begin_no_service();
}

/*!
 * \brief Arduino Sketch loop() hook.
 *
 * Called each time through Sketch main loop.
 */
void loop()
{
  // current up time
  t_cur = millis();

  //
  // Strobe power module's watchdog signal.
  //
  // RDK ignore watchdog strobe for now until system board logic is debugged
  //if( dt(t_cur, t_pm_wd) >= T_PM_WD_STROBE )
  //{
  //  strobe_pm_watchdog();
  //  t_pm_wd = t_cur;
  //}

  //
  // Execute state step.
  //
  switch( OpState )
  {
    case STATE_RUNNING:
      step_running();
      break;
    case STATE_HALTING:
      step_halting();
      break;
    case STATE_POWER_OFF:
      die_power_off();
      break;
    case STATE_INIT:
    default:
      step_no_service();
      break;
  }
}


//------------------------------------------------------------------------------
// State Functions
//------------------------------------------------------------------------------

/*!
 * \brief Enter the initialized, no service state.
 */
void begin_no_service()
{
  oldState  = OpState;
  OpState   = STATE_INIT;

  set_alarm_led(HIGH);
  set_status_led(HIGH);

  t_pm_wd     = t_cur;
  t_led_flash = t_cur;
}

/*!
 * \brief Execute one step in the initialized, no service state.
 *
 * The firmware is initalized. However the main processor has not communicated
 * with the Arduino. So no services are provided by the robot.
 */
void step_no_service()
{
  //
  // Fast flash LEDs.
  //
  if( dt(t_cur, t_led_flash) >= T_LED_FAST_FLASH )
  {
    toggle_both_leds();
    t_led_flash = t_cur;
  }

  //
  // Power module sent shutdown signal. Go to halting state.
  //
  if( pm_start_shutdown() )
  {
    begin_halting();
  }

  //
  // Main processor sent halted signal. Power off.
  //
  if( main_proc_halted() )
  {
    //Serial.println("step_no_service::main_proc_halted"); //BHW
    begin_power_off();
  }

  //
  // Process command.
  //
  else
  {
    switch( process_command() )
    {
    case CMD_NONE:        // no command
      break;
    case CMD_PROC_HALTED: // halt command - go to power-off state.
      //Serial.println("step_no_service::CMD_PROC_HALTED");  //BHW
      begin_power_off();
      break;
    default:              // normal command - go to running state.
      begin_running();
      break;
    }
  }
}

/*!
 * \brief Enter the normal running state.
 */
void begin_running()
{
  oldState  = OpState;
  OpState   = STATE_RUNNING;

  set_alarm_led(LOW);
  set_status_led(HIGH);

  t_fw_wd = t_cur;
}

/*!
 * \brief Execute one step in the running state.
 */
void step_running()
{
  //
  // Power module sent shutdown signal. Go to halting state.
  //
  if( pm_start_shutdown() )
  {
    begin_halting();
  }

  //
  // Main processor sent halted signal. Power off.
  //
  if( main_proc_halted() )
  {
    //Serial.println("step_running::main_proc_halted"); //BHW
    begin_power_off();
  }

  //
  // Process command.
  //
  else
  {
    switch( process_command() )
    {
    case CMD_NONE:        // no command - check for fw watchdog timeout.
      if( dt(t_cur, t_fw_wd) >= T_FW_WD_MAX )
      {
        begin_no_service();
      }
      break;
    case CMD_PROC_HALTED: // halt command - go to power-off state.
      //Serial.println("step_running::CMD_PROC_HALTED"); //BHW
      begin_power_off();
      break;
    default:              // normal command - reset firmware watchdog timer.
      t_fw_wd = t_cur;
      break;
    }
  }
}

/*!
 * \brief Enter the halting state.
 *
 * The robot will be shutdown after a grace period.
 */
void begin_halting()
{
  oldState  = OpState;
  OpState   = STATE_HALTING;

  // send signal to main processor to halt
  signal_main_proc_to_halt();

  set_alarm_led(HIGH);
  set_status_led(HIGH);

  t_led_flash = t_cur;
  t_fw_halt = t_cur;
}

/*!
 * \brief Execute one step in the halting state.
 */
void step_halting()
{
  //
  // Slow flash LEDs.
  //
  if( dt(t_cur, t_led_flash) >= T_LED_SLOW_FLASH )
  {
    toggle_both_leds();
    t_led_flash = t_cur;
  }

  //
  // Main processor sent halted signal. Power off.
  //
  if( main_proc_halted() )
  {
    //Serial.println("step_halting::main_proc_halted"); //BHW
    begin_power_off();
  }

  //
  // Main processor sent halted message. Power off.
  //
  else if( process_command() == CMD_PROC_HALTED )
  {
    //Serial.println("step_halting::CMD_PROC_HALTED"); //BHW
    begin_power_off();
  }

  //
  // Timed out waiting for main processor halt signal or message. Power off.
  //
  else if( dt(t_cur, t_fw_halt) >= T_FW_HALT_MAX )
  {
    //Serial.println("FW"); //BHW
    begin_power_off();
  }
}

/*!
 * \brief Enter the power-off state.
 */
void begin_power_off()
{
  oldState  = OpState;
  OpState   = STATE_POWER_OFF;

  set_alarm_led(HIGH);
  set_status_led(HIGH);
}

/*!
 * \brief Infinite loop until power module turns off the robot.
 */
void die_power_off()
{
  //Serial.println("Ki!");  //BHW
  delay(10); // wait a bit for halt to finish
  digitalWrite(PIN_PM_KILL, LOW);
  while( 1 )
  {
    delay(1000);
  }
}

/*!
 * \brief Elapse delta time.
 *
 * \param t1  Most recent time (e.g. current time).
 * \param t0  Some earlier, previous time.
 *
 * \return Milliseconds.
 */
unsigned long dt(unsigned long t1, unsigned long t0)
{
  // time t1 is later than t0
  if( t1 >= t0 )
  {
    return t1 - t0;
  }
  // must haved wrapped - happens about every 50 days of up time.
  else
  {
    return 4294967295 - t0 + t1;
  }
}


//------------------------------------------------------------------------------
// Response Functions
//------------------------------------------------------------------------------

/*!
 * \brief Send error response.
 *
 * \param ecode Error code.
 */
void send_error(byte ecode)
{
  Serial.write(RSP_ERR);
  Serial.write(ecode);
  Serial.write(EOR);
} 

/*!
 * \brief Send bad integer argument error response.
 *
 * \param rc  Parsed integer error return code.
 */
void send_bad_int_error(int rc)
{
  if( rc == -1 )
  {
    send_error(ERR_CMD_NO_ARG);
  }
  else // -2
  {
    send_error(ERR_BAD_ARG);
  }     
}

/*!
 * \brief Send an simple ok response.
 *
 * No response arguments are sent.
 */
void send_ok()
{
  Serial.write(RSP_OK);
  Serial.write(EOR);
} 

/*!
 * \brief Send an ok response with one argument.
 *
 * \param v   Byte argument. Sent as ASCII decimal.
 */
void send_rsp(byte v)
{
  Serial.write(RSP_OK);
  Serial.write(v);
  Serial.write(EOR);
}

/*!
 * \brief Send an ok response with one argument.
 *
 * \param v   Null terminated character string argument.
 */
void send_rsp(char v[])
{
  Serial.write(RSP_OK);
  Serial.write(v);
  Serial.write(EOR);
}

/*!
 * \brief Send an ok response with one argument.
 *
 * \param v   Integer argument. Sent as ASCII decimal.
 */
void send_rsp(int v)
{
  Serial.write(RSP_OK);
  Serial.print(v);
  Serial.write(EOR);
}

/*!
 * \brief Send an ok response with one argument.
 *
 * \param v   Integer argument. Sent as ASCII hexidecimal hh.
 */
void send_hex_rsp(int v)
{
  Serial.write(RSP_OK);
  if( v < 16 )
  {
    Serial.write('0');
  }
  Serial.print(v, HEX);
  Serial.write(EOR);
}


//------------------------------------------------------------------------------
// Command Functions
//------------------------------------------------------------------------------

/*!
 * \brief Receive and process command.
 *
 * The function accumulates command bytes from the serial port. When
 * a complete command is received, the command is then executed and a response
 * sent.
 *
 * \return
 * On execution, returns command identifier byte.
 * If no command was executed, returns CMD_NONE (0).
 */
byte process_command()
{
  byte  c;
  byte  cmd = CMD_NONE;

  // no bytes
  if( !Serial.available() )
  {
    return cmd;
  }

  // end of command - execute
  if( (c = Serial.read()) == EOC )
  {
    buf[buflen] = 0;  // null-terminate
    bufpos = 0;       // reset parse postion
    cmd = doCmd();    // execute
    buflen = 0;       // clear command buffer
  }

  // command too long
  else if( buflen >= MAX_CMD_LEN )
  {
    buflen = 0;
    send_error(ERR_CMD_TOO_LONG);
  }

  // new command
  else if( c == CMD_START_CHAR )
  {
    buflen = 1;
    buf[0] = c;
  }

  // accumulate command bytes
  else
  {
    buf[buflen++] = c;
  }

  //Serial.println(cmd);
  return cmd;
}  

/*!
 * \brief Execute command.
 *
 * Command arguments are parsed, validated, and if valid, executed.
 * An ok or error response is sent.
 *
 * \return
 * On execution, returns command identifier byte.
 * If no command was executed, returns CMD_NONE (0).
 */
byte doCmd()
{
  byte  cmd = CMD_NONE;   // no command
  int   arg1, arg2;       // command/ response arguments 1 and 2

  // No command, but not an error. Ignore newlines and carriage returns
  if( (buflen == 0) || (buf[0] == '\r') || (buf[0] == '\n') )
  {
    Serial.write(EOR);
    return CMD_NONE;
  }

  // All commands start with a bang ('!').
  else if( buf[0] != CMD_START_CHAR )
  {
    send_error(ERR_NO_CMD_START);
    return CMD_NONE;
  }

  cmd  = buf[1];

  //
  // Parse and execute command.
  //
  switch( cmd )
  {
    // Read version: !v
    case CMD_READ_VERSION:
      arg1 = read_version();
      send_rsp(arg1);
      Serial.flush();
      break;

    // Read pin state: !r pin_id
    case CMD_READ:
      // pin_id
      bufpos = 3;
      arg1 = parse_int();

      if( arg1 < 0 )
      {
        send_bad_int_error(arg1);
      }
      else if( arg1 < 2 || arg1 > 13 )
      {
        send_error(ERR_BAD_ARG);
      }
      else
      {
        send_rsp( read_pin(arg1) );
      }
      break;

    // Write pin state: !w pin_id val
    case CMD_WRITE:
      // pin id
      bufpos = 3;
      arg1 = parse_int();

      if( arg1 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else if( arg1 < 2 || arg1 > 7 )
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // val
      bufpos++;
      arg2 = parse_int();

      if( arg2 == 0 )
      {
        arg2 = LOW;
      }
      else if( arg2 == 1 )
      {
        arg2 = HIGH;
      }
      else if( arg2 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // write pin
      if( arg1 == PIN_ALARM_LED )
      {
        set_alarm_led(arg2);
      }
      else if( arg1 == PIN_STATUS_LED )
      {
        set_status_led(arg2);
      }
      else
      {
        write_pin(arg1, arg2);
      }
      send_ok();
      break;

    // Debugging: Configure pin direction: !c pin_id dir
    case CMD_CONFIG_PIN:
      // pin id
      bufpos = 3;
      arg1 = parse_int();

      if( arg1 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else if( (arg1 < 2) || (arg1 > 13) )
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // direction
      bufpos++;
      arg2 = parse_int();

      if( arg2 == 0 )
      {
        arg2 = INPUT;
      }
      else if( arg2 == 1 )
      {
        arg2 = OUTPUT;
      }
      else if( arg2 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // configure pin
      config_pin(arg1, arg2);
      send_ok();
      break;

    // Read optical limits: !o
    case CMD_READ_OPTICAL:
      arg1 = read_optical();
      send_hex_rsp(arg1);
      break;

    // Read end effector I/O: !e
    case CMD_READ_EE:
      arg1 = read_ee();
      send_hex_rsp(arg1);
      break;

    // Set LED state: !w led_id val
    case CMD_SET_LED:
      // led id
      bufpos = 3;
      arg1 = parse_int();

      if( arg1 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else if( (arg1 != CMD_ARG_LED_ALARM) && (arg1 != CMD_ARG_LED_STATUS) )
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // value
      bufpos++;
      arg2 = parse_int();

      if( arg2 == 0 )
      {
        arg2 = LOW;
      }
      else if( arg2 == 1 )
      {
        arg2 = HIGH;
      }
      else if( arg2 < 0 )
      {
        send_bad_int_error(arg1);
        break;
      }
      else
      {
        send_error(ERR_BAD_ARG);
        break;
      }

      // set LED state
      switch( arg1 )
      {
        case CMD_ARG_LED_ALARM:
          set_alarm_led(arg2);
          break;
        case CMD_ARG_LED_STATUS:
          set_status_led(arg2);
          break;
      }
      send_ok();
      break;

    // test communication
    case CMD_TEST:
      Serial.write(RSP_OK);
      Serial.print(OpState);
      Serial.print(' ');
      Serial.print(SeqNum);
      Serial.write(EOR);
      SeqNum = (SeqNum + 1) & 0xff;
      break;

    case CMD_PROC_HALTED:
    // Invalid command.
    default:
      send_error(ERR_CMD_UNKNOWN);
      break;
  }

  return cmd;
}

/*!
 * \brief Parse ASCII unsigned integer argument.
 *
 * Pushes buffer position forward as parse proceeds.
 *
 * \return Returns integer \h_ge 0 on success.\n
 * Returns -1 on no argument.\n
 * Returns -2 on parse error.
 */
int parse_int()
{
  char  c;
  int   val = 0;
  int   rc = -2;

  if( bufpos > buflen )
  {
    return -1;
  }

  while( bufpos < buflen )
  {
    c = buf[bufpos];
    if( c >= '0' && c <= '9' )
    {
      rc = 0;
      val *= 10;
      val += c-'0';
      bufpos++;
    }
    else
    {
      break;
    }
  }

  if( rc == 0 )
  {
    return val;
  } 
  else
  {
    return rc;
  }
}

/*!
 * \brief Read firmware version.
 *
 * \return Version number.
 */
int read_version()
{
  return VERSION;
}


//------------------------------------------------------------------------------
// Digital I/O
//------------------------------------------------------------------------------

/*!
 * \brief Strobe power module's watchdog line.
 *
 * During normal operation, the watchdog times out after 1.6 seconds. If the 
 * power module receives signal to shutdown via the power button, the watchdog
 * times out in 0.4 seconds.
 *
 * To pet the watchdog, a rising or falling edge is needed. So strobing only
 * involves toggling the strobe line state.
 */
void strobe_pm_watchdog()
{
  pm_wd_line_state = pm_wd_line_state == LOW? HIGH: LOW;
  digitalWrite(PIN_PM_WD_STROBE, pm_wd_line_state);
}


/*!
 * \brief Test if power module interrupt line is active initiate shutdown.
 *
 * Active: LOW
 *
 * \return Returns true or false.
 */
boolean pm_start_shutdown()
{
  // RDK ignore signal for now until system board logic is debugged
  return false;
  // return digitalRead(PIN_PM_SHUTDOWN) == LOW? true: false;
}

/*!
 * \brief Set alarm LED state.
 *
 * Active: HIGH
 *
 * \param state   LOW or HIGH.
 */
void set_alarm_led(int state)
{
  alarm_led_state = state;
  digitalWrite(PIN_ALARM_LED, alarm_led_state);
}

/*!
 * \brief Set status LED state.
 *
 * Active: HIGH
 *
 * \param state   LOW or HIGH.
 */
void set_status_led(int state)
{
  status_led_state = state;
  digitalWrite(PIN_STATUS_LED, status_led_state);
}

/*!
 * \brief Toggle both the alarm and status LED states.
 */
void toggle_both_leds()
{
  set_alarm_led(alarm_led_state == HIGH? LOW: HIGH);
  set_status_led(status_led_state == HIGH? LOW: HIGH);
}

/*!
 * \brief Signal main processor to start halt sequence.
 *
 * Active: LOW
 */
void signal_main_proc_to_halt()
{
  digitalWrite(PIN_MAIN_PROC_HALT, LOW);
}

/*!
 * \brief Test if received halt signal from main processor.
 *
 * Active: LOW
 *
 * Signal must be present for X seconds.
 *
 * \return Returns true or false.
 */
boolean main_proc_halted()
{
  int     new_state;
  boolean rc;

  // RDK ignore signal for now until system board logic is debugged
  return false;

  new_state = digitalRead(PIN_MAIN_PROC_HALTED);

  // inactive high
  if( new_state == HIGH )
  {
    rc = false;
  }

  // high to low
  if( (main_halted_state == HIGH) && (new_state == LOW) )
  {
    t_main_halted = millis();
    rc = false;
  }

  // active 
  else if( (main_halted_state == LOW) && (new_state == LOW) )
  {
    // not long enough
    if( dt(t_cur, t_main_halted) < T_MAIN_HALTED_MAX )
    {
      rc = false;
    }
    // long enough
    else
    {
      rc = true;
    }
  }

  main_halted_state = new_state;

  return rc;
}

/*!
 * \brief Read pin state and convert to ASCII.
 *
 * \param id  Pin id.
 *
 * \return Returns '0' or '1'.
 */
byte read_pin(int id)
{
  return digitalRead(id) == LOW? '0': '1';
}

/*!
 * \brief Write new pin state.
 *
 * \param id    Pin id.
 * \param state LOW or HIGH
 */
void write_pin(int id, int state)
{ 
  digitalWrite(id, state);
}

/*!
 * \brief Read optical limit states.
 *
 * \note Bit order must be mapped to what is expected by software, not by
 * digital I/O pin order.
 *
 * \return Return packed integer of bit states.
 */
unsigned int read_optical()
{
  unsigned int b0, b1, b2 , b3, b4, b5;

  b0 = digitalRead(PIN_LIM_BASE_ROT_0);
  b1 = digitalRead(PIN_LIM_BASE_ROT_180)  << 1;
  b2 = digitalRead(PIN_LIM_SHOULDER)      << 2;
  b3 = digitalRead(PIN_LIM_ELBOW)         << 3;
  b4 = digitalRead(PIN_LIM_WRIST_PITCH)   << 4;
  b5 = digitalRead(PIN_LIM_WRIST_ROT_0)   << 5;

  return (b5 | b4 | b3 | b2 | b1 | b0);
}

/*!
 * \brief Read end effector I/O states.
 *
 * \note Bit order must be mapped to what is expected by software, not by
 * digital I/O pin order.
 *
 * \return Return packed integer of bit states.
 */
unsigned int read_ee()
{
  return (digitalRead(PIN_EE_GPIO_1) << 1) | (digitalRead(PIN_EE_GPIO_2)); 
}

/*!
 * \brief Configure I/O pin's direction.
 *
 * \param id    Pin id.
 * \param dir   INPUT or OUTPUT
 */
int config_pin(int id, int dir)
{
  pinMode(id, dir);

  return 0;
}
