/* Hekateros Arduino Uno Firmware */

// DIGITAL PINS
const int PIN_0  = 0x00;
const int PIN_1  = 0x01;
const int PIN_2  = 0x02;
const int PIN_3  = 0x03;
const int PIN_4  = 0x04;
const int PIN_5  = 0x05;    // STATUS LED
const int PIN_6  = 0x06;    // EEF 1
const int PIN_7  = 0x07;    // EEF 2
const int PIN_8  = 0x08;    // Optical 1 
const int PIN_9  = 0x09;    // Optical 2
const int PIN_10 = 0x0A;    // Optical 3
const int PIN_11 = 0x0B;    // Optical 4
const int PIN_12 = 0x0C;    // Optical 5
const int PIN_13 = 0x0D;    // Optical 6

// serial itnerface
const int  MAX_CMD_LEN    = 16;
const byte CMD_START      = '!';

const byte READ_VERSION   = 'v';
const byte READ           = 'r';
const byte WRITE          = 'w';
const byte READ_OPTICAL   = 'o';
const byte READ_EE        = 'e';
const byte CONFIG_PIN     = 'c';

const byte RSP_OK         = '@';
const byte RSP_ERR        = '#';

const byte ERR_CMD_TOO_LONG     = '1';
const byte ERR_NO_CMD_START     = '2';
const byte ERR_CMD_UNKNOWN      = '3';
const byte ERR_CMD_INVALID_ARGS = '4';
const byte ERR_BAD_ARG          = '5';

const byte EOC = '\r';
const char EOR[] = "\n\r";

byte buf[MAX_CMD_LEN+1];
int buflen;
int bufpos;

boolean is_initialized;
boolean is_led_on;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(250);

  is_initialized = false;
  is_led_on = false;
  
  buflen = 0;
  bufpos = 0;

  pinMode(PIN_5,  OUTPUT);
  pinMode(PIN_6,  OUTPUT);
  pinMode(PIN_7,  OUTPUT);
  pinMode(PIN_8,  INPUT);
  pinMode(PIN_9,  INPUT);
  pinMode(PIN_10, INPUT);
  pinMode(PIN_11, INPUT);
  pinMode(PIN_12, INPUT);
  pinMode(PIN_13, INPUT);
}

void loop() {
  byte c;
  
  // Flash LED until version number is queried indicating arm OS is booted & running
  if (!is_initialized) {
    is_led_on = !is_led_on;
    is_led_on? digitalWrite(PIN_5, HIGH) : digitalWrite(PIN_5, LOW);
    delay(333);
  }
  
  if(!Serial.available()) {
    return;
  }

  if ((c=Serial.read()) == EOC)
  {
    buf[buflen] = 0;
    doCmd();
    buflen = 0;
  }
  else if (buflen >= MAX_CMD_LEN)
  {
    buflen = 0;
    sendErr(ERR_CMD_TOO_LONG);
  }
  else if (c == '!')
  {
    buflen = 1;
    buf[0] = c;
  }
  else
  {
    buf[buflen++] = c;
  }
}  

void sendErr(byte ecode) {
  Serial.write(RSP_ERR);
  Serial.write(ecode);
  Serial.write(EOR);
} 

void sendOk() {
  Serial.write(RSP_OK);
  Serial.write(EOR);
} 

void sendRsp(byte v) {
  Serial.write(RSP_OK);
  Serial.write(v);
  Serial.write(EOR);
}

void sendRsp(char v[]) {
  Serial.write(RSP_OK);
  Serial.write(v);
  Serial.write(EOR);
}

void sendRsp(int v) {
  Serial.write(RSP_OK);
  Serial.print(v);
  Serial.write(EOR);
}

void sendRspHex(int v) {
  Serial.write(RSP_OK);
  if(v < 16) {
    Serial.write('0');
  }
  Serial.print(v, HEX);
  Serial.write(EOR);
}

int parseInt() {
  int val = 0;
  int rc = -2;

  if (bufpos > buflen) {
    return -1;
  }

  char c;
  while(bufpos < buflen) {
    c = buf[bufpos];
    if (c >= '0' && c <= '9') {
      rc = 0;
      val *= 10;
      val += c-'0';
      bufpos++;
    }
    else {
      break;
    }
  }
  if (rc == 0) {
    return val;
  } 
  else {
    return rc;
  }
}

void doCmd()
{
  // ignore newlines and carriage returns
  if (buflen==0 || buf[0] == '\r' || buf[0] == '\n')
  {
    Serial.write(EOR);
    return;
  }

  // all commands start with a bang ('!')
  else if (buf[0] != CMD_START)
  {
    sendErr(ERR_NO_CMD_START);
    return;
  }

  int pin_id = 0;
  int val = 0;
  byte config='u'; // unspecified

  switch (buf[1]) {
  case READ_VERSION:
    sendRsp(read_version());
    break;

  case READ:
    bufpos = 3;
    pin_id = parseInt();
    if(pin_id == -1){
      sendErr(ERR_CMD_INVALID_ARGS);
    }
    else if(pin_id == -2){
      sendErr(ERR_BAD_ARG);
    }     
    else if(pin_id < 0 || pin_id > 13) {
      sendErr(ERR_BAD_ARG);
    }
    else {
      sendRsp(read_pin(pin_id));
    }
    break;

  case WRITE:
    bufpos = 3;
    pin_id = parseInt();
    if(pin_id == -1){
      sendErr(ERR_CMD_INVALID_ARGS);
      return;
    }
    else if(pin_id == -2){
      sendErr(ERR_BAD_ARG);
      return;
    }     
    else if(pin_id < 2 || pin_id > 7) {
      sendErr(ERR_BAD_ARG);
      return;
    }

    bufpos++;
    val = parseInt();
    if(val == 1 || val == 0) {
      write_pin(pin_id, val);
      sendOk();
    }
    else {
      sendErr(ERR_CMD_INVALID_ARGS);
    }
    break;

  case READ_OPTICAL:
    val = read_optical();
    sendRspHex(val);
    break;

  case READ_EE:
    val = read_ee();
    sendRspHex(val);
    break;

  case CONFIG_PIN:
    bufpos = 3;
    pin_id = parseInt();
    if(pin_id == -1){
      sendErr(ERR_CMD_INVALID_ARGS);
      return;
    }
    else if(pin_id == -2){
      sendErr(ERR_BAD_ARG);
      return;
    }     
    else if(pin_id < 2 || pin_id > 7 || pin_id == 5) {
      sendErr(ERR_BAD_ARG);
      return;
    }

    bufpos++;
    val = parseInt();
    if(val == 1 || val == 0) {
      config_pin(pin_id, val);
      sendOk();
    }
    else {
      sendErr(ERR_CMD_INVALID_ARGS);
    }
    break;

    // invalid command
  default:
    sendErr(ERR_CMD_UNKNOWN);
  }
}


byte read_pin(int id)
{
  if(digitalRead(id) == 0) { 
    return '0';
  }

  else {
    return '1';
  }
}

void write_pin(int id, int val)
{ 
  digitalWrite(id, val);
}

unsigned int read_optical() {
  unsigned int p8, p9, p10 , p11, p12, p13;
  p8 = digitalRead(PIN_8);
  p9 = digitalRead(PIN_9) << 1;
  p10 = digitalRead(PIN_10) << 2;
  p11 = digitalRead(PIN_11) << 3;
  p12 = digitalRead(PIN_12) << 4;
  p13 = digitalRead(PIN_13) << 5;

  return (p8 | p9 | p10 | p11 | p12 | p13);
}

unsigned int read_ee() {
  return (digitalRead(PIN_6) << 1) | (digitalRead(PIN_7)); 
}

int config_pin(int id, byte c) {
  switch (c) {
  case 0:
    pinMode(id, INPUT);
    break;
  case 1: 
    pinMode(id, OUTPUT);
    break;
  default:
    return -1;
  }

  return 0;
}

int read_version(){
  // Stop the flashing booting LED
  is_initialized = true;
  digitalWrite(PIN_5, LOW);
  is_led_on = false;
  
  // return version number
  return 1;
}

