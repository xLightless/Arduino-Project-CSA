#include <Arduino.h>

#include <LiquidCrystal.h>

#include <dht.h>

/*

  Arduino Project

  Use a photoresistor to transmit data to another arduino and vise versa.

  Communication is 9 bytes long so where 1 byte = 8 bits, 9 * 8 = 72 / 5 = 14400 seconds per message.

*/

// Define values of input pins
#define BUTTON_PIN 8
#define TILT_PIN 9
#define POT_PIN A1
#define JOYSTICK_SW_PIN 4
#define JOYSTICK_PIN_Y A2
#define FAN_MOTOR_PIN A5

// One digit 7 segment display
#define DISPLAY_A 8
#define DISPLAY_B 9
#define DISPLAY_C 10
#define DISPLAY_D 11
#define DISPLAY_E 12
#define DISPLAY_F 13
//#define DISPLAY_G 13

// Temperature and Humidity Sensor
dht DHT;
#define DHT_PIN 6

// Ultrasonic Sensor
#define ECHO_PIN 5
#define TRIG_PIN 6

// LCD display
//int RS = 4;
//int E = 2;
//int D4 = A5;
//int D5 = A4;
//int D6 = A3;
//int D7 = A2;
LiquidCrystal lcd(4, 2, A5, A4, A3, A2);

float analogCoeff = 100 * 8;      // x / 100 * 8 provides a range between 0..99.
long distance, duration;



int ledState = LOW;               // Set the state of the LED

char encrypt(char in_char)
{
  char out_char;
  
  out_char = in_char;
  
  return out_char;
}

char decrypt(char in_char)
{
  char out_char;
  
  out_char = in_char;
  
  return out_char;
}

void setup()
{
  // Set communication LED PIN to OUTPUT
  pinMode(3, OUTPUT);
  
  // Set any other explicit pins to read/write data
  pinMode(POT_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(FAN_MOTOR_PIN, OUTPUT);
  
  pinMode(DISPLAY_A, OUTPUT);
  pinMode(DISPLAY_B, OUTPUT);
  pinMode(DISPLAY_C, OUTPUT);
  pinMode(DISPLAY_D, OUTPUT);
  pinMode(DISPLAY_E, OUTPUT);
  pinMode(DISPLAY_F, OUTPUT);
//  pinMode(DISPLAY_G, OUTPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // Initialize LCD Display
  lcd.begin(16, 2);

  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

}


/*

  COMMUNICATION INPUTS

*/



int readPotInput() {
  // Handles potentiometer readings
  int potSensorValue = analogRead(POT_PIN);
  
  if (potSensorValue <= 511) {
    // Read Potentiometer as OFF state
    potSensorValue = 0;
    return potSensorValue;
  }

  // Read Potentiometer as HIGH state
  potSensorValue = 7;
  return potSensorValue;
}

int isSwitchPressed; // Store value of the pressed joystick

int readJoystickSwitchInput() {
  // Store actuated values of the joystick

  int switchY;
  
  // Vertical motion of joystick. Central location is 1023/2. LEFT = 0, RIGHT = 1023
  switchY = analogRead(JOYSTICK_PIN_Y);

  // If joystick is pressed down
  isSwitchPressed = 0;
  int switchValue = digitalRead(JOYSTICK_SW_PIN);
  if (switchValue == isSwitchPressed) {
    return 1;
  }

  // Return state as off
  return 0;
}

float readJoystickPitchInput() {
    // Read the joystick pitch value range

    float pitchValue = analogRead(JOYSTICK_PIN_Y);
    return (pitchValue / 100) * 8;
}

int readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration / 58.2;

  if ((distance > 0) && (distance < 100)) {
    return distance;
  }

  return 0;
}

long getDistance() {
  // Get the distance between the sensor and object

  long distance = readUltrasonicSensor();
  return distance;
}

int getLCDContrast() {
  // Get contrast of LCD display
  
  int sensorValue = analogRead(POT_PIN);
  int contrastPercent = map(sensorValue, 0, 1023, 101, 0); // return screen contrast as a percentage

  if ((contrastPercent > 0) && (contrastPercent < 100)) {
    return contrastPercent;
  }

  return 0;
}

//void readLCDInputs() {
//  int sensorValue = analogRead(POT_PIN);
//  int contrastPercent = map(sensorValue, 0, 1023, 101, 0); // return screen contrast as a percentage
//  long distance = readUltrasonicSensor();
//
//  return :
//}




/*

  COMMUNICATION TRANSMITTER AND RECIEVER

*/

const long txInterval = 200;              // interval at which to tx bit (milliseconds)
int tx_state = 0;                         // The state at which the transmission is occuring. Range is between 0 and 20.
// char tx_char;
char chr;                                 // TX CHAR encrypted value
unsigned long previousTxMillis = 0;        // will store last time LED was updated

#define STX 0x70 // 112 in binary. Starting bit not included
#define ETX 0x71 // 113 in binary.

char txButton, txTilt, txPot, txA, txB, txC, txD;
char txBuffer[8] = {0,0,0,0,0,0,0, ETX};
char rxBuffer[7];
char rxButton, rxTilt, rxPot, rxA, rxB, rxC, rxD;
int  rx_index;

void readInputs()
{
  // Gets project inputs
  // Uses txButton, txTilt, txPot, txA, txB, txC, txD

  txButton    = 0;     // Read switch value from joystick
  txTilt      = 0;
  txPot       = readPotInput();
  txA         = getDistance();      // Read distance from the ultasonic sensor
  txB         = getLCDContrast();
  txC         = 0;
  txD         = 0;
}

char rxButtonState, rxTiltState, rxPotState, rxAState, rxBState, rxCState, rxDState; // Store previously recieved values into these variables to continue actuation until the next message


void toggleFanOutput() {
  // Keep the fan in the previous state until the next recieved instruction.

  // Adjust fan speed based on joystick Y
  Serial.println(rxAState,DEC);

  if (rxAState > 50) {
    analogWrite(FAN_MOTOR_PIN, 1023);

  }
}

void writeDisplayValues() {

  // Set to 7 on the one-digit-display
  if (rxPotState == 7) {
    digitalWrite(DISPLAY_A, HIGH);
    digitalWrite(DISPLAY_B, HIGH);
    digitalWrite(DISPLAY_C, HIGH);
    digitalWrite(DISPLAY_D, LOW);
    digitalWrite(DISPLAY_E, LOW);
    digitalWrite(DISPLAY_F, LOW);
//    digitalWrite(DISPLAY_G, LOW);
  }
  // Set to 0 on the one-digit-display
  else if (rxPotState == 0) {
    digitalWrite(DISPLAY_A, HIGH);
    digitalWrite(DISPLAY_B, HIGH);
    digitalWrite(DISPLAY_C, HIGH);
    digitalWrite(DISPLAY_D, HIGH);
    digitalWrite(DISPLAY_E, HIGH);
    digitalWrite(DISPLAY_F, HIGH);
//    digitalWrite(DISPLAY_G, LOW);
  }
}

void writeLCDValues() {
  lcd.setCursor(0,0);
  lcd.print("Distance CM: ");
  lcd.print(rxA, DEC);
  
  lcd.setCursor(0,1);
  lcd.print("Contrast: ");
  lcd.print(rxB, DEC);
  
}

void writeOutputs()
{
  // Outputs breadboard projects to serial
  // Uses rxButton, rxTilt, rxPot, rxA, rxB, rxC, rxD;

  // Store RX values into intermedium states
  rxButtonState = rxButton;
  rxTiltState = rxTilt;
  rxPotState = rxPot;
  rxAState = rxA;
  rxBState = rxB;
  rxCState = rxC;
  rxDState = rxD;

  writeDisplayValues();
  writeLCDValues();
}

void printMessage() {
  Serial.print("Button Value = ");
  Serial.println(rxButton, DEC);
  
  Serial.print("Tilt Value = ");
  Serial.println(rxTilt, DEC);

  Serial.print("Potentiometer Value = ");
  Serial.println(rxPot, DEC);

  Serial.print("A Value = ");
  Serial.println(rxA, DEC);

  Serial.print("B Value = ");
  Serial.println(rxB, DEC);

  Serial.print("C Value = ");
  Serial.println(rxC, DEC);

  Serial.print("D Value = ");
  Serial.println(rxD, DEC);
}











#define TX_START_OF_TEXT -1
int tx_string_state = TX_START_OF_TEXT;

char getTxChar()
{
  char chr;

  switch (tx_string_state)
  {
    case TX_START_OF_TEXT:
    tx_string_state = 0;
    readInputs();             // Set tx inputs before putting into buffer and sending.
    txBuffer[0] = txButton;
    txBuffer[1] = txTilt;
    txBuffer[2] = txPot;
    txBuffer[3] = txA;
    txBuffer[4] = txB;
    txBuffer[5] = txC;
    txBuffer[6] = txD;
    return STX;
    break;
    
    default:
    chr = txBuffer[tx_string_state];
    // Serial.println(chr, DEC);
    tx_string_state++;
    
    if (chr == ETX)  /* End of string? */
    {
      tx_string_state = TX_START_OF_TEXT;  /* Update the tx string state to start sending the string again */
      return ETX;  /* Send End of Text character */
    }
    else
    {
      return chr;  /* Send a character in the string */
    }
    break;
  }
}


void txChar()
{
  unsigned long currentTxMillis = millis();

  if (currentTxMillis - previousTxMillis >= txInterval)
  {
    // save the last time you blinked the LED (improved)
    previousTxMillis = previousTxMillis + txInterval;  // this version catches up with itself if a delay was introduced
    switch (tx_state)
    {
      case 0:
        chr = encrypt(getTxChar());

        // chr values: 1 2 4 8 16 32 64 (-128) during transmission. Each number will left shift to the next in the range.

        digitalWrite(3, HIGH);  /* Transmit Start bit */
        tx_state++;
        break;

      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
        if ((chr & 0x40) != 0)   /* Transmit each bit in turn */
        {
          digitalWrite(3, HIGH);
        }
        else
        {
          digitalWrite(3, LOW);
        }
        chr = chr << 1;  /* Shift left to present the next bit */
        tx_state++;
        break;

      case 8:
        digitalWrite(3, HIGH);  /* Transmit Stop bit */
        tx_state++;
        break;

      default:
        digitalWrite(3, LOW);
        tx_state++;
        readInputs();
        if (tx_state > 20) tx_state = 0;  /* Start resending the character */
        break;
    }
  }
}



const long rxInterval = 20;              // interval at which to read bit (milliseconds)
int rx_state = 0;                        // RX STATE iterates from 0 to 100 to send the whole message. Each bit is 100/rx_bits.length. RX state means that the first 10 and the last 100 are start and end states.
char rx_char;
unsigned long previousRxMillis = 0;      // Will store last time LED was updated
int rx_bits[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


void rxChar()
{
  unsigned long currentRxMillis = millis();
  int sensorValue;
  int i;

  if (currentRxMillis - previousRxMillis >= rxInterval)
  {
    // Save the last time you read the analogue input
    previousRxMillis = previousRxMillis + rxInterval;  // this version catches up with itself if a delay was introduced

    sensorValue = analogRead(A0);

    switch (rx_state)
    {
      case 0:
        if (sensorValue >= 900)
        {
          rx_bits[0]++;
          rx_state++;
        }
        break;

      case 100:
        if ((rx_bits[0] >= 6) && (rx_bits[8] >= 6))  /* Valid start and stop bits */
        {
          rx_char = 0;

          for (i = 1; i < 8; i++)
          {
            rx_char = rx_char << 1;
            if (rx_bits[i] >= 6) rx_char = rx_char | 0x01;
          }
          rx_char = decrypt(rx_char);
          switch (rx_char)
          {
            case STX:
            Serial.println("0x70");
            rx_index = 0;
            break;
            
            case ETX:
            rxButton = rxBuffer[0];
            rxTilt = rxBuffer[1];
            rxPot = rxBuffer[2];
            rxA = rxBuffer[3];
            rxB = rxBuffer[4];
            rxC = rxBuffer[5];
            rxD = rxBuffer[6];
            printMessage();
            Serial.println("0x71");
            rx_index = 0;
            break;
            
            default:
            rxBuffer[rx_index] = rx_char;
            rx_index++;
            break;
          }
        }
        else
        {
          Serial.println("Rx error");

          // Prevent previous states from updating in case of an error.
          rxButton = txButton;
          rxTilt = txTilt;
          rxPot = txPot;
          rxA = txA;
          rxB = txB;
          rxC = txC;
          rxD = txD;


          
        }
        for (i = 0; i < 10; i++)  /* Print the recieved bit on the monitor - debug purposes */
        {
          // Serial.println(rx_bits[i]);
        }
        for (i = 0; i < 10; i++)
        {
          rx_bits[i] = 0;
        }
        rx_state = 0;
        break;

      default:
        if (sensorValue >= 900)
        {
          rx_bits[rx_state / 10]++;
        }
        rx_state++;
        break;
    }
  }

}

// Loop routine for handling arduino controls
void loop()
{  
  // Transmit the data to another arduino and receive the other arduinos transmit message.
  txChar();
  rxChar();
  writeOutputs();
}