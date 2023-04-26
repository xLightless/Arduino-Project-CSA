#include <Arduino.h>

/*

  Arduino Project

  Use a photoresistor to transmit data to another arduino and vise versa.

*/

// Define values of input pins
#define BUTTON_PIN 8
#define TILT_PIN 9
#define POT_PIN A1












int ledState = LOW;             // Set the state of the LED

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
  
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Set any other explicit pins
  pinMode(BUTTON_PIN, OUTPUT);















}

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

  txButton = digitalRead(BUTTON_PIN);
  txTilt = 1;
  txPot = 7;
  txA = 99;
  txB = 99;
  txC = 99;
  txD = 99;
}

void writeOutputs()
{
  // Outputs breadboard projects to serial
  // Uses rxButton, rxTilt, rxPot, rxA, rxB, rxC, rxD;

  Serial.println(rxButton, DEC);
  Serial.println(rxTilt, DEC);
  Serial.println(rxPot, DEC);
  Serial.println(rxA, DEC);
  Serial.println(rxB, DEC);
  Serial.println(rxC, DEC);
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
            Serial.println("0x71");
            rxButton = rxBuffer[0];
            rxTilt = rxBuffer[1];
            rxPot = rxBuffer[2];
            rxA = rxBuffer[3];
            rxB = rxBuffer[4];
            rxC = rxBuffer[5];
            rxD = rxBuffer[6];
            writeOutputs();
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



// the loop routine runs over and over again forever:
void loop()
{
  txChar();
  rxChar();

  // Execute mini project code below
  // If communication message has the correct range values from 0 to 99 then actuate

}
