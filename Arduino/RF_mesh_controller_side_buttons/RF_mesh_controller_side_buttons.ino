#include <SoftwareSerial.h>

#define ROSS_DEVL


/*****************
 * Configuration *
 *****************/

// HC-12 configuration for JUMP
#define HC12_TX_TO_ARDUINO_RX_PIN 10
#define HC12_RX_FROM_ARDUINO_TX_PIN 11

#ifndef ROSS_DEVL

// ----- button configuration for JUMP -----

static const int buttonPinA = A1;
static const int buttonPinB = A2;
static const int buttonPinC = A3;
static const int buttonPinD = A4;
static const int buttonPinE = A5;

// Use this when the buttons are active high.  Active high means that the pin is
// put in a high state when the button is pushed.  That is the case when the
// button's switch is the normally open type, one side of the switch is connected
// to the Arduino's pin, and the other side is connected to Vcc (5 V or 3.3. V).
#define BUTTON_PUSHED HIGH;

#else

// ----- button configuration for Ross's development board -----

static const int buttonPinA = 2;
static const int buttonPinB = 3;
static const int buttonPinC = 4;
static const int buttonPinD = 5;
static const int buttonPinE = 6;

// Use this when the buttons are active low.  Active low means that the pin is
// put in a low state when the button is pushed.  That is the case when the
// button's switch is the normally open type, one side of the switch is connected
// to the Arduino's pin, and the other side is connected to ground.  An advantage
// of active low is that the Arduino's internal pullup can be used to pull the
// pin high when the button is not pushed, eliminating the need for an external
// pullup resistor (for active low) or pulldown resistor (for active high).
#define BUTTON_PUSHED LOW;

// Enable this #define if we need the internal pullups.
#define ENABLE_INTERNAL_PULLUPS

#endif

// retransmitIntervalMs limits how often we will transmit a state value.
static const uint32_t retransmitIntervalMs = 500;


/***********
 * Globals *
 ***********/

static SoftwareSerial HC12(HC12_TX_TO_ARDUINO_RX_PIN, HC12_RX_FROM_ARDUINO_TX_PIN);


/**********************
 * Setup and Run Loop *
 **********************/

void setup()
{
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12

  // Configure the button pins as inputs.
  pinMode(buttonPinA, INPUT);
  pinMode(buttonPinB, INPUT);
  pinMode(buttonPinC, INPUT);
  pinMode(buttonPinD, INPUT);
  pinMode(buttonPinE, INPUT);

#ifdef ENABLE_INTERNAL_PULLUPS
  digitalWrite(buttonPinA, HIGH);
  digitalWrite(buttonPinB, HIGH);
  digitalWrite(buttonPinC, HIGH);
  digitalWrite(buttonPinD, HIGH);
  digitalWrite(buttonPinE, HIGH);
#endif
}


void loop()
{
  static uint32_t lastTxMs;

  // read buttons
  // TODO ross 10 Feb. 2019:  we should do debouncing so that we don't sporadically transmit the wrong state values
  int Index = digitalRead(buttonPinE) == BUTTON_PUSHED;
  int Middle = digitalRead(buttonPinD) == BUTTON_PUSHED;
  int Ring = digitalRead(buttonPinC) == BUTTON_PUSHED;
  int Pinky = digitalRead(buttonPinB) == BUTTON_PUSHED;
  int Thumb = digitalRead(buttonPinA) == BUTTON_PUSHED;

  /*
  Serial.println(Index);
  Serial.println(Middle);
  Serial.println(Ring);
  Serial.println(Pinky);
  Serial.println(Thumb);
  Serial.println();
  */

  //button scramble
  int state;        // this is the value representing a pattern number that we send to the receiver side
       if (Thumb == 1 && Index == 1 && Middle == 0 && Ring == 0 && Pinky == 0) state = 1;
  else if (Thumb == 1 && Index == 0 && Middle == 1 && Ring == 0 && Pinky == 0) state = 2;
  else if (Thumb == 1 && Index == 0 && Middle == 0 && Ring == 1 && Pinky == 0) state = 3;
  else if (Thumb == 1 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 1) state = 4;
  else if (Thumb == 0 && Index == 1 && Middle == 0 && Ring == 0 && Pinky == 0) state = 5;
  else if (Thumb == 0 && Index == 0 && Middle == 1 && Ring == 0 && Pinky == 0) state = 6;
  else if (Thumb == 0 && Index == 0 && Middle == 0 && Ring == 1 && Pinky == 0) state = 7;
  else if (Thumb == 0 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 1) state = 8;
  else if (              Index == 1 && Middle == 1 && Ring == 1 && Pinky == 1) state = 0;   // turn pattern off
  else                                                                         state = 86;  // send no signal

  uint32_t now = millis();
  if (state >= 0 && state <= 8 && now - lastTxMs >= retransmitIntervalMs) {     // state is valid and should be (re)sent?
    lastTxMs = now;

    HC12.print("^"); 
    HC12.print(state); 
    HC12.print("%");

    Serial.println(state);
  }
}

