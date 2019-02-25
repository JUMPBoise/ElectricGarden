#include <RBD_Button.h>
#include <SoftwareSerial.h>

//#define ROSS_DEVL

//#define HAS_BENDER_KNOBS


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

#ifdef HAS_BENDER_KNOBS

#define POT1_APIN A0
#define POT2_APIN A1
#define POT3_APIN A2

#define BENDER_BUTTON_PIN 9
#define BENDER_LED_PIN LED_BUILTIN

#define BENDER_LED_ON HIGH
#define BENDER_LED_OFF LOW

// benderRetransmitIntervalMs controls how often we will transmit the bender knob values.
static const uint32_t benderRetransmitIntervalMs = 667;

#endif


/***********
 * Globals *
 ***********/

static SoftwareSerial HC12(HC12_TX_TO_ARDUINO_RX_PIN, HC12_RX_FROM_ARDUINO_TX_PIN);

#ifdef HAS_BENDER_KNOBS
static RBD::Button benderButton(BENDER_BUTTON_PIN, true);  // true -> enable internal pullup
static bool benderIsEnabled;
#endif

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

#ifdef HAS_BENDER_KNOBS
  pinMode(POT1_APIN, INPUT);
  pinMode(POT2_APIN, INPUT);
  pinMode(POT3_APIN, INPUT);
  pinMode(BENDER_LED_PIN, OUTPUT);
  digitalWrite(BENDER_LED_PIN, BENDER_LED_OFF);
#endif
}


#ifdef HAS_BENDER_KNOBS
void readAndSendBenderValues(uint32_t now)
{
  static uint32_t lastBenderTxMs;

  if (benderButton.onPressed()) {
    benderIsEnabled = !benderIsEnabled;
    digitalWrite(BENDER_LED_PIN, benderIsEnabled ? BENDER_LED_ON : BENDER_LED_OFF);
    // Transmit bender message immediately after bender is enabled or disabled.
    lastBenderTxMs = 0L;
  }

  if (now - lastBenderTxMs >= benderRetransmitIntervalMs) {
    lastBenderTxMs = now;

    byte bender1;
    byte bender2;
    byte bender3;
    if (benderIsEnabled) {
      // We right shift the ADC values by 3 bits to essentially divide them by 8, thus
      // giving a range of 0 to 127 (instead of 0 - 1023).  We have to do this because
      // we can use only 7 bits for each bender value.
      bender1 = analogRead(POT1_APIN) >> 3;
      bender2 = analogRead(POT2_APIN) >> 3;
      bender3 = analogRead(POT3_APIN) >> 3;
    }
    else {
      // 64 means don't bend a value.
      bender1 = bender2 = bender3 = 64;
    }

    // Turn on the high-order bit of each bender value so that they
    // don't get misinterpreted as framing (start or stop) characters.
    bender1 |= 0x80;
    bender2 |= 0x80;
    bender3 |= 0x80;

    HC12.write(0x26);       // & - bender message start marker
    HC12.write(bender1);
    HC12.write(bender2);
    HC12.write(bender3);
    HC12.write(0x25);       // % - message end marker

    Serial.print("bender1=");
    Serial.print((int) bender1);
    Serial.print(" bender2=");
    Serial.print((int) bender2);
    Serial.print(" bender3=");
    Serial.println((int) bender3);
  }
}
#endif


void loop()
{
  static uint32_t lastTxMs;

  uint32_t now = millis();

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

  if (state >= 0 && state <= 8 && now - lastTxMs >= retransmitIntervalMs) {     // state is valid and should be (re)sent?
    lastTxMs = now;

    HC12.print("^");
    HC12.print(state);
    HC12.print("%");

    Serial.println(state);
  }

#ifdef HAS_BENDER_KNOBS
  readAndSendBenderValues(now);
#endif
}

