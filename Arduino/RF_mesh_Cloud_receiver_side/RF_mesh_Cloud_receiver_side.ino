#include <FastLED.h>
#include <SoftwareSerial.h>


/*****************
 * Configuration *
 *****************/

// fastLED
#define LED_STRIP_PIN 3
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define NUM_LEDS 300
#define BRIGHTNESS 255
#define FRAMES_PER_SECOND 30

// HC-12 configuration for JUMP
#define HC12_TX_TO_ARDUINO_RX_PIN 10
#define HC12_RX_FROM_ARDUINO_TX_PIN 11
// HC-12 configuration for Ross'd development board
//#define HC12_TX_TO_ARDUINO_RX_PIN 2
//#define HC12_RX_FROM_ARDUINO_TX_PIN 4


/***********
 * Globals *
 ***********/

static CRGB leds[NUM_LEDS];

static uint16_t dist;             // A random number for our noise generator.
static uint16_t scale = 30;       // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
static uint8_t maxChanges = 48;   // Value for blending between palettes.

// variables for aniamtion
static int waveA = 10;
static int waveB = 1;
static int waveC = 4;

static CRGBPalette16 currentPalette(RainbowColors_p);
static CRGBPalette16 targetPalette(PartyColors_p);

static SoftwareSerial HC12(HC12_TX_TO_ARDUINO_RX_PIN, HC12_RX_FROM_ARDUINO_TX_PIN);


/************
 * Patterns *
 ************/

// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.

// ---------- Noise Generator1 ----------
void fillnoise8()
{
  // Just ONE loop to fill up the LED array as all of the pixels change.
  for (int i = 0; i < NUM_LEDS; i++) {
    // Get a value from the noise function. I'm using both x and y axis.
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;
    // With that value, look up the 8 bit colour palette value and assign it to the current LED.
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);
  }
  // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
  dist += beatsin8(waveA, waveB, waveC);

  // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
}


void pattern0_off()
{
  // sine variables
  waveA = 12;   // high number more speratic 0-150 default 10
  waveB = .5;   // fast pulse a high number 0-5     default 1
  waveC = 2;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(CRGB::Black); }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern1_walkIn()
{
  // sine variables
  waveA = 12;   // high number more speratic 0-150 default 10
  waveB = .5;   // fast pulse a high number 0-5     default 1
  waveC = 2;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(RainbowColors_p);
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern2_ocean()
{
  // sine variables
  waveA = 25;   // high number more speratic 0-150 default 10
  waveB = .3;   // fast pulse a high number 0-5     default 1
  waveC = 1;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(OceanColors_p);
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern3_rainbowStripe()
{
  // sine variables
  waveA = 20;   // high number more speratic 0-150 default 10
  waveB = 1;    // fast pulse a high number 0-5     default 1
  waveC = 2;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(RainbowStripeColors_p);
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern4_party()
{
  // sine variables
  waveA = 26;   // high number more speratic 0-150 default 10
  waveB = 3.5;  // fast pulse a high number 0-5     default 1
  waveC = 16;   // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                          // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(PartyColors_p);   // max of 4 colors
 }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern5_cloud()
{
  // sine variables
  waveA = 2;    // high number more speratic 0-150 default 10
  waveB = .1;   // fast pulse a high number 0-5     default 1
  waveC = 2;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(CloudColors_p);   // max of 4 colors
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


void pattern6_lava()
{
  // sine variables
  waveA = 3;    // high number more speratic 0-150 default 10
  waveB = .3;   // fast pulse a high number 0-5     default 1
  waveC = 1;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(LavaColors_p); //max of 4 colors
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.
}


bool pattern15_startup()
{
  static unsigned int dot = 0;

  EVERY_N_MILLISECONDS(30) {
    leds[dot] = CRGB::Blue;
    FastLED.show();
    // clear this led for the next time around the loop
    leds[dot] = CRGB::Black;
    ++dot;
  }

  if (dot < NUM_LEDS) {
    return true;          // keep running
  }
  else {
    return false;         // switch to another pattern
  }
}


/**********************
 * Setup and Run Loop *
 **********************/

void setup()
{
  delay(3000);              // sanity delay

  Serial.begin(9600);       // Open serial port to computer
  HC12.begin(9600);         // Open serial port to HC12

  //fastled setup
  FastLED.addLeds<CHIPSET, LED_STRIP_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  dist = random16(12345);   // A semi-random number for our noise generator
}


void loop()
{
  static int Active;
  static int State = 15;    // the current pattern being displayed (or 86 if none)
  static String rxBuf;      // holds received data until we have a complete message

  // variables for activating if loop for parsing data
  const byte startMarker = '^';
  const byte endMarker = '%';
  boolean startBTdata = false;          // gets set true when we receive a start marker
  String BTdata = "";                   // if not empty, contains a complete message
  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {            // If HC-12 has data
    byte incomingByte = HC12.read();    // get each incoming byte from HC-12
    // Saves the data between the start and end markers.
    if (startBTdata == true) {          // We have received a start marker?
      if (incomingByte != endMarker) {
        rxBuf += char(incomingByte);    // Add the byte to the receive buffer.
      }
      else {                            // We've reached the end marker.
        startBTdata = false;            // Don't save any more characters.
        BTdata = rxBuf;
        rxBuf = "";
      }
    }
    // Checks whether the received message starts with the start marker.
    else if (incomingByte == startMarker) {
      startBTdata = true;               // start saving the message data
    }
  }

  // Bluetooth Comm -------------------------------------
  // convert
  // current = BTdata.toInt();

  if      (BTdata == "0") Active = 0;
  else if (BTdata == "1") Active = 1;
  else if (BTdata == "2") Active = 2;
  else if (BTdata == "3") Active = 3;
  else if (BTdata == "4") Active = 4;
  else if (BTdata == "5") Active = 5;
  else if (BTdata == "6") Active = 6;
  else if (BTdata == "")  Active = 86;

  // We're finished with the current message, so clear it.
  BTdata = "";

  // Change the state (pattern) if the message told us to do so.
  if (Active < 80) {
    State = Active;
  }

  Serial.print("BTdata=");
  Serial.print(BTdata);
  Serial.print(" Active=");
  Serial.print(Active);
  Serial.print(" State=");
  Serial.println(State);

  switch (State) {
    case 0:
      pattern0_off();
      break;
    case 1:
      pattern1_walkIn();
      break;
    case 2:
      pattern2_ocean();
      break;
    case 3:
      pattern3_rainbowStripe();
      break;
    case 4:
      pattern4_party();
      break;
    case 5:
      pattern5_cloud();
      break;
    case 6:
      pattern6_lava();
      break;
    case 15:
      // When pattern 15 is finished, switch to pattern 5.
      if (!pattern15_startup()) {
        State = 5;
      }
      break;
    //default:
      // TODO ross 9 Feb 2018:  We should indicate an internal error somehow if State is invalid.
  }
}

