
//fastLED 
#include <FastLED.h>
#define LED_PIN     3
#define COLOR_ORDER GRB
#define CHIPSET     WS2812B
#define NUM_LEDS    300
#define BRIGHTNESS  200
#define FRAMES_PER_SECOND 30
bool gReverseDirection = false;
CRGB leds[NUM_LEDS];

static uint16_t dist;         // A random number for our noise generator.
uint16_t scale = 30;          // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
uint8_t maxChanges = 48;      // Value for blending between palettes.

//variables for aniamtion
int waveA = 10;
int waveB = 1;
int waveC = 4;


CRGBPalette16 currentPalette(RainbowColors_p);
CRGBPalette16 targetPalette(PartyColors_p);


//HC12
#include <SoftwareSerial.h>
SoftwareSerial HC12(10, 11);         // HC-12 TX Pin, HC-12 RX Pin
byte incomingByte;
String BTdata = "";

int lead_dot = 150;
int counter;
int Active;
int State=1;
long randomNumber;
int randomNumberB = 0;
int randomNumberT = 150;
int distruptor=0;

//for cylcing the code without the controller
long timer = 60000;
long timekeeper = 0;

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void setup() {

 Serial.begin(9600);                   // Open serial port to computer
 HC12.begin(9600);                     // Open serial port to HC12


 
//fastled setup
  delay(3000); // sanity delay
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );
  dist = random16(12345);          // A semi-random number for our noise generator


}

//-------------------------------------------------Noise Generator1
void fillnoiseB() {
  for(int i = 0; i < 150; i++) {                                     // Just ONE loop to fill up the LED array as all of the pixels change.
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;                  // Get a value from the noise function. I'm using both x and y axis.
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);   // With that value, look up the 8 bit colour palette value and assign it to the current LED.
  }
  dist += beatsin8(waveA,waveB, waveC);                                               // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
                                                                           // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
} // fillnoise8()
//-------------------------------------------------Noise Generator1

//-------------------------------------------------Noise Generator2
void fillnoiseT() {
  for(int i = 150; i < 300; i++) {                                     // Just ONE loop to fill up the LED array as all of the pixels change.
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;                  // Get a value from the noise function. I'm using both x and y axis.
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);   // With that value, look up the 8 bit colour palette value and assign it to the current LED.
  }
  dist += beatsin8(waveA,waveB, waveC);                                               // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
                                                                           // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
} // fillnoise8()
//-------------------------------------------------Noise Generator2

void loop() {

//for cylcing the code without the controller
//unsigned long timerMillis = millis();
//if (timerMillis-timekeeper>timer){State=State+1; timekeeper=timerMillis;
//    if (State == 8) {State =0;}
//}


 //vairables for activating if loop for parsing data
  boolean startBTdata = false;

  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
                // Reads the data between the start "x" and end marker "y" ---------------- BTData
    if (startBTdata == true) {
      if (incomingByte != '%') {
        BTdata += char(incomingByte);    // Add each byte to ReadBuffer string variable
           }
      else {
        startBTdata = false;
      }
    }
    // Checks whether the received message statrs with the start marker "x"
    else if ( incomingByte == '^') {
      startBTdata = true; // If true start reading the message
      } // -------------------------------------------------------------------------BTData
  }
  //delay(200);
  
 

//Bluetooth Comm -------------------------------------
//convert
//current = BTdata.toInt();

if (BTdata == "1"){Active = 1;}
else if (BTdata == "0"){Active = 0;}
else if (BTdata == "2"){Active = 2;}
else if (BTdata == "3"){Active = 3;}
else if (BTdata == "4"){Active = 4;}
else if (BTdata == "5"){Active = 5;}
else if (BTdata == "6"){Active = 6;}
else if (BTdata == ""){Active = 86;}

//hold state
if (Active < 80) {State=Active;}
Serial.println(BTdata);
Serial.println(Active);
Serial.println(State);
Serial.println(distruptor);


 BTdata = "";

randomNumber = random(0,300); //random number generator
randomNumberB = random(0,150); //random number generator bottom
randomNumberT = random(150,300); //random number generator bottom
EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow

//execute sequence
if (State == 0) {  
  
  //--------------------------------animation loop-------************----------------------   MultiTrailz
 //sine variables 
 waveA = 12; //high number more speratic 0-150 default 10
 waveB = .5;  //fast pulse a high number 0-5     default 1
 waveC = 2;  //randomizes sine wave pulse higher number fast 0-80 default 4


  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoiseT();    fillnoiseB();                                                         // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one every 5 seconds.
     targetPalette = CRGBPalette16(CRGB::Black); }

  LEDS.show();                    // Display the LED's at every loop cycle.  
  //-------------------------------animation loop------************---------------------- MultiTrailz
}
else if (State == 1) {   
 //--------------------------------animation loop-------************---------------------- Party Sparkle
//sine variables 
 waveA = 25; //high number more speratic 0-150 default 10
 waveB = .8;  //fast pulse a high number 0-5     default 1
 waveC = 1;  //randomizes sine wave pulse higher number fast 0-80 default 4
 
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoiseT();  fillnoiseB();  addGlitter(80);                                                        // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one every 5 seconds.
    
  // Change the target palette to a random one every 5 seconds.
     targetPalette = CRGBPalette16(PartyColors_p);
     }

     
  
  LEDS.show();                    // Display the LED's at every loop cycle.  
  //-------------------------------animation loop------************----------------------  Party Sparkle
  }
else if (State == 2) {   
 //--------------------------------animation loop-------************----------------------  Single Trailz 
     // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 30);
  int pos = beatsin16( 30, 0, NUM_LEDS-1 );  //(speed, firstled, lastled)
  leds[pos] += CHSV( gHue, 255, 192);

  LEDS.show();                    // Display the LED's at every loop cycle.  
  //-------------------------------animation loop------************---------------------- Single Trailz 
  }
else if (State == 3) {   
 //--------------------------------animation loop-------************---------------------- Rainbow Sparkle
 // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
 addGlitter(20);
      
  LEDS.show();                    // Display the LED's at every loop cycle.  
  //-------------------------------animation loop------************---------------------- Rainbow Sparkle
  }
else if (State == 4) {   
 //--------------------------------animation loop-------************----------------------   Random Dots
 
  leds[ randomNumber ] += CHSV( gHue, 255, 192);

   fadeToBlackBy( leds, NUM_LEDS, 3);
      
  LEDS.show();                    // Display the LED's at every loop cycle. 

 leds[randomNumber] += CRGB::Black;

  //-------------------------------animation loop------************---------------------- Random Dots
  }

  else if (State == 5) {   
 //--------------------------------animation loop-------************----------------------   Lava MODE

 //sine variables 
 waveA = 3; //high number more speratic 0-150 default 10
 waveB = .3;  //fast pulse a high number 0-5     default 1
 waveC = 1;  //randomizes sine wave pulse higher number fast 0-80 default 4
  
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoiseT();  fillnoiseB();                                            // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one every 5 seconds.

targetPalette = CRGBPalette16(LavaColors_p); //max of 4 colors
  
    }

  LEDS.show();                    // Display the LED's at every loop cycle.  
  
  //-------------------------------animation loop------************---------------------- Lava
  }

 else if (State == 6) {   
 //--------------------------------animation loop-------************----------------------  Trailz and random

     // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 30);
  int pos = beatsin16( 5, 0, 150-1 );
  leds[pos] += CHSV( gHue, 255, 192);
  leds[ randomNumberT ] += CHSV( gHue, 255, 192);

  LEDS.show();                    // Display the LED's at every loop cycle.  
  
  //-------------------------------animation loop------************----------------------  Trailz and random
  }


    else if (State == 7) {   
 //--------------------------------animation loop-------************----------------------  Lava, Trailz, Random

 //sine variables 
 waveA = 3; //high number more speratic 0-150 default 10
 waveB = .3;  //fast pulse a high number 0-5     default 1
 waveC = 10;  //randomizes sine wave pulse higher number fast 0-80 default 4
  
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoiseB();                                                          // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one every 5 seconds.

targetPalette = CRGBPalette16(LavaColors_p); //max of 4 colors
  
    }

         // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 30);
  int pos = beatsin16( 15, 150, 300-1 );
  leds[pos] += CHSV( gHue, 255, 192);
  leds[ randomNumberT ] += CHSV( gHue, 255, 192);
  LEDS.show();                    // Display the LED's at every loop cycle.  
  
  //-------------------------------animation loop------************---------------------- Startup
  }

  else if (State == 8) {  
  
  //--------------------------------animation loop-------************----------------------   MultiTrailz
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 5; i++) {
    leds[beatsin16( i+1.5, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;}
 
      // Show the leds 
      FastLED.show();
  //-------------------------------animation loop------************---------------------- MultiTrailz
}
}



// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

