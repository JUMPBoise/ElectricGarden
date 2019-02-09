 
//buttons
const int buttonPinA = A1;
const int buttonPinB = A2;
const int buttonPinC = A3;
const int buttonPinD = A4;
const int buttonPinE = A5;

//buttons var
int Index = 0;
int Middle = 0;
int Ring = 0;
int Pinky = 0;
int Thumb = 0;

int transmit = 0;

char incomingByte;
String readBuffer = "";
boolean dataSwitch = false;

String RXdata = "";


//BT
int state =0; //variable for bt comm
int stateComp;
long BTappTimer = 0; // tracking App clicks
long halfMinute =   30000; //milliseconds in a 1/2 minute

#include <SoftwareSerial.h>
SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin
void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12

//buttons
pinMode(buttonPinA, INPUT);
pinMode(buttonPinB, INPUT);
pinMode(buttonPinC, INPUT);
pinMode(buttonPinD, INPUT);
pinMode(buttonPinE, INPUT);





  
}
void loop() {
 
//if(Serial.available()){state = Serial.read(); }

//button scramble
     if (Thumb == 0 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 0 ) {state = 86;} // send no signal
else if (Thumb == 1 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 0 ) {state = 86;} // send no signal
else if (Thumb == 1 && Index == 1 && Middle == 0 && Ring == 0 && Pinky == 0 ) {state = 1;} // send state
else if (Thumb == 1 && Index == 0 && Middle == 1 && Ring == 0 && Pinky == 0 ) {state = 2;} // send state
else if (Thumb == 1 && Index == 0 && Middle == 0 && Ring == 1 && Pinky == 0 ) {state = 3;} // send state
else if (Thumb == 1 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 1 ) {state = 4;} // send state
else if (Thumb == 0 && Index == 1 && Middle == 0 && Ring == 0 && Pinky == 0 ) {state = 5;} // send state
else if (Thumb == 0 && Index == 0 && Middle == 1 && Ring == 0 && Pinky == 0 ) {state = 6;} // send state
else if (Thumb == 0 && Index == 0 && Middle == 0 && Ring == 1 && Pinky == 0 ) {state = 7;} // send state
else if (Thumb == 0 && Index == 0 && Middle == 0 && Ring == 0 && Pinky == 1 ) {state = 8;} // send state
else if (Thumb == 1 && Index == 1 && Middle == 1 && Ring == 1 && Pinky == 1 ) {state = 0;} // send state  off
else if (Thumb == 0 && Index == 1 && Middle == 1 && Ring == 1 && Pinky == 1 ) {state = 0;} // send state  off


// read buttons
Index = digitalRead(buttonPinE);
Middle = digitalRead(buttonPinD);
Ring = digitalRead(buttonPinC);
Pinky = digitalRead(buttonPinB);
Thumb = digitalRead(buttonPinA);

/*
Serial.println(Index);
Serial.println(Middle);
Serial.println(Ring);
Serial.println(Pinky);
Serial.println(Thumb);
Serial.println();
*/



//if (state ==1){ transmit =1;}
//else if (state ==0){ transmit =0;}


if (state==1){ 
   HC12.print("^"); 
   HC12.print("1"); 
   HC12.print("%");
   delay(20);
}
else if (state==0){ 
   HC12.print("^"); 
   HC12.print("0"); 
   HC12.print("%");
   delay(20);
}
else if (state==2){ 
   HC12.print("^"); 
   HC12.print("2"); 
   HC12.print("%");
   delay(20);
}
else if (state==3){ 
   HC12.print("^"); 
   HC12.print("3"); 
   HC12.print("%");
   delay(20);
}
else if (state==4){ 
   HC12.print("^"); 
   HC12.print("4"); 
   HC12.print("%");
   delay(20);
}
else if (state==5){ 
   HC12.print("^"); 
   HC12.print("5"); 
   HC12.print("%");
   delay(20);
}
else if (state==6){ 
   HC12.print("^"); 
   HC12.print("6"); 
   HC12.print("%");
   delay(20);
}
else if (state==7){ 
   HC12.print("^"); 
   HC12.print("7"); 
   HC12.print("%");
   delay(20);
}
else if (state==8){ 
   HC12.print("^"); 
   HC12.print("8"); 
   HC12.print("%");
   delay(20);
}
Serial.println(state);
//Serial.println(transmit);
delay(500);
//transmit =0;
state = 86; //send no signal
}







