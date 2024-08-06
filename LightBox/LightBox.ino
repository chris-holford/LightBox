//Libraries
#include <FastLED.h>
#include <EncoderButton.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Global Variables/Fags/Defs

//--LED
#define FRAME_TIME 34 //ms, ~30fps
#define NUM_LEDS_SRND 132
#define NUM_LEDS_ARY 264
#define NUM_LEDS 396
#define LED_PIN 13
//0-1147 Int array(s)
int colorData[3];   //RGB 0-1147, the active color
int defaultColor1[3] = {1147,600,200};  //I mean, mess around, but...
int defaultColor2[3] = {1147,1116,945};
int defaultColor3[3] = {1112,1111,1053};
//RGB Arrays
byte surroundPixel[3][3]; //[pixelLED 0-2][r,g,b]
byte arrayPixel[3][3];    //[pixelLED 0-2][r,g,b]
CRGB leds[NUM_LEDS];

//--ENCODERS
#define ENC_RATE_LIMIT 300
#define ENC_R1_PIN 1
#define ENC_R2_PIN 2
#define ENC_G1_PIN 4
#define ENC_G2_PIN 5
#define ENC_B1_PIN 7
#define ENC_B2_PIN 8
//#define ENC_C1_PIN 10 //not yet using
//#define ENC_C2_PIN 11 //not yet using
//**Click in feature wired but not yet coded
EncoderButton rEnc(ENC_R1_PIN, ENC_R2_PIN);
EncoderButton gEnc(ENC_G1_PIN, ENC_G2_PIN);
EncoderButton bEnc(ENC_B1_PIN, ENC_B2_PIN);
//  Encoder cEnc(ENC_C1_PIN, ENC_C2_PIN); //not yet using
bool rHasChanged = false;
bool gHasChanged = false;
bool bHasChanged = false;
//bool cHasChanged = false; //not yet using
int rEncVal;
int gEncVal;
int bEncVal;
//int cEncVal; //not yet using

//--BUTTONS
#define BTN1_PIN 21
#define BTN2_PIN 20
#define BTN3_PIN 17
#define BTN4_PIN 16
#define BOUNCE_TIME 10 //ms
#define HOLD_TIME 3000 //ms, 3 seconds
//debounce objects -> make sure to set input pullup on button pins
Bounce btn1 = Bounce(BTN1_PIN, BOUNCE_TIME);
Bounce btn2 = Bounce(BTN2_PIN, BOUNCE_TIME);
Bounce btn3 = Bounce(BTN3_PIN, BOUNCE_TIME);
Bounce btn4 = Bounce(BTN4_PIN, BOUNCE_TIME);
//timers
unsigned long btn1Timer;
unsigned long btn2Timer;
unsigned long btn3Timer;
unsigned long btn4Timer;

//--EEPROM
int preset1RedAddress = 0;
int preset1GreenAddress = 2;
int preset1BlueAddress = 4;
int preset2RedAddress = 6;
int preset2GreenAddress = 8;
int preset2BlueAddress = 10;
int preset3RedAddress = 12;
int preset3GreenAddress = 14;
int preset3BlueAddress = 16;

//--LCD
#define LCD_REFRESH_TIME 300
LiquidCrystal_I2C lcd(0x3F, 16, 2); //object(address,chars per line/row, lines/rows)

//===========================================================================================


//                                        EEPROM


//===========================================================================================

void writeToEEPROM(int address, int val)
{ 
  EEPROM.write(address, val >> 8);
  EEPROM.write(address + 1, val & 0xFF);
}

int readFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

//--Buttons-----------------------------------------------------------------------------------

void recallPreset1(){
  rEncVal = readFromEEPROM(preset1RedAddress);
  gEncVal = readFromEEPROM(preset1GreenAddress);
  bEncVal = readFromEEPROM(preset1BlueAddress);
  rHasChanged = true;
  gHasChanged = true;
  bHasChanged = true;
}

void recallPreset2(){
  rEncVal = readFromEEPROM(preset2RedAddress);
  gEncVal = readFromEEPROM(preset2GreenAddress);
  bEncVal = readFromEEPROM(preset2BlueAddress);
  rHasChanged = true;
  gHasChanged = true;
  bHasChanged = true;
}

void recallPreset3(){
  rEncVal = readFromEEPROM(preset3RedAddress);
  gEncVal = readFromEEPROM(preset3GreenAddress);
  bEncVal = readFromEEPROM(preset3BlueAddress);
  rHasChanged = true;
  gHasChanged = true;
  bHasChanged = true;
}
void recallDefault(){
  rEncVal = defaultColor1[0];
  gEncVal = defaultColor1[1];
  bEncVal = defaultColor1[2];
  rHasChanged = true;
  gHasChanged = true;
  bHasChanged = true;
}
//===========================================================================================


//                                        LED


//===========================================================================================

void pixelMap(int x, int channel){  //0-1147 int to 2x(3)(r,g,b)
  int rgbData[2][3]; // [surround/array][fill1,center,fill2]
  //Safety First
  if(x>1147){x=1147;}else if(x<0){x=0;}
  
  //channelMap(x)
  if(x<383){//only surround needed
    //write empty to "array" and pixelize to "surround"
    rgbData[1][0]=0; rgbData[1][1]=0; rgbData[1][2]=0;
    x *= 2; //cause half as many leds, 2 steps at a time
    if(x<255){//only center led
      //write (0,x,0) to "surround" 
      rgbData[0][0]=0; rgbData[0][1]=x; rgbData[0][2]=0;
    } else{//center max plus fill
      x -= 254;
      int y = x/2;  //2 step IS both fill leds at once. no odd numbers...
      //write (y,MAX,y) to "surround"
      rgbData[0][0]=y; rgbData[0][1]=254; rgbData[0][2]=y;
    }
  } else{//write max to "surround", pixelize remaining to "array"
    rgbData[0][0]=255; rgbData[0][1]=254; rgbData[0][2]=255;
    x -= 382;
    if(x<256){//only center led
      //write (0,x,0) to "array" 
      rgbData[1][0]=0; rgbData[1][1]=x; rgbData[1][2]=0;
    } else{//center max plus fill
      x -= 255;
      if(x%2 == 0){//even number
        int y = x/2;
        //write (y,MAX,y) to "array"
        rgbData[1][0]=y; rgbData[1][1]=255; rgbData[1][2]=y;
      } else{//odd number
        int y = x/2;//cause integer math
        //write (y+1,MAX,y) to "array"
        rgbData[1][0]=y+1; rgbData[1][1]=255; rgbData[1][2]=y;
      }
    }
  }
  //rgbData is in integers, cast to bytes and push to global Pixel byte arrays
  for(int i=0; i<3; i++){
      surroundPixel[i][channel] = (byte)rgbData[0][i];
      arrayPixel[i][channel] = (byte)rgbData[1][i];
  }
}

//--------------------------------------------------------------------------------------------

void pushPixelsToLEDS(){
  for(int i=0; i<NUM_LEDS; i++){
    int pixelLED = i%3;
    if(i<NUM_LEDS_SRND){
      //write to surround
      leds[i][0] = surroundPixel[pixelLED][0];
      leds[i][1] = surroundPixel[pixelLED][1];
      leds[i][2] = surroundPixel[pixelLED][2];
    } else{
      //write to array
      leds[i][0] = arrayPixel[pixelLED][0];
      leds[i][1] = arrayPixel[pixelLED][1];
      leds[i][2] = arrayPixel[pixelLED][2];
    }
  }
}

//===========================================================================================


//                                        ENCODERS


//===========================================================================================

void onREnc(EncoderButton& rEnc) {
  rEncVal += rEnc.increment()*abs(rEnc.increment());
  if(rEncVal>1147){rEncVal=1147;}else if(rEncVal<0){rEncVal=0;}
  colorData[0] = rEncVal;
  rHasChanged = true;
}

void onGEnc(EncoderButton& gEnc) {
  gEncVal += gEnc.increment()*abs(gEnc.increment());
  if(gEncVal>1147){gEncVal=1147;}else if(gEncVal<0){gEncVal=0;}
  colorData[1] = gEncVal;
  gHasChanged = true;
}

void onBEnc(EncoderButton& bEnc) {
  bEncVal += bEnc.increment()*abs(bEnc.increment());
  if(bEncVal>1147){bEncVal=1147;}else if(bEncVal<0){bEncVal=0;}
  colorData[2] = bEncVal;
  bHasChanged = true;
}

//===========================================================================================


//                                         LCD


//===========================================================================================

void writeToLCD(){
  //write Red val
  lcd.clear();
  lcd.setCursor(0,0); //set to top left
  lcd.print("R: ");
  lcd.setCursor(3,0);
  lcd.print(String(rEncVal));
  lcd.setCursor(7,0);
  lcd.print(" ");
  //write Green val
  lcd.setCursor(8,0); //set to g start
  lcd.print("G: ");
  lcd.setCursor(11,0);
  lcd.print(String(gEncVal));
  //write Blue val
  lcd.setCursor(0,1); //set to bottom left
  lcd.print("B: ");
  lcd.setCursor(3,1);
  lcd.print(String(bEncVal));
  lcd.setCursor(7,1);
  lcd.print(" LP GRIP");
  
}
//===========================================================================================


//                                         MAIN


//===========================================================================================

void setup() {
  
  //Buttons
  pinMode(BTN1_PIN, INPUT_PULLUP);  //!!OFF FOR TESTING
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  //Encoders(Callbacks for update())
  rEnc.setEncoderHandler(onREnc);
  rEnc.setRateLimit(ENC_RATE_LIMIT);
  gEnc.setEncoderHandler(onGEnc);
  gEnc.setRateLimit(ENC_RATE_LIMIT);
  bEnc.setEncoderHandler(onBEnc);
  bEnc.setRateLimit(ENC_RATE_LIMIT);

  //LCD
  lcd.init();
  lcd.backlight();
  
  // LED boilerplate
  FastLED.addLeds<1, WS2813, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.clear(true);

  // Read defaults/settings(working light), push to arrays
  colorData[0] = defaultColor1[0];
  colorData[1] = defaultColor1[1];
  colorData[2] = defaultColor1[2];
  rEncVal = colorData[0];
  gEncVal = colorData[1];
  bEncVal = colorData[2];
  
  // Initialize modes and flags
  rHasChanged = true;
  gHasChanged = true;
  bHasChanged = true;
}

//--------------------------------------------------------------------------------------------

void loop() {
  
  // Listeners -> Callbacks (mode toggle, rotary encoders, brightness toggle)

//  //--Button Polling
//  if(btn1.update()){  //state change
//    if(btn1.fallingEdge()){ //button pressed
//      //start timer, if released before x time, single press, else hold
//      btn1Timer = millis();
//    } else if(btn1.risingEdge()){ //button released
//      //check for length of press/hold
//      if(millis() > (btn1Timer + HOLD_TIME)){ //hold
//        //store colorData to preset 1
//        writeToEEPROM(preset1RedAddress, colorData[0]);
//        writeToEEPROM(preset1GreenAddress, colorData[1]);
//        writeToEEPROM(preset1BlueAddress, colorData[2]);
//      } else{ //press
//        //recall preset 1 and push to colorData via encoder write
//        recallPreset1();
//      }
//    }
//  }
//  if(btn2.update()){  //state change
//    if(btn2.fallingEdge()){ //button pressed
//      //start timer, if released before x time, single press, else hold
//      btn2Timer = millis();
//    } else if(btn2.risingEdge()){ //button released
//      //check for length of press/hold
//      if(millis() > (btn2Timer + HOLD_TIME)){ //hold
//        //store colorData to preset 2
//        writeToEEPROM(preset2RedAddress, colorData[0]);
//        writeToEEPROM(preset2GreenAddress, colorData[1]);
//        writeToEEPROM(preset2BlueAddress, colorData[2]);
//      } else{ //press
//        //recall preset 2 and push to colorData via encoder write
//        recallPreset2();
//      }
//    }
//  }
//  if(btn3.update()){  //state change
//    if(btn3.fallingEdge()){ //button pressed
//      //start timer, if released before x time, single press, else hold
//      btn3Timer = millis();
//    } else if(btn3.risingEdge()){ //button released
//      //check for length of press/hold
//      if(millis() > (btn3Timer + HOLD_TIME)){ //hold
//        //store colorData to preset 3
//        writeToEEPROM(preset3RedAddress, colorData[0]);
//        writeToEEPROM(preset3GreenAddress, colorData[1]);
//        writeToEEPROM(preset3BlueAddress, colorData[2]);
//      } else{ //press
//        //recall preset 3 and push to colorData via encoder write
//        recallPreset3();
//      }
//    }
//  }
//  if(btn4.update()){  //state change
//    if(btn4.fallingEdge()){ //button pressed
//      //start timer, if released before x time, single press, else hold
//      btn4Timer = millis();
//    } else if(btn4.risingEdge()){ //button released
//      //check for length of press/hold
//      if(millis() > (btn4Timer + HOLD_TIME)){ //hold
//        //???
//      } else{ //press
//        //reset to default working light?
//        //write to encoders and let that function push data next frame
//        recallDefault();
//      }
//    }
//  }

  //--Encoder polling... every loop
  rEnc.update();
  gEnc.update();
  bEnc.update();

  //--LEDS
  EVERY_N_MILLISECONDS(FRAME_TIME){
    bool toShow = false;
    if(rHasChanged){
      //update color
      pixelMap(colorData[0], 0);
      rHasChanged = false;
      toShow = true;
    }
    if(gHasChanged){
      //update color
      pixelMap(colorData[1], 1);
      gHasChanged = false;
      toShow = true;
    }
    if(bHasChanged){
      //update color
      pixelMap(colorData[2], 2);
      bHasChanged = false;
      toShow = true;
    }
    if(toShow){
      pushPixelsToLEDS();
      FastLED.show();
    }
  }

  //--LCD Update(slower)
  EVERY_N_MILLISECONDS(LCD_REFRESH_TIME){
    writeToLCD();
  }
}
//===========================================================================================
