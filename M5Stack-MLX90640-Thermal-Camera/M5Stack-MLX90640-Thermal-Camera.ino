// update 3 :mirror image
// reverseScreen=true/false, turn=front camera, false=Selfie
/*
  Read the temperature pixels from the MLX90640 IR array
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14769

  This example initializes the MLX90640 and outputs the 768 temperature values
  from the 768 pixels.

  This example will work with a Teensy 3.1 and above. The MLX90640 requires some
  hefty calculations and larger arrays. You will need a microcontroller with 20,000
  bytes or more of RAM.

  This relies on the driver written by Melexis and can be found at:
  https://github.com/melexis/mlx90640-library

  Hardware Connections:
  Connect the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to the Qwiic board
  Connect the male pins to the Teensy. The pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
  Open the serial monitor at 9600 baud to see the output
*/

#include <M5Stack.h>
#include <Wire.h>

//#include "M5StackUpdater.h"

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

#define AMG_COLS 32
#define AMG_ROWS 24

float pixelsArraySize = AMG_COLS * AMG_ROWS;  //count 1 once

float pixels[AMG_COLS * AMG_ROWS];

//# define reversePixels [pixelsArraySize];
float reversePixels[AMG_COLS * AMG_ROWS];

//bool reverseScreen=false;
bool reverseScreen=true;

#define INTERPOLATED_COLS 16
#define INTERPOLATED_ROWS 16 

static float mlx90640To[AMG_COLS * AMG_ROWS];
paramsMLX90640 mlx90640;


float signedMag12ToFloat(uint16_t val);


//low range of the sensor (this will be blue on the screen)
int MINTEMP = 22;
int Default_MINTEMP = 20;

//high range of the sensor (this will be red on the screen)
int MAXTEMP = 26;
int Default_MAXTEMP = 26;

//the colors we will be using
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

int min_v = -40;
int min_cam_v = min_v;
int resetMinTemp = min_cam_v +10;
    
int max_v = 300;
int max_cam_v = max_v;
int resetMaxTemp = max_cam_v -10;

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);

int size = sizeof(reversePixels);
long loopTime,startTime,endTime,fps;

void setup()
{
  M5.begin();  
  Wire.begin();

/*  
 *   //M5StackUpdater
  if(digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }
*/
  
//  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Wire.setClock(600000); //Increase I2C clock speed to 600kHz
//Wire.setClock(800000); //Increase I2C clock speed to 800kHz
 
  //Serial.begin(9600);
  Serial.begin(115200);

  M5.Lcd.begin();
  M5.Lcd.setRotation(0);
  
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  
  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawCentreString("MLX90640 IR Array Example", 60, 4, 1);  

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  int SetRefreshRate;
    //Setting MLX90640 device at slave address 0x33 to work with 16Hz refresh rate:
  // 0x00 – 0.5Hz
  // 0x01 – 1Hz
  // 0x02 – 2Hz
  // 0x03 – 4Hz
  // 0x04 – 8Hz // OK @ I2C clock speed to 600kHz ~3fps
  // 0x05 – 16Hz // OK
  // 0x06 – 32Hz // Fail
  // 0x07 – 64Hz
  SetRefreshRate = MLX90640_SetRefreshRate (0x33,0x05);
  //Once params are extracted, we can release eeMLX90640 array

//Display left side colorList and info
//   M5.Lcd.fillScreen(TFT_BLACK);
//  int icolor = 255;
//  for (int irow = 16; irow <= 223;  irow++)
//  {
//    M5.Lcd.drawRect(0, 0, 35, irow, camColors[icolor]);
//    icolor--;
//  }


//Display bottom side colorList and info
   M5.Lcd.fillScreen(TFT_BLACK);
  int icolor = 0;
  for (int icol = 0; icol <= 248;  icol++)
  {
    M5.Lcd.drawRect(36, 208, icol, 284 , camColors[icolor]);
    icolor++;
  }
  infodisplay();  

}


void loop()
{
  loopTime = millis();
  startTime = loopTime;
//  Serial.print("Loop Start Time: ");  
//  Serial.print(loopTime); 
  ///////////////////////////////
  // Set Min Value - LongPress //
  ///////////////////////////////
  if (M5.BtnA.pressedFor(1000)) { 
    if (MINTEMP <= min_v +10)
    {
      MINTEMP = MAXTEMP - 10;
    }
    else
    {
      MINTEMP = MINTEMP - 10;
    }
    infodisplay();
  }

  ///////////////////////////////
  // Set Min Value - SortPress //
  ///////////////////////////////
  if (M5.BtnA.wasPressed()) {
    if (MINTEMP <= min_cam_v)
    {
      MINTEMP = MAXTEMP - 1;
    }
    else
    {
      MINTEMP--;
    }    
     infodisplay();
  }

  /////////////////////
  // Reset settings  //
  /////////////////////
  if (M5.BtnB.wasPressed()) {
    MINTEMP = max_v - 12;
    MAXTEMP = max_v - 2;
    infodisplay();
  }
  
  ////////////////
  // Power Off  //
  ////////////////
  if (M5.BtnB.pressedFor(1000)) {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.drawCentreString("Power Off...", 160, 80, 4);    
    delay(1000);
    M5.powerOFF();
  }

  ///////////////////////////////
  // Set Max Value - LongPress //
  ///////////////////////////////
  if (M5.BtnC.pressedFor(1000)) {
    if (MAXTEMP >= resetMaxTemp)
    {
      MAXTEMP = MINTEMP + 1;
    }
    else
    {     
        MAXTEMP = MAXTEMP + 10;      
    }    
    infodisplay();
  }

  ///////////////////////////////
  // Set Max Value - SortPress //
  ///////////////////////////////
  if (M5.BtnC.wasPressed()) {
    if (MAXTEMP >= max_cam_v )
    {
      MAXTEMP = MINTEMP + 1;
    }
    else
    {
      MAXTEMP++;
    }
    infodisplay();
  }

  M5.update();
  
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    //MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels); //save pixels temp to array (pixels)
  }


/*
   for (int x = 0 ; x < 768 ; x++)
  {
        if(x % 32 == 0) Serial.println();  //32 values wide
        Serial.print(pixels[x], 0);  //no fractions
        Serial.print(" ");  //space instead of comma

//        M5.Lcd.fillScreen(TFT_BLACK);
//        M5.Lcd.setTextColor(YELLOW, BLACK);
//        M5.Lcd.setCursor(0, 0);
//        M5.Lcd.setTextSize(1);
//        M5.Lcd.print(mlx90640To[x], 1); 
  }

  Serial.println("");
  Serial.println("");  //extra line
*/

  
  
//Reverse image (order of Integer array)
if (reverseScreen == 1)
{
   for (int x = 0 ; x < pixelsArraySize ; x++)   
   {    
//       Serial.print(String(pixels[x]));
//       Serial.print(" ");
       if(x % AMG_COLS == 0) //32 values wide
       { 
//          Serial.println(": pixels");  //space instead of comma
          for (int j = 0+x, k = AMG_COLS+x; j < AMG_COLS+x ; j++, k--)
          {
             reversePixels[j]=pixels[k];
//             Serial.print("j:" + String(j));  //no fractions
//             Serial.print(" k:" + String(k) + ", ");
//             if ( j >= AMG_COLS+x )    Serial.println(": reversePixels");  //space instead of comma
          }
       }
   }
}   
      
 //Serial.println("loop done.");

  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];


 // *  temp stop display image

if (reverseScreen == 1)
{
// ** reversePixels
  interpolate_image(reversePixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
}
else
{
//interpolate_image(float *src, src_rows, src_cols, *dest, dest_rows, dest_cols);
  interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
}

//  uint16_t boxsize = min(M5.Lcd.width() / INTERPOLATED_COLS, M5.Lcd.height() / INTERPOLATED_COLS);
uint16_t boxsize = min(M5.Lcd.width() / INTERPOLATED_ROWS, M5.Lcd.height() / INTERPOLATED_COLS);

//drawpixels(     *p,              rows,              cols, boxWidth, boxHeight, showVal)
//  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, false);

uint16_t boxWidth = M5.Lcd.width() / INTERPOLATED_ROWS;
uint16_t boxHeight = (M5.Lcd.height()-31) / INTERPOLATED_COLS; // 31 for bottom info
drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxWidth, boxHeight, false);

  max_v = INT_MIN;

  int spot_v = pixels[28];
//  int spot_v = pixels[32*12+6];
  for ( int itemp = 0; itemp < sizeof(pixels) / sizeof(pixels[0]); itemp++ )
//  for ( int itemp = 0; itemp < (32*24) / sizeof(pixels[0]); itemp++ )
  {
    if ( pixels[itemp] > max_v )
    {
      max_v = pixels[itemp];
      //max_i = itemp;
    }

    if ( pixels[itemp] < min_v )
    {
      min_v = pixels[itemp];
      //max_i = itemp;
    }
  }

 
  M5.Lcd.setTextSize(2);
//  M5.Lcd.fillRect(284, 18, 36, 16, TFT_BLACK);  // clear max temp text
  M5.Lcd.fillRect(164, 220, 75, 18, TFT_BLACK);  // clear max temp text

//  M5.Lcd.fillRect(284, 130, 36, 16, TFT_BLACK); // clear spot temp text
  M5.Lcd.fillRect(85, 220, 90, 18, TFT_BLACK); // clear spot temp text
  
//  M5.Lcd.setCursor(284, 18);      // update max temp text
//  M5.Lcd.setCursor(200, 224);      // update max temp text
    M5.Lcd.setCursor(163, 222);      // update max temp text with "Max"

  M5.Lcd.setTextColor(TFT_WHITE);
  if (max_v > max_cam_v | max_v < min_cam_v) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("Err", 1);
  }
  else
  { 
    //  M5.Lcd.setCursor(284, 0); 
    M5.Lcd.printf("Max", 1);
    M5.Lcd.print(max_v, 1);
    M5.Lcd.printf("C" , 1);

//    M5.Lcd.setCursor(284, 130); // update spot temp text
    M5.Lcd.drawCircle(95, 230, 6, TFT_WHITE);          // update spot icon
    M5.Lcd.drawLine(95, 221, 95, 239, TFT_WHITE);   // vertical line
    M5.Lcd.drawLine(86, 230, 104, 230, TFT_WHITE);   // horizontal line
    M5.Lcd.setCursor(106, 222); // update spot temp text
    M5.Lcd.print(spot_v, 1);
    M5.Lcd.printf("C" , 1);
    M5.Lcd.drawCircle(160, 120, 6, TFT_WHITE);          // update center spot icon
    M5.Lcd.drawLine(160, 110, 160, 130, TFT_WHITE);
    M5.Lcd.drawLine(150, 120, 170, 120, TFT_WHITE);

  }  

//delay(1000);  
//  Serial.print(", Loop End Time: ");
  loopTime = millis();
  endTime = loopTime;
//  Serial.print(loopTime); 
//  Serial.print(", ");
  fps=1000 / (endTime - startTime);
  
 M5.Lcd.fillRect(300, 209, 20, 12, TFT_BLACK); //Clear fps text area
// Serial.println( String( fps ) +"/fps"); 
 M5.Lcd.setTextSize(1);
 M5.Lcd.setCursor(284, 210);
 M5.Lcd.print("fps:"+ String( fps ));
 M5.Lcd.setTextSize(1);
}


/***infodisplay()*****/
void infodisplay(void) {
  M5.Lcd.setTextColor(TFT_WHITE);
  //     M5.Lcd.setCursor(288, 230);
  //   M5.Lcd.printf("Power" , 1);

  
  //M5.Lcd.fillRect(0, 0, 36, 16, TFT_BLACK);
  M5.Lcd.fillRect(284, 223, 320, 240, TFT_BLACK); //Clear MaxTemp area
  M5.Lcd.setTextSize(2);
//  M5.Lcd.setCursor(0, 1);

//  M5.Lcd.fillRect(284, 208, 36, 16, TFT_BLACK);
//  M5.Lcd.setCursor(284, 208); //show max_cam_v
//  M5.Lcd.print(max_cam_v , 1);
  
  M5.Lcd.setCursor(284, 222); //move to bottom right 
  M5.Lcd.print(MAXTEMP , 1);  // update MAXTEMP
  M5.Lcd.printf("C" , 1);
  
  
//  M5.Lcd.fillRect(0, 208, 36, 16, TFT_BLACK);
  M5.Lcd.setCursor(0, 208); //show min_cam_v  
//  M5.Lcd.print(min_cam_v , 1);
  M5.Lcd.print(size , 1);

  M5.Lcd.setCursor(0, 222);  // update MINTEMP text
  M5.Lcd.fillRect(0, 222, 36, 16, TFT_BLACK);
  M5.Lcd.print(MINTEMP , 1);
  M5.Lcd.printf("C" , 1);

  
//  M5.Lcd.setCursor(284, 100);
  M5.Lcd.setCursor(106, 224);
//  M5.Lcd.printf("Spot", 1);
//  M5.Lcd.drawCircle(300, 120, 6, TFT_WHITE);
//  M5.Lcd.drawLine(300, 110, 300, 130, TFT_WHITE);
//  M5.Lcd.drawLine(290, 120, 310, 120, TFT_WHITE);
    M5.Lcd.drawCircle(95, 231, 6, TFT_WHITE);          // update spot icon
    M5.Lcd.drawLine(95, 223, 95, 239, TFT_WHITE);
    M5.Lcd.drawLine(85, 231, 105, 231, TFT_WHITE);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal) {
  int colorTemp;
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float val = get_point(p, rows, cols, x, y);
      if (val >= MAXTEMP) colorTemp = MAXTEMP;
      else if (val <= MINTEMP) colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color;
      color = val * 2;
//      M5.Lcd.fillRect(40 + boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
      M5.Lcd.fillRect(boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
    }
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
