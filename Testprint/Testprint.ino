#include <OneButton.h>
#define PIN_INPUT 0
#define PIN_LED 19
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Bounce2.h>
#include <BleCombo.h>   
//#define I2C_SDA 23
//#define I2C_SCL 22

//v is x axis, h is y axis

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MPU6050 accelgyroh(0x69),accelgyrov(0x68); // default is 0x68; with AD0 high it's 0x69                          //
int16_t axh, ayh, azh;                                                                                          //
int16_t gxh, gyh, gzh;                                                                                          //
int16_t axv, ayv, azv;                                                                                          //
int16_t gxv, gyv, gzv;                                                                                          //
int16_t sensitivityDivider = 300;   //250                                                                       //
int deadZone = 100; // radius in which motion is ignored. Not used since much smaller than sensitivityDivider.   //
signed char mouseVX, mouseVY;                                                                                   //
//const unsigned long BTN_THROTTLE_DELAY = 40;                                                                  //
//const int btnPin[] = {10, 16, 14};                                                                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mpu6050_setOffsetsh(MPU6050 mpu)
{
//-3248  -6670 2475  153 13  -63
  
  mpu.setXAccelOffset(-3248);
  mpu.setYAccelOffset(-6670);
  mpu.setZAccelOffset(2475);
  mpu.setXGyroOffset(153);
  mpu.setYGyroOffset(13);
  mpu.setZGyroOffset(-63);
}

void mpu6050_setOffsetsv(MPU6050 mpu)
{


//-149  645 4783  3 -35 -56

  
  mpu.setXAccelOffset(-149);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(4783);
  mpu.setXGyroOffset(3);
  mpu.setYGyroOffset(-35);
  mpu.setZGyroOffset(-56);
}


/////////////////////////////////////////
OneButton button(PIN_INPUT, true);  ////
unsigned long pressStartTime;        //// Globals for Button
int ledState = LOW;                  ////
/////////////////////////////////////////

////////////////////////////////////////
void IRAM_ATTR checkTicks()         ////
{                                   ////  ISR
  button.tick();                    ////
}                                   ////
////////////////////////////////////////

void singleClick() //Single click
{
   Mouse.click(MOUSE_LEFT);
  //Serial.println("singleClick() detected.");
} 

void doubleClick() // Double click
{

   Mouse.click(MOUSE_LEFT);
   Mouse.click(MOUSE_LEFT);
  /*
 Serial.println("doubleClick() detected.");
 ledState = !ledState; // reverse the LED
 digitalWrite(PIN_LED, ledState);
 */
} 

void multiClick() //3 and 4 Clicks
{
  int n = button.getNumberClicks();
  if (n == 3) 
  {
     Mouse.click(MOUSE_RIGHT);
    //Serial.println("tripleClick detected.");
  } 
  else if (n == 4) 
  {
   // Serial.println("quadrupleClick detected.");
   Keyboard.press(KEY_LEFT_GUI);
   Keyboard.press(KEY_LEFT_CTRL);
   Keyboard.write('o');
   delay(100);
   Keyboard.releaseAll();
  } 
  else 
  {
   /*
   Serial.print("multiClick(");
   Serial.print(n);
   Serial.println(") detected.");
   */
  }

}

void pressStart()  //Long press hold start time
{
  Serial.println("pressStart()");
  pressStartTime = millis() - 1000; // as set in setPressTicks()
}

void pressStop() // pressStop()
{
  int x=0;
  x=millis()- pressStartTime;
  /*
  Serial.print("pressStop(");
 // Serial.print(millis() - pressStartTime);
 // Serial.println(") detected.");
  */
  if(x>3000)
  {
     Keyboard.press(KEY_LEFT_GUI);
     Keyboard.write('h');
     delay(100);
     Keyboard.releaseAll();
  }
  
} 


void setup() {
  Mouse.begin();
 Keyboard.begin();
 pinMode(2,OUTPUT);
 digitalWrite(2,HIGH);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(25,26);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
//Serial.println("Initializing MPU-6050");
  accelgyroh.initialize();
  accelgyrov.initialize();
  /*
  Serial.println("Testing device connection...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("Updating internal sensor offsets...");
  */
  mpu6050_setOffsetsh(accelgyroh);
  mpu6050_setOffsetsv(accelgyrov);

/*
  for (int i = 0; i < 3; i++) 
  {
    btn[i].attach(btnPin[i], INPUT_PULLUP);
    btn[i].interval(BTN_THROTTLE_DELAY);
  }
  */

 attachInterrupt(digitalPinToInterrupt(PIN_INPUT), checkTicks, CHANGE);
  button.attachClick(singleClick);
  button.attachDoubleClick(doubleClick);
  button.attachMultiClick(multiClick);

  button.setPressTicks(1000); // that is the time when LongPressStart is called
  button.attachLongPressStart(pressStart);
  button.attachLongPressStop(pressStop);
}
void loop() 
{
   button.tick();
   //accelgyroh.CalibrateGyro
  accelgyroh.getMotion6(&axh, &ayh, &azh, &gxh, &gyh, &gzh);
  accelgyrov.getMotion6(&axv, &ayv, &azv, &gxv, &gyv, &gzv);
  // Serial.print("\n");
   // Serial.print(-gx);
     /*   Serial.print("\n");
    Serial.print(gy);
        Serial.print("\n");
    Serial.print(gz);
  */
  processMouse(-gyh,gxv);
  delay(12);
}

void processMouse(int dX, int dY) 
{
  //Serial.print("\n");
    //Serial.print(dX);
  //dX-=deadZone;
 // dY-=deadZone;
  mouseVX = dX / sensitivityDivider;
  mouseVY = dY / sensitivityDivider;
  //Serial.print("\n");
    //Serial.print(mouseVX);
  if (mouseVX != 0 || mouseVY != 0) 
  {
   Mouse.move(mouseVX, mouseVY);
   /*
    Serial.print("$M 0 ");
    Serial.print(mouseVX);
    Serial.print(" ");
    Serial.print(mouseVY);
    Serial.print(" 0\n");
  */
  }
}