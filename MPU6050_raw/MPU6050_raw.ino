// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(2, 3); //Connect HC-06 TX,RX

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int aax,aay,aaz;
int ggx,ggy,ggz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

//header
static byte SUBSCRIBE=0x00;
static byte RESPONSE=0x01;
static byte UNSUBSCRIBE=0x02;
static byte MAJOR=0x03;

//body
static byte FRONT_FACE=0x01;
static byte BACK_FACE=0x00;

int header_body_switch = 0;//false is meaning of ready to get header.

byte a;
byte temp[1];
byte header;
byte body;
byte header2;
byte temp2[1];

int state = 0; //0 is stop , 1 is grap , 1 is throwing , 2 is landding

int waitcount=0;
boolean isSubscribingState = false;

boolean going=false;
/////////////////led
int redPin = 5;
int greenPin = 6;
int bluePin = 7;
///////////////////////


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);
    BTSerial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");



    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT); 

    setColor(0,255,255); //red
}

void loop() {
    // read raw accel/gyro measurements from device
    // these methods (and a few others) are also available
        /*
    setColor(255, 0, 0); //sky blue
    delay(500);
    setColor(0,255,0); //pinkl
    delay(500); 
    setColor(0,0,255);  //orange
    delay(500);
    setColor(255,255,0); //blue
    delay(500);
    setColor(255,0,255);  //green
    delay(500);
    setColor(0,255,255);  //red
    delay(500);
    */
    /////////////////////
        
    if(BTSerial.available())
    {
       setColor(255,0,0);
       BTSerial.readBytes((char *)temp,1);
      
      if(header_body_switch == 0){
        header = temp[0];
        header_body_switch = switching(header_body_switch);
      }else if(header_body_switch == 1){
        body = temp[0];
        header_body_switch = switching(header_body_switch);\
      }
      if(header_body_switch == 0){
        going=true;
      }
      ////////////////////
 
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState); 
    }
    if(going==true)
        toDo(header, body);
 }  
 ////////////////////////////////////////////////////////////////////////////////////


void activate()
{
   setColor(255,0,0); //sky blue
   while(1)
    {
           //stop state
         if(receiveProtocol())  break;
         
         if(state == 0){
               receiveProtocol();
               accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
               if(abs(gx) > 1000 && abs(gy) > 1000 && abs(gz) > 1000)
                      state=1;   
                      
              accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
              
              if(abs(gx) < 500 && abs(gy) < 500 && abs(gz) < 500 ){
                waitcount++;          
              }
              else
              {
                waitcount=0;
              }
      
              if(waitcount > 10000)
              {
                setColor(255,255,255);
                delay(1000);
              }      
                 
         }
         
         //grab state
         if(state == 1){
           setColor(255,0,255);
           accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
           if(abs(ax) >= 16000 && abs(ay) >= 16000 && abs(az) >= 16000){
                 
             if(isThrowing())
                   state=2;
             else
             {  setColor(255,0,0);               state=0;}
           }
           else if(abs(gx) >= 32000 && abs(gy) >= 32000 && abs(gz) >= 32000){
             if(isThrowing())
                   state=2;
             else
             {  setColor(255,0,0);               state=0;}
           }
           if(abs(gx) < 200 && abs(gy) < 200 && abs(gz) < 300 ){
               delay(500);
               accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
               if(abs(gx) < 200 && abs(gy) < 200 && abs(gz) < 300){
                  state=0;
                  break;
                }
             }
         }
         
         //throw state
         if(state == 2){
               setColor(0,0,255);
               throwing();
         }
         
         //landing state
         if(state == 3){
                 setColor(255,0,255);
                 Serial.println("result value");
                 calculator();
                 break;
         }        
      }
}

/////////////////////////////////////////////////////////////////////////////
void calculator(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(az > 0)
    {
      BTSerial.write(RESPONSE);
      BTSerial.write(FRONT_FACE);
      state=0;
    }
    else if(az < 0)
    {
      BTSerial.write(RESPONSE);
      BTSerial.write(BACK_FACE);
      state=0;
    }    
 }
  


void toDo(byte header, byte body){
  
  if(header == SUBSCRIBE){
    
       isSubscribingState = true;
       Serial.println("activate before");
       activate();
  }
  else if(header == UNSUBSCRIBE){
       Serial.println("UNSUBSCRIBE");
       isSubscribingState = false;
  }
}

 boolean isThrowing(){
      int16_t tax, tay, taz;
      int count1=0,count2=0,count3=0;
      tax=ax; tay=ay; taz=az;

      setColor(255,255,0);
      
      
      for(int i=0; i<50; i++){
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            Serial.print("throw join");
              
            if(abs(gx) < 2000 && abs(gy) < 2000 && abs(gz) < 2000)
              return false;
              
   
            //if(abs(gx) > 13000 && abs(gy) > 13000 && abs(gz) > 13000)
            //   return true;
             /*  
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            if( (tax > 0 && ax < 0) || (tax < 0 && ax > 0) ){
                            count1++;
                            tax=ax-(2*ax);
            }
            if( (tay > 0 && ay < 0) || (tay < 0 && ay > 0) ){
                            count2++;
                            tay=ay-(2*ay);
            }
            if( (taz > 0 && az < 0) || (taz < 0 && az > 0) ){
                            count3++;
                            taz=az-(2*az);
            }

            if( count1 >= 7 && count2 >= 7 && count3 >= 7)
                          return true;       
          */  
      }  
      
      return true;
 }
 //Yut throwing
 void throwing(){
            
      while(1){
         if(receiveProtocol())  break;
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
         Serial.print(gx);Serial.print("\t");
         Serial.print(gy);Serial.print("\t");
         Serial.println(gz); 
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                    
         if(abs(gx) < 200 && abs(gy) < 200 && abs(gz) < 300 ){
           delay(500);
           accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          
           if(abs(gx) < 200 && abs(gy) < 200 && abs(gz) < 300){
               state=3;
               break;
           }
         }
      }
 }

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue); 
}
int receiveProtocol(){
      if(BTSerial.available())
         {
             BTSerial.readBytes((char *)temp2,1);
             if(header_body_switch == 0){
                header2 = temp2[0];
                header_body_switch = switching(header_body_switch);
              }else if(header_body_switch == 1){
                body = temp2[0];
                header_body_switch = switching(header_body_switch);
              }
              
              if(header2 == MAJOR){
                calculator();
                return true;
              }
               if(header2 == UNSUBSCRIBE){
                 Serial.println("reset");
                 setColor(0,255,255);
                 softReset();
              }
         }
         return false;
}
//reset function
void softReset(){
  asm volatile ("  jmp  0");
}

int switching(int a){
  if(a == 0)
    return 1;
  else
    return 0;
}
