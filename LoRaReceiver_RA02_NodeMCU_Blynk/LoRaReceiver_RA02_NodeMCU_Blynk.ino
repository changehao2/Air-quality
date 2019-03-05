/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on NodeMCU.

  Note: This requires ESP8266 support package:
    https://github.com/esp8266/Arduino

  Please be sure to select the right NodeMCU module
  in the Tools -> Board menu!

  For advanced settings please follow ESP examples :
   - ESP8266_Standalone_Manual_IP.ino
   - ESP8266_Standalone_SmartConfig.ino
   - ESP8266_Standalone_SSL.ino

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <String.h>
SoftwareSerial mySerial(D2, D1); // RX, TX

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "8706293aa979429c9f7fa1fc1df3189c";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Kim  Hoang";
char pass[] = "01676109293";
BlynkTimer timer; // Create a Timer object called "timer"! 
String h, t, c, dust;
void setup()
{
  // Debug console
  Serial.begin(9600);
  mySerial.begin(9600);
  while(!Serial){
  }
   timer.setInterval(1000L, sendUptime); //  Here you set interval (1sec) and which function to call 

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8442);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8442);
}

void sendUptime()
{
  // This function sends Arduino up time every 1 second to Virtual Pin (V5)
  // In the app, Widget's reading frequency should be set to PUSH
  // You can send anything with any interval using this construction
  // Don't send more that 10 values per second
// put your main code here, to run repeatedly:
  if (mySerial.available()) {
    h = mySerial.readStringUntil('\n');
    t = mySerial.readStringUntil('\n');
    c = mySerial.readStringUntil('\n');
    dust = mySerial.readStringUntil('\n');
    // dataPackage = mySerial.readString();
    // Serial.println(dataPackage);
  }
  if(dust.length()){
  // Serial.print("Real: ");
  // Serial.println(dataPackage);
  Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Tempurature: ");
  Serial.println(t);
  Serial.print("Co2: ");
  Serial.println(c);
  Serial.print("Dust density: ");
  Serial.println(dust);  
  Blynk.virtualWrite(V1, h.toFloat());
  Blynk.virtualWrite(V2, t.toFloat());
  Blynk.virtualWrite(V3, c.toFloat());
  Blynk.virtualWrite(V4, dust.toFloat());     
  }
}

void loop()
{
  Blynk.run(); // all the Blynk magic happens here
  timer.run(); // BlynkTimer is working...
}
