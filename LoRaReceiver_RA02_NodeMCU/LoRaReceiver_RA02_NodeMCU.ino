#include <SoftwareSerial.h>
#include <String.h>
SoftwareSerial mySerial(D2, D1); // RX, TX

String dataPackage = "";
String h, t, c, dust;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
  while(!Serial){
  }
}

void loop() {
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
  }

}


