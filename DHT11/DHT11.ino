#include <dht.h>

dht sensor;

double sHumidity;
double sTemperature;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
      int check = sensor.read11(7);
      Serial.println(check);
      delay(2000);
      sHumidity = sensor.humidity;
      sTemperature = sensor.temperature;
      Serial.print("Humidity: ");
      Serial.println(sHumidity);
      Serial.print("Temperature: ");
      Serial.println(sTemperature);


}
