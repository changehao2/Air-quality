#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <LiquidCrystal.h>
#include <dht11.h>

LiquidCrystal lcd(22, 24, 26, 28, 30, 32); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7) 

dht11 DHT;
#define DHT11_PIN 4

#define USE_AVG
#define sharpLEDPin  7   // Arduino digital pin 7 connect to sensor LED.
#define sharpVoPin   A5   // Arduino analog pin 5 connect to sensor Vo.
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;
// Set the typical output voltage in Volts when there is zero dust. 
static float Voc = 0.6;
// Use the typical sensitivity in units of V per 100ug/m3.
const float K = 0.5;

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
void TaskDHT11( void *pvParameters );
void TaskMQ9( void *pvParameters );
void TaskDUST( void *pvParameters );
// void TaskLCD( void *pvParameters );
void TaskDEBUG( void *pvParameters );
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(sharpLEDPin, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  lcd.begin(20,4);
  lcd.setCursor(0,0); 
  lcd.print("Temperature: "); //13
  lcd.setCursor(0,1); 
  lcd.print("Humidity: "); 
  lcd.setCursor(0,2); 
  lcd.print("CO: ");
  lcd.setCursor(0,3); 
  lcd.print("Dust density: doing");
    
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

//  xTaskCreate(
//    TaskDUST
//    ,  (const portCHAR *) "ReadDustSensor"
//    ,  128  // Stack size
//    ,  NULL
//    ,  2  // Priority
//    ,  NULL );
  
  xTaskCreate(
    TaskDHT11
    ,  (const portCHAR *)"ReadDHT11"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  4  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskMQ9
    ,  (const portCHAR *) "ReadMQ9"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );
// xTaskCreate(
//     TaskLCD
//     ,  (const portCHAR *) "DisplayLCD"
//     ,  128  // Stack size
//     ,  NULL
//     ,  2  // Priority
//     ,  NULL );

//  xTaskCreate(
//    TaskDEBUG
//    ,  (const portCHAR *) "Debug"
//    ,  128  // Stack size
//    ,  NULL
//    ,  1  // Priority
//    ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.

}

void loop()
{
  // Empty. Things are done in Tasks.
  
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

 void TaskDHT11( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {
    int chk = DHT.read(DHT11_PIN);    // READ DATA
    lcd.setCursor(13,0); 
    lcd.print(DHT.temperature); 
    lcd.setCursor(13,1); 
    lcd.print(DHT.humidity); 
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.print("DHT11, \tSTATUS: ");
      switch (chk){
      case DHTLIB_OK:  
                Serial.print("OK,\t"); 
                break;
      case DHTLIB_ERROR_CHECKSUM: 
                Serial.print("Checksum error,\t"); 
                break;
      case DHTLIB_ERROR_TIMEOUT: 
                Serial.print("Time out error,\t"); 
                break;
      default: 
                Serial.print("Unknown error,\t"); 
                break;
      }
        Serial.print(DHT.humidity,1);
        Serial.print(",\t");
        Serial.println(DHT.temperature,1);
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

        vTaskDelay( 2000 / portTICK_PERIOD_MS );
  }
}

void TaskMQ9( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(A0);
    sensor_volt=(float)sensorValue/1024*5.0;
    RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL
 
    ratio = RS_gas/0.29;  // ratio = RS/R0 
  /*-----------------------------------------------------------------------*/
    lcd.setCursor(13,2); 
    lcd.print(ratio);  
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
//    Serial.print("sensor_volt = ");
//      Serial.println(sensor_volt);
//      Serial.print("RS_ratio = ");
//      Serial.println(RS_gas);
      Serial.print("CO = ");
      Serial.println(ratio);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay( 2000 / portTICK_PERIOD_MS );
  }
}

void TaskDUST( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  for (;;)
  { 
    vTaskDelay( 3000 / portTICK_PERIOD_MS );
    digitalWrite(sharpLEDPin, LOW);
  // Wait 0.28ms before taking a reading of the output voltage as per spec.
    vTaskDelay( 280 / portTICK_PERIOD_MS );
  // Record the output voltage. This operation takes around 100 microseconds.
    int VoRaw = analogRead(sharpVoPin); 
  // Turn the dust sensor LED off by setting digital pin HIGH.
    digitalWrite(sharpLEDPin, HIGH);
  // Wait for remainder of the 10ms cycle = 10000 - 280 - 100 microseconds.
    vTaskDelay( 9620 / portTICK_PERIOD_MS );
    float Vo = VoRaw;
  #ifdef USE_AVG
    VoRawTotal += VoRaw;
    VoRawCount++;
    if ( VoRawCount >= N ) {
      Vo = 1.0 * VoRawTotal / N;
      VoRawCount = 0;
      VoRawTotal = 0;
    } else {
    return;
    }
   #endif // USE_AVG
    Vo = Vo / 1024.0 * 5.0; // VOTLS :))
    float dV = Vo - Voc;
      if ( dV < 0 ) {
          dV = 0;
          Voc = Vo;
      }
    float dustDensity = dV / K * 100.0; 
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("Dust density: ");
      Serial.print(dustDensity);
      Serial.println(" ug/m3");

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay( 3000 / portTICK_PERIOD_MS );
  }
}

void TaskDEBUG( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println("DEBUG:)))");
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

        vTaskDelay( 2000 / portTICK_PERIOD_MS );
  }
}












