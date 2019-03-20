#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <LiquidCrystal.h>
#include <dht11.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
// LiquidCrystal lcd(22, 24, 26, 28, 30, 32); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7)

dht11 DHT;
#define DHT11_PIN 4

#define USE_AVG
#define sharpLEDPin  7   // Arduino digital pin 7 connect to sensor LED.
#define sharpVoPin   A5   // Arduino analog pin 5 connect to sensor Vo.
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;  // Set the typical output voltage in Volts when there is zero dust.
static float Voc = 0.6;     // Use the typical sensitivity in units of V per 100ug/m3.
const float K = 0.5;

const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
QueueHandle_t xQueue;
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
  pinMode(9, OUTPUT);
  LoRa.setPins(53, 9, 2);
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));  Serial.println("LoRa Sender");

  // lcd.begin(20,4);
  // lcd.setCursor(0,0);
  // lcd.print("Temperature: "); //13
  // lcd.setCursor(0,1);
  // lcd.print("Humidity: ");
  // lcd.setCursor(0,2);
  // lcd.print("CO: ");
  // lcd.setCursor(0,3);
  // lcd.print("Dust density: doing");

  while (!Serial);
  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  if (!LoRa.begin(433E6)) {
    //     if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    // {
    Serial.println("Starting LoRa failed!");
    // xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    // }
    while (1);
  }
  delay(1000);

  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.crc();

  xQueue = xQueueCreate( 5, sizeof( float ) );

  if ( xQueue != NULL ) {
    xTaskCreate(
      TaskMQ9
      ,  (const portCHAR *) "ReadMQ9"
      ,  200  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  NULL );

    xTaskCreate(
      TaskDHT11
      ,  (const portCHAR *)"ReadDHT11"  // A name just for humans
      ,  200  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );
    xTaskCreate(
      TaskDUST
      ,  (const portCHAR *) "ReadDustSensor"
      ,  1000  // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL );

    xTaskCreate(
      TaskLORA
      ,  (const portCHAR *) "Lora"
      ,  200  // Stack size
      ,  NULL
      ,  4  // Priority
      ,  NULL );
    // xTaskCreate(
    //     TaskLCD
    //     ,  (const portCHAR *) "DisplayLCD"
    //     ,  128  // Stack size
    //     ,  NULL
    //     ,  2  // Priority
    //     ,  NULL );

    // xTaskCreate(
    //   TaskDEBUG
    //   ,  (const portCHAR *) "Debug"
    //   ,  128  // Stack size
    //   ,  NULL
    //   ,  3  // Priority
    //   ,  NULL );

    // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
  }
  else
  {
    /* The queue could not be created. */
  }
}

void loop()
{
  // Empty. Things are done in Tasks.

}

/*---------------------- Tasks ---------------------*/
void TaskDHT11( void *pvParameters  )  // This is a Task.
{
  TickType_t xLastWakeTime;
  portBASE_TYPE xStatus;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    int chk = DHT.read(DHT11_PIN);    // READ DATA

    // lcd.setCursor(13,0);
    // lcd.print(DHT.temperature);
    // lcd.setCursor(13,1);
    // lcd.print(DHT.humidity);
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.print("DHT11, \tSTATUS: ");
      switch (chk) {
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
      Serial.print("xLastWakeTime DHT: ");
      Serial.println(xLastWakeTime);
      Serial.print(DHT.humidity);
      Serial.print(",\t");
      Serial.println(DHT.temperature);
      // Serial.println("FUCKING DHT_____");
      xStatus = xQueueSendToBack( xQueue, &DHT.humidity, 0 );
      xStatus = xQueueSendToBack( xQueue, &DHT.temperature, 0 );
      if ( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    // vTaskDelay( 10000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, 625 );
  }
}

void TaskMQ9( void *pvParameters  )  // This is a Task.
{
  TickType_t xLastWakeTime;
  portBASE_TYPE xStatus;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(A0);
    sensor_volt = (float)sensorValue / 1024 * 5.0;
    RS_gas = (5.0 - sensor_volt) / sensor_volt; // omit *RL

    // ratio = RS_gas/0.29;  // ratio = RS/R0 // Unline bro
    ratio = 99;
    /*-----------------------------------------------------------------------*/
    // lcd.setCursor(13,2);
    // lcd.print(ratio);
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      //    Serial.print("sensor_volt = ");
      //    Serial.println(sensor_volt);
      //    Serial.print("RS_ratio = ");
      //    Serial.println(RS_gas);
      // Serial.print("xLastWakeTime CO: ");
      // Serial.println(xLastWakeTime);
      Serial.print("CO = ");
      Serial.println(ratio);

      xStatus = xQueueSendToBack( xQueue, &ratio, 0 );
      // if( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    // vTaskDelay( 9000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, 625 );

  }
}

void TaskDUST( void *pvParameters )  // This is a Task.
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE xStatus;
  for (;;) {
    // vTaskDelay( 2000 / portTICK_PERIOD_MS );
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
    // #ifdef USE_AVG
    //   VoRawTotal += VoRaw;
    //   VoRawCount++;
    //   if ( VoRawCount >= N ) {
    //     Vo = 1.0 * VoRawTotal / N;
    //     VoRawCount = 0;
    //     VoRawTotal = 0;
    //   } else {
    //   return;
    //   }
    //  #endif // USE_AVG
    Vo = Vo / 1024.0 * 5.0; // VOTLS :))
    // float dV = Vo - Voc;
    //   if ( dV < 0 ) {
    //       dV = 0;
    //       Voc = Vo;
    //   }
    // float dustDensity = dV / K * 100.0;
    float dustDensity = 0.17 * Vo - 0.1;
    if ( dustDensity < 0)
    {
      dustDensity = 0.00;
    }

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("Dust density: ");
      Serial.print(dustDensity);
      Serial.println(" ug/m3");

      xStatus = xQueueSendToBack( xQueue, &dustDensity, 0 );
      // if( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    // vTaskDelay( 3000 / portTICK_PERIOD_MS );
    // vTaskDelayUntil( &xLastWakeTime, ( 12000 / portTICK_PERIOD_MS ) );
  }
}

void TaskDEBUG( void *pvParameters )  // This is a Task.
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println("DEBUG:)))");
      // Serial.print("Dust density: ");
      // Serial.println(" ug/m3");
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    // vTaskDelay( 2000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, ( 10000 / portTICK_PERIOD_MS ) );
  }
}

void TaskLORA( void *pvParameters)  // This is a Task.
{
  String dataPackage = "";
  float h, t, c, dust;
  char* arrData;
  portBASE_TYPE xStatus;
  // const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  // unsigned portBASE_TYPE check;
  for (;;) // A Task shall never return or exit.
  {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // xStatus = xQueueReceive( xQueue, &h, 0 );
      // if(xStatus == errQUEUE_FULL){
      //   Serial.println("DEBUG:))");
      //  }
      // check = uxQueueMEssagesWaiting (xQueue);

      // switch (uxQueueMessagesWaiting( xQueue )) {
      //   case 1:
      //     Serial.println("HERE 1");
      //     break;
      //   case 2:
      //     Serial.println("HERE 2");
      //     break;
      //   case 3:
      //     Serial.println("HERE 3");
      //     break;
      //   case 4:
      //     Serial.println("HERE 4");
      //     break;
      //   default: Serial.println("FULL");
      //     break;
      // }
      if ( uxQueueMessagesWaiting (xQueue) == 4 ) {
        // Serial.println("DEBUG:))");
        xStatus = xQueueReceive( xQueue, &h, 0 );
        xStatus = xQueueReceive( xQueue, &t, 0 );
        xStatus = xQueueReceive( xQueue, &c, 0 );
        xStatus = xQueueReceive( xQueue, &dust, 0 );
        Serial.println("_______________________________");
        Serial.print(h);
        Serial.print(t);
        Serial.print(c);
        Serial.println(dust);
        // sprintf(arrData, "%f\n%f\n%f\n%f\n", h, t, c, dust);
//        dataPackage = String(h) + "\n" + String(t) + "\n" + String(c) + "\n" + String(dust) + "\n";
//        Serial.println(dataPackage);
      
        // float x1 = 38;
        // float x2 = 40;
        // float x3 = 11.2;
        // float x4 = 300.03;
        // dataPackage = String(x1) + "\n" + String(x2) + "\n" + String(x3) + "\n" + String(x4) + "\n";
        LoRa.beginPacket();
        // LoRa.print("22\n33.3\n11.11\n300.03\n"); //For check Lora send
        LoRa.write((float)h);
        LoRa.endPacket();

      }

      // if( uxQueueMessagesWaiting( xQueue ) != 4 )
      // {
      //   Serial.println( "Queue should have been full!\r\n" );
      // }

      // xStatus = xQueueReceive( xQueue, &h, xTicksToWait );
      // xStatus = xQueueReceive( xQueue, &t, xTicksToWait );
      // xStatus = xQueueReceive( xQueue, &c, xTicksToWait );
      // xStatus = xQueueReceive( xQueue, &dust, xTicksToWait );
      // dataPackage = String(h) + "\n" + String(t) + "\n" + String(c) + "\n" + String(dust) + "\n";
      // Serial.println("_______________________________");
      // Serial.print(h);
      // Serial.print(t);
      // Serial.print(c);
      // Serial.println(dust);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.

    }
    vTaskDelay( 5000 / portTICK_PERIOD_MS );
  }
}


























































































































































