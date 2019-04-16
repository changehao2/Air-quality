#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>  
#include <dht11.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>

dht11 DHT;
#define DHT11_PIN 4
#define alarmPin 13
#define interruptPin 3
#define USE_AVG
#define sharpLEDPin  7   //sensor LED.
#define sharpVoPin   A5   //sensor Vo.
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0; 
static float Voc = 0.6;   
const float K = 0.5;

const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;


SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xBinarySemaphore;

QueueHandle_t xQueue;
QueueHandle_t xQueue2;


void TaskDHT11( void *pvParameters );
void TaskMQ9( void *pvParameters );
void TaskDUST( void *pvParameters );
void TaskLORA( void *pvParameters);
void TaskAlarm(void *pvParameters);  
// void TaskLCD( void *pvParameters );
void TaskDEBUG( void *pvParameters );

static void vHandlerTask( void *pvParameters );
static void vExampleInterruptHandler( void ); // ISR
boolean state = false;
void setup() {
  pinMode(sharpLEDPin, OUTPUT);

  Serial.begin(9600);
  pinMode(9, OUTPUT);
  LoRa.setPins(53, 9, 2);
  vSemaphoreCreateBinary( xBinarySemaphore );

  pinMode(alarmPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), vExampleInterruptHandler, CHANGE);  

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

  if ( xSerialSemaphore == NULL ) 
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  
      xSemaphoreGive( ( xSerialSemaphore ) ); 
  }

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  delay(1000);

  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.crc();

  xQueue = xQueueCreate( 3, sizeof( float ) );
  xQueue2 = xQueueCreate( 1, sizeof( float ) );

  if ( xQueue != NULL ) {
    xTaskCreate(
      TaskMQ9
      ,  (const portCHAR *) "ReadMQ9"
      ,  200  
      ,  NULL
      ,  1  
      ,  NULL );

    xTaskCreate(
      TaskDHT11
      ,  (const portCHAR *)"ReadDHT11"  
      ,  200  
      ,  NULL
      ,  2  
      ,  NULL );
    xTaskCreate(
      TaskDUST
      ,  (const portCHAR *) "ReadDustSensor"
      ,  1000  
      ,  NULL
      ,  3  
      ,  NULL );

    xTaskCreate(
      TaskLORA
      ,  (const portCHAR *) "Lora"
      ,  200  
      ,  NULL
      ,  4  
      ,  NULL );
//        xTaskCreate(
      // TaskAlarm
      // ,  (const portCHAR *) "Alarm"
      // ,  200  
      // ,  NULL
      // ,  6  
      // ,  NULL );

    xTaskCreate(  vHandlerTask, "Handler", 200, NULL, 5, NULL );
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

    }
  else
  {
    /* The queue could not be created. */
  }
}

void loop()
{
}


void TaskDHT11( void *pvParameters  )  
{
  TickType_t xLastWakeTime;
  portBASE_TYPE xStatus;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) 
  {
    int chk = DHT.read(DHT11_PIN);   

    // lcd.setCursor(13,0);
    // lcd.print(DHT.temperature);
    // lcd.setCursor(13,1);
    // lcd.print(DHT.humidity);
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // Serial.print("DHT11, \tSTATUS: ");
      switch (chk) {
        case DHTLIB_OK:
          Serial.println("DOING...\t");
          break;
        case DHTLIB_ERROR_CHECKSUM:
          Serial.println("Checksum error,\t");
          break;
        case DHTLIB_ERROR_TIMEOUT:
          Serial.println("Time out error,\t");
          break;
        default:
          Serial.println("Unknown error,\t");
          break;
      }
      // Serial.print("xLastWakeTime DHT: ");
      // Serial.println(xLastWakeTime);
      Serial.print("Humidity: ");
      Serial.println(DHT.humidity);
      Serial.print("Temperature: ");
      Serial.println(DHT.temperature);
      // Serial.println("FUCKING DHT_____");
      xStatus = xQueueSendToBack( xQueue, &DHT.humidity, 0 );
      xStatus = xQueueSendToBack( xQueue, &DHT.temperature, 0 );
      if ( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore ); 
    }

    // vTaskDelay( 10000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, 625 );
  }
}

void TaskMQ9( void *pvParameters  )  
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
    ratio = RS_gas/0.29;  // ratio = RS/R0 // Unline bro
    // ratio = 11.2;
    /*-----------------------------------------------------------------------*/
    // lcd.setCursor(13,2);
    // lcd.print(ratio);
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      //    Serial.print("sensor_volt = ");
      //    Serial.println(sensor_volt);
      //    Serial.print("RS_ratio = ");
      //    Serial.println(RS_gas);
      // Serial.print("xLastWakeTime CO: ");
      // Serial.println(xLastWakeTime);
      Serial.print("CO = ");
      Serial.print(ratio);
      Serial.print("\t");      
      Serial.println(sensor_volt);      

      xStatus = xQueueSendToBack( xQueue, &ratio, 0 );
      if( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore );
    }
    // vTaskDelay( 9000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, 625 );

  }
}

void TaskDUST( void *pvParameters )  
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE xStatus;
  for (;;) {
    // vTaskDelay( 2000 / portTICK_PERIOD_MS );
    digitalWrite(sharpLEDPin, LOW);
    vTaskDelay( 280 / portTICK_PERIOD_MS );
    int VoRaw = analogRead(sharpVoPin);
    digitalWrite(sharpLEDPin, HIGH);
    vTaskDelay( 9620 / portTICK_PERIOD_MS );
    float Vo = VoRaw;
    Vo = Vo / 1024.0 * 5.0; // VOTLS :))
    float dustDensity = 0.17 * Vo - 0.1;
    if ( dustDensity < 0)
    {
      dustDensity = 0.00;
    }

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("Dust density: ");
      Serial.print(dustDensity);
      Serial.println(" ug/m3");

      xStatus = xQueueSendToBack( xQueue2, &dustDensity, 0 );
      // if( xStatus != pdPASS ) Serial.println( "Could not send to the queue" );

      xSemaphoreGive( xSerialSemaphore );
    }
    // vTaskDelay( 3000 / portTICK_PERIOD_MS );
    // vTaskDelayUntil( &xLastWakeTime, ( 12000 / portTICK_PERIOD_MS ) );
  }
}

void TaskDEBUG( void *pvParameters )  
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) 
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println("DEBUG:)))");
      // Serial.print("Dust density: ");
      // Serial.println(" ug/m3");
      xSemaphoreGive( xSerialSemaphore ); 
    }
    // vTaskDelay( 2000 / portTICK_PERIOD_MS );
    vTaskDelayUntil( &xLastWakeTime, ( 10000 / portTICK_PERIOD_MS ) );
  }
}

void TaskLORA( void *pvParameters)  
{
  String dataPackage = "";
  float h, t, c, dust;
  char* arrData;
  portBASE_TYPE xStatus;
  // const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  // unsigned portBASE_TYPE check;
  for (;;) // A Task shall never return or exit.
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // xStatus = xQueueReceive( xQueue, &h, 0 );
      // if(xStatus == errQUEUE_FULL){
      //   Serial.println("DEBUG:))");
      //  }
      // check = uxQueueMEssagesWaiting (xQueue);

      // switch (uxQueueMessagesWaiting( xQueue )) { // For Debug
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
      if ( uxQueueMessagesWaiting (xQueue) == 3 && uxQueueMessagesWaiting (xQueue2) == 1) {
        // Serial.println("DEBUG:))");
        xStatus = xQueueReceive( xQueue, &h, 0 );
        LoRa.beginPacket();
        LoRa.print('1');
        LoRa.print('\n');
        LoRa.print(h);
        LoRa.print('\n');
        xStatus = xQueueReceive( xQueue, &t, 0 );
        LoRa.print(t);
        LoRa.print('\n');       
        xStatus = xQueueReceive( xQueue, &c, 0 );
        LoRa.print(c);
        LoRa.print('\n');       
        xStatus = xQueueReceive( xQueue2, &dust, 0 );
        LoRa.print(dust);
        LoRa.print('\n');
        LoRa.endPacket();        
  
        Serial.print("\nQueue received: ");
        Serial.print(h);
        Serial.print(t);
        Serial.print(c);
        Serial.println(dust);
        Serial.println("_______________________________");        
        // sprintf(arrData, "%f\n%f\n%f\n%f\n", h, t, c, dust);
//        dataPackage = String(h) + "\n" + String(t) + "\n" + String(c) + "\n" + String(dust) + "\n";
       // Serial.println(dataPackage);
      
        // float x1 = 38;
        // float x2 = 40;
        // float x3 = 11.2;
        // float x4 = 300.03;
        // dataPackage = String(x1) + "\n" + String(x2) + "\n" + String(x3) + "\n" + String(x4) + "\n";
//         LoRa.beginPacket();
//         LoRa.print("22\n33.3\n11.11\n300.03\n"); //For check Lora send
// //        LoRa.write((float)h);
//         LoRa.endPacket();

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
      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}

static void vHandlerTask( void *pvParameters )
{
  /* Note that when you create a binary semaphore in FreeRTOS, it is ready
  to be taken, so you may want to take the semaphore after you create it
  so that the task waiting on this semaphore will block until given by
  another task. */
  xSemaphoreTake( xBinarySemaphore, 0);

  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

    /* To get here the event must have occurred.  Process the event (in this
    case we just print out a message). */  
    // Serial.println( "Handler task - Processing event.\r\n" );
      // digitalWrite(alarmPin, state);
    if(state){
        LoRa.beginPacket();
        LoRa.print('0');
        LoRa.endPacket(); 
        state = false;
    }
  }
}

static void  vExampleInterruptHandler( void ){
  portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore to unblock the task. */
  xSemaphoreGiveFromISR( xBinarySemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  if( xHigherPriorityTaskWoken == pdTRUE )
  {
    /* Giving the semaphore unblocked a task, and the priority of the
    unblocked task is higher than the currently running task - force
    a context switch to ensure that the interrupt returns directly to
    the unblocked (higher priority) task.

    NOTE: The syntax for forcing a context switch is different depending
    on the port being used.  Refer to the examples for the port you are
    using for the correct method to use! */
    // portSWITCH_CONTEXT();
    // for (;;) {

    //   }
    // Serial.println("This is interrupt :)))");
    state = true;
    vPortYield();
  }
}

























































































































































































































