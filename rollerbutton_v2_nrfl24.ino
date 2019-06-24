/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// define shift register
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_HIGH
//#define MY_RADIO_RFM69
//#define MY_IS_RFM69HWw
#define MY_REPEATER_NODE
#define MY_NODE_ID AUTO
//#define MY_NODE_ID 1

//BUGGY PCB (requires this workarround)
#define MY_SOFTSPI
#define MY_SOFT_SPI_SCK_PIN 10
#define MY_SOFT_SPI_MISO_PIN 11
#define MY_SOFT_SPI_MOSI_PIN 12
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 13

#include <SPI.h>
#include <MySensors.h> 
#include <avr/wdt.h>
#include <Bounce2.h>

// Enable debug prints to serial monitor

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//Sketch information
#define SKETCH_INFO         "Roller Switch Button"
#define SKETCH_VERSION      "1.3"
#define ROLLERL_ID_INFO     "Local Roller"
#define ROLLERL_ID 0
#define LPUP_ID_INFO        "Long Press Up"
#define LPUP_ID 1
#define LPDOWN_ID_INFO      "Long Press Down"
#define LPDOWN_ID 2
#define SCENE_ID 1
#define SCENE_ID_INFO       "Scene Controller"

#define SCENE_UP 1
#define SCENE_DOWN 2

#define LOCAL_ROLLER_ID 5
#define LOCAL_ROLLER_SENSOR_ID 0

//Auto-reset
#define MESSAGES_FAILED_REBOOT 20
#define CYCLES_REBOOT 2880

//Configurable ACK Timeout
#define ACK_TIMEOUT 1000

//BAUD RATE
#define BAUD_RATE 115200

//Switches definitions
#define SWITCH_CLOSED  LOW
#define SWITCH_OPEN HIGH
#define SWITCH1_PIN 3
#define SWITCH2_PIN 4
#define SWITCH_TIMEOUT 2000

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 20
#define LOW_PRIORITY_RETRIES 5

Bounce switch1 = Bounce(); 
Bounce switch2 = Bounce(); 

MyMessage message;

unsigned int messagesFailed=0;
volatile int isACKed = false;
volatile int gateway_request= -1;
unsigned int i;
int s1_state, s2_state;

boolean ack = true;
boolean metric, changed, double_press = false, s2_complete = false, s1_complete = false;

unsigned long currentMillis,loopMillis, s1_millis, s2_millis;

void before() {
  //disable watchdog timer
  MCUSR = 0;
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));
  
  //blink LED on power up
  pinMode(13,OUTPUT);
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }

  // PIN setup
  pinMode(SWITCH1_PIN,INPUT_PULLUP);
  pinMode(SWITCH2_PIN,INPUT_PULLUP);
  // Bounce instance
  switch1.attach(SWITCH1_PIN);
  switch1.interval(5); // interval in ms
  switch2.attach(SWITCH2_PIN);
  switch2.interval(5); // interval in ms
  
}

void setup()  
{
  Serial.println("");
  Serial.println(F("Listening for Commands"));
  
  metric = getConfig().isMetric;
  //activate watchdog timer
  wdt_enable(WDTO_8S);
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{ 
  if (millis() >= 86400000) {
    Serial.println(F("daily Reboot reached"));
    asm volatile ( "jmp 0");
    wait(100);
  }
  //
  //Process change for switch 1 (UP commands)
  //
  changed = switch1.update();
  // Get the update value
  s1_state = switch1.read();

  if (changed) {
    if (s1_state == SWITCH_CLOSED) {
      s1_millis = millis();
      double_press=false;
      Serial.println(F("Switch1 closed (UP)"));
    } else {  
      if (double_press == false) {
        //detect if it was a short press, local UP if so informs the roller node
        if (long (millis() - s1_millis) < SWITCH_TIMEOUT) {
          Serial.println(F("Switch1 short release (UP LOCAL)"));
          //switch was released, send command to roller
          resend(message.setType(V_UP).setSensor(LOCAL_ROLLER_SENSOR_ID).setDestination(LOCAL_ROLLER_ID).set(1),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
          //switch was released, send command to gateway
          resend(message.setType(V_UP).setSensor(ROLLERL_ID).setDestination(0).set(1),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
        }
      }
      s1_complete = false;       
    }
  } else {
    //detect if it is a long press and not changed, remote UP, if so informs the gateway
    if ((s1_state == SWITCH_CLOSED) && (long (millis() - s1_millis) > SWITCH_TIMEOUT) && (s1_complete == false) && (double_press == false)) {
      Serial.println(F("Switch1 Long release (UP REMOTE)"));
      //switch was released, send command to gateway
      resend(message.setType(V_SCENE_ON).setSensor(SCENE_ID).setDestination(0).set(SCENE_UP),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
      //reset long press timer
      s1_complete = true;    
    }
  }

  //
  //Process change for switch 2 (DOWN commands)
  //
  changed = switch2.update();
  // Get the update value
  s2_state = switch2.read();

  if (changed) {
    if (s2_state == SWITCH_CLOSED) {
      s2_millis = millis();
      double_press=false;
      Serial.println(F("Switch2 closed (DOWN)"));
    } else {
      if (double_press == false) {
        //detect if it was a short press, local DOWN if so informs the roller node
        if (long (millis() - s2_millis) < SWITCH_TIMEOUT) {  
          Serial.println(F("Switch2 Short release (DOWN LOCAL)"));
          //switch was released, send short key press
          resend(message.setType(V_DOWN).setSensor(LOCAL_ROLLER_SENSOR_ID).setDestination(LOCAL_ROLLER_ID).set(1),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
          //switch was released, send command to gateway
          resend(message.setType(V_DOWN).setSensor(ROLLERL_ID).setDestination(0).set(1),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
        }
      }
      s2_complete = false;
    }
  } else {
    //detect if it is a long press and not changed, remote DOWN, if so informs the gateway
    if ((s2_state == SWITCH_CLOSED) && (long (millis() - s2_millis) > SWITCH_TIMEOUT) && (s2_complete == false) && (double_press == false)) {
      Serial.println(F("Switch2 Long release (DOWN REMOTE)"));
      //switch was released, send command to gateway
      resend(message.setType(V_SCENE_ON).setSensor(SCENE_ID).setDestination(0).set(SCENE_DOWN),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
      //reset long press timer
      s2_complete = true;
    }
  }

  //If both UP and DOWN are pressed at the same time, STOP local ROLLER
  if ((s2_state == SWITCH_CLOSED) && (s1_state == SWITCH_CLOSED) && (double_press == false)) {
    Serial.println(F("Both Switch's pressed (STOP LOCAL)"));
    resend(message.setType(V_STOP).setSensor(LOCAL_ROLLER_SENSOR_ID).setDestination(LOCAL_ROLLER_ID).set(1),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
    double_press = true;
  }
  
  _process();
  wdt_reset();

  //new gateway request for local roller
  if (gateway_request != -1) {
    Serial.println(F("Sending gateway request to local roller"));
    resend(message.setType(gateway_request).setSensor(LOCAL_ROLLER_SENSOR_ID).setDestination(LOCAL_ROLLER_ID).set(1),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
    //reset gateway request
    gateway_request = -1;
  }
}

void gwPresent () {

  Serial.println(F("Presenting"));
 
  sendSketchInfo(SKETCH_INFO, SKETCH_VERSION);
  present(ROLLERL_ID, S_COVER, ROLLERL_ID_INFO);
  present(SCENE_ID, S_SCENE_CONTROLLER, SCENE_ID_INFO);
}

void resend(MyMessage &msg, int repeats, int timeout)
{
  int repeat = 0;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    send(msg,true);

    if (waitACK(timeout)) {
      sendOK = true;
      messagesFailed = 0;
    } else {
      sendOK = false;
      Serial.print("Retry ");
      Serial.print(repeat);
      Serial.print(" Failed ");
      Serial.println(messagesFailed);
      repeatdelay += 500;
      wdsleep(repeatdelay);
    }
    repeat++; 
  }
  if (sendOK == false) {
    if (messagesFailed > MESSAGES_FAILED_REBOOT) {
      //wdt_enable(WDTO_15MS);
      asm volatile ( "jmp 0");
      wait(100);
    }
    messagesFailed++;
  }
}

boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == true) {
      isACKed = false;
      Serial.print(F("Reply "));
      Serial.print(timeout);
      Serial.print(" ");
      Serial.println((millis() - startTime));
      return true;
    }
  }
  return false;
}

void receive (const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  if (message.isAck()) {
    Serial.println(F("This is an ack from gateway."));
    isACKed = true;
  } else if (message.sensor==ROLLERL_ID) {
    Serial.println(F("Command Received for local roller"));
    gateway_request=message.type;
  }  else {
    Serial.print(F("Incoming change for sensor:"));
    Serial.println(message.sensor);
  }
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
#if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
     _process();
    wdt_reset();
  }
    wdt_reset();
#else
  sleep(ms);
#endif
}

