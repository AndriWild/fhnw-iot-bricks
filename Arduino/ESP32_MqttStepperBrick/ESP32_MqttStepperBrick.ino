#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <AccelStepper.h>
#include <math.h>

#define STEPS               516
#define MAX_DEGREE          360 
#define MQTT_CONN_KEEPALIVE 30
#define DEFAULT_SPEED       60 
#define MAX_SPEED           1000 
#define MAX_ACCEL           5000.0 

const char *ssid           = "MY_SSID"; // TODO
const char *password       = ""; // TODO
const char *host           = "test.mosquitto.org"; // TODO
const char *topicStrTarget = "bricks/0000-0008/target"; // TODO
const char *topicStrActual = "bricks/0000-0008/actual"; // TODO

const int port     = 8883;

WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, host, port);
Adafruit_MQTT_Subscribe topic(&mqtt, topicStrTarget);

AccelStepper stepper (AccelStepper::FULL4WIRE, 32, 14, 22, 23);

int standbyPin   = 26;
int sensorPin    = 4;
int allSteps     = 0;
int stepperAngle = 0;
double error     = 0;

void connectWifi(){
    // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  delay(1000);

  WiFi.begin(ssid, password);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  client.setInsecure();
}

void setup() {
  Serial.begin(9600);

  pinMode(sensorPin,  INPUT);
  pinMode(standbyPin, OUTPUT);

  stepper.setEnablePin(standbyPin);
  stepper.setMaxSpeed       (MAX_SPEED);
  stepper.setAcceleration   (MAX_ACCEL);
  stepper.setCurrentPosition(0);

  connectWifi();

  while (!Serial);
  Serial.println("Stepper test!");

  zeroing();
  Serial.println("Zeroing done.");

  topic.setCallback(handleMqttMessage);
  mqtt.subscribe(&topic);
  MQTT_connect();
}

void loop() {
  if (mqtt.connected()) {
    //Serial.println("MQTT: Connected");
    mqtt.processPackets(10000); // ms, calls callbacks
    if (!mqtt.ping()) {
      mqtt.disconnect();  
    }
  } else {
    int result = mqtt.connect(); // calls client.connect()
    if (result != 0) {
      Serial.println(mqtt.connectErrorString(result));
      delay(3000);
    }
  }
}

void zeroing(){
  stepper.enableOutputs();
  // running to zero point
  stepper.setSpeed(400);
  while(digitalRead(sensorPin)) { stepper.runSpeed(); }
  stepper.stop();

  // move back after detecting zero point
  stepper.move(-20);
  stepper.runToPosition();

  // running slowly to zero
  stepper.setSpeed(30);
  while(digitalRead(sensorPin)) { stepper.runSpeed(); }
  // zero position found
  stepper.stop();

  stepper.setCurrentPosition(0);
  stepper.setSpeed(DEFAULT_SPEED);
  stepper.move(STEPS/2);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);
  stepperAngle = 0;
  stepper.disableOutputs();
}

void setStepperPosition(int targetAngle){
  
    // angel to move to reach the new target
    int deltaAngle = shortestDistance(targetAngle, stepperAngle);

    // steps to move deltaAngle
    double deltaSteps = ((STEPS / 360.0) *  deltaAngle);

    // effective steps to move by adding error of previous calculations
    int stepsToMove = deltaSteps + error;

    // calculation of the data type convertion error
    error = error + deltaSteps - stepsToMove;

    stepper.move(stepsToMove);
    stepper.runToPosition();
    stepperAngle = targetAngle;
    delay(100);

    float batt = 3.7; // V, TODO
    int b = batt * 100.0f;

    uint8_t payload[] = {
      highByte(b), lowByte(b),
      highByte(targetAngle), lowByte(targetAngle)
    };
    printf("batt = %.2f, newAngle= %d\n", batt, targetAngle);
    printf("publish to %s\n", topicStrTarget);
    mqtt.publish(topicStrActual, payload, sizeof(payload));
}

void handleMqttMessage(char *buf, uint16_t len) {
  if (len == 2) {
    int pos = (int) ((buf[0] << 8) | buf[1]);
    stepper.enableOutputs();
    setStepperPosition(pos);
    stepper.disableOutputs(); // save power by turning off the motor coils
  }
}

int shortestDistance(int target, int stepperAngle){
  // without passing zero
  int delta = target - stepperAngle;
  // passing zero ACW
  int pZeroACW = delta - 360;
  // passing zero CW
  int pZeroCW  = delta + 360;

  int curShortest = abs(delta);

  if(abs(pZeroACW) < curShortest){
    curShortest = abs(pZeroACW);
    delta = pZeroACW;
  }

  if(abs(pZeroCW) < curShortest){
    curShortest = abs(pZeroCW);
    delta = pZeroCW;
  }
  return delta;
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
