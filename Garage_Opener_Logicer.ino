// Libraries
#include "Arduino.h"
#include "Button.h"

//Definitions
const int trigPin = 4;
const int echoPin = 5;
const int lightPin = A0;

long duration;
float distanceCm;

#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701
#define MICROSWITCH_PIN_COM	0
#define LDR_PIN_SIG	A0

Button microSwitch(MICROSWITCH_PIN_COM);

int LDRValue = 0;
bool car_inside = false;


void setup() {

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(lightPin, INPUT);
  microSwitch.begin();
  Serial.begin(9600);

}


void loop() {


//Light Sensor Read
  LDRValue = analogRead(lightPin);
  Serial.print("Light Value: ");
  Serial.println(LDRValue);

//Distance Sensor Ping
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
 
  // Calculate the distance
  distanceCm = duration * SOUND_VELOCITY/2;
  
  // Prints the distance on the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);

  delay(1000);

  if (distanceCm == 0.00) {
    Serial.println("Bilen är hemma!");
    }



//microSwitch Function (klistrat från Button.h-exempel)
  if (microSwitch.toggled()) {
      if (microSwitch.read() == Button::PRESSED)
          Serial.println("Garage Open");

      else
          Serial.println("Garage Closed");
    }
}
