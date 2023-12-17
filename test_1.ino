#include <Arduino.h>

int inputPins[] = {2, 3, 4, 5}; // Inputs
int outputPin = A3;
float referenceVoltage = 5.0; // Maximum voltage 5V
int potentiometerPin = A1; // Pin for changing frequency with a potentiometer
int Btn = 10; // Pin for button

int currentFunction = 0;


void inputs();
void frequency();
void sinus();
void triangle();
void root();

void setup() {
  // Set input pins
  for (int i = 0; i < 4; i++) {
    pinMode(inputPins[i], INPUT);
  }
  pinMode(outputPin, OUTPUT);
  pinMode(potentiometerPin, INPUT);
  pinMode(10, INPUT);
  Serial.begin(115200);
}

void loop() {
  int buttonState = digitalRead(Btn); // Read the state of the button connected to D10 pin

  if (buttonState == HIGH) { // If the button is pressed
    currentFunction++; // Increment the current function counter

    // Check the current function and perform the corresponding action
    switch (currentFunction % 3) {
      case 0:
        sinus();
        break;
      case 1:
        triangle();
        break;
      case 2:
        root();
        break;
    }

    // Wait until the button is released
    while (digitalRead(Btn) == HIGH) {
      delay(100);
    }

    delay(1000); // Add a small delay to debounce the button
  }
}



//KT, check what is happening in scheme.
void inputs(){
  int input = 0;

  // Read binary value from inputs
  while (input <= 15) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(inputPins[i], bitRead(input, i) ? HIGH : LOW);
    }

    // Calculate voltage based on binary value
    float voltage = referenceVoltage * input / 15.0;

    // Set output voltage
    if (voltage > referenceVoltage) {
      voltage = referenceVoltage;
    }
    analogWrite(outputPin, voltage / referenceVoltage * 255);

    // Print input voltage and output voltage
    Serial.print("Binary value: ");
    Serial.print(input, BIN);
    Serial.print("\\\\tDecimal value: ");
    Serial.print(input);
    Serial.print("\\\\tOutput voltage: ");
    Serial.print(voltage, 3);
    Serial.println();

    input++;
    delay(1000);
  }
}

//KT, check if frequency is changed.
void frequency () {
  // Read potentiometer value to get frequency
  int F = map(analogRead(potentiometerPin), 0, 1023, 20, 20000);  // Frequency range is 20 Hz to 20 kHz
  // Info
  Serial.print("Frequency: ");
  Serial.println(F);
  Serial.print(" Hz ");
  delay(1000);
}

void sinus() {
   int delay = 1000;

  while ( delay > 0 ){
    int F = map(analogRead(potentiometerPin), 0, 1023, 20, 20000);  // Frequency range is 20 Hz to 20 kHz
    int T = 1000000 / F; // Calculate period in microseconds

    for (int t = 0; t < T; t++) {
      float angle = 2 * PI * t / T; // Calculate angle
      float value = sin(angle); // Calculate sine value
      
      //y-telje sättimine
      Serial.print(1.1);
      Serial.print(", ");
      Serial.print(-1.1);
      Serial.print(", ");

      // Print value to serial plotter
      Serial.println(value);
    }
    // Check if button is pressed
    int buttonState = digitalRead(Btn);
    if (buttonState == HIGH) {
      break;
    }
  delay--;
  }
}

void triangle() {
  int delay = 1000;

  while ( delay > 0 ){
    int F = map(analogRead(potentiometerPin), 0, 1023, 20, 20000);  // Frequency range is 20 Hz to 20 kHz
    int T = 1000000 / F; // Calculate period in microseconds
    int halfT = T / 2; // Half period
    for (int t = 0; t < T; t++) {
      
      float value;

      if (t < halfT) {
        value = map(t, 0, halfT, 0, 15);
      } else {
        value = map(t, halfT, T, 15, 0);
      }

      //y-telje sättimine
      Serial.print(15.1);
      Serial.print(", ");
      Serial.print(-0.1);
      Serial.print(", ");

      // Print value to serial plotter
      Serial.println(value);
    }
    // Check if button is pressed
    int buttonState = digitalRead(Btn);
    if (buttonState == HIGH) {
      break;
    }
  delay--;
  }
}

void root() {
   int delay = 1000;

  while ( delay > 0 ){
    int F = map(analogRead(potentiometerPin), 0, 1023, 20, 20000);  // Frequency range is 20 Hz to 20 kHz
    int T = 1000000 / F; // Calculate period in microseconds
    int halfT = T / 2; // Half period

    for (int t = 0; t < T; t++) {
      float value;

      if (t < halfT) {
        value = 1;
      } else {
        value = 0;
      }

      //y-telje sättimine
      Serial.print(1.1);
      Serial.print(", ");
      Serial.print(0);
      Serial.print(", ");

      // Print value to serial plotter
      Serial.println(value);
    }
    // Check if button is pressed
    int buttonState = digitalRead(Btn);
    if (buttonState == HIGH) {
      break;
    }
  delay--;
  }
}
