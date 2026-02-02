# Sensors and Microcontroller Information

## Microcontroller: ARD R3 DIP Precision

| Component                  | Purpose / Function                 |
|----------------------------|----------------------------------|
| Digital Pins (D0-D13)      | Read/write digital signals       |
| Analog Pins (A0-A5)        | Read analog signals              |
| 5V Pin                      | Power supply output              |
| GND Pin                     | Common ground reference          |
| VCC Pin                     | Power supply input for modules  |
| PWM Pins (D3, D5, D6, etc.)| Generate PWM signals             |
| Serial Pins (TX/RX)         | Serial communication interface  |

## Basic Blink Example for ARD R3 DIP Precision

```cpp
// Basic Blink Example for ARD R3 DIP Precision

// Pin where the LED is connected
int ledPin = 13; // Most Arduino boards have a built-in LED on pin 13

void setup() {
  // Initialize the digital pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);   // Turn the LED on
  delay(1000);                  // Wait for 1 second (1000 milliseconds)
  digitalWrite(ledPin, LOW);    // Turn the LED off
  delay(1000);                  // Wait for 1 second
}
```

## KY-018 Sensor Information
| KY-018 Pin         | Microcontroller Pin | Notes                        |
| ------------------ | ------------------- | ---------------------------- |
| VCC                | 5V                  | Power supply for sensor      |
| GND                | GND                 | Common ground                |
| Analog Output (A0) | A0                  | Reads analog light intensity |


```cpp
// KY-018 Photoresistor Example for ARD R3 DIP Precision

int photoPin = A0;  // Analog pin connected to KY-018
int sensorValue = 0; // Variable to store sensor reading

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop() {
  sensorValue = analogRead(photoPin); // Read the analog value from KY-018
  Serial.print("Light Intensity: ");
  Serial.println(sensorValue);        // Print the value to Serial Monitor
  delay(500);                         // Wait half a second before next reading
}
```

Sensor Type: Light Sensor (Photoresistor)

Function:
Detects the intensity of ambient light
Converts light levels into analog voltage

Used in:
Automatic street lights
Light-activated alarms
Brightness control for electronic devices
Arduino and IoT light sensing projects

Applications:
Home automation systems
Robotics for light-following or light-sensitive tasks
Environmental monitoring
Educational electronics projects

Behavior when running the code:
The microcontroller reads the voltage from the KY-018
Higher light intensity produces higher analog values
Values are printed to the Serial Monitor
LEDs or other devices can be controlled based on these values

##  KY-039 Heartbeat Sensor Information

| KY-039 Pin       | Microcontroller Pin | Notes                         |
| ---------------- | ------------------- | ----------------------------- |
| VCC              | 5V                  | Power supply for sensor       |
| GND              | GND                 | Common ground                 |
| Signal (Digital) | D2                  | Digital pin to read heartbeat |






