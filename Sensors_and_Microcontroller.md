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

## SEN-DHT22 Temperature Sensor 
| SEN-DHT22 Pin | Microcontroller Pin | Notes                                         |
| ------------- | ------------------- | --------------------------------------------- |
| VCC           | 5V                  | Power supply for sensor                       |
| GND           | GND                 | Common ground                                 |
| Data (Signal) | D2                  | Digital pin for temperature/humidity readings |

```cpp
// SEN-DHT22 Example for ARD R3 DIP Precision
#include "DHT.h"

#define DHTPIN 2        // Digital pin connected to the Data pin
#define DHTTYPE DHT22   // DHT 22 (AM2302) sensor type

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(2000); // Wait 2 seconds before next reading
}
```


SEN-DHT22 Temperature & Humidity Sensor Information

Sensor Type: Digital Temperature & Humidity Sensor

Function:
Measures ambient temperature and relative humidity
Converts physical values into digital signals

Used in:
Weather monitoring systems
Home automation
IoT temperature & humidity projects
Educational electronics experiments

Applications:
Environmental monitoring
Greenhouse climate control
Arduino-based weather stations
Laboratory experiments for temperature & humidity

Behavior when running the code:
The microcontroller requests data from the SEN-DHT22
Temperature and humidity readings are retrieved digitally
Values are printed to the Serial Monitor
These readings can be used to trigger devices or log environmental data



##  KY-039 Heartbeat Sensor Information

| KY-039 Pin       | Microcontroller Pin | Notes                         |
| ---------------- | ------------------- | ----------------------------- |
| VCC              | 5V                  | Power supply for sensor       |
| GND              | GND                 | Common ground                 |
| Signal (Digital) | D2                  | Digital pin to read heartbeat |

```cpp
// KY-039 Heartbeat Sensor Example for ARD R3 DIP Precision

int pulsePin = 2;     // Digital pin connected to KY-039 Signal
int ledPin = 13;      // Built-in LED to indicate heartbeat
int pulseState = 0;   // Variable to store current pulse reading

void setup() {
  pinMode(pulsePin, INPUT);   // Set pulse pin as input
  pinMode(ledPin, OUTPUT);    // Set LED pin as output
  Serial.begin(9600);         // Start serial communication
}

void loop() {
  pulseState = digitalRead(pulsePin); // Read heartbeat signal
  if (pulseState == HIGH) {
    digitalWrite(ledPin, HIGH);       // Turn LED on when pulse detected
    Serial.println("Heartbeat detected");
  } else {
    digitalWrite(ledPin, LOW);        // Turn LED off otherwise
  }
  delay(50);                           // Small delay for stability
}
```
KY-039 Heartbeat / Pulse Sensor Information

Sensor Type: Heartbeat / Pulse Sensor

Function:
Detects the heartbeat by measuring changes in blood flow
Converts heartbeat signals into digital pulses

Used in:
Heart rate monitoring devices
Fitness trackers
Arduino and IoT health projects
Educational electronics projects

Applications:
Health monitoring systems
Wearable electronics
Robotics and bio-feedback projects
Laboratory experiments for learning about heart rate

Behavior when running the code:

The microcontroller reads digital pulses from the KY-039
Each pulse corresponds to a heartbeat
The code counts pulses per minute to calculate heart rate
Values are printed to the Serial Monitor
LEDs or displays can be triggered to indicate heartbeat

## KY-037 Sound Sensor
| KY-037 Pin          | Microcontroller Pin | Notes                                           |
| ------------------- | ------------------- | ----------------------------------------------- |
| VCC                 | 5V                  | Power supply for sensor                         |
| GND                 | GND                 | Common ground                                   |
| Analog Output (A0)  | A0                  | Analog pin to read sound level                  |
| Digital Output (D2) | D2                  | Optional digital output for threshold detection |

```cpp
// KY-037 Sound Sensor Example for ARD R3 DIP Precision

int analogPin = A0;   // Analog pin connected to KY-037
int digitalPin = 2;   // Digital pin for threshold output
int soundValue = 0;   // Variable to store analog reading

void setup() {
  pinMode(digitalPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  soundValue = analogRead(analogPin); // Read analog sound intensity
  Serial.print("Sound Level: ");
  Serial.println(soundValue);

  int threshold = digitalRead(digitalPin); // Read digital threshold
  if(threshold == HIGH){
    Serial.println("Sound threshold exceeded!");
  }

  delay(500); // Wait half a second before next reading
}
```


KY-037 Sound Sensor Module Information

Sensor Type: Sound / Microphone Sensor

Function:
Detects sound intensity in the environment
Converts sound vibrations into analog or digital signals

Used in:
Sound-activated alarms
Noise monitoring systems
Arduino projects that respond to claps or noise

Applications:
Home security systems
Interactive electronics projects
Robotics for sound-based triggers
Educational electronics experiments

Behavior when running the code:
The microcontroller reads the analog voltage from KY-037
Higher sound intensity produces higher analog values
Digital output goes HIGH when sound exceeds a certain threshold
Values are printed to the Serial Monitor
Can be used to trigger LEDs, buzzers, or other output devices

## KY-038 Sound Sensor Module
| KY-038 Pin          | Microcontroller Pin | Notes                                 |
| ------------------- | ------------------- | ------------------------------------- |
| VCC                 | 5V                  | Power supply for sensor               |
| GND                 | GND                 | Common ground                         |
| Analog Output (A0)  | A0                  | Reads sound intensity as analog value |
| Digital Output (D2) | D2                  | HIGH when sound exceeds set threshold |

```cpp
// KY-038 Sound Sensor Example for ARD R3 DIP Precision

int analogPin = A0;   // Analog output from KY-038
int digitalPin = 2;   // Digital threshold output
int soundValue;

void setup() {
  pinMode(digitalPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  soundValue = analogRead(analogPin);   // Read sound intensity
  Serial.print("Sound Level: ");
  Serial.println(soundValue);

  if (digitalRead(digitalPin) == HIGH) {
    Serial.println("Sound threshold exceeded");
  }

  delay(500);
}
```
KY-038 Sound Sensor Module Information

Sensor Type: Sound / Microphone Sensor

Function:
Detects sound levels using a microphone
Provides both analog and digital sound output

Used in:
Noise detection systems
Sound-activated switches
Arduino sound-based projects

Applications:
Noise level monitoring
Clap-controlled lights
Security and alert systems
Educational electronics experiments

Behavior when running the code:
The microcontroller continuously reads sound intensity
Analog values increase as sound becomes louder
Digital output turns HIGH when sound crosses a preset threshold
Readings are displayed on the Serial Monitor
The sensor can be used to trigger LEDs, alarms, or relays

## HW-489 Infrared Emission Sensor
| HW-489 Pin | Microcontroller Pin | Notes                              |
| ---------- | ------------------- | ---------------------------------- |
| VCC        | 5V                  | Power supply for IR emitter        |
| GND        | GND                 | Common ground                      |
| Signal     | D3                  | Digital pin to control IR emission |

```cpp
// HW-489 Infrared Emitter Example for ARD R3 DIP Precision

int irPin = 3;   // Digital pin connected to HW-489 Signal

void setup() {
  pinMode(irPin, OUTPUT);
}

void loop() {
  digitalWrite(irPin, HIGH); // Turn IR emitter ON
  delay(1000);

  digitalWrite(irPin, LOW);  // Turn IR emitter OFF
  delay(1000);
}
```
HW-489 Infrared Emission Sensor Information

Sensor Type: Infrared (IR) Emitter

Function:
Emits infrared light when powered
Used to transmit IR signals to receivers

Used in:
Remote control systems
Infrared communication projects
Object detection systems (with IR receiver)

Applications:
TV and appliance remote controls
IR-based communication systems
Obstacle detection when paired with IR receivers
Educational Arduino projects

Behavior when running the code:
The microcontroller sends HIGH or LOW signals to the HW-489
When the signal pin is HIGH, the IR LED emits infrared light
When the signal pin is LOW, IR emission stops
The emitted IR light can be detected by an IR receiver
Emission can be controlled using delays or signal patterns

## HW-040 Rotary Encoder Module
| HW-040 Pin | Microcontroller Pin | Notes                           |
| ---------- | ------------------- | ------------------------------- |
| VCC        | 5V                  | Power supply for rotary encoder |
| GND        | GND                 | Common ground                   |
| CLK        | D2                  | Clock signal output             |
| DT         | D3                  | Data signal output              |
| SW         | D4                  | Push-button switch output       |

```cpp
// HW-040 Rotary Encoder Example for ARD R3 DIP Precision

int clkPin = 2;   // CLK pin
int dtPin  = 3;   // DT pin
int swPin  = 4;   // Switch pin

int lastClkState;
int counter = 0;

void setup() {
  pinMode(clkPin, INPUT);
  pinMode(dtPin, INPUT);
  pinMode(swPin, INPUT_PULLUP);

  Serial.begin(9600);
  lastClkState = digitalRead(clkPin);
}

void loop() {
  int currentClkState = digitalRead(clkPin);

  if (currentClkState != lastClkState) {
    if (digitalRead(dtPin) != currentClkState) {
      counter++;
    } else {
      counter--;
    }
    Serial.print("Encoder Value: ");
    Serial.println(counter);
  }

  lastClkState = currentClkState;

  if (digitalRead(swPin) == LOW) {
    Serial.println("Button Pressed");
    delay(300);
  }
}

HW-040 Rotary Encoder Module Information

Module Type: Rotary Encoder with Push Button

Function:
Detects rotational movement and direction
Provides digital pulses as the knob is rotated

Used in:
Volume control systems
Menu navigation interfaces
Position and speed detection projects

Applications:
Brightness and volume control in electronics
User input devices for embedded systems
Arduino-based control panels
Educational projects for learning digital inputs

Behavior when running the code:
The microcontroller detects pulses from CLK and DT pins
Rotation direction is determined by comparing CLK and DT states
Each step of rotation changes a counter value
Pressing the knob triggers the switch input
Values are displayed on the Serial Monitor or used to control devices
```
