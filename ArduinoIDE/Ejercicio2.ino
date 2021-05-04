#define LED_PIN 14
#define BUTTON_PIN 2 // The number of the pushbutton pin (2-> digital pin)
#define ECHO_PIN 12 // Analog input that receives the echo signal
#define TRIG_PIN 13 // Digital output that sends the trigger signal

#define DARKNESS_RES    1000  // Resistance in darkness in KΩ
#define BRIGHTNESS_RES  15    // Resistance in brightness (10 Lux) in KΩ
#define CALIBRARION_RES 10    // Calibration resistance in KΩ
#define LDR_PIN         33    // LDR Pin
#define BUZZER_PIN      2

#define ANALOG_BIT_RESOLUTION 12.0

int voltage_measure;
int lux;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.println("\nBooting device...");
    
    pinMode(LED_PIN, OUTPUT); // Pinout as output
    pinMode(BUTTON_PIN, INPUT); // Initialize the button pin as an input
    //pinMode(MOTION_PIN, INPUT); // Initialize the button pin as an input
    pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output

    analogReadResolution(ANALOG_BIT_RESOLUTION);  // Sets the reading resolution value to 12 bits (0-4095)

    pinMode(BUZZER_PIN, OUTPUT); // Pinout as output
}

void loop(){
  //Ejercicio2_1();
  //Ejercicio2_2();
  //Ejercicio2_3();
  Ejercicio2_4();
  }

void Ejercicio2_1() {
  // put your main code here, to run repeatedly:
  static int buttonState; // Variable for reading the pushbutton status

  buttonState = digitalRead(BUTTON_PIN); // Read the state of the button value
  // Show the state of button on serial monitor
  if (buttonState == HIGH) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Button is pressed and led on");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("Button is not pressed and led off");
  }
  delay(100);// Check the button every 1000 miliseconds
}

void Ejercicio2_3(){
  static float distance;

  distance = getDistance();
  Serial.println("Distance to the object: " + String(distance) + " cm");
  LedOnIfPresence(distance);
  
  delay(100); // Check the disntace every 1000 miliseconds
}

/* Additional functions */
float getDistance() {
  digitalWrite(TRIG_PIN, LOW); // Clear the TRIG_PIN by setting it LOW
  delayMicroseconds(5);

  // Trigger the sensor by setting the TRIG_PIN to HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH); // pulseIn() returns the duration (length of the pulse) in microseconds

  return duration * 0.034 / 2; // Returns the distance in cm
  }

void LedOnIfPresence(float distance) {
  if (distance < 10.0) {
    digitalWrite(LED_PIN, HIGH);
    }
  else {
    digitalWrite(LED_PIN, LOW);
    }
  }


void Ejercicio2_4(){
  voltage_measure = analogRead(LDR_PIN);  // Reads the value from the pin in a 0-4095 resolution corresponding to a linear 0-3.3V

  lux = voltage_measure * DARKNESS_RES * 10 / (BRIGHTNESS_RES * CALIBRARION_RES * (pow(2.0, ANALOG_BIT_RESOLUTION) - voltage_measure)); // Use with LDR & Vcc

  Serial.println("Light intensity: " + String(lux) + " lux");

  if (lux > 15) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  
  delay(1000);  // Check the light every 1000 miliseconds
  }
