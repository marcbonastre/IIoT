#define LED_PIN 25
#define LED_RED 12
#define LED_BLUE 13
#define BUZZER 14

int ledsEj1_2[3] = {0,0,0};

void setup() {
  Serial.begin(9600);
  Serial.println("\nBooting device...");

  pinMode(LED_PIN, OUTPUT); // Pinout as output
  pinMode(LED_RED, OUTPUT); // Pinout as output
  pinMode(LED_BLUE, OUTPUT); // Pinout as output
  pinMode(BUZZER, OUTPUT); // Pinout as output
  
}

void loop() {
  //loop_ejercicio1_1();
  //loop_ejercicio1_2();
  loop_ejercicio1_3();
  }

void loop_ejercicio1_1() {
  setColor("red");
  delay(1000);
  setColor("green");
  delay(1000);
  setColor("blue");
  delay(1000);
}

void loop_ejercicio1_2(){
  for (int i = 0; i < 3; i++) {
      if (ledsEj1_2[i] == 0) {
        ledsEj1_2[i] = 1;
        setRGB(ledsEj1_2[0], ledsEj1_2[1], ledsEj1_2[2]);
        delay(1000);
      } else {
        ledsEj1_2[i] = 0;
        setRGB(ledsEj1_2[0], ledsEj1_2[1], ledsEj1_2[2]);
        delay(1000);
      }
    }
  }

void loop_ejercicio1_3(){
  loop_ejercicio1_1();
  loop_ejercicio1_1();
  delay(1000);
  loop_ejercicio1_2();
  loop_ejercicio1_2();
  digitalWrite(BUZZER, HIGH);
  delay(1000);
  digitalWrite(BUZZER, LOW);
  }

/* Additional functions */
void setRGB(int R, int G, int B) {
  digitalWrite(LED_PIN, R);
  digitalWrite(LED_RED, G);
  digitalWrite(LED_BLUE, B);
}

void setColor(const char* color) {
  if (color == "red") {
    setRGB(1, 0, 0);
  } else if (color == "green") {
    setRGB(0, 1, 0);
  } else if (color == "blue") {
    setRGB(0, 0, 1);
  }
}
