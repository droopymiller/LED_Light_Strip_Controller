#include <inttypes.h>

//Define Input Pins
#define brightness A0
#define R_IN A1
#define G_IN A2
#define B_IN A3

//Define Output Pins
#define R_OUT 3
#define G_OUT 5
#define B_OUT 6
#define MAX_VAL 1046529 // Max value of two analogReads multiplied (1023*1023)ph

#define DEBUG_ENABLE 0 // 0 for disabled, 1 for enabled


int32_t RED_VAL = 0;
int32_t BLUE_VAL = 0;
int32_t GREEN_VAL = 0;
int16_t BRIGHT_VAL = 0;


int32_t RED_IN;
int32_t GREEN_IN;
int32_t BLUE_IN;

uint8_t RED_OUT;
uint8_t GREEN_OUT;
uint8_t BLUE_OUT;

void setup() {
  //Set Input Pinmodes
  pinMode(brightness, INPUT);
  pinMode(R_IN, INPUT);
  pinMode(G_IN, INPUT);
  pinMode(B_IN, INPUT);

  //Set Output Pinmodes
  pinMode(R_OUT, OUTPUT);
  pinMode(G_OUT, OUTPUT);
  pinMode(B_OUT, OUTPUT);
  // Set up timer0
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  OCR0A = 0;
  OCR0B = 0;

  // Set up timer2
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2B = 0;

  if (DEBUG_ENABLE) {
    Serial.begin(9600);
  }

}

void loop() {

  RED_IN = analogRead(R_IN);
  GREEN_IN = analogRead(G_IN);
  BLUE_IN = analogRead(B_IN);

  BRIGHT_VAL = analogRead(brightness);
  RED_VAL = RED_IN * BRIGHT_VAL;
  GREEN_VAL = GREEN_IN * BRIGHT_VAL;
  BLUE_VAL = BLUE_IN * BRIGHT_VAL;

  RED_OUT = map(RED_VAL, 0, MAX_VAL, 0, 255);
  GREEN_OUT = map(GREEN_VAL, 0, MAX_VAL, 0, 255);
  BLUE_OUT = map(BLUE_VAL, 0, MAX_VAL, 0, 255);

  if (DEBUG_ENABLE) {
    debug_interface();
  }

  analogWrite(R_OUT, RED_OUT);
  // OCR2B = RED_OUT;
  analogWrite(G_OUT, GREEN_OUT);
  // OCR0B = GREEN_OUT; // Set green value
  analogWrite(B_OUT, BLUE_OUT);
  // OCR0A = BLUE_OUT; // Set blue value
  delay(50);
}

void debug_interface() {
  Serial.print("Brightness: ");
  Serial.println(BRIGHT_VAL);
  Serial.println();

  Serial.print("Red Analog Read: ");
  Serial.println(RED_IN);
  Serial.print("Red Brightness: ");
  Serial.println(RED_VAL);
  Serial.print("Red Output (Mapped)");
  Serial.println(RED_OUT);
  Serial.println();

  Serial.print("Green Analog Read: ");
  Serial.println(GREEN_IN);
  Serial.print("Green Brightness: ");
  Serial.println(GREEN_VAL);
  Serial.print("Green Output (Mapped)");
  Serial.println(GREEN_OUT);
  Serial.println();

  Serial.print("Blue Analog Read: ");
  Serial.println(BLUE_IN);
  Serial.print("Blue Brightness: ");
  Serial.println(BLUE_VAL);
  Serial.print("Blue Output (Mapped)");
  Serial.println(BLUE_OUT);

  Serial.println();
  Serial.println();
}
