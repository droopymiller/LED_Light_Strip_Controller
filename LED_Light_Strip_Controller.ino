//Define Input Pins
#define brightness A0
#define R_IN A1
#define G_IN A2
#define B_IN A3

//Define Output Pins
#define R_OUT 3
#define G_OUT 5
#define B_OUT 6

//
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
  
}

void loop() {
  analogWrite(R_OUT, map(analogRead(R_IN)*analogRead(brightness), 0, 1023*1023, 0, 255));
  analogWrite(G_OUT, map(analogRead(G_IN)*analogRead(brightness), 0, 1023*1023, 0, 255));
  analogWrite(B_OUT, map(analogRead(B_IN)*analogRead(brightness), 0, 1023*1023, 0, 255));
}
