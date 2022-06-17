#define PWM1 9
#define PWM2 3
#define M1CW 11
#define M1CCW 12
#define M2CW 14
#define M2CCW 15

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(M1CW, OUTPUT);
  pinMode(M1CCW, OUTPUT);
  pinMode(M2CW, OUTPUT);
  pinMode(M2CCW, OUTPUT);  
  //Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(M1CW, HIGH);
digitalWrite(M1CCW, LOW);
analogWrite(PWM1, 100);
digitalWrite(M2CW, HIGH);
digitalWrite(M2CCW, LOW);
analogWrite(PWM2, 100);

}
