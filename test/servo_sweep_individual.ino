Servo baseServo;     // A - IO16
Servo shoulderServo; // C - IO2  (boot strap pin)
Servo elbowServo;    // B - IO17
Servo clawServo;     // D - IO4

void setup() {
  delay(500); // let ESP32 finish booting (helps with GPIO2)
  baseServo.attach(16, 500, 2400);
  shoulderServo.attach(2, 500, 2400);
  elbowServo.attach(17, 500, 2400);
  clawServo.attach(4, 500, 2400);

  // Park neutral so first motion isn't a slam
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  clawServo.write(90);
  delay(500);
}

static void sweep(Servo &s, int lo=20, int hi=160, int step=5, int dt=20){
  for (int a=lo; a<=hi; a+=step){ s.write(a); delay(dt); }
  for (int a=hi; a>=lo; a-=step){ s.write(a); delay(dt); }
}

void loop() {
  sweep(baseServo);     delay(300);
  sweep(shoulderServo); delay(300);
  sweep(elbowServo);    delay(300);
  sweep(clawServo);     delay(600);
}


