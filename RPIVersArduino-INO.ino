void setup() {
  Serial2.begin(115200);
}

void loop() {
  if (Serial2.available() > 0) {
    String data = Serial2.readStringUntil('\n');
    Serial2.print("Reçu: ");
    Serial2.println(data);
  }
}