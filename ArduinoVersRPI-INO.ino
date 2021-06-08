void setup() {
  Serial2.begin(115200);
}
void loop() {
  Serial2.println("Serial2");
  delay(1000);
}