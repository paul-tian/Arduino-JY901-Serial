void setup() {
  while(!Serial) {
    ;
  }
  int16_t a = -648;
  uint8_t ar[4] = {0, 0, 0, 0};
 
  Serial.print("hex_a = ");
  Serial.println(a, HEX);
  Serial.print("a = ");
  Serial.println(a);
  memcpy(&ar[1], &a, 2);
  Serial.print("ar[0] = ");
  Serial.println(ar[0], HEX);
  Serial.print("ar[1] = ");
  Serial.println(ar[1], HEX);
  Serial.print("ar[2] = ");
  Serial.println(ar[2], HEX);
  Serial.print("ar[3] = ");
  Serial.println(ar[3], HEX);
}

void loop() {
  // put your main code here, to run repeatedly:

}
