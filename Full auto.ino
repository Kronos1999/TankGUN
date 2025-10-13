void performAction1() {
  // Full auto sequence
  mot1.writeMicroseconds(1500);
  mot2.writeMicroseconds(1500);
  Serial.println("Motors running at half throttle.");
  loa.write(110); // Send payload
  delay(300);
  loa.write(30); // Return to home
  delay(350); // Additional delay between actions
}
