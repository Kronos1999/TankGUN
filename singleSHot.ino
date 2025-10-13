void performAction3() {
  // One-shot sequence
  mot1.writeMicroseconds(1500);
  mot2.writeMicroseconds(1500);
  delay(1000); // Hold for 1 second
  loa.write(110); // Send payload
  delay(300);
  loa.write(30); // Return to home
  for (int i = 1500; i >= 1000; i -= 10) {
    mot1.writeMicroseconds(i);
    mot2.writeMicroseconds(i);
    delay(20); // Small delay for smooth transition
  }
  Serial.println("One-shot sequence completed.");
}
