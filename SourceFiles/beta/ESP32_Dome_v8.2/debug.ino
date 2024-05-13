
void debugRoutines() {
#ifdef debugBody
// if (incomingPSI != 0 || incomingBTN != 0 || incomingBAT != 0.00) {
  Serial.print(F("incomingPSI: ")); Serial.print(incomingPSI); Serial.print('\t'); 
  Serial.print(F("incomingBTN: ")); Serial.print(incomingBTN); Serial.print('\t');
  // Serial.print(F("BTNstate: ")); Serial.print(BTNstate); Serial.print('\t');
  Serial.print(F("incomingBAT: ")); Serial.print(incomingBAT); Serial.println('\t'); 
  // Serial.print(F("Dome/BAT: ")); Serial.print(sendBAT); Serial.print('\t'); 
  // Serial.print(F("incomingPSI: ")); Serial.print(incomingPSI); Serial.println('\t');  
// }
#endif
}
