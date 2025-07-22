// For test
#include <Arduino.h>


void cliChangeDispenserVars()
{
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();                     

  if (line.startsWith("STATE")) {
    line = line.substring(5); line.trim();   
    
    if (isdigit(line[0])) {
      int v = line.toInt();
      if (v >= STARTUP && v <= SUPPLY) {
        dispenserState = static_cast<STATE>(v);
        Serial.print(F("dispenserState = "));
        Serial.println(dispenserState);
      } else {
        Serial.println(F("State must be 0-9"));
      }
      return;
    }

    // symbolic?
    if      (line == "STARTUP")        dispenserState = STARTUP;
    else if (line == "IDLE")           dispenserState = IDLE;
    else if (line == "FILLING_START")  dispenserState = FILLING_START;
    else if (line == "PRESSURE_SPIKE") dispenserState = PRESSURE_SPIKE;
    else if (line == "FILLING")        dispenserState = FILLING;
    else if (line == "LEAK_CHECK")     dispenserState = LEAK_CHECK;
    else if (line == "VENTING")        dispenserState = VENTING;
    else if (line == "ABORT")          dispenserState = ABORT;
    else if (line == "OFFTAKE_250")    dispenserState = OFFTAKE_250;
    else if (line == "SUPPLY")         dispenserState = SUPPLY;
    else { Serial.println(F("Unknown state")); return; }

    Serial.print(F("dispenserState = "));
    Serial.println(dispenserState);
    return;
  }

  Serial.println(F("Use  state <value>  or  mode <value>"));
}