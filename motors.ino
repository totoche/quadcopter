char motors[4] = {6, 5, 4, 3};

void pulsout (int pin, int duration) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(duration);
  digitalWrite(pin, LOW);
}

void init_motors()
{
  for(int i = 0 ; i < 4; i++) {
    pinMode(motors[i], OUTPUT);
    pulsout(motors[i] ,2000);
    speed_motors[i] = 0;
    
  }

  delay(5000);
}

void output_motors()
{  
  for (int i = 0 ; i < 4; i++)
  {
    Serial.print(i); // Print the value of 
    Serial.print(" : ");
    Serial.println(speed_motors[i]);        // each channel
  }
}


void run_motors()
{
  for(int i = 0 ; i < 4 ; i++)
  {
    pulsout(motors[i], map(speed_motors[i], 0, 700, 1000, 2000)); // full is 1600 not 1400
  }
}
