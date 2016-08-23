/*************************************
* radio : turnigy 9x
*
* Roll     : ch1 -> pin 13
* Pitch    : ch2 -> pin 12
* Throttle : ch3 -> pin 11
* Yaw      : ch4 -> pin 10
*
**************************************/

void init_radio()
{
  pinMode(13, INPUT); // Set our input pins as such
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
}

void read_radio()
{
  int ch1 = pulseIn(13, HIGH, 25000);
  radio_roll = map(ch1, 906, 1620, -500, 500); //center over zero
  radio_roll = constrain(radio_roll, -250, 250); //only pass values whose absolutes are
                                                 //valid pwm values
  int ch2 = pulseIn(12, HIGH, 25000);
  radio_pitch = map(ch2, 906, 1620, -500, 500);
  radio_pitch = constrain(radio_pitch, -250, 250);
  
  int ch3 = pulseIn(11, HIGH, 25000);
  radio_throttle = map(ch3, 906, 1620, 0, 600);
  
  int ch4 = pulseIn(10, HIGH, 25000);
  radio_yaw = map(ch4, 906, 1620, -500, 500);
  radio_yaw = constrain(radio_yaw, -250, 250);
}

void output_radio()
{
  Serial.print("pitch : "); // Print the value of 
  Serial.println(radio_pitch);        // each channel

  Serial.print("roll : ");
  Serial.println(radio_roll);

  Serial.print("throttle : ");
  Serial.println(radio_throttle);
  
  Serial.print("yaw : ");
  Serial.println(radio_yaw);
}
