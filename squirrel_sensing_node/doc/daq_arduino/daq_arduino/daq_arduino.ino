
void setup(){
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  //led on: not connected
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(38400); //initializes the serial port with the pc
  
  while(!Serial);  //get stuck until you are connected to PC
  
  //led off: connected
  digitalWrite(LED_BUILTIN, LOW);
}


void loop(){
  
//as arduino sdk stripped out proper interrupts, here we don't do anything

}

//surrogate of interrupt
void serialEvent()
{
  
  //only 1 char is allowed
  int cmd=Serial.read();  //you have 1 sec to do something
  
  if(cmd==103)  //103 is 'g'
  {
  
    digitalWrite(LED_BUILTIN, HIGH);
  
    //we write as quickly as we can, not to upset the PC, since every call is slow
    for(byte i=0;i<15;i++){ //reads from pin 0 to pin 14
        Serial.print(analogRead(i));
        Serial.print(" ");
    }
    Serial.println();
  
     digitalWrite(LED_BUILTIN, LOW);  //led blink: command received and executed
  }
    
 


}

