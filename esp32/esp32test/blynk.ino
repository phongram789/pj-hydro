BLYNK_CONNECTED(){
  
  
  }
BLYNK_WRITE(V16){
  int light1State = param.asInt();
  if(light1State == 1){
    digitalWrite(LIGHT1,LOW);
    
  }
  else{
    digitalWrite(LIGHT1,HIGH);
  }
}
BLYNK_WRITE(V17){
  int light2State = param.asInt();
  if(light2State == 1){
    digitalWrite(LIGHT2,LOW);
    
  }
  else{
    digitalWrite(LIGHT2,HIGH);
  }
}
BLYNK_WRITE(V18){
  int light3State = param.asInt();
  if(light3State == 1){
    digitalWrite(LIGHT3,LOW);
    
  }
  else{
    digitalWrite(LIGHT3,HIGH);
  }
}
BLYNK_WRITE(V19){
  int light4State = param.asInt();
  if(light4State == 1){
    digitalWrite(LIGHT4,LOW);
    
  }
  else{
    digitalWrite(LIGHT4,HIGH);
  }
}
BLYNK_WRITE(V20){
  int pumpState = param.asInt();
  if(pumpState == 1){
    digitalWrite(PUMP,LOW);
    
  }
  else{
    digitalWrite(PUMP,HIGH);
  }
}
BLYNK_WRITE(V21){
  int ABState = param.asInt();
  if(ABState == 1){
    digitalWrite(AB,LOW);
    
  }
  else{
    digitalWrite(AB,HIGH);
  }
}
BLYNK_WRITE(V22){
  int PHState = param.asInt();
  if(PHState == 1){
    digitalWrite(PH,LOW);
    
  }
  else{
    digitalWrite(PH,HIGH);
  }
}
BLYNK_WRITE(V23){
  int VALVEState = param.asInt();
  if(VALVEState == 1){
    digitalWrite(VALVE,LOW);
    
  }
  else{
    digitalWrite(VALVE,HIGH);
  }
}
