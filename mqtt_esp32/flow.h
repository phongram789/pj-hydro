#include <arduino.h>

class FLOW{
  private:  
    int t_pin;
    int X;
    int Y;
    float TIME = 0;
    float FREQUENCY = 0;
    float WATER = 0;
    float TOTAL = 0;
    float LS = 0;

  public:
    FLOW(int pin){
      t_pin = pin;
      pinMode(t_pin,INPUT_PULLUP);
    }
  
    float get_flow(){
      return WATER;
    }
    float readwaterflow(){
      X = pulseIn(t_pin, HIGH);
      Y = pulseIn(t_pin, LOW);
      TIME = X + Y;
      FREQUENCY = 1000000/TIME;
      WATER = FREQUENCY/7.5; // 7.5 is value of calibration
      LS = WATER/60;
      return WATER;
      if(FREQUENCY >= 0){
        if(isinf(FREQUENCY)){
         //Serial.println("Not flowing");
         //Serial.println(TIME);
         // TIME/100
         }
         else{
          //Serial.println("Water is flowing");
          //TOTAL = TOTAL + LS;
          //Serial.println(FREQUENCY);
          //Serial.println(WATER);
          //Serial.println(TOTAL);
          // TOTAL L/H
          }
       }
    }
  
};//end of class
