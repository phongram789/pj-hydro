#include <arduino.h>

class swInput{
  private:  
    int t_pin;

  public:
    swInput(int pin){
      t_pin = pin;
      pinMode(t_pin,INPUT);
    }
  
    char get_status(){
      return digitalRead(t_pin);
      
    }
  
};//end of class
