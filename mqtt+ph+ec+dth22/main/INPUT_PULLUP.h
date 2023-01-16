#include <arduino.h>

class InputPullup{
  private:  
    int t_pin;

  public:
    swInput(int pin){
      t_pin = pin;
      pinMode(t_pin,INPUT_PULLUP);
    }
  
    char get_status(){
      return digitalRead(t_pin);
      
    }
  
};//end of class