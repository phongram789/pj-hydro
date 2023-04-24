#include <arduino.h>

class swInput{
  private:  
    int t_pin;

  public:
    swInput(int pin){
      t_pin = pin;
      pinMode(t_pin,INPUT_PULLUP);
    }
  
    bool get_status(){
      return digitalRead(t_pin);
      
    }
  
};
//end of class

/*
#include <arduino.h>

class swInput {
  private:
    int t_pin;
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 200;
    bool lastButtonState;
  public:
    swInput(int pin) {
      t_pin = pin;
      //this->debounceDelay = debounceDelay;
      pinMode(t_pin, INPUT_PULLUP);
    }

    bool get_status() {
      int buttonState = digitalRead(t_pin);
      if (buttonState != lastButtonState) {
        lastDebounceTime = millis();
      }
      if ((millis() - lastDebounceTime) > debounceDelay) {
        lastButtonState = buttonState;
        return buttonState;
      }
      return lastButtonState;
    }
};*/

