#define TDSPIN 36

class TDS{
  private:
    int t_pin;
    float analogValue;
    float voltage;
    float aref = 3.3;
    float adcRange = 4096.0;
    float ecValue,ecValue25;
    float temperature = 25;
    float tdsValue;
    float kValue = 1.0;

  public:
    TDS(int pin){
      t_pin = pin;
    }
    void calTDS(){
      //analogValue = analogRead(t_pin);
      analogValue = analogRead(t_pin);      
      voltage = analogValue/(adcRange*aref);
      ecValue = (133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*kValue;
      ecValue25 = ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
      //tdsValue = ecValue25 * TdsFactor;
    }
    float getEC(){
      return ecValue25;
    }


};
float ECvalue;
TDS tds(TDSPIN);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  tds.calTDS();
  ECvalue = tds.getEC();
  Serial.println(ECvalue);
  delay(1000);


}
