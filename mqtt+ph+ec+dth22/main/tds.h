#include<Arduino.h>
class TDS{
  private:
    float TdsFactor = 0.5;  // tds = ec / 2
    int t_pin;
    float analogValue;
    float voltage;
    float aref = 3.3;
    float adcRange = 4096.0;
    float ecValue,ecValue25;
    float temperature = 25;
    float tdsValue;
    float kValue = 1.38;
    float KValueTemp,rawECsolution;

  public:
    TDS(int pin){
      t_pin = pin;
    }
    void calTDS(){
      //analogValue = analogRead(t_pin);
      analogValue = analogRead(t_pin);
      voltage =analogValue/adcRange*aref;
      ecValue = (133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*kValue;
      ecValue25 = ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
      tdsValue = ecValue25 * TdsFactor;
    }
    float getEC(){
      return ecValue25;
    }
    float getTds(){
      return tdsValue;
    }
    void getkkValue(){
      rawECsolution = 707/(float)(TdsFactor);
      rawECsolution = rawECsolution*(1.0+0.02*(temperature-25.0));
      KValueTemp = rawECsolution/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);  //calibrate in the  buffer solution, such as 707ppm(1413us/cm)@25^c
      kValue =  KValueTemp;
      Serial.print("voltage: ");
      Serial.println(voltage);
      Serial.print("rawECsolution: ");
      Serial.println(rawECsolution);
      Serial.print("kValue is: ");
      Serial.println(kValue);
      Serial.print("KValueTemp is: ");
      Serial.println(KValueTemp);
    }


};
