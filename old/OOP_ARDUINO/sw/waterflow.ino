
// reading liquid flow rate using Seeeduino and Water Flow Sensor from Seeedstudio.com
// Code adapted by Charles Gantt from PC Fan RPM code written by Crenn @thebestcasescenario.com
// http:/themakersworkbench.com http://thebestcasescenario.com http://seeedstudio.com

volatile int NbTopsFan; //measuring the rising edges of the signal
int Calc;                               
int hallsensor = 2;    //The pin location of the sensor

void rpm ()     //This is the function that the interupt calls 
{ 
 NbTopsFan++;  //This function measures the rising and falling edge of the 

hall effect sensors signal
} 
// The setup() method runs once, when the sketch starts

// the loop() method runs over and over again,
// as long as the Arduino has power
void sen ()    
{
 NbTopsFan = 0;      //Set NbTops to 0 ready for calculations
 sei();            //Enables interrupts
 delay (1000);      //Wait 1 second
 cli();            //Disable interrupts
 Calc = (NbTopsFan * 60 / 7.5); //(Pulse frequency x 60) / 7.5Q, = flow rate 

in L/hour 
 Serial.print (Calc, DEC); //Prints the number calculated above
 Serial.print (" L/hour\r\n"); //Prints "L/hour" and returns a  new line
}
