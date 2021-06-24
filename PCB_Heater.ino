int PWM_pin = 9;
int ThermistorPin = 2;
int Vo;
float R1 = 10000; //Thermistor Resistor
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; //Thermistor Temperature Conversions


//Variables
float temperature_read = 0.0;
float set_temperature = 120;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;



//PID constants
int kp = 9.1;   int ki = 0.3;   int kd = 1.8;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;



void setup() {
  pinMode(PWM_pin, OUTPUT);

  Time = millis();
  Serial.begin(9600);
}


void loop() {
  //Read and Convert Thermistor
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;
  T = (T * 9.0) / 5.0 + 32.0;

  temperature_read = T;
  Serial.println(T);
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if (-3 < PID_error < 3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  //Now we can calculate the D calue
  PID_d = kd * ((PID_error - previous_error) / elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if (PID_value < 0)
  {
    PID_value = 0;
  }
  if (PID_value > 255)
  {
    PID_value = 255;
  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin, PID_value);
  //Serial.println(PID_value);
  //Serial.println("------------------------------");
  previous_error = PID_error;     //Remember to store the previous error for next loop.

  delay(300);

}
