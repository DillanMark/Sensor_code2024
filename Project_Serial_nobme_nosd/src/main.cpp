#include <Arduino.h>
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/
#include <bme68xLibrary.h> // https://github.com/boschsensortec/Bosch-BME68x-Library/tree/master/src
#include<Adafruit_ADS1X15.h> // https://github.com/adafruit/Adafruit_ADS1X15
#include <Adafruit_MCP4728.h> // https://github.com/adafruit/Adafruit_MCP4728/tree/master


// REF_RESISTANCE 42.7 at 25.43C //in ohms #### @23C ##very dodgy##




#define INITIAL_DAC0_VOLTAGE 4.38
#define DAC1_CURRENT_SET_VOLTAGE 1 //in volts dac1
#define SAMPLING_PERIOD 20 // in ms 20    ADC really determins the speed
#define HEAT_CURRENT0_PIN 2 //inputs into the adc
#define HEAT_CURRENT1_PIN 3 //inputs into the adc
#define TEMP_CURRENT0_PIN 1 //inputs into the adc
#define TEMP_CURRENT1_PIN 0 //inputs into the adc
#define U6_GAIN 8 //Differential op amp Gain
#define BMEPERIOD 1000 //Sampling Period of BME
#define PLOTTER 0 // 0 for datalogger, 1 for plotting sensor_v, 2 for plotting temperature with arduino plotter


// function declarations

void bmesetup(); 
float flusso_measure(int8_t);
void adc_setup();
void dac_setup();


Adafruit_MCP4728 mcp;
Adafruit_ADS1115 ads;
Bme68x bme;



void setup() {  
  Serial.begin(38400); 
  while (!Serial){;};//wait until serial connection

  //set opamp gain gain - 1 = 2^(A1 A0 bin 2 dec)

  pinMode(26,OUTPUT);
  pinMode(27,OUTPUT);
  
  #if U6_GAIN == 1
    digitalWrite(27, LOW); //A1
    digitalWrite(26, LOW); //A0
  #elif U6_GAIN == 2
    digitalWrite(27, LOW); //A1
    digitalWrite(26, HIGH); //A0
  #elif U6_GAIN == 4
    digitalWrite(27, HIGH); //A1
    digitalWrite(26, LOW); //A0
  #elif U6_GAIN == 8
    digitalWrite(27, HIGH); //A1
    digitalWrite(26, HIGH); //A0
  #else
    Serial.print("Error U6 gain");
    while(1){;}
  #endif

  dac_setup(); //set up dac values

  adc_setup(); // set up ADC

  bmesetup(); //set up BME

#if !PLOTTER // headers for datalogger file - will disable when using arduino plotter
  Serial.println("Sample_number,time,V_REF,V_heater_curr,U6_gain,V_sensor,V_ref,T_bme,RHa_bme,pressure_bme,V_curr_heater,V_curr_ref,sampling_period");
#endif
}

void loop() {
  /*###### VALUES #########*/

  static long Sample_number = 0;    // sample number
  static unsigned long start_time = millis(); //starting time in reference in cpu time
  static unsigned long previous_time = 0; //previous sample time
  static unsigned long time = 0; // current time
  float measure = 0; // initial measure value - other ADC measurements
  static float measureH = 0; // initial measure value - sensor_v 
  static float ambtemperature = 0; // ambient temperature reading from bme
  static float ambRha = 0; // ambient relative humidity reading from bme
  static float ambpressure = 0; // ambient pressure reading from bme

  static bme68xData data; 
  static unsigned long bmeprev = 0; //previous bme sample time
  static unsigned long dacprev = 0;//previous DAC0 sample time
  static unsigned long dac2prev = 0; //previous DAC1 sample time

  static float DAC0_REF_VOLTAGE = INITIAL_DAC0_VOLTAGE; // DAC0 voltage value
  static float DAC1_VOLTAGE = DAC1_CURRENT_SET_VOLTAGE; // DAC1 voltage value


if ((time - dacprev) >= 500){
    dacprev = time;
    if (measureH >= 230){
      DAC0_REF_VOLTAGE = DAC0_REF_VOLTAGE + 0.005;
    } else if (measureH <= 20){
      DAC0_REF_VOLTAGE = DAC0_REF_VOLTAGE - 0.005;
    }
    mcp.setChannelValue(MCP4728_CHANNEL_A, (DAC0_REF_VOLTAGE * 4096)/5); //vref is 5volts
  }  


/* //this section is altering heater current. uncomment when need this. BE CAREFUL OF DAMAGING EQUIPTMENT
  if ((time - dac2prev) >= 30000){
    dac2prev = time;
    DAC1_VOLTAGE = DAC1_VOLTAGE + 0.02;
    mcp.setChannelValue(MCP4728_CHANNEL_B, (DAC1_VOLTAGE * 4096)/2.048, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X); //vref is internal 2.048
  }
*/ 

while (!Serial){;}; //stop if serial gets disconnected

  if (time - previous_time >= SAMPLING_PERIOD){
    previous_time = time;

    Sample_number++;
    #if (PLOTTER == 1) //to plot sensor_V
      measureH = flusso_measure(HEAT_CURRENT0_PIN);
      Serial.println(measureH,3);
    #elif (PLOTTER == 2)//to plot bme temperature
      bme.setOpMode(BME68X_FORCED_MODE);
      delayMicroseconds(bme.getMeasDur());

      if (bme.fetchData())
      {
        bme.getData(data);
        ambtemperature = data.pressure;
      }
      Serial.println(ambtemperature);
    #else
        
      //print files to serial, with comma (for commadelimited CSV)
      Serial.print(Sample_number); 
      Serial.print(",");
      Serial.print(time); 
      Serial.print(",");
      Serial.print(DAC0_REF_VOLTAGE);
      Serial.print(",");
      Serial.print(DAC1_VOLTAGE);
      Serial.print(",");
      Serial.print(U6_GAIN);
      Serial.print(",");
      measureH = flusso_measure(HEAT_CURRENT0_PIN);
      Serial.print(measureH, 3); //sensor voltage measurement
      Serial.print(",");
      measure = flusso_measure(TEMP_CURRENT0_PIN);
      Serial.print(measure, 3);//reference sensor voltage measurement
      Serial.print(",");

      if ((time - bmeprev) >= BMEPERIOD){
        bmeprev = time;
        //bme_count = 4;
        
        bme.setOpMode(BME68X_FORCED_MODE);
        delayMicroseconds(bme.getMeasDur());

        if (bme.fetchData())
        {
          bme.getData(data);
          ambtemperature = data.temperature;
          ambRha = data.humidity;
          ambpressure = data.pressure;
        }

      }
        
      Serial.print(ambtemperature);
      Serial.print(",");
      Serial.print(ambRha);
      Serial.print(",");
      Serial.print(ambpressure); //pressure in PA
      
      Serial.print(",");
      measure = flusso_measure(HEAT_CURRENT1_PIN);
      Serial.print(measure, 3); //sensor current measurement
      Serial.print(",");
      measure = flusso_measure(TEMP_CURRENT1_PIN);
      Serial.print(measure, 3); //reference sensor current measurement
      Serial.print(",");
      Serial.println(SAMPLING_PERIOD);
  #endif
  }
  time = millis() - start_time; // current time
}



// put function definitions here:

void bmesetup(){
  bme.begin(BME68X_I2C_ADDR_LOW, Wire); //begin bme i2c

  bme.setTPH();

  bme68xData data;

  bme.setOpMode(BME68X_FORCED_MODE); //using forced mode - one call for one measurement. try parallel in future?
  delayMicroseconds(bme.getMeasDur());

  if (bme.fetchData()) //gets rid of noise
  {
    bme.getData(data);  //pressure in hpa and humidity as % 
  }

}


void adc_setup(){
  ads.setGain(GAIN_SIXTEEN); // use datasheet for this. The limits restrict as gain increases
  
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  
  //128 is good precision and 20Hz| 250 can do 30ms period, 475 can do 50Hz but less precise
  ads.setDataRate(RATE_ADS1115_475SPS); //250data rate     //total not per channel

  flusso_measure(HEAT_CURRENT0_PIN); //warm up the adc
  flusso_measure(HEAT_CURRENT1_PIN);
  flusso_measure(TEMP_CURRENT0_PIN);
  flusso_measure(TEMP_CURRENT1_PIN);


}

void dac_setup(){
  // Try to initialize!
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {;}
  }


  mcp.setChannelValue(MCP4728_CHANNEL_B, (DAC1_CURRENT_SET_VOLTAGE * 4096)/2.048, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X); //vref is internal 2.048
  mcp.setChannelValue(MCP4728_CHANNEL_A, (INITIAL_DAC0_VOLTAGE * 4096)/5); //vref is 5volts

}


inline float flusso_measure(int8_t in){
  int16_t a = ads.readADC_SingleEnded(in);
  //16-bit analogue, ADC Range: +/- 6.144V (1 bit = 0.1875mV/ADS1115)
  float measurement = ads.computeVolts(a) * 1000; // in millivolts to volts

  return measurement;
}