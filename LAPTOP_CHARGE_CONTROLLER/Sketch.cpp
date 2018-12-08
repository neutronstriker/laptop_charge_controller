/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

#define RELAY_PIN 3
#define ACS_CURRENT_SENSE_PIN_WITH_LPF  A0	//added low pass filter R=1K,C=0.1uF, Fc=1591Hz
#define ACS_CURRENT_SENSE_PIN_RAW A1
#define VCC_PROBE_PIN  A5		//uses a resistor divider 30K/5K which gives a 1/7 ratio divider.
#define AC_SUPPLY_VOLTAGE_SENSE_PIN A3 //uses a resistor network to sense AC voltage Safely;ac_current_sensing_for_arduino_2.asc; LTSPICE schematic

uint8_t CURRENT_STREAM_FLAG = 0;
//Parts of the project ac_current_sensing_for_arduino_2.asc; LTSPICE schematic
//Power_data_analysis.xlsx contains power measurement test results and will help in improving measurement accuracy.
//[Update]:2017-06-18: After fitting together everything inside the enclosure the noise seems to have increased.
//Could be because of Bluetooth SMPS being closer to the Hall sensor.


//Todo: 1.Need to make a block level schematic for this project.
//		2.Make a Python App which will be able to control the relay and measure power.

uint16_t getVccValue()
{
	// I didn't calculate the resistors properly and placed a VCC/2 voltage divider which is not useful in anyway.
	//so until I fix the resistor values, we will not build this function.
	//2018-06-17: Not it has been fixed resistor combination used is 30K||5K, so voltage read by ADC will be 1/7 of VCC.
	analogReference(INTERNAL);
	delay(100); //allow stabilization of Internal reference, need to check if this necessary and if yes what is the minimum required time.
	//uint16_t vccVoltage = (analogRead(VCC_PROBE_PIN) * 1100 * 7)/1024;
	uint16_t vccVoltage = (analogRead(VCC_PROBE_PIN) * 1.0742 * 7);
	analogReference(DEFAULT);
	delay(100);
	return vccVoltage;
}

int16_t getAcSupplyVoltage()
{
	//AC_LIVE=(758*ADC_IN-1510)/3, take a look at ac_voltage_sense_for_arduino_2.asc, Ltspice schematic.
	uint16_t acSupplyPinAdcVoltageMilliVolts = (analogRead(AC_SUPPLY_VOLTAGE_SENSE_PIN)*4.882);
	float acSupplyPinAdcVoltageVolts = (acSupplyPinAdcVoltageMilliVolts*1.0)/1000;
	int16_t returnVal =  (((758*acSupplyPinAdcVoltageVolts)-1510))/3;
	return returnVal;
}

uint16_t largestSample(uint16_t * array, uint8_t len)
{
	uint16_t largest=array[0];
	for (int i=0;i<len;i++)
	{
		if (largest < array[i])
		{
			largest = array[i];
		}
	}	
	return largest;
}

uint16_t getAcRmsVoltage()
{
	uint16_t acVoltagePosCycleSamples[128]={0};
	uint8_t index = 0;
	while(getAcSupplyVoltage() < 0 );//wait if negative cycle is running
	int acVoltageSample = getAcSupplyVoltage();
	while (acVoltageSample >= 0)
	{
		acVoltagePosCycleSamples[index] = acVoltageSample;
		index++;
		acVoltageSample = getAcSupplyVoltage();
	}
	float rmsVoltage= largestSample(acVoltagePosCycleSamples,index+1)/1.414; //Dividing peak value by crest factor of sine wave, sqrt(2).
	return (uint16_t)rmsVoltage;
}

int16_t getAcsCurrentPinVoltage()
{
	//return (analogRead(ACS_CURRENT_SENSE_PIN_WITH_LPF)>>2); //shifting 2 bits to scale down 10 bit data to 8 bits, since for testing it will simpler to send 8bit data through UART.
	//update: since I am using currently Arduino's Serial plotter, it can automatically do the conversion from string to numbers and plot so directly sending measured current.
	int acsVoltage_offsetRemoved = (analogRead(ACS_CURRENT_SENSE_PIN_WITH_LPF)*4.88)-2500;
	return acsVoltage_offsetRemoved;
	//return (acsVoltage_offsetRemoved*1000.0)/185;
}

int getAcCurrentMilliAmps()
{
	return getAcsCurrentPinVoltage()*5.405; //multiplying with 1000/185
}

int acsCurrentAveragedMilliAmps(uint8_t averageSampleCount)
{
	int sum = 0;	//take care of this var, if sample count is too high and the scaling factor in above function is also high it could be possible that, the combination could overflow the var range and calculation will be inaccurate.
	for (uint8_t i=0;i<averageSampleCount;i++)
	{
		sum = sum + getAcCurrentMilliAmps();
	}
	return sum/averageSampleCount;
}

void setup() {
  // put your setup code here, to run once:
	Serial.begin(57600);
	Serial.println("LAPTOP CHARGER CONTROLLER -- INITIALIZED");
	Serial.println("SUPPLY COMES FROM NC OF RELAY, DEFAULT IS POWER ON");
	Serial.println("SEND>\r\n1: POWER ON\r\n0: POWER OFF");
	Serial.println("SEND>\r\nA: ENABLE_CURRENT_STREAM\r\nB: DISABLE_CURRENT_STREAM");
	Serial.println("Initialization Complete");
	
	pinMode(RELAY_PIN,OUTPUT);
	digitalWrite(RELAY_PIN,HIGH);
	pinMode(ACS_CURRENT_SENSE_PIN_WITH_LPF,INPUT);
	pinMode(ACS_CURRENT_SENSE_PIN_RAW,INPUT);
	pinMode(VCC_PROBE_PIN,INPUT);
	pinMode(AC_SUPPLY_VOLTAGE_SENSE_PIN,INPUT);
	digitalWrite(ACS_CURRENT_SENSE_PIN_WITH_LPF,LOW);
	digitalWrite(AC_SUPPLY_VOLTAGE_SENSE_PIN,LOW);
	analogReference(DEFAULT);
}

void loop() {
  // put your main code here, to run repeatedly:
	if (Serial.available())
	{
		char c = Serial.read();
		switch(c)
		{
			case '0': digitalWrite(RELAY_PIN,LOW);
					Serial.println("POWER_OFF");
					break;
			case '1': digitalWrite(RELAY_PIN,HIGH);
					Serial.println("POWER_ON");
					break;
			case 'A':CURRENT_STREAM_FLAG = 1;
					//Serial.println("STREAMING_ON");
					break;
			case 'B':CURRENT_STREAM_FLAG = 0;
					//Serial.println("STREAMING_OFF");
					break;
			default:Serial.println("UNKNOWN_OPTION");
					break;
		}	
	}
	
	if (CURRENT_STREAM_FLAG)
	{
		//Serial.println(getAcsCurrentPinVoltage());
		//Serial.println(acsCurrentAveragedMilliAmps(20));
		//Serial.println(getVccValue());
		//Serial.println(getAcSupplyVoltage());
		//Serial.println(getAcRmsVoltage());
		//Serial.println(getAcCurrentMilliAmps());
		
		//I have decided that all the processing will be done in the host computer,
		//so here I will just stream the voltage and current ADC raw data in real time, minimum lag between current and voltage measurements
		
		/*
		Serial.print(analogRead(ACS_CURRENT_SENSE_PIN_WITH_LPF));
		Serial.print('\t');
		Serial.println(analogRead(AC_SUPPLY_VOLTAGE_SENSE_PIN));
		*/
		
		
		//my test results show that the phase relationship between current and voltage with the below statements 
		//is still preserved, however I will have to test more by using a incandescent bulb. This test I did using laptop charger.
		//the captured data was plotted in excel, arduino plotter seems to used to different Y-axis scales for both plots but shows only one of them.
		Serial.print(acsCurrentAveragedMilliAmps(4));
		Serial.print('\t');
		Serial.println(getAcSupplyVoltage());
		CURRENT_STREAM_FLAG = 0;//Python code was getting hung because of data stream overflow, so now I have implemented it as a function call
								//for testing.
	}
	
	
	
	
}
