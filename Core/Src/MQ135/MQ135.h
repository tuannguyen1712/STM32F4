/***********************************************
HOW TO USE
1. Declare a Air_param_t variable to write and read air value.
2. Get the ADC value from module MQ135.
3. If you have temperature and humidity value, use function getAQI_Correctval(...)
 	 else use function getAQI_val

*********************************************/

#include <stdio.h>
#include <math.h>

#define RL 		10				// kOhms
#define Vin 	5
#define Res 	4096			// Resolution
#define ATMOCO2 397.13

//Correction Values
#define CorrA 	-0.000002469136
#define CorrB 	0.00048148148
#define CorrC 	0.0274074074
#define CorrD 	1.37530864197
#define CorrE 	0.0019230769
#define R0 		76.63			//	Average value of R0 (kOhms)
/// Helper to calculate Voltage from Input
/// Voltage = input * Vin / (Resolution - 1)


typedef struct {
	uint16_t Acetone;
	uint16_t Alcohol;
	uint16_t CO;
	uint16_t CO2;
	uint16_t NH4;
	uint16_t Toluene;
} Air_param_t;

//struct MQ135 {
//	uint8_t pin;
//};

double MQ135_getVoltage(uint32_t ADC_val);
double MQ135_getCorrectionFactor(float temparature, float humidity);
double MQ135_getResistance(uint32_t ADC_val);
double MQ135_getCorrectResistance(double tem, double hum, uint32_t ADC_val);
// a and b dependent on type of gas
double MQ135_getPPM(float a, float b, uint32_t ADC_val);
double MQ135_getCorrectPPM(float a, float b, float tem, float hum, uint32_t ADC_val);
double MQ135_getPPMLinear(float a, float b, uint32_t ADC_val);
double MQ135_getAcetone(uint32_t ADC_val);
double MQ135_getCorrectAcetone(float tem, float hum, uint32_t ADC_val);
double MQ135_getAlcohol(uint32_t ADC_val);
double MQ135_getCorrectAlcohol(float tem, float hum, uint32_t ADC_val);
double MQ135_getCO2(uint32_t ADC_val);
double MQ135_getCorrectCO2(float tem, float hum, uint32_t ADC_val);
double MQ135_getCO(uint32_t ADC_val);
double MQ135_getCorrectCO(float tem, float hum, uint32_t ADC_val);
double MQ135_getNH4(uint32_t ADC_val);
double MQ135_getCorrectNH4(float tem, float hum, uint32_t ADC_val);
double MQ135_getToluene(uint32_t ADC_val);
double MQ135_getCorrectToluene(float tem, float hum, uint32_t ADC_val);
void getAQI_val(Air_param_t *aqi, uint32_t ADC_val);
void getAQI_Correctval(Air_param_t *aqi, int tem, int hum, uint32_t ADC_val);
