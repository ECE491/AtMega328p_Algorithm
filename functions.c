/*
 * Capstone.c
 *
 * Created: 2021-02-27 1:28:00 PM
 * Author : Dustin
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#define PI 3.14159
#define F_CPU 1000000UL
#define PD0 0 // SPDT Control Signal
#define PD1 1 // SP4T Control Signal 1
#define PD2 2 // SP4T Control Signal 2
#define PD3 3 // LTC5597 Control Signal
// For ADC_Sim only
#define PC5 5
#define PC4 4
#define PC3 3
#define PC2 2
#define PC1 1
#define PC0 0
#define PB2 2
#define PB1 1

void ADC_init(void);
uint16_t ADC_read(uint16_t ADC_Channel);
void ADC_sim_init();
double ADC_sim();
void Sim_Search(float *PowerData);
volatile float RefreshRate(float v1[],float v2[],float v3[],float v4[],float DeltaT,float Scale_factor);
void Search(float *PowerData);
void Transmit(float *PowerData);
int ClockCheck(void);
float CheckStart(float v1[],float v2[],float v3[],float phi1);
float CheckEnd(float v1[],float v2[],float v3[],float phi2);

int main(void)
{
	DDRD |= (1<<PD0)|(1<<PD1)|(1<<PD2)|(1<<PD3); // Set ports as outputs
	PORTD &= (0<<PD0)|(0<<PD1)|(0<<PD2)|(0<<PD3); // Set outputs low initially
	
	TCCR1B |= (1<<CS10);
	
	float PowerData1[4] = {0};
	float PowerData2[4] = {0};
	//Search(PowerData1);
	Sim_Search(PowerData1);
	float t1 = ClockCheck();
	Transmit(PowerData1);
    while (1) 
    {
		//Search(PowerData2);
		Sim_Search(PowerData2);
		float t2 = ClockCheck();
		Transmit(PowerData2);
		
		float DeltaT = t2-t1;
		if(t2<t1) {
			DeltaT = DeltaT + 65536;
		};
		DeltaT = DeltaT/1000000;
		
		volatile float v1[2] = {PowerData1[0],PowerData2[0]};
		volatile float v2[2] = {PowerData1[1],PowerData2[1]};
		volatile float v3[2] = {PowerData1[2],PowerData2[2]};
		volatile float v4[2] = {PowerData1[3],PowerData2[3]};

		volatile float RefreshPeriod;
		RefreshPeriod = RefreshRate(v1,v2,v3,v4,DeltaT,1000);
		int j;
		for(j=0;j<4;j++) {
			PowerData1[j] = PowerData2[j];	
		};
		t1 = t2;
		int k;
		for(k=0;k<RefreshPeriod;k++)
		{
			_delay_ms(100);
		};
		_delay_ms(5000);
    }
}

//INITIALIZE ANALOG-TO-DIGITAL CONVERTER
void ADC_init(void)
{
	// Vref = AVcc; REFS1:0 = 0b01;
	ADMUX |= (1<<REFS0);
	// ADC Enable
	ADCSRA |= (1<<ADEN);
	// Set pre-scaling factor to 128
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t ADC_read(uint16_t ADC_Channel)
{
	// Select Channel and Mask MUX3:0
	ADMUX =	(0xF0)|(ADC_Channel & 0x0F);
	// Start Conversion
	ADCSRA |= (1<<ADSC);
	// Run continuously until conversion is complete
	while(ADCSRA & (1<<ADSC));
	
	return(ADC);
}

void ADC_sim_init(void)
{
	DDRC = 0;
	DDRB = 0;
}

double ADC_sim(void)
{
	double AnalogVoltage = 0;
	AnalogVoltage = ((PINC & (1<<PC5))>0)*128 + ((PINC & (1<<PC4))>0)*64 + ((PINC & (1<<PC3))>0)*32 + ((PINC & (1<<PC2))>0)*16 + ((PINC & (1<<PC1))>0)*8 + ((PINC & (1<<PC0))>0)*4 + ((PINB & (1<<PB2))>0)*2 +((PINB & (1<<PB1))>0);
	AnalogVoltage = AnalogVoltage*5/255;
	
	return AnalogVoltage;
}

void Sim_Search(float *PowerData)
{
	PORTD |= (1<<PD0); //Switch SPDT to Searching Mode
	PORTD |= (1<<PD3); //Switch Power Detector on
	_delay_ms(1000);
	
	PORTD &= ~((1<<PD2)|(1<<PD1)); // Switch SPDT to Search Mode and enable power detector
	_delay_ms(1000);
	PowerData[0] = ADC_sim();
	PORTD &= ~(1<<PD2);
	PORTD |= (1<<PD1);
	_delay_ms(1000);
	PowerData[1] = ADC_sim();
	PORTD &= ~(1<<PD1);
	PORTD |= (1<<PD2);
	_delay_ms(1000);
	PowerData[2] = ADC_sim();
	PORTD |= (1<<PD2)|(1<<PD1);
	_delay_ms(1000);
	PowerData[3] = ADC_sim();
}

volatile float RefreshRate(float v1[],float v2[],float v3[],float v4[],float DeltaT,float Scale_factor)
{
	volatile float vA1 = 0;
	volatile float vB1 = 0;
	volatile float vA2 = 0;
	volatile float vB2 = 0;
	volatile float A1 = 0;
	volatile float A2 = 0;
	volatile float A3 = 0;
	volatile float B1 = 0;
	volatile float B2 = 0;
	volatile float B3 = 0;
	
	if((v1[1]<v2[1])&&(v1[1]<v3[1])) //V1 has the weakest signal
	{
		vA1 = v2[1];
		vB1 = v3[1];
		A1 = 1;
	};
	
	if((v2[1]<v1[1])&&(v2[1]<v3[1])) //V2 has the weakest signal
	{
		vA1 = v3[1];
		vB1 = v1[1];
		A2 = 1;
	};
	
	if((v3[1]<v2[1])&&(v3[1]<v1[1])) //V3 has the weakest signal
	{
		vA1 = v1[1];
		vB1 = v2[1];
		A3 = 1;
	};
	////////////////////////////////////////
	
	if((v1[2]<v2[2])&&(v1[2]<v3[2])) //V1 has the weakest signal
	{
		vA2 = v2[2];
		vB2 = v3[2];
		B1 = 1;
	};
	
	if((v2[2]<v1[2])&&(v2[2]<v3[2])) //V2 has the weakest signal
	{
		vA2 = v3[2];
		vB2 = v1[2];
		B2 = 1;
	};
	
	if((v3[2]<v2[2])&&(v3[2]<v1[2])) //V3 has the weakest signal
	{
		vA2 = v1[2];
		vB2 = v2[2];
		B3 = 1;
	};
	
	// Azimuthal angle
	volatile float rho1 = sqrt(pow((vA1-vB1/2),2)+3*pow(vB1/2,2));
	volatile float phi1 = acos(vB1/(2*rho1)) + (2*3.14159/3)*(A1*B3+A2*B1+A3*B2);
	volatile float rho2 = sqrt(pow(vA2-vB2/2,2)+3*pow(vB2/2,2));
	volatile float phi2 = acos(vB2/(2*rho2)) + (2*3.14159/3)*(A1*B2+A2*B3+A3*B1);
	phi1 = CheckStart(v1,v2,v3,phi1);
	phi2 = CheckEnd(v1,v2,v3,phi2);
	volatile float Omega_phi = (phi2-phi1)/(DeltaT);
	volatile float Refresh_Period_phi = Scale_factor*2*PI/Omega_phi;
	// Inclination angle
	volatile float theta1 = atan(rho1/v4[1]);
	volatile float theta2 = atan(rho1/v4[2]);
	volatile float Omega_theta = (theta2-theta1)/(DeltaT);
	volatile float Refresh_Period_theta = Scale_factor*2*PI/Omega_theta;
	
	volatile float Refresh_Period = fminf(Refresh_Period_phi,Refresh_Period_theta);
	Refresh_Period = fminf(Refresh_Period,100);
	
	return Refresh_Period;
}

void Search(float *PowerData)
{
	PORTD |= (1<<PD0); //Switch SPDT to Searching Mode
	PORTD |= (1<<PD3); //Switch Power Detector on
	uint16_t ADC_Channel = 1;
	_delay_ms(1000);
	
	PORTD &= ~((1<<PD2)|(1<<PD1)); // Switch SPDT to Search Mode and enable power detector
	_delay_ms(1000);
	PowerData[0] = ADC_read(ADC_Channel);
	PORTD &= ~(1<<PD2);
	PORTD |= (1<<PD1);
	_delay_ms(1000);
	PowerData[1] = ADC_read(ADC_Channel);
	PORTD &= ~(1<<PD1);
	PORTD |= (1<<PD2);
	_delay_ms(1000);
	PowerData[2] = ADC_read(ADC_Channel);
	PORTD |= (1<<PD2)|(1<<PD1);
	_delay_ms(1000);
	PowerData[3] = ADC_read(ADC_Channel);
}

void Transmit(float *PowerData)
{
	PORTD &= ~(1<<PD0); // Switch SPDT to Transmit Mode
	PORTD &= ~(1<<PD3); // Switch OFF Power Detector
	float max = 0;
	int max_index = 0;
	int i;
	for(i=0; i<4; i++)
	{
		if(max<PowerData[i])
		{
			max = PowerData[i];
			max_index = i;
		}
	}
	switch(max_index)
	{
		case 0:
			PORTD &= ~((1<<PD2)|(1<<PD1));
			break;
		case 1:
			PORTD &= ~(1<<PD2);
			PORTD |= (1<<PD1);
			break;
		case 2:
			PORTD &= ~(1<<PD1);
			PORTD |= (1<<PD2);
			break;
		case 3:
			PORTD |= (1<<PD2)|(1<<PD1);
			break;
	}
}

int ClockCheck(void) 
{
	int ClockValue = TCNT1;
	
	return ClockValue;
}

float CheckStart(float v1[],float v2[],float v3[],float phi1)
{
	if((v1[1]==v2[1])&&(v1[2]>v2[2]))
	{
		phi1 = 0;
	};
	
	if((v1[1]==v2[1])&&(v1[2]<v2[2]))
	{
		phi1 = 120;
	};
	
	if((v2[1]==v3[1])&&(v2[2]>v3[2]))
	{
		phi1 = 0;
	};
	
	if((v2[1]==v3[1])&&(v2[2]<v3[2]))
	{
		phi1 = 120;
	};
	
	if((v3[1]==v1[1])&&(v3[2]>v1[2]))
	{
		phi1 = 0;
	};
	
	if((v3[1]==v1[1])&&(v3[2]<v1[2]))
	{
		phi1 = 120;
	};
	return phi1;
}

float CheckEnd(float v1[],float v2[],float v3[],float phi2)
{
	if((v1[2]==v2[2])&&(v1[1]>v2[1]))
	{
		phi2 = 0;
	};
	
	if((v1[2]==v2[2])&&(v1[1]<v2[1]))
	{
		phi2 = 120;
	};
	
	if((v2[2]==v3[2])&&(v2[1]>v3[1]))
	{
		phi2 = 0;
	};
	
	if((v2[2]==v3[2])&&(v2[1]<v3[1]))
	{
		phi2 = 120;
	};
	
	if((v3[2]==v1[2])&&(v3[1]>v1[1]))
	{
		phi2 = 0;
	};
	
	if((v3[2]==v1[2])&&(v3[1]<v1[1]))
	{
		phi2 = 120;
	};
	return phi2;
}