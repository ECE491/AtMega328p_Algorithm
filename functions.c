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
#define PD0 0 // SPDT Control Signal
#define PD1 1 // SP4T Control Signal 1
#define PD2 2 // SP4T Control Signal 2
#define PD3 3 // LTC5597 Control Signal


int main(void)
{
	DDRD |= (1<<PD0)|(1<<PD1)|(1<<PD2)|(1<<PD3); // Set ports as outputs
	PORTD |= (0<<PD0)|(0<<PD1)|(0<<PD2)|(0<<PD3); // Set outputs low initially
	
	TCCR1B |= (1<<CS10);
	
	float PowerData1[4] = Search(0);
	float t1 = ClockCheck()/1000000;
	Transmit(PowerData1);
	
    /* Replace with your application code */
    while (1) 
    {
		float PowerData2[4] = Search(0);
		float t2 = ClockCheck()/1000000;
		Transmit(PowerData2);
		float DeltaT = t2-t1;
		if(t2<t1) {
			DeltaT = DeltaT + 65536;
		}
		float v1[2] = ConvertPowerData(PowerData1,PowerData2,1);
		float v2[2] = ConvertPowerData(PowerData1,PowerData2,2);
		float v3[2] = ConvertPowerData(PowerData1,PowerData2,3);
		float v4[2] = ConvertPowerData(PowerData1,PowerData2,4);
		float RefreshPeriod = RefreshRate(v1,v2,v3,v4,DeltaT,1);
		PowerData1 = PowerData2;
		t1 = t2;
		//Delay(RefreshPeriod)
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
	ADMUX =	(ADMUX & 0xF0)|(ADC_Channel & 0x0F);
	// Start Conversion
	ADCSRA |= (1<<ADSC);
	// Run continuously until conversion is complete
	while(ADCSRA & (1<<ADSC));
	
	return(ADC);
}

float RefreshRate(float v1[],float v2[],float v3[],float v4[],float DeltaT,float Scale_factor)
{
	float vA1 = 0;
	float vB1 = 0;
	float vA2 = 0;
	float vB2 = 0;
	float A1 = 0;
	float A2 = 0;
	float A3 = 0;
	float B1 = 0;
	float B2 = 0;
	float B3 = 0;
	
	if((v1[1]<v2[1])&&(v1[1]<v3[1])) //V1 has the weakest signal
	{
		vA1 = v2[1];
		vB1 = v3[1];
		A1 = 1;
	}
	
	if((v2[1]<v1[1])&&(v2[1]<v3[1])) //V2 has the weakest signal
	{
		vA1 = v3[1];
		vB1 = v1[1];
		A2 = 1;
	}
	
	if((v3[1]<v2[1])&&(v3[1]<v1[1])) //V3 has the weakest signal
	{
		vA1 = v1[1];
		vB1 = v2[1];
		A3 = 1;
	}
	////////////////////////////////////////
	
	if((v1[2]<v2[2])&&(v1[2]<v3[2])) //V1 has the weakest signal
	{
		vA2 = v2[2];
		vB2 = v3[2];
		B1 = 1;
	}
	
	if((v2[2]<v1[2])&&(v2[2]<v3[2])) //V2 has the weakest signal
	{
		vA2 = v3[2];
		vB2 = v1[2];
		B2 = 1;
	}
	
	if((v3[2]<v2[2])&&(v3[2]<v1[2])) //V3 has the weakest signal
	{
		vA2 = v1[2];
		vB2 = v2[2];
		B3 = 1;
	}
	
	// Azimuthal angle
	float rho1 = sqrt((vA1-vB1/2)^2+3*(vB1/2)^2);
	float phi1 = acos(vA1/rho1) + (2*PI/3)*(A1*B3+A2*B1+A3*B2);
	float rho2 = sqrt((vA2-vB2/2)^2+3*(vB2/2)^2);
	float phi2 = acos(vA2/rho1) + (2*PI/3)*(A1*B2+A2*B3+A3*B1);
	phi1 = CheckStart(v1,v2,v3,phi1);
	phi2 = CheckEnd(v1,v2,v3,phi2);
	float Omega_phi = (phi2-phi1)/(DeltaT);
	float Refresh_Period_phi = Scale_factor*2*PI/Omega_phi;
	// Inclination angle
	float theta1 = atan(rho1/v4[1]);
	float theta2 = atan(rho1/v4[2]);
	float Omega_theta = (theta2-theta1)/(DeltaT);
	float Refresh_Period_theta = Scale_factor*2*PI/Omega_theta;
	
	float Refresh_Period = fminf(Refresh_Period_phi,Refresh_Period_theta);
	
	return Refresh_Period;
}

float Search(uint16_t ADC_Channel)
{
	float PowerData[4];
	
	PORTD |= (1<<PD3)|(0<<PD2)|(0<<PD1)|(1<<PD0); // Switch SPDT to Search Mode and enable power detector
	//Delay (For power detector to turn on and SPDT to switch)
	PowerData[0] = ADC_read(ADC_Channel);
	PORTD |= (0<<PD2)|(1<<PD1);
	//Delay
	PowerData[1] = ADC_read(ADC_Channel);
	PORTD |= (1<<PD2)|(0<<PD1);
	//Delay
	PowerData[2] = ADC_read(ADC_Channel);
	PORTD |= (1<<PD2)|(1<<PD1);
	//Delay
	PowerData[3] = ADC_read(ADC_Channel);
	
	return PowerData;
}

void Transmit(float PowerData[])
{
	float max = 0;
	int max_index;
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
			PORTD |= (0<<PD2)|(0<<PD1);
			break;
		case 1:
			PORTD |= (0<<PD2)|(1<<PD1);
			break;
		case 2:
			PORTD |= (1<<PD2)|(0<<PD1);
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

float ConvertPowerData(float PowerData1[], float PowerData2[],int index) 
{
	float vX[2] = {PowerData1[index],PowerData2[index]};
		
	return vX;
}

float CheckStart(float v1[],float v2[],float v3[],float phi1)
{
	if((v1[1]==v2[1])&&(v1[2]>v2[2]))
	{
		phi1 = 0;
	}
	
	if((v1[1]==v2[1])&&(v1[2]<v2[2]))
	{
		phi1 = 120;
	}
	
	if((v2[1]==v3[1])&&(v2[2]>v3[2]))
	{
		phi1 = 0;
	}
	
	if((v2[1]==v3[1])&&(v2[2]<v3[2]))
	{
		phi1 = 120;
	}
	
	if((v3[1]==v1[1])&&(v3[2]>v1[2]))
	{
		phi1 = 0;
	}
	
	if((v3[1]==v1[1])&&(v3[2]<v1[2]))
	{
		phi1 = 120;
	}
	return phi1;
}

float CheckEnd(float v1[],float v2[],float v3[],float phi2)
{
	if((v1[2]==v2[2])&&(v1[1]>v2[1]))
	{
		phi2 = 0;
	}
	
	if((v1[2]==v2[2])&&(v1[1]<v2[1]))
	{
		phi2 = 120;
	}
	
	if((v2[2]==v3[2])&&(v2[1]>v3[1]))
	{
		phi2 = 0;
	}
	
	if((v2[2]==v3[2])&&(v2[1]<v3[1]))
	{
		phi2 = 120;
	}
	
	if((v3[2]==v1[2])&&(v3[1]>v1[1]))
	{
		phi2 = 0;
	}
	
	if((v3[2]==v1[2])&&(v3[1]<v1[1]))
	{
		phi2 = 120;
	}
	return phi2;
}
