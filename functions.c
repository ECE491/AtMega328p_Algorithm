/*
 * CFile1.c
 *
 * Created: 2021-03-02 7:12:42 AM
 *  Author: Dustin
 */ 

#include <avr/io.h>
#include <util/delay.h>
#define PI 3.14159

//INITIALIZE ANALOG-TO-DIGITAL CONVERTER
void ADC_init(void)
{
	// Vref = AVcc; REFS1:0 = 0b01;
	ADMUX |= (1<<REFS0);
	// ADC Enable
	ADCSRA |= (1<<ADEN);
	// Set pre-scaling factor to 128
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	// Not sure if necessary
}

uint16_t ADC_read(uint16_t ADC_Channel)
{
	// Select Channel and Mask MUX3:0
	ADMUX =	(ADMUX & 0xF0)|(ADC_Channel & 0x0F);
	// Start Conversion
	ADCSRA |= (1<<ADSC);
	// Run continuously until conversion is complete
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}

float RefreshRate(float v1[],float v2[],float v4[],float t1, float t2,float Scale_factor)
{
	// Azimuthal angle
	float rho1 = sqrt((v1[1]-v2[1]/2)^2+3*(v2[1]/2)^2);
	float phi1 = acos(v1[1]/rho1);
	float rho2 = sqrt((v1[2]-v2[2]/2)^2+3*(v2[2]/2)^2);
	float phi2 = acos(v1[2]/rho1);
	float Omega_phi = (phi2-phi1)/(t2-t1);
	float Refresh_Period_phi = Scale_factor*2*PI/Omega_phi;
	// Inclination angle
	float theta1 = atan(rho1/v4[1]);
	float theta2 = atan(rho1/v4[2]);
	float Omega_theta = (theta2-theta1)/(t2-t1);
	float Refresh_Period_theta = Scale_factor*2*PI/Omega_theta;
	
	float Refresh_Period = (Refresh_Period_phi > Refresh_Period_theta ) ? Refresh_Period_theta : Refresh_Period_phi;
	
	return Refresh_Period;
}
