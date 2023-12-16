/*
 * ultra_sonic_interface.h
 *
 *  Created on: Nov 30, 2023
 *      Author: msame
 */

#ifndef ULTRA_SONIC_INTERFACE_H_
#define ULTRA_SONIC_INTERFACE_H_


#include "main.h"



#define TOTAL_ULTRA_SOINC    6
#define NUMBER_OF_READS		 10

typedef enum
{

	FORWARD_SENSOR = 0,
	FORWARD_LEFT_SENSOR = 1,
	FORWARD_RIGHT_SENSOR,
	BACKWARD_SENSOR,
	BACKWARD_LEFT_SENSOR,
	BACKWARD_RIGHT_SENSOR

}Ultra_sonic_t;


typedef enum
{
	PACKAGE_1 = 0,
	PACKAGE_2	 ,
	PACKAGE_3	 ,
	PACKAGE_4

}Ultrasonic_Packages_t;



void Ultrasonic_vidInit(void);

void Ultrasonic_vidTrigger (Ultra_sonic_t Sensor_type);




#endif /* ULTRA_SONIC_INTERFACE_H_ */
