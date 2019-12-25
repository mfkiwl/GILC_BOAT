/*
 * GILC_VehicleMeasure_lib.h
 *
 *  Created on: May 9, 2018
 *      Author: dsf
 *        Form: ANSI
*/

#ifndef GILC_VEHICLE_INNER_H
#define GILC_VEHICLE_INNER_H

#include "GILC_Boat_lib.h"

#define GILC_SENSOR_USE__GNSS          0x01
#define GILC_SENSOR_USE__IMU           0x02
#define GILC_SENSOR_USE__BODY_SPEED    0x04
#define GILC_SENSOR_USE__BODY_HEADING  0x08

#ifndef STM32
#define GILC_DEBUG_FILE 1
#define MX_GNSS_DELAY_MS 150
#else
#define MX_GNSS_DELAY_MS 200
#endif

#pragma pack (4)
typedef struct gilc_result_inner {
	double second;    //sec of gnss week
	int week;
	double dpos_sync[3];//enu m//m//m
}gilc_result_inner_t;
#pragma pack ()

void printf_rtkplot(FILE *fd,int week,double imutimetarge,double pos[3],int stat);
double difpos_enu(double pospre[3], double poscur[3], double dpos_enu[3]);
void GILC_process_backup(void);
void GILC_process_recovery(void);
int GILC_get_result_inner(gilc_result_inner_t *pstRstInner);

#endif
