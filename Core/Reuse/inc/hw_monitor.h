/**
 ********************************************************************************
 * @file    hw_monitor.h
 * @author  Mikolaj Pieklo
 * @date    29.11.2024
 * @brief
 ********************************************************************************
 */

#ifndef __HW_MONITOR_H__
#define __HW_MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void HW_Software_Reset(void);

void HW_PVD_Start(void);


#ifdef __cplusplus
}
#endif

#endif