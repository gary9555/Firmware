/** This topic is for reading the proximity sensor output values at the adc.*/


#pragma once

#include <stdint.h>
#include <uORB/uORB.h>


#ifndef __cplusplus

#endif

/**
 * @addtogroup topics
 * @{
 */


#ifdef __cplusplus
struct __EXPORT adc_prox_s {
#else
struct adc_prox_s {
#endif
    uint64_t timestamp;
    uint32_t data;
    uint8_t prox_num;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(adc_prox);

