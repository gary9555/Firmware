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
struct __EXPORT test_uorb_s {
#else
struct test_uorb_s {
#endif
    uint64_t timestamp;
    uint64_t shuai;
#ifdef __cplusplus

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(test_uorb);
