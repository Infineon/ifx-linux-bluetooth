/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
*
* List of parameters and defined functions needed to utilize the
*
*/

#ifndef __WICED_HAL_RAND_H__
#define __WICED_HAL_RAND_H__

#include "wiced.h"

/*  
*   Linux RandomNumberGenerator Random Number Generator (LRNG)
*/
/******************************************************************************
*** Function prototypes.
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// Generates and returns a random 32-bit integer. 
///
/// \param none
///
/// \return A randomly generated 32-bit integer.
///////////////////////////////////////////////////////////////////////////////
uint32_t wiced_hal_rand_gen_num(void);

#endif // __WICED_RAND_H__
