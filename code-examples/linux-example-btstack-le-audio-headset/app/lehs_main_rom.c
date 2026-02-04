/*
 * $ Copyright Cypress Semiconductor $
 */
#ifdef MAIN_ROM 
#include "stdint.h"

#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_transport.h"

extern void APPLICATION_START(void);

int get_spy_instance(void)
{
    return 0;
}

void application_start(void)
{
    APPLICATION_START();
}

int main(void)
{
    APPLICATION_START();

    return 0;
}
#endif
