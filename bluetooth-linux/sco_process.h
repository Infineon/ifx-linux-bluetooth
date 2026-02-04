#ifndef SCO_PROCESS_H
#define SCO_PROCESS_H

#include "userial.h"
#include "wiced_bt_types.h"
#include "wiced_memory.h"
#include "data_types.h"
#include "hci_uart_linux.h"

#ifndef SCO_PKT_NUM
#define SCO_PKT_NUM          (10U)  //this number can change.
#endif

extern void sco_enqueue(tUART_RX *sco_pkt);
extern BOOL32 init_sco_queue(void);
extern BOOL32 sco_get_queue_ready(void);

#endif

