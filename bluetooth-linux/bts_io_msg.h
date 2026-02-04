#ifndef _BT_TTY_SDIO_IO_MESSAGE_H
#define _BT_TTY_SDIO_IO_MESSAGE_H

#define BTDLFW_FILE_PATH_SIZE   256
#define BTDLFW_REG_DATA_SIZE    256

typedef struct
{
    unsigned int    sz;
    char            path[BTDLFW_FILE_PATH_SIZE];
} BTDLFW_Msg;

typedef struct
{
    unsigned int    addr;
    unsigned int    sz;
    unsigned int    value;
    unsigned char   data[BTDLFW_REG_DATA_SIZE];
} BT_RWMem_Msg;

#endif  /* _BT_TTY_SDIO_IO_MESSAGE_H */
