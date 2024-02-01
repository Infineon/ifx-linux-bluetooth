/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * This file contains functions for processing AT commands and responses
 *
 */

#include "wiced_bt_hfp_ag_int.h"
#include "string.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

#define BTA_AG_ERR_OP_NOT_ALLOWED   3       /* Operation not allowed */
#define BTA_AG_ERR_OP_NOT_SUPPORTED 4       /* Operation not supported */

#define BTA_AG_CMD_MAX_VAL      32767       /* Maximum value is signed 16-bit value */
#define BTA_AG_NUM_INDICATORS    7
#define BTA_AG_CIND_INFO        "(\"call\",(0,1)),(\"callsetup\",(0-3)),(\"callheld\",(0-2)),(\"service\",(0,1),(\"battchg\",(0-5)),(\"signal\",(0-5)),(\"roam\",(0-1))"
static char  BTA_AG_CIND_VALUES[20] = { '0', ',' , '0' , ',' , '0' , ',' , '1' ,',','5',',' ,'5',',' ,'0'};

/* enumeration of HFP AT commands matches HFP command interpreter table */
enum
{
    BTA_AG_HF_CMD_A,
    BTA_AG_HF_CMD_D,
    BTA_AG_HF_CMD_VGS,
    BTA_AG_HF_CMD_VGM,
    BTA_AG_HF_CMD_CCWA,
    BTA_AG_HF_CMD_CHLD,
    BTA_AG_HF_CMD_CHUP,
    BTA_AG_HF_CMD_CIND,
    BTA_AG_HF_CMD_CLIP,
    BTA_AG_HF_CMD_CMER,
    BTA_AG_HF_CMD_VTS,
    BTA_AG_HF_CMD_BINP,
    BTA_AG_HF_CMD_BLDN,
    BTA_AG_HF_CMD_BVRA,
    BTA_AG_HF_CMD_BRSF,
    BTA_AG_HF_CMD_NREC,
    BTA_AG_HF_CMD_CNUM,
    BTA_AG_HF_CMD_BTRH,
    BTA_AG_HF_CMD_CLCC,
    BTA_AG_HF_CMD_COPS,
    BTA_AG_HF_CMD_CMEE,
    BTA_AG_HF_CMD_BIA,
    BTA_AG_HF_CMD_CBC,
    BTA_AG_HF_CMD_BCC,
    BTA_AG_HF_CMD_BCS,
    BTA_AG_HF_CMD_BIND,
    BTA_AG_HF_CMD_BIEV,
    BTA_AG_HF_CMD_BAC,
    BTA_AG_HF_CMD_CKPD
};

/* AT command interpreter table for HFP */
static const struct
{
    char        *p_cmd;                     /* AT command string */
    uint8_t     arg_type;                   /* allowable argument type syntax */
#define BTA_AG_AT_NONE          0x01        /* no argument */
#define BTA_AG_AT_SET           0x02        /* set value */
#define BTA_AG_AT_READ          0x04        /* read value */
#define BTA_AG_AT_TEST          0x08        /* test value range */
#define BTA_AG_AT_FREE          0x10        /* freeform argument */

    uint8_t     fmt;                        /* whether arg is int or string */
#define BTA_AG_AT_STR           0           /* string */
#define BTA_AG_AT_INT           1           /* integer */

    uint8_t     min;                        /* minimum value for int arg */
    int16_t     max;                        /* maximum value for int arg */

} bta_ag_hfp_cmd[] =
{
    {"A",       BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"D",       (BTA_AG_AT_NONE | BTA_AG_AT_FREE),  BTA_AG_AT_STR,   0,   0},
    {"+VGS",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,  15},
    {"+VGM",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,  15},
    {"+CCWA",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+CHLD",   (BTA_AG_AT_SET | BTA_AG_AT_TEST),   BTA_AG_AT_STR,   0,   4},
    {"+CHUP",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+CIND",   (BTA_AG_AT_READ | BTA_AG_AT_TEST),  BTA_AG_AT_STR,   0,   0},
    {"+CLIP",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+CMER",   BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+VTS",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+BINP",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   1,   1},
    {"+BLDN",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BVRA",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+BRSF",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   BTA_AG_CMD_MAX_VAL},
    {"+NREC",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   0},
    {"+CNUM",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BTRH",   (BTA_AG_AT_READ | BTA_AG_AT_SET),   BTA_AG_AT_INT,   0,   2},
    {"+CLCC",   BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+COPS",   (BTA_AG_AT_READ | BTA_AG_AT_SET),   BTA_AG_AT_STR,   0,   0},
    {"+CMEE",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   1},
    {"+BIA",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   20},
    {"+CBC",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   100},
    {"+BCC",    BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0},
    {"+BCS",    BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0,   BTA_AG_CMD_MAX_VAL},
    {"+BIND",   BTA_AG_AT_SET | BTA_AG_AT_READ | BTA_AG_AT_TEST , BTA_AG_AT_STR,   0,   0},
    {"+BIEV",   BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+BAC",    BTA_AG_AT_SET,                      BTA_AG_AT_STR,   0,   0},
    {"+CKPD",   BTA_AG_AT_SET,                      BTA_AG_AT_INT,   0, 200}, //SC for HSP
    {"",        BTA_AG_AT_NONE,                     BTA_AG_AT_STR,   0,   0}
};

/* AT result code table element */
typedef struct
{
    const char  *p_res;         /* AT result string */
    uint8_t       fmt;            /* whether argument is int or string */
} tBTA_AG_RESULT;

/* AT result code argument types */
enum
{
    BTA_AG_RES_FMT_NONE,       /* no argument */
    BTA_AG_RES_FMT_INT,        /* integer argument */
    BTA_AG_RES_FMT_STR         /* string argument */
};

/* enumeration of AT result codes, matches constant table */
enum
{
    BTA_AG_RES_OK,
    BTA_AG_RES_ERROR,
    BTA_AG_RES_CHLD,
    BTA_AG_RES_CIND,
    BTA_AG_RES_CIEV,
    BTA_AG_RES_BVRA,
    BTA_AG_RES_BRSF,
    BTA_AG_RES_BCS,
    BTA_AG_RES_COPS,
    BTA_AG_RES_RING,
    BTA_AG_RES_VGM,
    BTA_AG_RES_VGS,
    BTA_AG_RES_CLCC,
    BTA_AG_RES_MAX,
};

/* AT result code constant table  (Indexed by result code) */
const tBTA_AG_RESULT bta_ag_result_tbl[] =
{
    {"OK",      BTA_AG_RES_FMT_NONE},
    {"ERROR",   BTA_AG_RES_FMT_NONE},
    {"+CHLD: ", BTA_AG_RES_FMT_STR},
    {"+CIND: ", BTA_AG_RES_FMT_STR},
    {"+CIEV: ", BTA_AG_RES_FMT_STR},
    {"+BVRA: ", BTA_AG_RES_FMT_INT},
    {"+BRSF: ", BTA_AG_RES_FMT_INT},
    {"+BCS: ",  BTA_AG_RES_FMT_INT},
    {"+COPS: ", BTA_AG_RES_FMT_STR},
    {"RING",    BTA_AG_RES_FMT_NONE},
    {"+VGM=",   BTA_AG_RES_FMT_INT},
    {"+VGS=",   BTA_AG_RES_FMT_INT},
    {"+CLCC: ", BTA_AG_RES_FMT_STR},
    {"",        BTA_AG_RES_FMT_STR}
};

extern wiced_bt_hfp_ag_hci_send_ag_event_cback_t hfp_ag_hci_send_ag_event;
/*******************************************************************************
**
** Function         _send_result_to_hf
**
** Description      Send an AT result code.
**
**
** Returns          void
**
*******************************************************************************/
static uint8_t _send_result_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t code, char *p_arg, int16_t int_arg)
{
    char    *buf = (char*)wiced_bt_get_buffer(HFP_AG_AT_MAX_LEN + 16);
    if(buf == NULL)
    {
        WICED_BT_TRACE("[%s]:OUT OF MEMORY",__FUNCTION__);
        return FALSE;
    }
    char    *p = buf;
    wiced_bt_rfcomm_result_t    ret;

    /* init with \r\n */
    *p++ = '\r';
    *p++ = '\n';

    /* copy result code string */
    BCM_STRNCPY_S(p, sizeof(buf), bta_ag_result_tbl[code].p_res, HFP_AG_AT_MAX_LEN);
    p += strlen(bta_ag_result_tbl[code].p_res);

    /* copy argument if any */
    if (bta_ag_result_tbl[code].fmt == BTA_AG_RES_FMT_INT)
    {
        p += utl_itoa((uint16_t) int_arg, p);
    }
    else if (bta_ag_result_tbl[code].fmt == BTA_AG_RES_FMT_STR)
    {
        if(p_arg)
        {
            BCM_STRNCPY_S(p, sizeof(buf), p_arg, HFP_AG_AT_MAX_LEN);
            p += strlen(p_arg);
        }
    }

    /* finish with \r\n */
    *p++ = '\r';
    *p++ = '\n';

    /* send to RFCOMM */
#if BTSTACK_VER >= 0x03000001
    ret = wiced_bt_rfcomm_write_data( p_scb->rfc_conn_handle, buf, ( uint16_t )( p - buf ) );
#else
    ret = wiced_bt_rfcomm_write_data( p_scb->rfc_conn_handle, buf, ( uint16_t )( p - buf ), &len );
#endif

    if ( ret != WICED_BT_RFCOMM_SUCCESS)
    {
        WICED_BT_TRACE( "[%u]Sent AT fail[0x%02x][ret %d]\n", p_scb->app_handle, code, ret );
        wiced_bt_free_buffer(buf);
        return FALSE;
    }

    WICED_BT_TRACE( "[%u]Sent AT response[0x%02x][ret %d][hndl %d]: %s", p_scb->app_handle, code, ret, p_scb->rfc_conn_handle, &buf[2] );

    return TRUE;
}

/*******************************************************************************
**
** Function         _send_OK_to_hf
**
** Description      Send an OK result code.
**
**
** Returns          void
**
*******************************************************************************/
static void _send_OK_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb)
{
    WICED_BT_TRACE("[%s]",__FUNCTION__);
    _send_result_to_hf (p_scb, BTA_AG_RES_OK, NULL, 0);
}

/*******************************************************************************
**
** Function         _send_error_to_hf
**
** Description      Send an ERROR result code.
**                  errcode - used to send verbose errocode
**
**
** Returns          void
**
*******************************************************************************/
static void _send_error_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, int16_t errcode)
{
    /* If HFP and extended audio gateway error codes are enabled */
    _send_result_to_hf (p_scb, BTA_AG_RES_ERROR, NULL, 0);
}

/*******************************************************************************
**
** Function         _parse_bia_command
**
** Description      Parse AT+BIA parameter string.
**
** Returns          None.
**
*******************************************************************************/
static void _parse_bia_command (wiced_bt_hfp_ag_session_cb_t *p_scb, char *p_s)
{
    int                 indicator_index = 0;

    while (p_s)
    {
        switch(*p_s)
        {
            case ',':
                indicator_index++;
                break;
            case '0':
                // Don't disable mandatory indicators
                if (indicator_index > 2)
                    p_scb->indicator_bit_map &= ~(1<<indicator_index);
                break;
            case '1':
                p_scb->indicator_bit_map |= (1<<indicator_index);
                break;
            case 0:
            default:
                return;
        }
        if (indicator_index >= BTA_AG_NUM_INDICATORS)
            return;
        p_s++;
    }
}


#if (BTM_WBS_INCLUDED == TRUE )
/*******************************************************************************
**
** Function         _parse_bac_command
**
** Description      Parse AT+BAC parameter string.
**
** Returns          Returns bitmap of supported codecs.
**
*******************************************************************************/
static void _parse_bac_command (wiced_bt_hfp_ag_session_cb_t *p_scb, char *p_s)
{
    uint16_t              codec;
    wiced_bool_t cont = FALSE; /* Continue processing */
    char                *p;

    p_scb->peer_supports_msbc = FALSE;

    while (p_s)
    {
        /* skip to comma delimiter */
        for (p = p_s; *p != ',' && *p != 0; p++)
            ;

        /* get integer value */
        if (*p != 0)
        {
            *p = 0;
            cont = TRUE;
        }
        else
            cont = FALSE;

        codec = utl_str2int(p_s);
        switch (codec)
        {
            case HFP_CODEC_CVSD:
                break;
            case HFP_CODEC_MSBC:
                p_scb->peer_supports_msbc = TRUE;
                break;
            default:
                /* Ignore any proprietary codecs */
                WICED_BT_TRACE ("Unknown Codec UUID(%d) received", codec);
                break;
        }

        if (cont)
            p_s = p + 1;
        else
            break;
    }
}
#endif


/*******************************************************************************
**
** Function         _handle_command_from_hf
**
** Description      AT command processing.
**
** Returns          void
**
*******************************************************************************/
static void _handle_command_from_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, uint16_t cmd, uint8_t arg_type, char *p_arg, int16_t int_arg)
{
    WICED_BT_TRACE ("HFP AT cmd:%d arg_type:%d arg:%d arg:%s\n", cmd, arg_type, int_arg, p_arg);

    switch (cmd)
    {
        case BTA_AG_HF_CMD_D:
            if (arg_type == BTA_AG_AT_NONE)
                _send_OK_to_hf(p_scb);                          /* send OK */
            break;
        case BTA_AG_HF_CMD_A:
        case BTA_AG_HF_CMD_VGS:
        case BTA_AG_HF_CMD_VGM:
        case BTA_AG_HF_CMD_CHUP:
        case BTA_AG_HF_CMD_CBC:
        case BTA_AG_HF_CMD_CNUM:
        case BTA_AG_HF_CMD_CCWA:
        case BTA_AG_HF_CMD_CMEE:
        case BTA_AG_HF_CMD_VTS:
            _send_OK_to_hf(p_scb);                          /* send OK */
            break;
        case BTA_AG_HF_CMD_BIA:
            _parse_bia_command(p_scb, p_arg);
            _send_OK_to_hf(p_scb);                          /* send OK */
            break;
        case BTA_AG_HF_CMD_CKPD:                            /* for HSP */
            _send_OK_to_hf(p_scb);                          /* send OK */

            if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HEADSET)
            {
                if(!p_scb->b_call_is_up)
                {
                    WICED_BT_TRACE("HSP call is up.\n");
                    p_scb->b_call_is_up = WICED_TRUE;
                    if(!p_scb->b_sco_opened)
                        wiced_bt_hfp_ag_audio_open( p_scb->app_handle );
                }
                else
                {
                    WICED_BT_TRACE("HSP call is down.\n");
                    p_scb->b_call_is_up = WICED_FALSE;
                    if(p_scb->b_sco_opened)
                        wiced_bt_hfp_ag_audio_close( p_scb->app_handle );
                }
            }
            break;
        case BTA_AG_HF_CMD_CLCC:
            if(hfp_ag_hci_send_ag_event)
                hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_CLCC_REQ, p_scb->app_handle, NULL );
            break;
        case BTA_AG_HF_CMD_BINP:
        case BTA_AG_HF_CMD_NREC:
        case BTA_AG_HF_CMD_BTRH:
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
            break;

        case BTA_AG_HF_CMD_CHLD:
            if (arg_type == BTA_AG_AT_TEST)
            {
                /* send CHLD string followed by "OK" */
                _send_result_to_hf(p_scb, BTA_AG_RES_CHLD, "(0,1,2,3,4)", 0);

                /* if service level conn. not already open and our features or
                ** peer features does not have HF indicator, service level conn. now open
                ** Otherwise, SLC will be open later.
                */
                if ( !((ag_features & HFP_AG_FEAT_HF_IND) && (p_scb->hf_features & HFP_HF_FEAT_HF_IND)) )
                {
                    hfp_ag_service_level_up (p_scb);
                }
            }
            else if (arg_type == BTA_AG_AT_SET)
            {
                // receive enhance call control command and device won't support
                if ( (strlen(p_arg) == 2) && !(ag_features & HFP_AG_FEAT_ECC))
                {
                    _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
                    return;
                }
            }
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_CIND:
            if (arg_type == BTA_AG_AT_TEST)
            {
                _send_result_to_hf (p_scb, BTA_AG_RES_CIND, BTA_AG_CIND_INFO, 0);
            }
            else if (arg_type == BTA_AG_AT_READ)
            {
                _send_result_to_hf (p_scb, BTA_AG_RES_CIND, BTA_AG_CIND_VALUES, 0);
            }
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_CLIP:
            /* store setting, send OK */
            p_scb->clip_enabled = (wiced_bool_t)int_arg;
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_BVRA:
            _send_OK_to_hf (p_scb);
            if (arg_type == BTA_AG_AT_SET)
            {
                if ((int_arg == 1) && (!p_scb->b_sco_opened))
                    hfp_ag_sco_create (p_scb, TRUE);
                else if ((int_arg == 0) && (p_scb->b_sco_opened))
                    hfp_ag_sco_close (p_scb);
            }
            break;

        case BTA_AG_HF_CMD_BRSF:
            /* store peer features */
            p_scb->hf_features = (uint16_t) int_arg;
            WICED_BT_TRACE ("BRSF HF : 0x%x , phone : 0x%x", p_scb->hf_features, ag_features);

            /* send BRSF, send OK */
            _send_result_to_hf(p_scb, BTA_AG_RES_BRSF, NULL, (int16_t)ag_features);
            _send_OK_to_hf(p_scb);
            break;

        case BTA_AG_HF_CMD_COPS:
            _send_OK_to_hf(p_scb);
            if (arg_type == BTA_AG_AT_READ)
            {
                _send_result_to_hf(p_scb, BTA_AG_RES_COPS, "0", 0);
            }
            break;

        case BTA_AG_HF_CMD_CMER:
            _send_OK_to_hf(p_scb);                          /* send OK */
            /*IOP issue with BOSE headset check BTSTACK-3518
            check for spaces in the receives command */
            utl_ignore_spaces(p_arg);
            // 3,0,0,1 or 3,0,0,0
            if (p_arg[6] == '1')
            {
                p_scb->cmer_enabled = WICED_TRUE;
                hfp_ag_service_level_up (p_scb);
            }
            else if (p_arg[6] == '0')
            {
                p_scb->cmer_enabled = WICED_FALSE;
            }
            break;

#if (BTM_WBS_INCLUDED == TRUE )
        case BTA_AG_HF_CMD_BAC:
            _send_OK_to_hf(p_scb);

            /* parse codecs and set a flag if peer supports mSBC */
            _parse_bac_command (p_scb, p_arg);
            break;

        case BTA_AG_HF_CMD_BCS:
            _send_OK_to_hf(p_scb);

            /* stop cn timer */
            wiced_stop_timer(&p_scb->cn_timer);

            if (int_arg == HFP_CODEC_MSBC)
                p_scb->msbc_selected = WICED_TRUE;
            else
                p_scb->msbc_selected = WICED_FALSE;

            hfp_ag_sco_create (p_scb, TRUE);
            break;

        case BTA_AG_HF_CMD_BCC:
            _send_OK_to_hf(p_scb);
            hfp_ag_sco_create (p_scb, TRUE);
            break;
#endif

        default:
            break;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_hfp_ag_send_BVRA_to_hf
**
** Description      Send +BVRA AT unsolicited response to peer.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_hfp_ag_send_BVRA_to_hf(wiced_bt_hfp_ag_session_cb_t *p_scb, wiced_bool_t b_is_active)
{
    _send_result_to_hf (p_scb, BTA_AG_RES_BVRA, NULL, b_is_active ? 1 : 0);
}

void wiced_bt_hfp_ag_send_RING_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb)
{
    _send_result_to_hf (p_scb, BTA_AG_RES_RING, NULL, 0);
}

uint8_t wiced_bt_hfp_ag_send_VGM_to_hf(wiced_bt_hfp_ag_session_cb_t *p_scb, int16_t gain)
{
    return _send_result_to_hf (p_scb, BTA_AG_RES_VGM, NULL, gain);
}

uint8_t wiced_bt_hfp_ag_send_VGS_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, int16_t gain)
{
    return _send_result_to_hf (p_scb, BTA_AG_RES_VGS, NULL, gain);
}

static wiced_bool_t hfp_ag_is_ciev_allowed(wiced_bt_hfp_ag_session_cb_t *p_scb, char *data)
{
    char at_cmd[10] = "+CIEV: ";

    if ( memcmp(data, at_cmd, strlen(at_cmd)) == 0)
    {
        int indicator_index = data[7]-'0';
        if (p_scb->cmer_enabled == WICED_FALSE)
            return WICED_FALSE;

        return (p_scb->indicator_bit_map & (1 << (indicator_index-1)));
    }
    return WICED_TRUE;
}

uint8_t wiced_bt_hfp_ag_send_cmd_str_to_hf(wiced_bt_hfp_ag_session_cb_t *p_scb, char *data)
{
    // For CIEV verify indicator is enabled or not
    if ( hfp_ag_is_ciev_allowed(p_scb, data) )
    {
        return _send_result_to_hf(p_scb, BTA_AG_RES_MAX, data, 0);
    }
    return 0;
}

void wiced_bt_hfp_ag_send_OK_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb)
{
    _send_OK_to_hf(p_scb);
}

void wiced_bt_hfp_ag_send_Error_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, int error_code)
{
    _send_error_to_hf(p_scb, error_code);
}


void wiced_bt_hfp_ag_set_cind(char *cind_val, uint8_t length)
{
    if (length <= sizeof(BTA_AG_CIND_VALUES))
        memcpy(BTA_AG_CIND_VALUES, cind_val, length);
}

#if (BTM_WBS_INCLUDED == TRUE )
/*******************************************************************************
**
** Function         wiced_bt_hfp_ag_send_BCS_to_hf
**
** Description      Send +BCS AT unsolicited response to peer.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_hfp_ag_send_BCS_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb)
{
    int16_t codec_uuid;

    /* Try to use mSBC if the peer supports it */
    if (p_scb->peer_supports_msbc)
        codec_uuid = HFP_CODEC_MSBC;
    else
        codec_uuid = HFP_CODEC_CVSD;

    /* send +BCS */
    WICED_BT_TRACE ("send +BCS codec is %d", codec_uuid);

    _send_result_to_hf (p_scb, BTA_AG_RES_BCS, NULL, codec_uuid);
}
#endif

/******************************************************************************
**
** Function         hfp_ag_parse_AT_command
**
** Description      Parse AT commands.  This function will take the input
**                  character string and parse it for AT commands according to
**                  the AT command table passed in the control block.
**
** Returns          void
**
******************************************************************************/
void hfp_ag_parse_AT_command (wiced_bt_hfp_ag_session_cb_t *p_scb)
{
    int         i;
    uint16_t      idx;
    uint8_t       arg_type;
    char        *p_arg;
    int16_t       int_arg = 0;
    wiced_bt_hfp_ag_at_cmd_t at_cmd;

    /* Remove CR/LF from the buffer */
    for (i = 0; i < p_scb->res_len; i++)
        if ( (p_scb->res_buf[i] == '\r') || (p_scb->res_buf[i] == '\n') )
          p_scb->res_buf[i] = 0;

    at_cmd.cmd_ptr = (uint8_t *)p_scb->res_buf;
    at_cmd.cmd_len = p_scb->res_len;

    for (i = 0; i < p_scb->res_len; i++)
    {
        if ( ((p_scb->res_buf[i]   != 'A') && (p_scb->res_buf[i]   != 'a'))
          || ((p_scb->res_buf[i+1] != 'T') && (p_scb->res_buf[i+1] != 't')) )
            continue;

        /* Got an "AT" */
        i += 2;

        /* loop through at command table looking for match */
        for (idx = 0; bta_ag_hfp_cmd[idx].p_cmd[0] != 0; idx++)
        {
            if (!utl_strucmp(bta_ag_hfp_cmd[idx].p_cmd, &p_scb->res_buf[i]))
                break;
        }

        if (bta_ag_hfp_cmd[idx].p_cmd[0] == 0)
        {
            WICED_BT_TRACE("hfp_ag_parse_AT_command: BTA_AG_ERR_OP_NOT_SUPPORTED\n");
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
            break;
        }

        /* Got a match; verify argument type */
        /* start of argument is p + strlen matching command */
        p_arg = p_scb->res_buf + i + strlen(bta_ag_hfp_cmd[idx].p_cmd);

        if (p_arg[0] == 0)                                  /* if no argument */
        {
            arg_type = BTA_AG_AT_NONE;
        }
        else if (p_arg[0] == '?' && p_arg[1] == 0)          /* else if arg is '?' and it is last character */
        {
            /* we have a read */
            arg_type = BTA_AG_AT_READ;
        }
        else if (p_arg[0] == '=' && p_arg[1] != 0)          /* else if arg is '=' */
        {
            if (p_arg[1] == '?' && p_arg[2] == 0)
            {
                /* we have a test */
                arg_type = BTA_AG_AT_TEST;
            }
            else
            {
                /* we have a set */
                arg_type = BTA_AG_AT_SET;

                /* skip past '=' */
                p_arg++;
            }
        }
        else                                                /* else it is freeform argument */
        {
            arg_type = BTA_AG_AT_FREE;
        }

        /* if arguments match command capabilities */
        if ((arg_type & bta_ag_hfp_cmd[idx].arg_type) != 0)
        {
            /* if it's a set integer check max, min range */
            if (arg_type == BTA_AG_AT_SET && bta_ag_hfp_cmd[idx].fmt == BTA_AG_AT_INT)
            {
                int_arg = utl_str2int(p_arg);

                if (int_arg < (int16_t) bta_ag_hfp_cmd[idx].min || int_arg > (int16_t) bta_ag_hfp_cmd[idx].max)
                {
                    WICED_BT_TRACE ("arg out of range: value: %u\n", int_arg);

                    /* arg out of range; error */
                    _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
                }
                else
                {
                    _handle_command_from_hf (p_scb, idx, arg_type, p_arg, int_arg);
                }
            }
            else
            {
                _handle_command_from_hf (p_scb, idx, arg_type, p_arg, int_arg);
            }
        }
        else                /* else error */
        {
            WICED_BT_TRACE("hfp_ag_parse_AT_command: Out of range %d\n", arg_type);
            _send_error_to_hf (p_scb, BTA_AG_ERR_OP_NOT_SUPPORTED);
        }

        break;              /* Only process one command at a time */
    }

    if(hfp_ag_hci_send_ag_event)
        hfp_ag_hci_send_ag_event(WICED_BT_HFP_AG_EVENT_AT_CMD, p_scb->app_handle, (wiced_bt_hfp_ag_event_data_t *)&at_cmd);
    p_scb->res_len = 0;
}
