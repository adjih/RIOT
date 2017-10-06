/*
 * Copyright (C) 2017 Inria
 *               2017 Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     test
 *
 * @file
 * @brief       Semtech LoRaMAC test application
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Jose Alamos <jose.alamos@inria.cl>
 */

#include <string.h>

#include "msg.h"

#include "loramac/params.h"
#include "loramac/board.h"

#include "LoRaMac.h"
#include "region/Region.h"

#include "net/netdev.h"
#include "net/loramac.h"

#include "sx127x.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define LORAMAC_MSG_QUEUE       (16U)
#define LORAMAC_STACKSIZE       (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR                   (0x3456)
#define MSG_TYPE_RX_TIMEOUT            (0x3457)
#define MSG_TYPE_TX_TIMEOUT            (0x3458)
#define MSG_TYPE_LORAMAC_CMD           (0x3460)

#define LORAMAC_CMD_ENABLE_ADR         (0x01)
#define LORAMAC_CMD_SET_PUBLIC         (0x02)
#define LORAMAC_CMD_SET_REGION         (0x03)

static char stack[LORAMAC_STACKSIZE];
static kernel_pid_t _loop_pid;

static sx127x_t sx127x;
static netdev_t *netdev;

#define APP_TX_DUTYCYCLE                            5000

#define APP_TX_DUTYCYCLE_RND                        1000

#define LORAWAN_DEFAULT_DATARATE                    DR_0

#define LORAWAN_CONFIRMED_MSG_ON                    (LORAMAC_DEFAULT_TX_MODE != LORAMAC_TX_CNF)

#define LORAWAN_ADR_ON                              1

#if defined(REGION_EU868)

#include "LoRaMacTest.h"

#define LORAWAN_DUTYCYCLE_ON                        true
#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if (USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1)
#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }
#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

#define LORAWAN_PUBLIC_NETWORK                      true

#define LORAWAN_NETWORK_ID                          1

#define LORAMAC_USE_OTAA   (LORAMAC_DEFAULT_JOIN_PROCEDURE != LORAMAC_JOIN_OTAA)

/*!
 * User application data buffer size
 */
#if defined(REGION_CN779) || defined(REGION_EU868) || defined(REGION_IN865) || \
    defined(REGION_KR920)
#define LORAWAN_APP_DATA_SIZE                       16
#elif defined(REGION_AS923) || defined(REGION_AU915) || defined(REGION_US915) || \
      defined(REGION_US915_HYBRID)
#define LORAWAN_APP_DATA_SIZE                       11
#else
#error "Please define a region in the compiler options."
#endif

static char payload[LORAWAN_APP_DATA_SIZE] = { 0 };

static uint8_t DevEui[] = LORAMAC_DEV_EUI_DEFAULT;
static uint8_t AppEui[] = LORAMAC_APP_EUI_DEFAULT;
static uint8_t AppKey[] = LORAMAC_APP_KEY_DEFAULT;

#if !(LORAMAC_USE_OTAA)
static uint8_t NwkSKey[] = LORAMAC_NET_SKEY_DEFAULT;
static uint8_t AppSKey[] = LORAMAC_APP_SKEY_DEFAULT;
static uint32_t DevAddr = LORAMAC_DEV_ADDR_DEFAULT;
#endif

static uint8_t AppPort = LORAWAN_APP_PORT;

static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

#define LORAWAN_APP_DATA_MAX_SIZE                           242

static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

static uint32_t TxDutyCycleTime;

static TimerEvent_t TxNextPacketTimer;

static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} DeviceState;

struct ComplianceTest_s {
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
} ComplianceTest;


uint8_t BoardGetBatteryLevel(void) {
    return 50;
}

void BoardGetUniqueId(uint8_t *DevEui) {
    *DevEui = 10;
}


/* Prepares the payload of the frame */
static void PrepareTxFrame(uint8_t port)
{
    switch(port) {
        case 2:
        {
            size_t p = 0;
            p += snprintf(payload, 14, "This is RIOT!");
            payload[p] = '\0';
            memcpy(AppData, payload, p);
            break;
        }

        case 224:
            if (ComplianceTest.LinkCheck == true) {
                ComplianceTest.LinkCheck = false;
                AppDataSize = 3;
                AppData[0] = 5;
                AppData[1] = ComplianceTest.DemodMargin;
                AppData[2] = ComplianceTest.NbGateways;
                ComplianceTest.State = 1;
            }
            else {
                switch(ComplianceTest.State) {
                    case 4:
                        ComplianceTest.State = 1;
                        break;
                    case 1:
                        AppDataSize = 2;
                        AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                        AppData[1] = ComplianceTest.DownLinkCounter;
                        break;
                }
            }
            break;

        default:
            break;
    }
}

/* Prepares the payload of the frame */
static bool SendFrame(void)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if (LoRaMacQueryTxPossible(AppDataSize, &txInfo) != LORAMAC_STATUS_OK) {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else {
        if (IsTxConfirmed == false) {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if (LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK) {
        return false;
    }

    return true;
}

/* Function executed on TxNextPacket Timeout event */
static void OnTxNextPacketTimerEvent(void)
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mibReq);

    if (status == LORAMAC_STATUS_OK) {
        if (mibReq.Param.IsNetworkJoined == true) {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/* MCPS-Confirm event function */
static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
    if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        switch(mcpsConfirm->McpsRequest) {
            case MCPS_UNCONFIRMED:
                /* Check Datarate
                   Check TxPower */
                break;

            case MCPS_CONFIRMED:

                /* Check Datarate
                   Check TxPower
                   Check AckReceived
                   Check NbTrials */
                break;

            case MCPS_PROPRIETARY:
                break;

            default:
                break;
        }
    }
    NextTx = true;
}

/* MCPS-Indication event function */
static void McpsIndication(McpsIndication_t *mcpsIndication)
{
    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
        return;
    }

    switch (mcpsIndication->McpsIndication) {
        case MCPS_UNCONFIRMED:
            break;

        case MCPS_CONFIRMED:
            break;

        case MCPS_PROPRIETARY:
            break;

        case MCPS_MULTICAST:
            break;

        default:
            break;
    }

    /* Check Multicast
       Check Port
       Check Datarate
       Check FramePending
       Check Buffer
       Check BufferSize
       Check Rssi
       Check Snr
       Check RxSlot */

    if (ComplianceTest.Running == true) {
        ComplianceTest.DownLinkCounter++;
    }

    if (mcpsIndication->RxData == true) {
        switch(mcpsIndication->Port) {
            case 1:
            case 2:
                /* if (mcpsIndication->BufferSize == 1) {
                    AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                } */
                break;

            case 224:
                if (ComplianceTest.Running == false) {
                    /* Check compliance test enable command (i) */
                    if ((mcpsIndication->BufferSize == 4) &&
                        (mcpsIndication->Buffer[0] == 0x01) &&
                        (mcpsIndication->Buffer[1] == 0x01) &&
                        (mcpsIndication->Buffer[2] == 0x01) &&
                        (mcpsIndication->Buffer[3] == 0x01)) {
                        IsTxConfirmed = false;
                        AppPort = 224;
                        AppDataSize = 2;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.LinkCheck = false;
                        ComplianceTest.DemodMargin = 0;
                        ComplianceTest.NbGateways = 0;
                        ComplianceTest.Running = true;
                        ComplianceTest.State = 1;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = true;
                        LoRaMacMibSetRequestConfirm(&mibReq);

#if defined(REGION_EU868)
                        LoRaMacTestSetDutyCycleOn(false);
#endif
                    }
                }
                else {
                    ComplianceTest.State = mcpsIndication->Buffer[0];
                    switch(ComplianceTest.State) {
                        case 0: /* Check compliance test disable command (ii) */
                            IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                            AppPort = LORAWAN_APP_PORT;
                            AppDataSize = LORAWAN_APP_DATA_SIZE;
                            ComplianceTest.DownLinkCounter = 0;
                            ComplianceTest.Running = false;

                            MibRequestConfirm_t mibReq;
                            mibReq.Type = MIB_ADR;
                            mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                            LoRaMacMibSetRequestConfirm(&mibReq);
#if defined(REGION_EU868)
                            LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif
                            break;
                        case 1: /* (iii, iv) */
                            AppDataSize = 2;
                            break;

                        case 2: /* Enable confirmed messages (v) */
                            IsTxConfirmed = true;
                            ComplianceTest.State = 1;
                            break;

                        case 3:  /* Disable confirmed messages (vi) */
                            IsTxConfirmed = false;
                            ComplianceTest.State = 1;
                            break;

                        case 4: /* (vii) */
                            AppDataSize = mcpsIndication->BufferSize;

                            AppData[0] = 4;
                            for (uint8_t i = 1; i < MIN(AppDataSize, LORAWAN_APP_DATA_MAX_SIZE); i++) {
                                AppData[i] = mcpsIndication->Buffer[i] + 1;
                            }
                            break;

                        case 5: /* (viii) */
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_LINK_CHECK;
                            LoRaMacMlmeRequest(&mlmeReq);
                        }
                            break;

                        case 6: /* (ix) */
                        {
                            MlmeReq_t mlmeReq;

                            /* Disable TestMode and revert back to normal operation */
                            IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                            AppPort = LORAWAN_APP_PORT;
                            AppDataSize = LORAWAN_APP_DATA_SIZE;
                            ComplianceTest.DownLinkCounter = 0;
                            ComplianceTest.Running = false;

                            MibRequestConfirm_t mibReq;
                            mibReq.Type = MIB_ADR;
                            mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                            LoRaMacMibSetRequestConfirm(&mibReq);
#if defined(REGION_EU868)
                            LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif

                            mlmeReq.Type = MLME_JOIN;

                            mlmeReq.Req.Join.DevEui = DevEui;
                            mlmeReq.Req.Join.AppEui = AppEui;
                            mlmeReq.Req.Join.AppKey = AppKey;
                            mlmeReq.Req.Join.NbTrials = 3;

                            LoRaMacMlmeRequest(&mlmeReq);
                            DeviceState = DEVICE_STATE_SLEEP;
                        }
                            break;
                        case 7: /* (x) */
                            if (mcpsIndication->BufferSize == 3) {
                                MlmeReq_t mlmeReq;
                                mlmeReq.Type = MLME_TXCW;
                                mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
                                LoRaMacMlmeRequest(&mlmeReq);
                            }
                            else if (mcpsIndication->BufferSize == 7) {
                                MlmeReq_t mlmeReq;
                                mlmeReq.Type = MLME_TXCW_1;
                                mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
                                mlmeReq.Req.TxCw.Frequency = (uint32_t)((mcpsIndication->Buffer[3] << 16) | (mcpsIndication->Buffer[4] << 8) | mcpsIndication->Buffer[5]) * 100;
                                mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                                LoRaMacMlmeRequest(&mlmeReq);
                            }
                            ComplianceTest.State = 1;
                            break;
                        default:
                            break;
                    }

                }
                break;

            default:
                break;
        }
    }
}

/*MLME-Confirm event function */
static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
    switch (mlmeConfirm->MlmeRequest) {
        case MLME_JOIN:
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                /* Status is OK, node has joined the network */
                DeviceState = DEVICE_STATE_SEND;
            }
            else {
                /* Join was not successful. Try to join again */
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        case MLME_LINK_CHECK:
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                /* Check DemodMargin
                   Check NbGateways */
                if (ComplianceTest.Running == true) {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;

        default:
            break;
    }

    NextTx = true;
}

static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    size_t len;
    netdev_sx127x_lora_packet_info_t packet_info;
    RadioEvents_t *events = radio_get_event_ptr();

	msg_t msg;
	msg.content.ptr = dev;

    switch (event) {

        case NETDEV_EVENT_ISR:
            msg.type = MSG_TYPE_ISR;
            if (msg_send(&msg, _loop_pid) <= 0) {
                DEBUG("[semtech-loramac] test: possibly lost interrupt.\n");
            }
            break;

        case NETDEV_EVENT_TX_COMPLETE:
            events->TxDone();
            DEBUG("Transmission completed\n");
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            msg.type = MSG_TYPE_TX_TIMEOUT;

            if (msg_send(&msg, _loop_pid) <= 0) {
                DEBUG("[semtech-loramac] test: TX timeout, possibly lost "
                      "interrupt.\n");
            }
            break;

        case NETDEV_EVENT_RX_COMPLETE:
            len = dev->driver->recv(dev, NULL, 0, 0);
            dev->driver->recv(dev, payload, len, &packet_info);
            events->RxDone((uint8_t*)payload, len, packet_info.rssi,
                           packet_info.snr);
            printf("{Payload: \"%s\" (%d bytes), RSSI: %i, SNR: %i, TOA: %i}\n",
                   payload, (int)len,
                   packet_info.rssi, (int)packet_info.snr,
                   (int)packet_info.time_on_air);
            break;

        case NETDEV_EVENT_RX_TIMEOUT:
            msg.type = MSG_TYPE_RX_TIMEOUT;

            if (msg_send(&msg, _loop_pid) <= 0) {
                DEBUG("[semtech-loramac] test: RX timeout, possibly lost "
                      "interrupt.\n");
            }
            break;

        case NETDEV_EVENT_CRC_ERROR:
            DEBUG("[semtech-loramac] test: RX CRC error\n");
            events->RxError();
            break;

        case NETDEV_EVENT_FHSS_CHANGE_CHANNEL:
            DEBUG("[semtech-loramac] test: FHSS channel change\n");
            events->FhssChangeChannel(sx127x._internal.last_channel);
            break;

        case NETDEV_EVENT_CAD_DONE:
            DEBUG("[semtech-loramac] test: CAD done\n");
            events->CadDone(sx127x._internal.is_last_cad_success);
            break;

        default:
            DEBUG("[semtech-loramac] test: unexpected netdev event "
                  "received: %d\n", event);
            break;
    }
}

void *_event_loop(void *arg)
{
    static msg_t _msg_q[LORAMAC_MSG_QUEUE];
    msg_init_queue(_msg_q, LORAMAC_MSG_QUEUE);
    RadioEvents_t *events;
	msg_t reply;

    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;

	DEBUG("[semtech-loramac] test: initializing loramac\n");
	LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
	LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
	LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
	LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
#if defined(REGION_AS923)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_AS923);
#elif defined(REGION_AU915)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_AU915);
#elif defined(REGION_CN779)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_CN779);
#elif defined(REGION_EU868)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_EU868);
#elif defined(REGION_IN865)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_IN865);
#elif defined(REGION_KR920)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_KR920);
#elif defined(REGION_US915)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_US915);
#elif defined(REGION_US915_HYBRID)
	LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks,
						  LORAMAC_REGION_US915_HYBRID);
#else
#error "Please define a region in the compiler options."
#endif


    while (1) {
        msg_t msg;
        msg_receive(&msg);
		events = radio_get_event_ptr();
        switch(msg.type) {
            case MSG_TYPE_ISR:
            {
                netdev_t *dev = msg.content.ptr;
                dev->driver->isr(dev);
            }
                break;

            case MSG_TYPE_RX_TIMEOUT:
                DEBUG("[semtech-loramac] test: RX timeout\n");
                events->RxTimeout();
                break;

            case MSG_TYPE_TX_TIMEOUT:
                DEBUG("[semtech-loramac] test: TX timeout\n");
                events->TxTimeout();
                break;

			case MSG_TYPE_MAC_TIMEOUT:
                DEBUG("[semtech-loramac] test: MAC timeout\n");
				void (*function)(void) = msg.content.ptr;
				function();
				break;
			case MSG_TYPE_LORAMAC_CMD:
                DEBUG("[semtech-loramac] test: MAC timeout\n");
				void (*cmd)(void) = msg.content.ptr;
				cmd();
				msg_reply(&msg, &reply);
				break;
            default:
                DEBUG("Unexpected msg type: %04x\n", msg.type);
                break;
        }
    }
}

typedef void LORAMAC_CMD;
LORAMAC_CMD _loramac_enable_adr(void)
{
    MibRequestConfirm_t mibReq;
	mibReq.Type = MIB_ADR;
	mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
	LoRaMacMibSetRequestConfirm(&mibReq);
}

LORAMAC_CMD _loramac_set_public(void)
{
    MibRequestConfirm_t mibReq;
	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
	LoRaMacMibSetRequestConfirm(&mibReq);
}

LORAMAC_CMD _loramac_setup_region(void)
{
    MibRequestConfirm_t mibReq;
#if defined(REGION_EU868)
	LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);

#if (USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1)
	LoRaMacChannelAdd(3, (ChannelParams_t)LC4);
	LoRaMacChannelAdd(4, (ChannelParams_t)LC5);
	LoRaMacChannelAdd(5, (ChannelParams_t)LC6);
	LoRaMacChannelAdd(6, (ChannelParams_t)LC7);
	LoRaMacChannelAdd(7, (ChannelParams_t)LC8);
	LoRaMacChannelAdd(8, (ChannelParams_t)LC9);
	LoRaMacChannelAdd(9, (ChannelParams_t)LC10);

	mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
	mibReq.Param.Rx2DefaultChannel = (Rx2ChannelParams_t){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_RX2_CHANNEL;
	mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
}

void exec_loramac_cmd(void (*cb)(void))
{
	msg_t msg;
	msg.type = MSG_TYPE_LORAMAC_CMD;
	msg.content.ptr = cb;
	msg_send_receive(&msg, &msg, _loop_pid);
}
/**
 * Main application entry point.
 */
int main( void )
{
    sx127x_setup(&sx127x, &sx127x_params[0]);
    netdev = (netdev_t*) &sx127x;
    netdev->driver = &sx127x_driver;
    netdev->event_callback = _event_cb;
    MibRequestConfirm_t mibReq;
	(void) mibReq;

    radio_set_ptr(&sx127x);
    xtimer_init();

    DeviceState = DEVICE_STATE_INIT;

    _loop_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _event_loop, NULL,
                              "recv_thread");

    if (_loop_pid <= KERNEL_PID_UNDEF) {
        DEBUG("Creation of receiver thread failed\n");
        return -1;
    }

	set_mac_pid(_loop_pid);
    while(1) {
        switch(DeviceState) {
            case DEVICE_STATE_INIT:
            {
                TimerInit(&TxNextPacketTimer, OnTxNextPacketTimerEvent);

				exec_loramac_cmd(_loramac_enable_adr);
				exec_loramac_cmd(_loramac_set_public);
				exec_loramac_cmd(_loramac_setup_region);

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }

            case DEVICE_STATE_JOIN:
            {
#if (LORAMAC_USE_OTAA)
                DEBUG("[semtech-loramac] test: starting OTAA join\n");
                MlmeReq_t mlmeReq;

                /* Initialize LoRaMac device unique ID */
                BoardGetUniqueId(DevEui);

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if (NextTx == true) {
                    LoRaMacMlmeRequest(&mlmeReq);
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else /* ABP join procedure */
                DEBUG("[semtech-loramac] test: starting ABP join\n");
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm(&mibReq);

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }

            case DEVICE_STATE_SEND:
                DEBUG("[semtech-loramac] test: sending frame\n");
                if (NextTx) {
                    PrepareTxFrame(AppPort);

                    NextTx = SendFrame();
                }
                if (ComplianceTest.Running) {
                    /* Schedule next packet transmission */
                    TxDutyCycleTime = 5000; /* 5000 ms */
                }
                else {
                    /* Schedule next packet transmission */
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;

            case DEVICE_STATE_CYCLE:
                DEBUG("[semtech-loramac] test: schedule next TX\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue(&TxNextPacketTimer, TxDutyCycleTime);
                TimerStart(&TxNextPacketTimer);
                break;

            case DEVICE_STATE_SLEEP:
                /* Wake up through events */
                break;

            default:
                DeviceState = DEVICE_STATE_INIT;
                break;
        }

        xtimer_usleep(1);
    }
}
