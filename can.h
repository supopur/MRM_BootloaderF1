#ifndef MAT_CAN_H
#define MAT_CAN_H

#if HSE_VALUE == 8000000
#define CAN_BR_1000 6
#define CAN_BR_500 12
#define CAN_BR_250 24
#define CAN_BR_125 48
#define CAN_BR_100 60
#define CAN_BR_50 120
#define CAN_BR_20 300
#define CAN_BR_10 600
#endif

//-----------------------------------------------------------------------------
// CAN standard-ID packing
//
// The 11-bit standard ID is split as:
//   bits [10:3] (8 bits) - target node address
//   bits [2:0]  (3 bits) - message type
//-----------------------------------------------------------------------------

// reserved / special target addresses
#define CAN_ADDR_LOCAL 0x00 // never appears on the wire
#define CAN_ADDR_BROADCAST_NOSELF 0xFE // broadcast, sender excludes itself
#define CAN_ADDR_BROADCAST 0xFF // broadcast to everyone

// message types (bits 2:0 of the standard ID)
#define CAN_MSGTYPE_OTA_DATA 0x00 // firmware update data, always wins bus arbitration
#define CAN_MSGTYPE_DHCP 0x01 // address assignment (DISC/OFFR/ACK)
#define CAN_MSGTYPE_MASTER_OUT 0x02 // master -> slave(s), Data[0] = sub-message id
#define CAN_MSGTYPE_SLAVE_OUT 0x03 // slave -> master/slaves, Data[0] = sub-message id
#define CAN_MSGTYPE_TIME_SYNC 0x05 // best-effort light/siren sync

#define CAN_MK_ID(target, msgtype) ((uint32_t)(((uint32_t)(target) << 3) | ((msgtype) & 0x07)))
#define CAN_ID_TARGET(id) ((uint8_t)((id) >> 3))
#define CAN_ID_MSGTYPE(id) ((uint8_t)((id) & 0x07))

void can_init(uint16_t br);

// id/msk are plain 11-bit standard IDs (NOT pre-shifted into the filter
// register layout - can_filter() does that internally). Only ever matches
// standard-frame (IDE=0) traffic.
uint8_t can_filter(uint32_t id, uint32_t msk, uint8_t canfilnum);
uint8_t can_tx(CanTxMsg* msg);
uint8_t can_rx(CanRxMsg* msg);

#endif
