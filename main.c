/**
blcan

@file		main.c
@author		Matej Kogovsek (matej@hamradio.si)
@copyright	LGPL 2.1

@note CAN protocol
	Standard ID = [10:3] target node address | [2:0] message type
	(see can.h: CAN_MK_ID / CAN_ID_TARGET / CAN_ID_MSGTYPE).

	This bootloader only ever looks at CAN_MSGTYPE_OTA_DATA (0x00), addressed
	either to CAN_ADDR_BROADCAST(_NOSELF) or to our own node address, if we
	have one. Everything else (DHCP, MASTER_OUT, SLAVE_OUT, TIME_SYNC) is
	filtered out in hardware and never reaches this code - that includes the
	"reboot into bootloader" MASTER_OUT command, which the *application*
	handles by writing MAGIC_ADDR/NODEADDR_ADDR and resetting.

	OTA_DATA payload, Data[0] = sub-opcode:
	  OTA_OP_INFO  [op][product_type][page_count]  - announce an update, reset state
	  OTA_OP_DATA  [op][up to 7 raw fw bytes]       - firmware bytes, in order
	  OTA_OP_END   [op][crc32 LE]                   - finalize + verify whole image
	  OTA_OP_ABORT [op]                             - cancel in-flight update

	product_type must match this build's PRODUCT_TYPE (set at compile time,
	one value per product line) or the update is rejected outright - this is
	what stops product A from flashing product B's firmware when an update is
	broadcast to the whole bus. INFO always gets an ACK/NACK so the master
	knows who is and isn't participating. DATA/END/ABORT get a reply only
	from devices that are actively participating in the update (i.e. already
	accepted a matching INFO) - a device sitting out someone else's update
	stays completely silent instead of NACKing every frame.

	Every reply (when sent) is a 3-byte broadcast SLAVE_OUT frame:
	[our_addr][op][err_code]. err_code 0 = OK, see OTA_ERR_* below.

	A page auto-flushes (erase+program) to flash the instant PAGE_BYTES worth
	of data has been received - no separate "commit page" step. On OTA_OP_END,
	any partial final page is padded with 0xFF (the erased-flash value) before
	the whole-image CRC check, so a sender computing the expected CRC for a
	non-page-aligned image must pad the same way.

	Once OTA_OP_END verifies OK, the {page_count, crc} info page is written
	and the existing "reboot into app once the bus goes quiet" logic in
	main()'s loop (unchanged) takes over - no immediate reset is forced here.
*/

#include "stm32f10x.h"
#include "core_cm3.h"
#include "can.h"

#include <string.h>

//-----------------------------------------------------------------------------
//  Defines
//-----------------------------------------------------------------------------

#define LED_PORT GPIOC
#define LED_PIN GPIO_Pin_13

#define TMR_ID_DELAY 0
#define TMR_ID_LED 1
#define TMR_ID_NUM 2

#define PAGE_WORDS 0x100              // words per flash page (STM32F1 page = 1KB)
#define PAGE_BYTES (PAGE_WORDS * 4)

//-----------------------------------------------------------------------------
//  Typedefs
//-----------------------------------------------------------------------------

struct bl_pvars_t // size has to be a multiple of 4
{
	uint32_t app_page_count;
	uint32_t app_crc;
};

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

static const uint32_t MAGIC_VAL = (uint32_t)(0x36051bf3);
static uint32_t* const MAGIC_ADDR = (uint32_t*)(SRAM_BASE + 0x1000);

// node address, written by the application before it reboots into this
// bootloader. Valid only if the top 24 bits equal NODEADDR_MAGIC; if not
// present/valid we simply only answer to broadcast traffic.
#define NODEADDR_MAGIC 0x00C0FFEEUL
static uint32_t* const NODEADDR_ADDR = (uint32_t*)(SRAM_BASE + 0x1004);

static const uint32_t* APP_BASE = (uint32_t*)(0x08002000);
static const uint16_t PAGE_COUNT = 64 - 8;

// which product line this build is for - must match the product_type byte
// in OTA_OP_INFO or the update is rejected. Override in the makefile per
// product, e.g. -DPRODUCT_TYPE=2.
#ifndef PRODUCT_TYPE
#define PRODUCT_TYPE 0
#endif

// OTA_DATA sub-opcodes (payload[0])
#define OTA_OP_INFO  0x00
#define OTA_OP_DATA  0x01
#define OTA_OP_END   0x02
#define OTA_OP_ABORT 0x03

// OTA ack error codes (payload[2] of the SLAVE_OUT reply)
#define OTA_ERR_OK         0
#define OTA_ERR_STATE      1 // reserved (non-participants now stay silent instead)
#define OTA_ERR_PRODUCT    2 // product_type in INFO doesn't match this device
#define OTA_ERR_SIZE       3 // page_count is 0 or too big for this device
#define OTA_ERR_OVERFLOW   4 // more DATA received than page_count allows
#define OTA_ERR_INCOMPLETE 5 // END received before all pages were received
#define OTA_ERR_CRC        6 // final image CRC does not match
#define OTA_ERR_FLASH      7 // erase/program/verify failed

static const uint32_t NOCANRX_TO = 5;

//-----------------------------------------------------------------------------
//  Global variables
//-----------------------------------------------------------------------------

static volatile uint32_t uptime;
static volatile uint32_t lastcanrx;

static volatile uint32_t tmr_cnt[TMR_ID_NUM];
static uint32_t tmr_top[TMR_ID_NUM];

static uint8_t my_addr;
static uint8_t have_addr;

static uint32_t pagebuf[PAGE_WORDS];
static uint16_t page_off;    // bytes filled in pagebuf so far
static uint8_t cur_page;     // next page index to write
static uint8_t total_pages;  // expected page count for this update (from INFO)
static uint8_t ota_active;   // set once a valid INFO has been received

//-----------------------------------------------------------------------------
//  newlib required functions
//-----------------------------------------------------------------------------

void _exit(int status)
{
	while( 1 );
}

//-----------------------------------------------------------------------------
//  Timers
//-----------------------------------------------------------------------------

void tmr_set(uint8_t tid, uint32_t cnt)
{
	tmr_top[tid] = cnt;
	tmr_cnt[tid] = cnt;
}

void tmr_reset(uint8_t tid)
{
	tmr_cnt[tid] = tmr_top[tid];
}

uint8_t tmr_elapsed(uint8_t tid)
{
	return (tmr_cnt[tid] == 0);
}

void tmr_tick(void)
{
	uint8_t i;
	for( i = 0; i < TMR_ID_NUM; ++i ) {
		if( tmr_cnt[i] > 0 )
			--(tmr_cnt[i]);
	}
}

//-----------------------------------------------------------------------------
//  SysTick handler
//-----------------------------------------------------------------------------

void SysTick_Handler(void)
{
	tmr_tick();

	static uint16_t mscnt = 0;
	if( ++mscnt == 1000 ) {
		mscnt = 0;
		++uptime;
	}
}

//-----------------------------------------------------------------------------
//  delays number of ms
//-----------------------------------------------------------------------------

void _delay_ms (uint32_t ms)
{
	tmr_set(TMR_ID_DELAY, ms);
	while( !tmr_elapsed(TMR_ID_DELAY));
}

//-----------------------------------------------------------------------------
//  utility functions
//-----------------------------------------------------------------------------

uint8_t __fls_wr(const uint32_t* page, const uint32_t* buf, uint32_t len)
{
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	if( FLASH_COMPLETE != FLASH_ErasePage((uint32_t)page) ) {
		return 1;
	}

	uint32_t i;

	for( i = 0; i < len; ++i ) {
		if( FLASH_COMPLETE != FLASH_ProgramWord((uint32_t)page, *buf) ) {
			return 2;
		}
		++page;
		++buf;
	}

	return 0;
}

uint8_t fls_wr(const uint32_t* page, const uint32_t* buf, uint32_t len)
{
	// does flash equal buffer already?
	if( 0 == memcmp(page, buf, 4*len) ) {
		return 0;
	}

	FLASH_Unlock();
	uint8_t r = __fls_wr(page, buf, len);
	FLASH_Lock();
	if( r ) {
		return r;
	}

  // verify
	if( 0 != memcmp(page, buf, 4*len) ) {
		return 10;
	}

	return 0;
}

void DDR(GPIO_TypeDef* port, uint16_t pin, GPIOMode_TypeDef mode)
{
	if( port == GPIOA ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); }
	if( port == GPIOB ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); }
	if( port == GPIOC ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); }
	if( port == GPIOD ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); }
	if( port == GPIOE ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); }
	if( port == GPIOF ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); }
	if( port == GPIOG ) { RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); }

	GPIO_InitTypeDef iotd;
	iotd.GPIO_Pin = pin;
	iotd.GPIO_Speed = GPIO_Speed_2MHz;
	iotd.GPIO_Mode = mode;
	GPIO_Init(port, &iotd);
}

void PreSystemInit(void)
{
	if( *(MAGIC_ADDR) == MAGIC_VAL ) {
		*(MAGIC_ADDR) = 0;
		uint32_t sp = *(APP_BASE);
		asm volatile ("MSR msp, %0" : : "r" (sp) : );
		uint32_t app = *(APP_BASE + 1); // +1 = 4 bytes since uint32_t
		asm("bx %0\n"::"r" (app):);
	}
}

//-----------------------------------------------------------------------------
//  OTA update handling
//-----------------------------------------------------------------------------

static void ota_ack(uint8_t op, uint8_t err)
{
	CanTxMsg m;
	m.StdId = CAN_MK_ID(CAN_ADDR_BROADCAST, CAN_MSGTYPE_SLAVE_OUT);
	m.IDE = CAN_Id_Standard;
	m.RTR = CAN_RTR_Data;
	m.DLC = 3;
	m.Data[0] = have_addr ? my_addr : CAN_ADDR_BROADCAST;
	m.Data[1] = op;
	m.Data[2] = err;
	can_tx(&m);
}

static void ota_reset(void)
{
	ota_active = 0;
	page_off = 0;
	cur_page = 0;
	total_pages = 0;
}

// erases+programs flash page cur_page from pagebuf (nwords words), advances
// cur_page and re-primes pagebuf with the erased-flash fill value on success.
static uint8_t ota_flush_page(uint16_t nwords)
{
	if( fls_wr(APP_BASE + (uint32_t)cur_page * PAGE_WORDS, pagebuf, nwords) ) {
		return 0;
	}
	++cur_page;
	page_off = 0;
	memset(pagebuf, 0xFF, sizeof(pagebuf));
	return 1;
}

void process_ota_msg(CanRxMsg* msg)
{
	if( msg->DLC < 1 ) return;
	lastcanrx = uptime;

	uint8_t op = msg->Data[0];

	if( op == OTA_OP_INFO ) {
		if( msg->DLC < 3 ) return;
		uint8_t prod = msg->Data[1];
		uint8_t pc = msg->Data[2];
		ota_reset();
		if( prod != PRODUCT_TYPE ) {
			ota_ack(op, OTA_ERR_PRODUCT);
			return;
		}
		if( (pc == 0) || (pc > PAGE_COUNT) ) {
			ota_ack(op, OTA_ERR_SIZE);
			return;
		}
		total_pages = pc;
		ota_active = 1;
		memset(pagebuf, 0xFF, sizeof(pagebuf));
		ota_ack(op, OTA_ERR_OK);
		return;
	}

	if( op == OTA_OP_DATA ) {
		if( !ota_active ) return; // not participating in this update - stay silent
		if( cur_page >= total_pages ) {
			ota_ack(op, OTA_ERR_OVERFLOW);
			ota_reset();
			return;
		}

		uint8_t n = msg->DLC - 1;
		uint8_t* pb8 = (uint8_t*)pagebuf;
		uint8_t i;
		for( i = 0; i < n; ++i ) {
			if( page_off >= PAGE_BYTES ) break; // ignore stray extra bytes
			pb8[page_off++] = msg->Data[1 + i];
		}

		if( page_off >= PAGE_BYTES ) {
			if( !ota_flush_page(PAGE_WORDS) ) {
				ota_ack(op, OTA_ERR_FLASH);
				ota_reset();
			}
		}
		return;
	}

	if( op == OTA_OP_END ) {
		if( msg->DLC < 5 ) return;
		if( !ota_active ) return; // not participating in this update - stay silent

		if( page_off > 0 ) {
			uint16_t nwords = (page_off + 3) / 4;
			if( !ota_flush_page(nwords) ) {
				ota_ack(op, OTA_ERR_FLASH);
				ota_reset();
				return;
			}
		}

		if( cur_page != total_pages ) {
			ota_ack(op, OTA_ERR_INCOMPLETE);
			ota_reset();
			return;
		}

		uint32_t crc;
		memcpy(&crc, msg->Data + 1, 4); // little-endian, matches Cortex-M3

		CRC_ResetDR();
		CRC_CalcBlockCRC((uint32_t*)APP_BASE, (uint32_t)total_pages * PAGE_WORDS);
		if( CRC_GetCRC() != crc ) {
			ota_ack(op, OTA_ERR_CRC);
			ota_reset();
			return;
		}

		struct bl_pvars_t pv;
		pv.app_page_count = total_pages;
		pv.app_crc = crc;
		if( fls_wr(APP_BASE - PAGE_WORDS, (uint32_t*)&pv, sizeof(pv) / 4) ) {
			ota_ack(op, OTA_ERR_FLASH);
		} else {
			ota_ack(op, OTA_ERR_OK); // app boots once the bus goes quiet
		}
		ota_reset();
		return;
	}

	if( op == OTA_OP_ABORT ) {
		if( !ota_active ) return; // nothing to abort, stay silent
		ota_reset();
		ota_ack(op, OTA_ERR_OK);
		return;
	}
}

//-----------------------------------------------------------------------------
//  MAIN function
//-----------------------------------------------------------------------------

int main(void)
{
	if( SysTick_Config(SystemCoreClock / 1000) ) { // setup SysTick Timer for 1 msec interrupts
		while( 1 );                                  // capture error
	}

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32); // approx 3s
	IWDG_SetReload(0xfff);
	IWDG_ReloadCounter();
	IWDG_Enable();

	#ifdef LED_PIN
	DDR(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
	#endif

	// do we have a node address left behind by the application?
	uint32_t nv = *NODEADDR_ADDR;
	have_addr = ((nv >> 8) == NODEADDR_MAGIC);
	my_addr = have_addr ? (uint8_t)nv : 0;

	can_init(CAN_BR_100);

	// filter 0: OTA_DATA sent to broadcast (0xFF) or broadcast-except-self
	// (0xFE) - these two addresses differ only in their LSB, so one
	// mask-based filter catches both.
	can_filter(CAN_MK_ID(CAN_ADDR_BROADCAST_NOSELF, CAN_MSGTYPE_OTA_DATA), 0x7F7, 0);

	// filter 1: OTA_DATA sent to our own address, if we have one
	if( have_addr ) {
		can_filter(CAN_MK_ID(my_addr, CAN_MSGTYPE_OTA_DATA), 0x7FF, 1);
	}

	tmr_set(TMR_ID_LED, 100);

	while( 1 ) {
		// toggle LED (if defined)
		if( tmr_elapsed(TMR_ID_LED) ) {
			tmr_reset(TMR_ID_LED);
			#ifdef LED_PIN
			if( GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == Bit_SET ) {
				GPIO_ResetBits(LED_PORT, LED_PIN);
			} else {
				GPIO_SetBits(LED_PORT, LED_PIN);
			}
			#endif
		}

		// Process CAN messages
		CanRxMsg msg;
		if( can_rx(&msg) ) {
			process_ota_msg(&msg);
		}

		// reset if no relevant CAN messages received
		if( uptime - lastcanrx > NOCANRX_TO ) {
			struct bl_pvars_t pv;
			memcpy(&pv, APP_BASE - PAGE_WORDS, sizeof(pv));

			if( (pv.app_page_count > 0) && (pv.app_page_count <= PAGE_COUNT) ) {
				CRC_ResetDR();
				CRC_CalcBlockCRC((uint32_t*)APP_BASE, pv.app_page_count * PAGE_WORDS);
				if( CRC_GetCRC() == pv.app_crc ) {
					*(MAGIC_ADDR) = MAGIC_VAL;
				}
			}

			NVIC_SystemReset();
		}

		// feed watchdog
		IWDG_ReloadCounter();
	}
}
