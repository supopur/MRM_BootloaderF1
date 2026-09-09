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
	  OTA_OP_INFO     [op][product_type][page_count]     - announce an update, reset state
	  OTA_OP_DATA     [op][up to 7 raw fw bytes]          - firmware bytes, in order
	  OTA_OP_PAGE_END [op][crc32 LE][page_index]          - commit + verify one page
	  OTA_OP_END      [op][crc32 LE]                      - finalize + verify whole image
	  OTA_OP_ABORT    [op]                                - cancel in-flight update

	product_type must match this build's PRODUCT_TYPE (set at compile time,
	one value per product line) or the update is rejected outright - this is
	what stops product A from flashing product B's firmware when an update is
	broadcast to the whole bus. INFO always gets an ACK/NACK so the master
	knows who is and isn't participating. DATA/PAGE_END/END/ABORT get a reply
	only from devices that are actively participating in the update (i.e.
	already accepted a matching INFO) - a device sitting out someone else's
	update stays completely silent.

	Transfer is stop-and-wait, one page (PAGE_BYTES = 1024) at a time:
	the sender blasts all of a page's OTA_OP_DATA frames back-to-back with
	no per-frame reply (DATA is never ACKed or NACKed - keeps the bus free
	of chatter and lets frames go out at full CAN rate), then sends a single
	OTA_OP_PAGE_END carrying the CRC32 of that page (same STM32 hardware
	CRC32 algorithm as OTA_OP_END, i.e. no input/output reflection, no final
	XOR) and the 0-based index of the page it believes it just sent. Only
	OTA_OP_PAGE_END and OTA_OP_END ever touch flash - DATA only fills a RAM
	buffer - so a bad/incomplete page never gets written.

	The reply to OTA_OP_PAGE_END is always a 4-byte broadcast SLAVE_OUT
	frame: [our_addr][op][err_code][cur_page], where cur_page is *our*
	authoritative "next page expected" counter. This lets the sender
	recover from a lost ACK without corrupting the wrong page: if its
	page_index doesn't match cur_page, we reply OTA_ERR_PAGE_INDEX instead
	of touching anything, and the sender resyncs off cur_page (if we're
	already one page ahead, our previous OK was simply lost on the wire and
	the sender just moves on; otherwise it resends the page cur_page names).
	If page_index matches but the buffer isn't full (OTA_ERR_PAGE_INCOMPLETE)
	or its CRC doesn't match (OTA_ERR_PAGE_CRC), the whole page is discarded
	and must be resent from scratch - no partial resend bookkeeping, which
	is what made the old per-frame sequence-number scheme both slow (it
	required a reply loop practically per frame) and buggy (its sequence
	counter wrapped at 256 frames, silently corrupting any resend past the
	first ~1.5KB of a page). Since every page sent over the wire is always
	exactly PAGE_BYTES (the sender pads the final page of the image with
	0xFF, the erased-flash value), OTA_OP_PAGE_END never has to deal with a
	short final page.

	Every other reply (INFO/END/ABORT) stays the old 3-byte format:
	[our_addr][op][err_code]. err_code 0 = OK, see OTA_ERR_* below.

	On OTA_OP_END, the whole image (all total_pages pages, already verified
	and flashed page-by-page) is CRC32'd again as a final end-to-end check
	before the {page_count, crc} info page is written. That done, the
	existing "reboot into app once the bus goes quiet" logic in main()'s
	loop (unchanged) takes over - no immediate reset is forced here.
*/

#include "stm32f10x.h"
#include "core_cm3.h"
#include "can.h"

#include <string.h>

//-----------------------------------------------------------------------------
//  Defines
//-----------------------------------------------------------------------------

#define LED_PORT GPIOB
#define LED_PIN GPIO_Pin_8

#define TMR_ID_DELAY 0
#define TMR_ID_LED 1
#define TMR_ID_NUM 2

#define PAGE_WORDS 0x100              // words per flash page (STM32F1 page = 1KB)
#define PAGE_BYTES (PAGE_WORDS * 4)

#define FLASH_SIZE_REG  (*(__IO uint16_t*)0x1FFFF7E0UL)

// Bootloader layout
#define BOOT_APP_ADDRESS 0x08002000UL
#define BOOT_INFO_ADDRESS 0x08001C00UL
#define BOOT_RESERVED_KIB 8UL
#define BOOT_PAGE_BYTES 1024UL
#define BOOT_PAGE_WORDS 256UL
#define BOOT_REQUEST_ADDRESS 0x20001000UL
#define BOOT_NODE_ADDRESS 0x20001004UL
#define BOOT_REQUEST_MAGIC 0x36051BF3UL
#define BOOT_NODE_MAGIC 0x00C0FFEEUL
#define BOOT_NODE_SHIFT 8U
#define BOOT_C6_FLASH_KIB 32U
#define BOOT_C8_FLASH_KIB 64U
#define BOOT_C6_RAM_BYTES 10240UL
#define BOOT_C8_RAM_BYTES 20480UL
#define BOOT_STACK_ALIGNMENT 8UL
#define BOOT_THUMB_BIT 1UL
#define BOOT_VECTOR_BYTES 8UL

//-----------------------------------------------------------------------------
//  Typedefs
//-----------------------------------------------------------------------------

typedef struct {
    uint32_t pageCount;
    uint32_t crc;
} BootInfo_t;

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

static const uint32_t MAGIC_VAL = BOOT_REQUEST_MAGIC;
static uint32_t* const MAGIC_ADDR = (uint32_t*)BOOT_REQUEST_ADDRESS;
static uint32_t* const NODEADDR_ADDR = (uint32_t*)BOOT_NODE_ADDRESS;
#define NODEADDR_MAGIC  0x00C0FFEEUL

static const uint32_t* APP_BASE = (uint32_t*)BOOT_APP_ADDRESS;
static uint16_t PAGE_COUNT;

// which product line this build is for - must match the product_type byte
// in OTA_OP_INFO or the update is rejected. Override in the makefile per
// product, e.g. -DPRODUCT_TYPE=2.
#ifndef PRODUCT_TYPE
#define PRODUCT_TYPE 0
#endif

// OTA_DATA sub-opcodes (payload[0])
#define OTA_OP_INFO     0x00
#define OTA_OP_DATA     0x01
#define OTA_OP_END      0x02
#define OTA_OP_ABORT    0x03
#define OTA_OP_PAGE_END 0x04

// OTA ack error codes (payload[2] of the SLAVE_OUT reply)
#define OTA_ERR_OK              0
#define OTA_ERR_STATE           1 // reserved (non-participants now stay silent instead)
#define OTA_ERR_PRODUCT         2 // product_type in INFO doesn't match this device
#define OTA_ERR_SIZE            3 // page_count is 0 or too big for this device
#define OTA_ERR_OVERFLOW        4 // more DATA received than page_count allows
#define OTA_ERR_INCOMPLETE      5 // END received before all pages were received
#define OTA_ERR_CRC              6 // final whole-image CRC does not match
#define OTA_ERR_FLASH            7 // erase/program/verify failed
#define OTA_ERR_PAGE_INCOMPLETE  8 // PAGE_END: buffer isn't full yet - resend this whole page
#define OTA_ERR_PAGE_CRC         9 // PAGE_END: page CRC mismatch - resend this whole page
#define OTA_ERR_PAGE_INDEX      10 // PAGE_END: page_index != our cur_page - resync off cur_page

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

//-----------------------------------------------------------------------------
//  Boot‑selection helpers (new)
//-----------------------------------------------------------------------------

static uint16_t flash_size_kib(void)
{
    // FLASH_SIZE_REG is already the value, not a pointer
    return FLASH_SIZE_REG;
}

static uint16_t application_capacity(void)
{
    uint16_t flashSize_kib = flash_size_kib();

    if (flashSize_kib != BOOT_C6_FLASH_KIB &&
        flashSize_kib != BOOT_C8_FLASH_KIB) {
        return 0;
    }

    return flashSize_kib - BOOT_RESERVED_KIB;
}

static uint8_t application_matches(uint32_t pageCount, uint32_t crc)
{
    volatile const uint32_t *p_vectors =
        (volatile const uint32_t *)BOOT_APP_ADDRESS;
    uint32_t capacity = application_capacity();
    uint32_t ramBytes;
    uint32_t imageEnd;
    uint32_t stack;
    uint32_t entry;

    if (!capacity || !pageCount || pageCount > capacity) {
        return 0;
    }

    ramBytes = flash_size_kib() == BOOT_C6_FLASH_KIB ?
        BOOT_C6_RAM_BYTES : BOOT_C8_RAM_BYTES;
    imageEnd = BOOT_APP_ADDRESS + pageCount * BOOT_PAGE_BYTES;
    stack = p_vectors[0];
    entry = p_vectors[1];

    if ((stack & (BOOT_STACK_ALIGNMENT - 1UL)) ||
        stack <= SRAM_BASE || stack > SRAM_BASE + ramBytes ||
        !(entry & BOOT_THUMB_BIT)) {
        return 0;
    }

    entry &= ~BOOT_THUMB_BIT;
    if (entry < BOOT_APP_ADDRESS + BOOT_VECTOR_BYTES ||
        entry >= imageEnd) {
        return 0;
    }

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    CRC_ResetDR();

    return CRC_CalcBlockCRC((uint32_t *)BOOT_APP_ADDRESS,
                           pageCount * BOOT_PAGE_WORDS) == crc;
}

static uint8_t application_is_valid(void)
{
    volatile const BootInfo_t *p_info =
        (volatile const BootInfo_t *)BOOT_INFO_ADDRESS;

    return application_matches(p_info->pageCount, p_info->crc);
}

static uint8_t application_invalidate(void)
{
    volatile const uint32_t *p_info =
        (volatile const uint32_t *)BOOT_INFO_ADDRESS;
    FLASH_Status status;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR |
                    FLASH_FLAG_WRPRTERR);
    status = FLASH_ErasePage(BOOT_INFO_ADDRESS);
    FLASH_Lock();

    if (status != FLASH_COMPLETE) {
        return 0;
    }

    for (uint32_t word = 0; word < BOOT_PAGE_WORDS; ++word) {
        if (p_info[word] != UINT32_MAX) {
            return 0;
        }
    }

    return 1;
}

/* GCC: no C code may execute after changing MSP. */
__attribute__((naked, noreturn))
static void application_branch(uint32_t stack, uint32_t entry)
{
    __asm volatile (
        "msr msp, r0\n"
        "cpsie i\n"
        "bx r1\n"
    );
}

/* Not naked – it can call CMSIS inline functions */
__attribute__((noreturn))
static void application_jump(void)
{
    volatile const uint32_t *p_vectors =
        (volatile const uint32_t *)BOOT_APP_ADDRESS;
    uint32_t stack = p_vectors[0];
    uint32_t entry = p_vectors[1];

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (uint32_t bank = 0;
         bank < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]);
         ++bank) {
        NVIC->ICER[bank] = UINT32_MAX;
        NVIC->ICPR[bank] = UINT32_MAX;
    }

    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
    RCC_DeInit();

    SCB->VTOR = BOOT_APP_ADDRESS;
	__asm volatile ("msr CONTROL, %0" :: "r"(0u) : "memory");
	__asm volatile ("msr BASEPRI, %0" :: "r"(0u) : "memory");
	__asm volatile ("msr FAULTMASK, %0" :: "r"(0u) : "memory");
    __DSB();
    __ISB();

    application_branch(stack, entry);
}

// empty hook – called by startup before main
void PreSystemInit(void)
{
}

//-----------------------------------------------------------------------------
//  OTA update handling
//-----------------------------------------------------------------------------

// Reply used for OTA_OP_PAGE_END only - always reports cur_page (our
// authoritative "next page expected" counter) alongside the error code, so
// the sender can resync after a lost reply instead of guessing.
static void ota_ack_page(uint8_t op, uint8_t err, uint8_t page)
{
	CanTxMsg m;
	m.StdId = CAN_MK_ID(CAN_ADDR_BROADCAST, CAN_MSGTYPE_SLAVE_OUT);
	m.IDE = CAN_Id_Standard;
	m.RTR = CAN_RTR_Data;
	m.DLC = 4;
	m.Data[0] = have_addr ? my_addr : CAN_ADDR_BROADCAST;
	m.Data[1] = op;
	m.Data[2] = err;
	m.Data[3] = page;
	can_tx(&m);
}

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
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
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
	// Basic frame sanity
	if (msg->IDE != CAN_Id_Standard ||
	    msg->RTR != CAN_RTR_Data ||
	    msg->DLC == 0 || msg->DLC > sizeof(msg->Data)) {
	    return;
	}
	if( msg->DLC < 1 ) return;
	lastcanrx = uptime;

	uint8_t op = msg->Data[0];

	if( op == OTA_OP_INFO ) {
		if( msg->DLC < 3 ) return;
		uint8_t prod = msg->Data[1];
		uint8_t pc = msg->Data[2];
		// Validate product and size first
		if( prod != PRODUCT_TYPE ) {
			ota_ack(op, OTA_ERR_PRODUCT);
			return;
		}
		if( (pc == 0) || (pc > PAGE_COUNT) ) {
			ota_ack(op, OTA_ERR_SIZE);
			return;
		}
		// Reset state and invalidate old metadata before accepting new image
		ota_reset();
		if (!application_invalidate()) {
			ota_ack(op, OTA_ERR_FLASH);
			return;
		}
		total_pages = pc;
		ota_active = 1;
		memset(pagebuf, 0xFF, sizeof(pagebuf));
		ota_ack(op, OTA_ERR_OK);
		return;
	}

	if( op == OTA_OP_DATA ) {
		// DATA is never ACKed or NACKed - it only fills the RAM page
		// buffer. Correctness (and any resend) is entirely handled by
		// OTA_OP_PAGE_END below, so the sender can blast a whole page's
		// worth of frames back-to-back at full CAN rate.
		if( !ota_active ) return;
		if( cur_page >= total_pages ) {
			ota_ack(op, OTA_ERR_OVERFLOW); // sender bug - it kept going past total_pages
			ota_reset();
			return;
		}

		uint8_t byteCount = msg->DLC - 1U;
		// Detect overflow; if too many bytes, flag an error but keep the buffer
		// so that PAGE_END will fail its CRC check.
		if (page_off > PAGE_BYTES ||
		    byteCount > PAGE_BYTES - page_off) {
		    page_off = PAGE_BYTES + 1U;  // intentionally oversize
		    return;
		}
		memcpy((uint8_t *)pagebuf + page_off, msg->Data + 1, byteCount);
		page_off += byteCount;
		return;
	}

	if( op == OTA_OP_PAGE_END ) {
		if( !ota_active ) return;
		if( msg->DLC < 6 ) return;

		uint32_t page_crc;
		memcpy(&page_crc, msg->Data + 1, 4); // little-endian, matches Cortex-M3
		uint8_t sender_page = msg->Data[5];

		// First, handle index mismatch – resync without touching flash
		if( sender_page != cur_page ) {
		    // Discard any stale page data to avoid mixing with a future retry
		    page_off = 0;
		    memset(pagebuf, 0xFF, sizeof(pagebuf));
		    ota_ack_page(op, OTA_ERR_PAGE_INDEX, cur_page);
		    return;
		}

		// At this point sender_page == cur_page. Ensure we haven't already
		// finished all pages (e.g. a spurious duplicate PAGE_END).
		if( cur_page >= total_pages ) {
		    ota_ack_page(op, OTA_ERR_OVERFLOW, cur_page);
		    return;
		}

		if( page_off != PAGE_BYTES ) {
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
			ota_ack_page(op, OTA_ERR_PAGE_INCOMPLETE, cur_page);
			page_off = 0; // discard - sender resends this whole page
			memset(pagebuf, 0xFF, sizeof(pagebuf));
			return;
		}

		CRC_ResetDR();
		CRC_CalcBlockCRC(pagebuf, PAGE_WORDS);
		if( CRC_GetCRC() != page_crc ) {
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
			ota_ack_page(op, OTA_ERR_PAGE_CRC, cur_page);
			page_off = 0; // discard - sender resends this whole page
			memset(pagebuf, 0xFF, sizeof(pagebuf));
			return;
		}

		GPIO_ResetBits(GPIOB, GPIO_Pin_9);

		if( !ota_flush_page(PAGE_WORDS) ) { // erase+program; advances cur_page on success
			ota_ack_page(op, OTA_ERR_FLASH, cur_page);
			ota_reset(); // local flash hardware failure - whole update is dead
			return;
		}

		ota_ack_page(op, OTA_ERR_OK, cur_page); // cur_page already advanced past the committed page
		return;
	}

	if( op == OTA_OP_END ) {
		if( msg->DLC < 5 ) return;
		if( !ota_active ) return; // not participating in this update - stay silent

		// Check that all pages have been received and buffer is empty
		if( cur_page != total_pages || page_off != 0 ) {
			ota_ack(op, OTA_ERR_INCOMPLETE);
			ota_reset();
			return;
		}

		uint32_t crc;
		memcpy(&crc, msg->Data + 1, 4);

		// Use the same validation function that we use at boot time
		if (!application_matches(total_pages, crc)) {
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
			ota_ack(op, OTA_ERR_CRC);
			ota_reset();
			return;
		}

		// Write the metadata page
		BootInfo_t pv;
		pv.pageCount = total_pages;
		pv.crc = crc;
		if( fls_wr((uint32_t*)BOOT_INFO_ADDRESS, (uint32_t*)&pv, sizeof(pv) / 4) ) {
			ota_ack(op, OTA_ERR_FLASH);
			ota_reset();
			return;
		}

		// Verify that the metadata was written correctly
		if (!application_is_valid()) {
			ota_ack(op, OTA_ERR_FLASH);
			ota_reset();
			return;
		}

		// Send OK reply; then the main loop will reset the device after bus idle.
		ota_ack(op, OTA_ERR_OK);
		// Do NOT reset immediately – let the host receive the ACK.
		// The existing idle‑timeout will trigger a reset.
		ota_reset(); // clear active state but keep the written image valid
		return;
	}

	if( op == OTA_OP_ABORT ) {
		if( !ota_active ) return; // nothing to abort, stay silent
		ota_reset();
		ota_ack(op, OTA_ERR_OK);
		return;
	}
}

static void SystemClock_Config(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	while (!RCC_WaitForHSEStartUp());

	FLASH_SetLatency(FLASH_Latency_2);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);   // APB1 = 36 MHz
	RCC_PCLK2Config(RCC_HCLK_Div1);

	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
	RCC_PLLCmd(ENABLE);
	while (!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while (RCC_GetSYSCLKSource() != 0x08);

	SystemCoreClock = 72000000;
}

//-----------------------------------------------------------------------------
//  MAIN function
//-----------------------------------------------------------------------------

int main(void)
{
	// --- Early boot decision (must run before any clock/peripheral init) ---
	volatile uint32_t *p_request =
	    (volatile uint32_t *)BOOT_REQUEST_ADDRESS;
	volatile uint32_t *p_node =
	    (volatile uint32_t *)BOOT_NODE_ADDRESS;

	uint32_t resetFlags = RCC->CSR;
	uint32_t request = *p_request;
	uint32_t node = *p_node;

	// Clear the request magic so it is not reused
	*p_request = 0;
	*p_node = 0;
	__DSB();
	RCC_ClearFlag();

	uint8_t requested =
	    (resetFlags & RCC_CSR_SFTRSTF) &&
	    !(resetFlags & RCC_CSR_PORRSTF) &&
	    request == BOOT_REQUEST_MAGIC;

	PAGE_COUNT = application_capacity();

	have_addr = requested &&
	    (node >> BOOT_NODE_SHIFT) == BOOT_NODE_MAGIC &&
	    (uint8_t)node != CAN_ADDR_LOCAL &&
	    (uint8_t)node < CAN_ADDR_BROADCAST_NOSELF;
	my_addr = have_addr ? (uint8_t)node : CAN_ADDR_LOCAL;

	// If we were not explicitly requested to stay in bootloader and the
	// application is valid, jump to it immediately.
	if (!requested && application_is_valid()) {
	    application_jump();
	}

	// --- Continue with bootloader initialisation ---
	SystemClock_Config();
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
	DDR(LED_PORT, GPIO_Pin_9, GPIO_Mode_Out_PP);
	#endif

	// (PAGE_COUNT already set above, have_addr/my_addr too)

	// Initialise CAN with automatic bus‑off recovery
	can_init(CAN_BR_125);
	// We could check return value here, but can_init is currently void.
	// We'll modify can.c to return status and handle it here.

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

		// Reset to application if the bus has been quiet long enough and
		// the image is valid.
		if( uptime - lastcanrx > 2 ) { // 2 seconds idle
			if (application_is_valid()) {
				// Write the boot‑request magic to trigger a jump on next reset
				*MAGIC_ADDR = MAGIC_VAL;
				// Write the node address that was used (or broadcast)
				if (have_addr) {
					*NODEADDR_ADDR = (NODEADDR_MAGIC << 8) | my_addr;
				}
				NVIC_SystemReset();
			}
		}

		// feed watchdog
		IWDG_ReloadCounter();
	}
}