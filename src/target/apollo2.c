#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "adiv5.h"

#define APOLLO2_DEVICE_ID 0x40020000

#define APOLLO2_UID_HIGH 0x40020004
#define APOLLO2_UID_LOW 0x40020008
#define APOLLO2_CHIP_REV 0x4002000C

#define APOLLO2_SRAM_ADDR   0x10000000
#define APOLLO2_SRAM_SIZE   0x40000

#define APOLLO2_FLASH_ADDR                   0x00000000
#define APOLLO2_FLASH_PAGE_SIZE              ( 8 * 1024 )
#define APOLLO2_FLASH_INSTANCE_SIZE          ( 512 * 1024 )
#define APOLLO2_FLASH_INSTANCE_PAGES         ( APOLLO2_FLASH_INSTANCE_SIZE / APOLLO2_FLASH_PAGE_SIZE )
#define APOLLO2_FLASH_TOTAL_SIZE             ( APOLLO2_FLASH_INSTANCE_SIZE * 2 )
#define APOLLO2_FLASH_LARGEST_VALID_ADDR     ( APOLLO2_FLASH_ADDR + APOLLO2_FLASH_TOTAL_SIZE - 1 )

#define APOLLO2_PROGRAM_KEY 0x12344321
#define APOLLO2_OTP_PROGRAM_KEY 0x87655678

// Bootloader functions
#define APOLLO2_FLASH_PROGRAM_MAIN_SRAM         0x0800005d
#define APOLLO2_FLASH_PROGRAM_INFO_SRAM         0x08000061
#define APOLLO2_FLASH_ERASE_MAIN_PAGES_SRAM     0x08000065
#define APOLLO2_FLASH_MASS_ERASE_SRAM           0x08000069
#define APOLLO2_FLASH_ERASE_INFO_SRAM           0x08000085
#define APOLLO2_FLASH_ERASE_MAIN_PLUS_INFO_SRAM 0x0800008d

/** Wait for halt after each command. 3s */
#define WAITHALT_TIMEOUT    (3000)

/** Breakpoint for Bootloader, loaded to sram location of return codes. */
#define APOLLO2_BREAKPOINT                  (0xfffffffe)

/** Apollo2 Bootloader Write Buffer Start. */
#define APOLLO2_WRITE_BUFFER_START   (0x10001000)


/** Bootloader visible at 0x00000000 (0x1). */
#define REG_CONTROL_BOOTLOADERLOW   (0x400201a0)

//
// Convert an absolute flash address to a instance
//
#define AM_HAL_FLASH_ADDR2INST(addr)        ( ( addr >> 19 ) & 1 )

//
// Convert an absolute flash address to a page number relative to the instance
//
#define AM_HAL_FLASH_ADDR2PAGE(addr)        ( ( addr >> 13 ) & 0x3F )

//
// Convert an absolute flash address to an absolute page number
//
#define AM_HAL_FLASH_ADDR2ABSPAGE(addr)     ( addr >> 13 )

// Reset AIRCR
// Reset POR
// Reset POI

static bool clear_sram_parameters(target_s *target, uint32_t pSram, uint32_t pStart) {
    if (pSram < pStart) {
		DEBUG_INFO("sram pointer %u less than start address %u\n", pSram, pStart);
		return false;
	}
	while (pSram > pStart) {
		pSram -= sizeof(uint32_t);
		target_mem_write32(target, pSram, 0);
	}

	return true;
}

static uint32_t setup_sram(target_s *target, int width, uint32_t arr[width]) {
    uint32_t pSramRetval = 0;
    uint32_t pSram = APOLLO2_SRAM_ADDR;

	for (int i = 0; i < width; i++) {
		DEBUG_INFO("pSram[0x%08X] 0x%08X\n", pSram, arr[i]);
		if (arr[i] == APOLLO2_BREAKPOINT)
			pSramRetval = pSram;
		target_mem_write32(target, pSram, arr[i]);

		pSram += sizeof(uint32_t);
	}

	DEBUG_INFO("pSram[pSramRetval] 0x%08X\n", pSramRetval);
	return pSramRetval;
}

static bool check_flash_status(target_s *target, target_addr_t addr) {
    uint32_t rc;
    rc = target_mem_read32(target, addr);
    
    if (rc != 0) {
        DEBUG_WARN("Flash not happy: status(0x%" PRIx32 ")\n", rc);
        return false;
    }

    return true;
}

static bool exec_command(target_s *target, uint32_t command, uint32_t flash_return_address) {
    DEBUG_INFO("pROM[Bootloader] 0x%08X\n", command);

    /* Set up for the call to the IAP ROM */
	uint32_t regs[target->regs_size / sizeof(uint32_t)];
	target_regs_read(target, regs);
	regs[REG_MSP] = 0x10000000U + 1024 - 32;
	regs[REG_LR] = 0x10000000U | 1U;
	regs[REG_PC] = command;
    regs[REG_XPSR] = CORTEXM_XPSR_THUMB;
	target_regs_write(target, regs);

    platform_timeout_s progress_timeout;
	platform_timeout_set(&progress_timeout, 150);

    platform_timeout_s command_timeout;
    platform_timeout_set(&command_timeout, WAITHALT_TIMEOUT);
	/* Start the target and wait for it to halt again */

	target_halt_resume(target, false);
	while (!target_halt_poll(target, NULL) && !platform_timeout_is_expired(&command_timeout)) {
		target_print_progress(&progress_timeout);
	}

    if (platform_timeout_is_expired(&command_timeout)) {
        DEBUG_WARN("Command timeout\n");
        return false;
    }

    if (!check_flash_status(target, flash_return_address)) {
        return false;
    }

    return true;
}

static bool exec_sram_command(target_s *target, uint32_t command, const char* cmd_name, int width, uint32_t arr[]) {
    if(cmd_name) {
        DEBUG_INFO("Starting %s\n", cmd_name);
    }
    
    uint32_t return_address = setup_sram(target, width, arr);
    bool success = exec_command(target, command, return_address);
    clear_sram_parameters(target, return_address, APOLLO2_SRAM_ADDR);

    if(cmd_name) {
        DEBUG_INFO("Finished %s\n", cmd_name);
    }

    return success;
}

static bool exec_main_command(target_s *target, uint32_t command, const char* cmd_name, int width, uint32_t arr[]) {
    target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x0);
    
    bool success = exec_sram_command(target, command, cmd_name, width, arr);
    
    target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x1);

    return success;
}

static bool apollo2_flash_erase(target_flash_s *f, target_addr_t addr, size_t len) {
    int first_page = AM_HAL_FLASH_ADDR2PAGE(addr);
    int last_page = AM_HAL_FLASH_ADDR2PAGE((addr + len - 1));
    int num_pages = last_page - first_page + 1;

    target_s *target = f->t;

    uint32_t sram_args[] = {
		AM_HAL_FLASH_ADDR2INST(addr),
        num_pages,	/* Number of pages to erase. */
		APOLLO2_PROGRAM_KEY,
		APOLLO2_BREAKPOINT,
		first_page,
        ARM_THUMB_BREAKPOINT
	};

    target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x0);
    bool success = exec_sram_command(target, APOLLO2_FLASH_ERASE_MAIN_PAGES_SRAM, "Page Erase", ARRAY_LENGTH(sram_args), sram_args);
    if (first_page==0) {
        target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x1);
    }
    
    return success;
}

static bool apollo2_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t len) {
    target_s *target = f->t;
    target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x0);

    target_mem_write(target, APOLLO2_WRITE_BUFFER_START, src, len);

    uint32_t sram_args[] = {dest, len/4, APOLLO2_PROGRAM_KEY, APOLLO2_BREAKPOINT, ARM_THUMB_BREAKPOINT};

    bool success = exec_sram_command(target, APOLLO2_FLASH_PROGRAM_MAIN_SRAM, "Write", ARRAY_LENGTH(sram_args), sram_args);

    target_mem_write32(target, REG_CONTROL_BOOTLOADERLOW, 0x0);

	return success;
}

static bool apollo2_erase_instance(target_s *t, uint32_t instance) {
    uint32_t sram_args[] = {
		instance,
		APOLLO2_PROGRAM_KEY,
		APOLLO2_BREAKPOINT,
        ARM_THUMB_BREAKPOINT
	};

    return exec_main_command(t, APOLLO2_FLASH_MASS_ERASE_SRAM, "Mass Erase", ARRAY_LENGTH(sram_args), sram_args);
}

static bool apollo2_mass_erase(target_s *t) {
    return apollo2_erase_instance(t, 0) && apollo2_erase_instance(t, 1);
}

static bool apollo2_cmd_read_uid(target_s *t, int argc, const char **argv);

const command_s apollo2_cmd_list[] = {
    {"readuid", apollo2_cmd_read_uid, "Read out the 8-byte UID."},
    {NULL, NULL, NULL}
};

static void apollo2_add_flash(target_s *t, uint32_t addr, size_t len, size_t erase_size) {
    target_flash_s *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = len;
	f->blocksize = erase_size;
	f->erase = apollo2_flash_erase;
	f->write = apollo2_flash_write;
	f->writesize = erase_size;
	f->erased = 0xff;
	target_add_flash(t, f);
}

bool apollo2_probe(target_s *t) {
    uint32_t idcode;

    idcode = target_mem_read32(t, APOLLO2_DEVICE_ID);
    switch (idcode) {
    case 0x036422c9:
        t->driver = "Apollo2";
        t->mass_erase = apollo2_mass_erase;
        target_add_ram(t, APOLLO2_SRAM_ADDR, APOLLO2_SRAM_SIZE);
        apollo2_add_flash(t, APOLLO2_FLASH_ADDR, APOLLO2_FLASH_INSTANCE_SIZE, APOLLO2_FLASH_PAGE_SIZE);
        apollo2_add_flash(t, APOLLO2_FLASH_ADDR+APOLLO2_FLASH_INSTANCE_SIZE, APOLLO2_FLASH_INSTANCE_SIZE, APOLLO2_FLASH_PAGE_SIZE);
        target_add_commands(t, apollo2_cmd_list, "Apollo2");
        return true;
    }

    if (idcode) {
		DEBUG_INFO("Apollo2: Unknown IDCODE 0x%08" PRIx32 "\n", idcode);
	}
    return false;
}

static bool apollo2_cmd_read_uid(target_s *t, int argc, const char **argv) {
    (void) argc;
    (void) argv;

    uint32_t uid_high;
    uint32_t uid_low;

    uid_high = target_mem_read32(t, APOLLO2_UID_HIGH);
    uid_low = target_mem_read32(t, APOLLO2_UID_LOW);
    tc_printf(t, "UID: 0x");
    tc_printf(t, "%08x", uid_high);
    tc_printf(t, "%08x", uid_low);
    tc_printf(t, "\n");

    return true;
}