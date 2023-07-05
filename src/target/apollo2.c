#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "adiv5.h"

#define APOLLO2_DEVICE_ID 0x40020000


bool apollo2_probe(target_s *t) {
    uint32_t idcode;

    idcode = target_mem_read32(t, APOLLO2_DEVICE_ID);
    switch (idcode) {
    case 0x036422c9:
        t->driver = "Apollo2";
        return true;
    }

    if (idcode) {
		DEBUG_INFO("Apollo2: Unknown IDCODE 0x%08" PRIx32 "\n", idcode);
	}
    return false;
}