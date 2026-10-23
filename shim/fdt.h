#pragma once
#include <stdint.h>

/* Read a u32/u64 (cell-size agnostic) property of /chosen. Returns 0 if absent. */
int fdt_chosen_u64(uintptr_t fdt, const char *prop, uint64_t *out);

/* Print root ids and every touch-controller node on the shim console. */
void fdt_dump_touch(uintptr_t fdt);

/* Does /chosen/bootargs contain the space-separated token `arg`? */
int fdt_bootarg_has(uintptr_t fdt, const char *arg);
