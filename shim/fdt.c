/*
 * Just enough flattened-device-tree parsing to find what ABL put in
 * /chosen (initrd location). Big-endian, 4-byte aligned tokens.
 */
#include "fdt.h"
#include "console.h"

#define FDT_MAGIC	0xd00dfeed
#define FDT_BEGIN_NODE	1
#define FDT_END_NODE	2
#define FDT_PROP	3
#define FDT_NOP		4
#define FDT_END		9

static uint32_t be32(uintptr_t a)
{
	return __builtin_bswap32(*(volatile uint32_t *)a);
}

static int streq(const char *a, const char *b)
{
	while (*a && *a == *b)
		a++, b++;
	return *a == *b;
}

static uintptr_t align4(uintptr_t a)
{
	return (a + 3) & ~3UL;
}

static uintptr_t skip_str(uintptr_t p)
{
	/* byte loads are fine on Device memory as long as they are aligned to 1 */
	while (*(volatile char *)p)
		p++;
	return p + 1;
}

int fdt_chosen_u64(uintptr_t fdt, const char *prop, uint64_t *out)
{
	if (!fdt || be32(fdt) != FDT_MAGIC)
		return 0;
	uintptr_t p = fdt + be32(fdt + 8);		/* off_dt_struct */
	uintptr_t strings = fdt + be32(fdt + 12);	/* off_dt_strings */
	int depth = 0, in_chosen = 0;

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE: {
			const char *name = (const char *)p;
			depth++;
			in_chosen = depth == 2 && streq(name, "chosen");
			p = align4(skip_str(p));
			break;
		}
		case FDT_END_NODE:
			if (in_chosen)
				return 0;
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			if (in_chosen && streq((const char *)(strings + nameoff), prop)) {
				if (len == 4)
					*out = be32(val);
				else if (len == 8)
					*out = (uint64_t)be32(val) << 32 | be32(val + 4);
				else
					return 0;
				return 1;
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:	/* FDT_END or garbage */
			return 0;
		}
	}
}

/* Does /chosen/bootargs contain `arg` (space separated)? */
int fdt_bootarg_has(uintptr_t fdt, const char *arg)
{
	if (!fdt || be32(fdt) != FDT_MAGIC)
		return 0;
	uintptr_t p = fdt + be32(fdt + 8);
	uintptr_t strings = fdt + be32(fdt + 12);
	int depth = 0, in_chosen = 0;

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE: {
			const char *name = (const char *)p;
			depth++;
			in_chosen = depth == 2 && streq(name, "chosen");
			p = align4(skip_str(p));
			break;
		}
		case FDT_END_NODE:
			if (in_chosen)
				return 0;
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			if (in_chosen &&
			    streq((const char *)(strings + nameoff), "bootargs")) {
				const char *s = (const char *)val;
				const char *end = s + len;
				for (; s < end; s++) {
					const char *a = s, *b = arg;
					while (a < end && *b && *a == *b)
						a++, b++;
					if (!*b)
						return 1;
				}
				return 0;
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:
			return 0;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Hardware identification dump: touch controller nodes and root ids. */

static int has(const char *s, const char *needle)
{
	for (; *s; s++) {
		const char *a = s, *b = needle;
		while (*a && *b && *a == *b)
			a++, b++;
		if (!*b)
			return 1;
	}
	return 0;
}

static int is_touch_node(const char *n)
{
	return has(n, "focal") || has(n, "goodix") || has(n, "novatek") ||
	       has(n, "synaptics") || has(n, "ft5") || has(n, "nt36");
}

static void cells(const char *label, uintptr_t val, uint32_t len)
{
	static const char hx[] = "0123456789ABCDEF";
	char buf[64];
	int n = 0;
	for (uint32_t i = 0; i + 3 < len && n < 48; i += 4) {
		uint32_t v = be32(val + i);
		for (int k = 7; k >= 0; k--)
			buf[n++] = hx[(v >> (4 * k)) & 0xf];
		buf[n++] = ' ';
	}
	buf[n] = 0;
	con_str(label, buf);
}

void fdt_dump_touch(uintptr_t fdt)
{
	if (!fdt || be32(fdt) != FDT_MAGIC) {
		con_clear(0xff000000);
		con_str("DT", "no fdt");
		return;
	}
	uintptr_t p = fdt + be32(fdt + 8);
	uintptr_t strings = fdt + be32(fdt + 12);
	const char *names[16];
	int depth = 0, match = 0, printed = 0, actives = 0;

	con_clear(0xff000000);
	con_str("SHIM", "device tree: touch / ids");
	con_skip();

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE: {
			const char *name = (const char *)p;
			depth++;
			if (depth < 16)
				names[depth] = name;
			p = align4(skip_str(p));
			int fixups = depth >= 2 && has(names[2], "fixup");
			int fp = has(name, "finger") || has(name, "fp");
			if (depth > 1 && printed < 2 && !fixups && !fp &&
			    is_touch_node(name)) {
				printed++;
				match = depth;
				con_str("NODE", name);
				con_str("  IN", names[depth - 1]);
			}
			break;
		}
		case FDT_END_NODE:
			if (match && depth == match) {
				match = 0;
				con_skip();
			}
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			const char *pn = (const char *)(strings + nameoff);
			if (depth == 1) {
				if (streq(pn, "model"))
					con_str("MODEL", (const char *)val);
				else if (streq(pn, "compatible"))
					con_str("COMPAT", (const char *)val);
				else if (streq(pn, "qcom,board-id"))
					cells("BOARDID", val, len);
				else if (streq(pn, "qcom,msm-id"))
					cells("MSMID", val, len);
			} else if (streq(pn, "qcom,i2c-touch-active")) {
				if (actives++ < 2)
					con_str("ACTIVE", (const char *)val);
			} else if (match && depth == match) {
				if (streq(pn, "compatible") || streq(pn, "status")) {
					con_str(pn, (const char *)val);
				} else if (streq(pn, "reg") ||
					   streq(pn, "interrupts") ||
					   streq(pn, "qcom,display-coords") ||
					   has(pn, "gpio") || has(pn, "avdd") ||
					   has(pn, "vdd") || has(pn, "supply")) {
					con_str("  PROP", pn);
					cells("    =", val, len);
				}
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:
			return;
		}
	}
}
