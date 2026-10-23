/*
 * Milestone 5 probe: can we talk to the internal UFS that ABL booted from?
 *
 * Runs with the MMU off (all memory Device, so DMA is coherent with no cache
 * maintenance). Dumps the UFSHCI registers ABL left behind and, if the host
 * is enabled and the link is up, points the transfer request list at our own
 * buffers and issues READ-ONLY SCSI commands on LUN 0. Nothing is ever
 * written to the device: there is no SCSI WRITE in this file.
 *
 * Register layout and descriptor formats: JEDEC UFSHCI (JESD223).
 */
#include <stdint.h>
#include "console.h"
#include "ufs.h"

#define UFS_BASE	0x04804000UL	/* ufshc@4804000, topaz live FDT */

#define REG_CAP		0x00
#define REG_VER		0x08
#define REG_IS		0x20
#define REG_HCS		0x30
#define REG_HCE		0x34
#define REG_UTRLBA	0x50
#define REG_UTRLBAU	0x54
#define REG_UTRLDBR	0x58
#define REG_UTRLCLR	0x5c
#define REG_UTRLRSR	0x60

#define HCS_DP		(1u << 0)
#define HCS_UTRLRDY	(1u << 1)
#define HCS_UCRDY	(1u << 3)

#define BLOCK		4096u

static inline uint32_t ufs_rd(uint32_t off) { return *(volatile uint32_t *)(UFS_BASE + off); }
static inline void ufs_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(UFS_BASE + off) = v; }

/* UTP transfer request descriptor (32 bytes). */
struct utrd {
	uint32_t dw0;		/* [31:28] CT, [26:25] DD, [24] I */
	uint32_t dw1;
	uint32_t dw2;		/* [7:0] OCS */
	uint32_t dw3;
	uint32_t ucdba;		/* command descriptor base, 128-byte aligned */
	uint32_t ucdbau;
	uint32_t resp;		/* [31:16] response UPIU offset, [15:0] length (DWORDs) */
	uint32_t prdt;		/* [31:16] PRDT offset, [15:0] entries (DWORDs/entries) */
};

/* UTP command descriptor: command UPIU, response UPIU, PRDT. */
struct ucd {
	uint8_t cmd[512];
	uint8_t resp[512];
	struct {
		uint32_t addr_lo, addr_hi, rsvd, dbc;	/* dbc = byte count - 1 */
	} prdt[1];
};

static struct utrd utrl[32] __attribute__((aligned(1024)));
static struct ucd ucd __attribute__((aligned(128)));
static uint8_t data[2 * BLOCK] __attribute__((aligned(4096)));

static uint64_t ticks(void)
{
	uint64_t t;
	__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(t));
	return t;
}

static void zero(void *p, unsigned n)
{
	volatile uint8_t *b = p;
	for (unsigned i = 0; i < n; i++)
		b[i] = 0;
}

static uint32_t be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/*
 * Issue one SCSI command (read direction or no data) in slot 0.
 * Returns 0 on success, otherwise a code describing the failure stage.
 */
static uint32_t scsi(uint8_t lun, const uint8_t *cdb, unsigned cdb_len, void *buf, uint32_t len,
    uint32_t *ocs_out, uint32_t *status_out)
{
	static uint8_t tag;
	zero(&utrl[0], sizeof(utrl[0]));
	zero(&ucd, sizeof(ucd));

	/* Command UPIU: header (12 bytes), expected length, CDB. */
	ucd.cmd[0] = 0x01;			/* transaction type: COMMAND */
	ucd.cmd[1] = len ? 0x40 : 0x00;		/* flags: R (data in) */
	ucd.cmd[2] = lun;
	ucd.cmd[3] = ++tag;
	ucd.cmd[12] = len >> 24;
	ucd.cmd[13] = len >> 16;
	ucd.cmd[14] = len >> 8;
	ucd.cmd[15] = len;
	for (unsigned i = 0; i < cdb_len; i++)
		ucd.cmd[16 + i] = cdb[i];

	uintptr_t ucd_pa = (uintptr_t)&ucd;
	utrl[0].dw0 = (1u << 28) | ((len ? 2u : 0u) << 25) | (1u << 24);	/* UFS storage, dev->host */
	utrl[0].dw2 = 0x0f;			/* OCS: invalid until the host updates it */
	utrl[0].ucdba = (uint32_t)ucd_pa;
	utrl[0].ucdbau = (uint32_t)(ucd_pa >> 32);
	utrl[0].resp = (512u / 4) << 16 | (512u / 4);
	if (len) {
		uintptr_t buf_pa = (uintptr_t)buf;
		ucd.prdt[0].addr_lo = (uint32_t)buf_pa;
		ucd.prdt[0].addr_hi = (uint32_t)(buf_pa >> 32);
		ucd.prdt[0].dbc = len - 1;
		utrl[0].prdt = (1024u / 4) << 16 | 1;
	}
	__asm__ volatile("dsb sy" ::: "memory");

	ufs_wr(REG_UTRLDBR, 1);
	uint64_t start = ticks();
	while (ufs_rd(REG_UTRLDBR) & 1) {
		if (ticks() - start > 19200000) {	/* 1 s */
			ufs_wr(REG_UTRLCLR, ~1u);	/* clear slot 0 (write 0 to the bit) */
			return 1;			/* timeout */
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
	*ocs_out = utrl[0].dw2 & 0xff;
	*status_out = ucd.resp[7];		/* SCSI status in the response UPIU */
	if (*ocs_out)
		return 2;
	if (ucd.resp[0] != 0x21)		/* RESPONSE UPIU */
		return 3;
	return *status_out ? 4 : 0;
}

static uint32_t read_blocks(uint32_t lba, uint32_t count, void *buf)
{
	uint8_t cdb[10] = { 0x28, 0, lba >> 24, lba >> 16, lba >> 8, lba, 0, count >> 8, count, 0 };
	uint32_t ocs, st;
	return scsi(0, cdb, sizeof(cdb), buf, count * BLOCK, &ocs, &st);
}

static uint64_t le64(const uint8_t *p)
{
	uint64_t v = 0;
	for (int i = 7; i >= 0; i--)
		v = v << 8 | p[i];
	return v;
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* Name the filesystem whose superblock starts at block lba (f2fs/ext4). */
static const char *fs_type(uint32_t lba)
{
	if (read_blocks(lba, 1, data))
		return "read error";
	const uint8_t *sb = data + 1024;
	if (sb[0] == 0x10 && sb[1] == 0x20 && sb[2] == 0xf5 && sb[3] == 0xf2)
		return "f2fs";
	if (sb[56] == 0x53 && sb[57] == 0xef)
		return "ext4";
	return "other";
}

/* One line per GPT entry of at least 64 MiB: name, first LBA, size in MiB (4K blocks). */
static void list_partitions(uint32_t entry_lba, uint32_t nent)
{
	static uint8_t ents[128 * 128] __attribute__((aligned(4096)));
	uint32_t blocks = (nent * 128 + BLOCK - 1) / BLOCK;
	if (read_blocks(entry_lba, blocks, ents)) {
		con_str("UFS PART", "read error");
		return;
	}
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = ents + i * 128;
		uint64_t first = le64(e + 32), last = le64(e + 40);
		if (!first)
			continue;
		char name[17];
		for (int c = 0; c < 16; c++) {
			uint8_t ch = e[56 + 2 * c];
			name[c] = (ch >= 32 && ch < 127) ? ch : (ch ? '?' : 0);
		}
		name[16] = 0;
		if (((last - first + 1) >> 8) < 64)	/* keep the screen to the big ones */
			continue;
		con_hex2(name, "lba=", (uint32_t)first, "MiB=", (uint32_t)((last - first + 1) >> 8));
		if (!__builtin_strcmp(name, "userdata") || !__builtin_strcmp(name, "cust") ||
		    !__builtin_strcmp(name, "cache"))
			con_str(" fs", fs_type((uint32_t)first));
	}
}

void test_ufs(void)
{
	con_str("UFS", "milestone 5 probe (read-only)");
	uint32_t cap = ufs_rd(REG_CAP), hcs = ufs_rd(REG_HCS), hce = ufs_rd(REG_HCE);
	con_hex2("UFS", "ver=", ufs_rd(REG_VER), "cap=", cap);
	con_hex2("UFS", "hce=", hce, "hcs=", hcs);
	con_hex2("UFS", "is=", ufs_rd(REG_IS), "rsr=", ufs_rd(REG_UTRLRSR));
	con_hex2("UFS", "utrlba=", ufs_rd(REG_UTRLBA), "hi=", ufs_rd(REG_UTRLBAU));
	con_hex2("UFS", "dbr=", ufs_rd(REG_UTRLDBR), "nutrs=", (cap & 0x1f) + 1);

	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY)) {
		con_str("UFS", "host not enabled/ready: stop");
		return;
	}
	if (ufs_rd(REG_UTRLDBR)) {
		con_str("UFS", "requests outstanding: stop");
		return;
	}

	/* Take over the transfer request list: stop, point at ours, run. */
	ufs_wr(REG_UTRLRSR, 0);
	uintptr_t list = (uintptr_t)utrl;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	con_hex2("UFS", "ours=", (uint32_t)list, "rsr=", ufs_rd(REG_UTRLRSR));
	/*
	 * Never ring a doorbell unless the host really uses our list: otherwise
	 * it would execute whatever ABL left in its own slot 0 (maybe a write).
	 */
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || ufs_rd(REG_UTRLBAU) != (uint32_t)(list >> 32) ||
	    !(ufs_rd(REG_UTRLRSR) & 1)) {
		con_str("UFS", "list base not taken: stop");
		return;
	}

	uint32_t ocs = 0, st = 0, rc;
	static const uint8_t tur[6] = { 0x00 };
	/* A first command may report a unit attention; retry a few times. */
	for (int i = 0; i < 3; i++) {
		rc = scsi(0, tur, sizeof(tur), 0, 0, &ocs, &st);
		con_hex2("UFS TUR", "rc=", rc, "ocs/st=", ocs << 8 | st);
		if (!rc)
			break;
	}

	static const uint8_t cap10[10] = { 0x25 };	/* READ CAPACITY(10) */
	rc = scsi(0, cap10, sizeof(cap10), data, 8, &ocs, &st);
	con_hex2("UFS CAP", "rc=", rc, "ocs/st=", ocs << 8 | st);
	if (!rc)
		con_hex2("UFS LUN0", "lastlba=", be32(data), "blk=", be32(data + 4));

	/* READ(10) LBA 0, 2 blocks: protective MBR and the GPT header. */
	static const uint8_t rd10[10] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 2, 0 };
	rc = scsi(0, rd10, sizeof(rd10), data, 2 * BLOCK, &ocs, &st);
	con_hex2("UFS READ", "rc=", rc, "ocs/st=", ocs << 8 | st);
	if (rc)
		return;
	con_hex2("UFS MBR", "sig=", data[510] | data[511] << 8, "type=", data[450]);
	const uint8_t *gpt = data + BLOCK;
	char sig[9];
	for (int i = 0; i < 8; i++)
		sig[i] = (gpt[i] >= 32 && gpt[i] < 127) ? gpt[i] : '.';
	sig[8] = 0;
	con_str("UFS GPT", sig);
	uint32_t nent = gpt[80] | gpt[81] << 8 | gpt[82] << 16, esz = gpt[84] | gpt[85] << 8;
	uint32_t elba = gpt[72] | gpt[73] << 8;
	con_hex2("UFS GPT", "entries=", nent, "lba=", elba);
	if (esz != 128 || nent > 128)
		return;
	list_partitions(elba, nent);
}

/* ------------------------------------------------------------------ */
/* Persistent console ring (kernel patch 0047) reader.                */
/*
 * The kernel keeps the last 4 MiB + 4 KiB of every console line in the
 * userdata partition: device block 0 of that area is a header
 * ("SWEETLOG", u32 version, u32 ring size, u64 total bytes written,
 * u64 flush sequence) and the ring follows. Byte n of the console
 * stream lives at ring offset n % size, so the newest text is right
 * before offset total % size. Boot this shim after a run that hung and
 * before the next one: it reads the ring left from that run.
 */

#define SWEETLOG_BLOCKS	1025u		/* 1 header block + 4 MiB ring */
#define LOG_WIN		32768u		/* bytes of the stream we scan */
#define LOG_COLS	44		/* console columns at scale 3 (1080/24) */
#define LOG_MAXLINES	20

static uint8_t log_win[LOG_WIN] __attribute__((aligned(4096)));
static uint8_t log_blk[BLOCK] __attribute__((aligned(4096)));
static uint8_t log_ents[128 * 128] __attribute__((aligned(4096)));

/* Same takeover test as test_ufs(), minus the reporting. */
static int log_host_ready(void)
{
	uint32_t hcs = ufs_rd(REG_HCS), hce = ufs_rd(REG_HCE);
	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY))
		return 1;
	if (ufs_rd(REG_UTRLDBR))
		return 2;
	ufs_wr(REG_UTRLRSR, 0);
	uintptr_t list = (uintptr_t)utrl;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || ufs_rd(REG_UTRLBAU) != (uint32_t)(list >> 32) ||
	    !(ufs_rd(REG_UTRLRSR) & 1))
		return 3;
	static const uint8_t tur[6] = { 0 };
	uint32_t ocs, st;
	for (int i = 0; i < 3; i++)
		if (!scsi(0, tur, sizeof(tur), 0, 0, &ocs, &st))
			return 0;
	return 4;
}

/* Byte-wise compare/substring: no libc (no memcmp) in a freestanding image. */
static int log_eq(const uint8_t *a, const char *b, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++)
		if (a[i] != (uint8_t)b[i])
			return 0;
	return 1;
}

static int log_has(const uint8_t *s, uint32_t n, const char *w, uint32_t wl)
{
	for (uint32_t i = 0; i + wl <= n; i++) {
		uint32_t j = 0;
		while (j < wl && s[i + j] == (uint8_t)w[j])
			j++;
		if (j == wl)
			return 1;
	}
	return 0;
}

static int log_interesting(const uint8_t *s, uint32_t n)
{
	return log_has(s, n, "SWEET", 5) || log_has(s, n, "CRASH", 5) ||
	       log_has(s, n, "panic", 5) || log_has(s, n, "SMP", 3) ||
	       log_has(s, n, "FPS", 3);
}

/* One log line on the console: first LOG_COLS chars, and for longer lines
 * the tail (the values usually sit there) as a second row, so a wrapped line
 * never pushes the newer lines off the screen. */
static void log_print(const uint8_t *s, uint32_t n)
{
	char buf[LOG_COLS + 1];

	if (!n) {
		con_raw("");
		return;
	}
	uint32_t first = n < LOG_COLS ? n : LOG_COLS;
	for (uint32_t i = 0; i < first; i++)
		buf[i] = (s[i] >= 32 && s[i] < 127) ? (char)s[i] : '.';
	buf[first] = 0;
	con_raw(buf);
	if (n <= LOG_COLS)
		return;
	buf[0] = '~';
	uint32_t start = n - (LOG_COLS - 1);
	for (uint32_t i = 0; i < LOG_COLS - 1; i++)
		buf[i + 1] = (s[start + i] >= 32 && s[start + i] < 127) ? (char)s[start + i] : '.';
	buf[LOG_COLS] = 0;
	con_raw(buf);
}

void ufs_log_dump(void)
{
	con_str("LOGDUMP", "reading the SWEETLOG ring from userdata");

	int rc = log_host_ready();
	if (rc) {
		con_hex2("LOGDUMP", "host rc=", (uint32_t)rc, 0, 0);
		return;
	}

	/* GPT header, then the entries; find "userdata". */
	if (read_blocks(0, 2, data)) {
		con_str("LOGDUMP", "GPT header read error");
		return;
	}
	const uint8_t *gpt = data + BLOCK;
	if (!log_eq(gpt, "EFI PART", 8)) {
		con_str("LOGDUMP", "no GPT signature");
		return;
	}
	uint32_t nent = gpt[80] | gpt[81] << 8 | gpt[82] << 16;
	uint32_t elba = gpt[72] | gpt[73] << 8;
	if (gpt[84] != 128 || gpt[85] != 0 || nent == 0 || nent > 128) {
		con_str("LOGDUMP", "unexpected GPT entry size");
		return;
	}
	if (read_blocks(elba, (nent * 128 + BLOCK - 1) / BLOCK, log_ents)) {
		con_str("LOGDUMP", "GPT entries read error");
		return;
	}
	uint64_t first = 0, last = 0;
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = log_ents + i * 128;
		char name[17];
		for (int c = 0; c < 16; c++) {
			uint8_t ch = e[56 + 2 * c];
			name[c] = (ch >= 32 && ch < 127) ? (char)ch : 0;
		}
		name[16] = 0;
		if (!__builtin_strcmp(name, "userdata")) {
			first = le64(e + 32);
			last = le64(e + 40);
			break;
		}
	}
	if (!first || last < first) {
		con_str("LOGDUMP", "no userdata partition");
		return;
	}
	uint64_t base = first + (last - first + 1) - SWEETLOG_BLOCKS;
	con_hex2("LOGDUMP", "first=", (uint32_t)first, "n=", (uint32_t)(last - first + 1));
	con_hex("LOGDUMP base", base);

	if (read_blocks((uint32_t)base, 1, log_blk)) {
		con_str("LOGDUMP", "log header read error");
		return;
	}
	if (!log_eq(log_blk, "SWEETLOG", 8)) {
		con_str("LOGDUMP", "no SWEETLOG magic (never flushed?)");
		return;
	}
	uint32_t size = le32(log_blk + 12);
	uint64_t total = le64(log_blk + 16);
	if (!size || size > 16 * 1024 * 1024) {
		con_hex2("LOGDUMP", "bad ring size=", size, 0, 0);
		return;
	}

	/* Copy the last LOG_WIN bytes of the stream, following the wrap. */
	uint32_t len = total > LOG_WIN ? LOG_WIN : (uint32_t)total;
	uint64_t n = total - len;
	uint32_t done = 0;
	while (n < total) {
		uint32_t r = (uint32_t)(n % size);
		uint32_t off = r % BLOCK;
		if (read_blocks((uint32_t)(base + 1 + r / BLOCK), 1, log_blk)) {
			con_str("LOGDUMP", "ring read error");
			return;
		}
		uint32_t chunk = BLOCK - off;
		if (chunk > len - done)
			chunk = len - done;
		for (uint32_t i = 0; i < chunk; i++)
			log_win[done + i] = log_blk[off + i];
		done += chunk;
		n += chunk;
	}

	/* One pass: keep the last LOG_MAXLINES interesting line starts. */
	uint32_t ring[LOG_MAXLINES];
	uint32_t seen = 0, s = 0;
	for (uint32_t i = 0; i <= done; i++) {
		if (i != done && log_win[i] != '\n')
			continue;
		uint32_t m = i - s;
		while (m && log_win[s + m - 1] == '\r')
			m--;
		if (m && log_interesting(log_win + s, m)) {
			ring[seen % LOG_MAXLINES] = s;
			seen++;
		}
		s = i + 1;
	}

	uint32_t np = seen < LOG_MAXLINES ? seen : LOG_MAXLINES;
	con_hex2("LOGDUMP", "total=", (uint32_t)total, "keep=", (uint32_t)np);
	if (!np) {
		con_str("LOGDUMP", "no SWEET/CRASH/SMP lines in window");
		return;
	}
	/* Draw the kept lines oldest-first so time reads downward. */
	for (uint32_t k = 0; k < np; k++) {
		s = ring[(seen - np + k) % LOG_MAXLINES];
		uint32_t e = s;
		while (e < done && log_win[e] != '\n')
			e++;
		uint32_t m = e - s;
		while (m && log_win[s + m - 1] == '\r')
			m--;
		log_print(log_win + s, m);
	}
}
