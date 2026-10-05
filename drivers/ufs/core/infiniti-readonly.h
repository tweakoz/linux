/* SPDX-License-Identifier: GPL-2.0 */
#ifndef INFINITI_READONLY_H
#define INFINITI_READONLY_H

/* Diagnostic-only allowlist. Unknown commands fail closed, including commands
 * with no outbound payload (UNMAP, FORMAT, WRITE SAME, vendor commands, etc.).
 * Kept independent of kernel types so all opcode/direction combinations can
 * be exercised on the host without a UFS device.
 */
static inline int infiniti_scsi_read_allowed(const unsigned char *cdb,
		unsigned int len, int data_in, int data_none)
{
	if (!len || (!data_in && !data_none))
		return 0;
	switch (cdb[0]) {
	case 0x00: /* TEST UNIT READY */
		return len == 6 && data_none;
	case 0x1b: /* START STOP UNIT, UFS power-mode form only: no START/LOEJ bits */
		return len == 6 && data_none && cdb[1] == 0 && !(cdb[4] & 0x0f);
	case 0x03: /* REQUEST SENSE */
	case 0x08: /* READ(6) */
	case 0x12: /* INQUIRY */
	case 0x1a: /* MODE SENSE(6) */
		return len == 6 && data_in;
	case 0x25: /* READ CAPACITY(10) */
	case 0x28: /* READ(10) */
	case 0x5a: /* MODE SENSE(10) */
		return len == 10 && data_in;
	case 0x88: /* READ(16) */
		return len == 16 && data_in;
	case 0x9e: /* Only READ CAPACITY(16), not other service actions. */
		return len == 16 && cdb[1] == 0x10 && data_in;
	case 0xa0: /* REPORT LUNS */
	case 0xa8: /* READ(12) */
		return len == 12 && data_in;
	default:
		return 0;
	}
}

static inline int infiniti_query_allowed(unsigned char opcode,
		unsigned char idn, unsigned char index, unsigned char selector)
{
	/* Read descriptor, read attribute, read flag. */
	if (opcode == 1 || opcode == 3 || opcode == 5)
		return 1;
	/* Required device-initialization handshake; no other flag changes. */
	return opcode == 6 && idn == 1 && index == 0 && selector == 0;
}
/* CPH2749 captured userdata geometry, in 4096-byte UFS logical blocks.
 * This optional policy is deliberately fixed to this device's current GPT.
 * It does not make Android encrypted data readable or preserve that data.
 */
#define INFINITI_USERDATA_FIRST 4551404ULL
#define INFINITI_USERDATA_BLOCKS 57297167ULL

/* Generated bounded write policy; userdata constants are NOT permission ranges. */
#define INFINITI_WRITE_POLICY_MODE "experimental"
#define INFINITI_WRITE_PROFILE_SHA256 "90df78e0166cb0fc9cf5e06b38ccc9f1ba62b18517645fff7bcb17d2af543363"
#define INFINITI_WRITE_LAYOUT_SHA256 "98f0ef83ff67492debff4e633bd0ed26530b6e67fef541de74ac8df8ac7d6f73"
#define INFINITI_WRITE_RANGE_COUNT 2
/* experimental: [21332716, 38109932) in 4096-byte UFS blocks. */
#define INFINITI_WRITE_RANGE_0_FIRST 21332716ULL
#define INFINITI_WRITE_RANGE_0_BLOCKS 16777216ULL
/* shared: [38109932, 61848300) in 4096-byte UFS blocks. */
#define INFINITI_WRITE_RANGE_1_FIRST 38109932ULL
#define INFINITI_WRITE_RANGE_1_BLOCKS 23738368ULL

static inline unsigned long long infiniti_cdb_be(const unsigned char *p,
		unsigned int n)
{
	unsigned long long v = 0;
	while (n--)
		v = (v << 8) | *p++;
	return v;
}

static inline int infiniti_userdata_write_allowed(const unsigned char *cdb,
		unsigned int len, unsigned long long lun,
		unsigned int sector_size, unsigned long long bytes,
		int data_out, int data_none)
{
	unsigned long long lba, blocks;
	unsigned int i;

	if (!INFINITI_WRITE_RANGE_COUNT || !len || lun != 0 || sector_size != 4096)
		return 0;
	/* Full-LUN cache flush only. This commits previous permitted writes;
	 * it does not request new data, layout or provisioning changes.
	 */
	if ((cdb[0] == 0x35 && len == 10) ||
	    (cdb[0] == 0x91 && len == 16)) {
		if (!data_none || data_out || bytes)
			return 0;
		for (i = 1; i < len; i++)
			if (cdb[i])
				return 0;
		return 1;
	}
	if ((len != 10 && len != 16) || !data_out || data_none ||
	    (cdb[1] & ~0x08))
		return 0; /* FUA allowed, protection/other flags rejected. */
	if (cdb[0] == 0x2a && len == 10) {
		if (cdb[6] || cdb[9])
			return 0;
		lba = infiniti_cdb_be(cdb + 2, 4);
		blocks = infiniti_cdb_be(cdb + 7, 2);
	} else if (cdb[0] == 0x8a && len == 16) {
		if (cdb[14] || cdb[15])
			return 0;
		lba = infiniti_cdb_be(cdb + 2, 8);
		blocks = infiniti_cdb_be(cdb + 10, 4);
	} else {
		return 0;
	}
	/* No addition of untrusted LBA/count: reject overflow and every region seam. */
	if (!blocks || bytes != blocks * 4096ULL)
		return 0;
	if (lba >= INFINITI_WRITE_RANGE_0_FIRST &&
	    lba - INFINITI_WRITE_RANGE_0_FIRST < INFINITI_WRITE_RANGE_0_BLOCKS &&
	    blocks <= INFINITI_WRITE_RANGE_0_BLOCKS - (lba - INFINITI_WRITE_RANGE_0_FIRST))
		return 1;
	if (lba >= INFINITI_WRITE_RANGE_1_FIRST &&
	    lba - INFINITI_WRITE_RANGE_1_FIRST < INFINITI_WRITE_RANGE_1_BLOCKS &&
	    blocks <= INFINITI_WRITE_RANGE_1_BLOCKS - (lba - INFINITI_WRITE_RANGE_1_FIRST))
		return 1;
	return 0;
}

#endif
