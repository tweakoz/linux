/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef IRIS_PSR_SOCINFO_H
#define IRIS_PSR_SOCINFO_H

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/soc/qcom/smem.h>

/* Pinned OEM socinfo layout, not the Iris WRAPPER_EFUSE_MONITOR bitmap.
 * The pure decoder is separate so malformed firmware data can be tested.
 * Success alone writes *part; errors never select a default SKU.
 */
static inline int iris_psr_decode_video_part(const void *data, size_t size,
					    u32 *part)
{
	static const u16 header_sizes[] = {172, 176, 192, 196, 204,
					 216, 220, 224, 228, 236};
	const u32 video_index = 2; /* OEM enum: UNKNOWN=0, GPU=1, VIDEO=2. */
	const u8 *p = data;
	u32 version, count, offset;
	size_t header;

	if (!p || !part || size < 4)
		return -EINVAL;
	version = get_unaligned_le32(p);
	/* Only the exact published 0.14..0.23 layouts are understood. */
	if (version < 14 || version > 23)
		return -EOPNOTSUPP;
	header = header_sizes[version - 14];
	if (size < header)
		return -EINVAL;
	count = get_unaligned_le32(p + 164);
	offset = get_unaligned_le32(p + (version >= 21 ? 220 : 168));
	/* Subtract/divide before comparison to avoid offset+count overflow. */
	if (!count || video_index >= count || offset < header ||
	    offset > size || count > (size - offset) / sizeof(u32))
		return -EINVAL;
	*part = get_unaligned_le32(p + offset + (size_t)video_index * sizeof(u32));
	return 0;
}

static inline int iris_psr_video_part(u32 *part)
{
	size_t size;
	const void *data = qcom_smem_get(QCOM_SMEM_HOST_ANY, 137, &size);

	if (IS_ERR(data))
		return PTR_ERR(data);
	return iris_psr_decode_video_part(data, size, part);
}

#endif
