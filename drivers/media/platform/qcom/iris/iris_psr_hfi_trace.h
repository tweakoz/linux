/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __IRIS_PSR_HFI_TRACE_H__
#define __IRIS_PSR_HFI_TRACE_H__

#include <linux/atomic.h>
#include <linux/of.h>
#include "iris_hfi_gen2_packet.h"

/* Diagnostic only: bounded metadata, no payload dump or device access. */
static atomic_t iris_psr_hfi_trace_budget = ATOMIC_INIT(256);

static inline bool iris_psr_hfi_trace_take(struct iris_core *core)
{
	return of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris") &&
		atomic_dec_if_positive(&iris_psr_hfi_trace_budget) >= 0;
}

static inline void iris_psr_hfi_trace_packet(struct iris_core *core,
		const char *event, u32 session, u32 header_id,
		const struct iris_hfi_packet *pkt, u32 available, int ret, u32 state)
{
	struct iris_hfi_packet packet;
	struct iris_hfi_buffer buffer;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris") ||
	    available < sizeof(packet))
		return;
	memcpy(&packet, pkt, sizeof(packet));
	if (packet.size < sizeof(packet) || packet.size > available ||
	    !iris_psr_hfi_trace_take(core))
		return;

	dev_info(core->dev,
		 "PSR HFI flow %s session=%#x header=%u packet=%u type=%#x port=%u flags=%#x payload=%u size=%u ret=%d state=%u\n",
		 event, session, header_id, packet.packet_id, packet.type,
		 packet.port, packet.flags, packet.payload_info, packet.size, ret, state);
	if (packet.type != HFI_CMD_BUFFER ||
	    packet.payload_info != HFI_PAYLOAD_STRUCTURE ||
	    packet.size - sizeof(packet) < sizeof(buffer))
		return;
	memcpy(&buffer, (const u8 *)pkt + sizeof(packet), sizeof(buffer));
	dev_info(core->dev,
		 "PSR HFI buffer %s session=%#x packet=%u type=%u index=%u flags=%#x size=%u data_offset=%u data_size=%u ret=%d\n",
		 event, session, packet.packet_id, buffer.type, buffer.index,
		 buffer.flags, buffer.buffer_size, buffer.data_offset, buffer.data_size, ret);
}

static inline void iris_psr_hfi_trace_header(struct iris_core *core,
		const char *event, const void *raw, u32 available, int ret)
{
	struct iris_hfi_header header;
	struct iris_hfi_packet packet;
	const u8 *cursor;
	u32 remaining, i;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris") ||
	    available < sizeof(header) || available > IFACEQ_CORE_PKT_SIZE)
		return;
	memcpy(&header, raw, sizeof(header));
	if (header.size < sizeof(header) || header.size > available)
		return;
	remaining = header.size - sizeof(header);
	if (header.num_packets > remaining / sizeof(packet))
		return;
	cursor = (const u8 *)raw + sizeof(header);
	for (i = 0; i < header.num_packets; i++) {
		if (remaining < sizeof(packet))
			return;
		memcpy(&packet, cursor, sizeof(packet));
		if (packet.size < sizeof(packet) || packet.size > remaining)
			return;
		iris_psr_hfi_trace_packet(core, event, header.session_id, header.header_id,
					 (const struct iris_hfi_packet *)cursor,
					 packet.size, ret, core->state);
		remaining -= packet.size;
		cursor += packet.size;
	}
}

static inline void iris_psr_hfi_trace_done(struct iris_core *core, u32 session,
		u32 type, u32 index, u32 attr, u32 data_size, int ret)
{
	if (iris_psr_hfi_trace_take(core))
		dev_info(core->dev,
			 "PSR HFI vb2-done session=%#x type=%u index=%u attr=%#x data_size=%u ret=%d\n",
			 session, type, index, attr, data_size, ret);
}

#endif
