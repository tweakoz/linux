/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __IRIS_PSR_HFI_TRACE_H__
#define __IRIS_PSR_HFI_TRACE_H__

#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/of.h>
#include "iris_hfi_gen2_packet.h"

/* Diagnostic only: finite per-translation-unit metadata; no device access. */
#define IRIS_PSR_TRACE_SESSIONS 4
#define IRIS_PSR_TRACE_EVENTS 256
static DEFINE_SPINLOCK(iris_psr_hfi_trace_lock);
static unsigned int iris_psr_hfi_trace_budget = IRIS_PSR_TRACE_EVENTS;
static unsigned int iris_psr_hfi_trace_sessions;
static struct {
	struct iris_core *core;
	u32 session;
	unsigned int remaining;
} iris_psr_hfi_trace_slots[IRIS_PSR_TRACE_SESSIONS];

/* Only successful enqueue/receive OPEN begins a new, finite session slot. */
static inline void iris_psr_hfi_trace_open(struct iris_core *core,
		const char *event, u32 session, const struct iris_hfi_packet *packet,
		const struct iris_hfi_packet *raw, int ret)
{
	unsigned long flags;
	unsigned int slot;

	if (packet->type != HFI_CMD_OPEN || ret)
		return;
	if (!strcmp(event, "enqueue")) {
		if (packet->payload_info != HFI_PAYLOAD_U32 ||
		    packet->size - sizeof(*packet) < sizeof(session))
			return;
		memcpy(&session, (const u8 *)raw + sizeof(*packet), sizeof(session));
	} else if (strcmp(event, "receive") ||
		   packet->flags != HFI_FW_FLAGS_SUCCESS) {
		return;
	}
	if (!session)
		return;

	spin_lock_irqsave(&iris_psr_hfi_trace_lock, flags);
	if (iris_psr_hfi_trace_sessions < IRIS_PSR_TRACE_SESSIONS) {
		slot = iris_psr_hfi_trace_sessions++;
		iris_psr_hfi_trace_slots[slot].core = core;
		iris_psr_hfi_trace_slots[slot].session = session;
		iris_psr_hfi_trace_slots[slot].remaining = IRIS_PSR_TRACE_EVENTS;
	}
	spin_unlock_irqrestore(&iris_psr_hfi_trace_lock, flags);
}

static inline bool iris_psr_hfi_trace_take(struct iris_core *core, u32 session)
{
	unsigned int *budget = &iris_psr_hfi_trace_budget;
	unsigned long flags;
	bool taken = false;
	int i;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris"))
		return false;
	spin_lock_irqsave(&iris_psr_hfi_trace_lock, flags);
	/* Newest wins when a later OPEN reuses the driver's hashed session ID. */
	for (i = (int)iris_psr_hfi_trace_sessions - 1; i >= 0; i--) {
		if (iris_psr_hfi_trace_slots[i].core == core &&
		    iris_psr_hfi_trace_slots[i].session == session) {
			budget = &iris_psr_hfi_trace_slots[i].remaining;
			break;
		}
	}
	if (*budget) {
		--*budget;
		taken = true;
	}
	spin_unlock_irqrestore(&iris_psr_hfi_trace_lock, flags);
	return taken;
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
	if (packet.size < sizeof(packet) || packet.size > available)
		return;
	iris_psr_hfi_trace_open(core, event, session, &packet, pkt, ret);
	if (!iris_psr_hfi_trace_take(core, session))
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
	if (iris_psr_hfi_trace_take(core, session))
		dev_info(core->dev,
			 "PSR HFI vb2-done session=%#x type=%u index=%u attr=%#x data_size=%u ret=%d\n",
			 session, type, index, attr, data_size, ret);
}

#endif
