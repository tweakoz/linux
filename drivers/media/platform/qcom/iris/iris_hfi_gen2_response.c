// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-mem2mem.h>

#include "iris_hfi_gen2.h"
#include "iris_hfi_gen2_defines.h"
#include "iris_hfi_gen2_packet.h"
#include "iris_vdec.h"
#include "iris_vpu_buffer.h"
#include "iris_vpu_common.h"
#include "iris_vpu_register_defines.h"
#include "iris_psr_hfi_trace.h"

struct iris_hfi_gen2_core_hfi_range {
	u32 begin;
	u32 end;
	int (*handle)(struct iris_core *core, struct iris_hfi_packet *pkt);
};

struct iris_hfi_gen2_inst_hfi_range {
	u32 begin;
	u32 end;
	int (*handle)(struct iris_inst *inst, struct iris_hfi_packet *pkt);
};

struct iris_hfi_gen2_packet_handle {
	enum hfi_buffer_type type;
	int (*handle)(struct iris_inst *inst, struct iris_hfi_packet *pkt);
};

static u32 iris_hfi_gen2_buf_type_to_driver(struct iris_inst *inst,
					    enum hfi_buffer_type buf_type)
{
	switch (buf_type) {
	case HFI_BUFFER_BITSTREAM:
		return BUF_INPUT;
	case HFI_BUFFER_RAW:
		return BUF_OUTPUT;
	case HFI_BUFFER_BIN:
		return BUF_BIN;
	case HFI_BUFFER_ARP:
		return BUF_ARP;
	case HFI_BUFFER_COMV:
		return BUF_COMV;
	case HFI_BUFFER_NON_COMV:
		return BUF_NON_COMV;
	case HFI_BUFFER_LINE:
		return BUF_LINE;
	case HFI_BUFFER_DPB:
		if (inst->domain == DECODER)
			return BUF_DPB;
		else
			return BUF_SCRATCH_2;
	case HFI_BUFFER_PERSIST:
		return BUF_PERSIST;
	case HFI_BUFFER_PARTIAL_DATA:
		return BUF_PARTIAL;
	case HFI_BUFFER_VPSS:
		return BUF_VPSS;
	default:
		return 0;
	}
}

static bool iris_hfi_gen2_is_valid_hfi_buffer_type(u32 buffer_type)
{
	switch (buffer_type) {
	case HFI_BUFFER_BITSTREAM:
	case HFI_BUFFER_RAW:
	case HFI_BUFFER_BIN:
	case HFI_BUFFER_ARP:
	case HFI_BUFFER_COMV:
	case HFI_BUFFER_NON_COMV:
	case HFI_BUFFER_LINE:
	case HFI_BUFFER_DPB:
	case HFI_BUFFER_PERSIST:
	case HFI_BUFFER_VPSS:
	case HFI_BUFFER_PARTIAL_DATA:
		return true;
	default:
		return false;
	}
}

static bool iris_hfi_gen2_is_valid_hfi_port(u32 port, u32 buffer_type)
{
	if (port == HFI_PORT_NONE && buffer_type != HFI_BUFFER_PERSIST)
		return false;

	if (port != HFI_PORT_BITSTREAM && port != HFI_PORT_RAW)
		return false;

	return true;
}

static int iris_hfi_gen2_get_driver_buffer_flags(struct iris_inst *inst, u32 hfi_flags)
{
	struct iris_inst_hfi_gen2 *inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);
	u32 keyframe = HFI_GEN2_PICTURE_IDR | HFI_GEN2_PICTURE_I |
		HFI_GEN2_PICTURE_CRA | HFI_GEN2_PICTURE_BLA;
	u32 driver_flags = 0;

	if (inst_hfi_gen2->hfi_frame_info.picture_type & HFI_GEN2_PICTURE_NOSHOW)
		driver_flags |= V4L2_BUF_FLAG_ERROR;
	else if (inst_hfi_gen2->hfi_frame_info.picture_type & keyframe)
		driver_flags |= V4L2_BUF_FLAG_KEYFRAME;
	else if (inst_hfi_gen2->hfi_frame_info.picture_type & HFI_GEN2_PICTURE_P)
		driver_flags |= V4L2_BUF_FLAG_PFRAME;
	else if (inst_hfi_gen2->hfi_frame_info.picture_type & HFI_GEN2_PICTURE_B)
		driver_flags |= V4L2_BUF_FLAG_BFRAME;

	if (inst_hfi_gen2->hfi_frame_info.data_corrupt || inst_hfi_gen2->hfi_frame_info.overflow)
		driver_flags |= V4L2_BUF_FLAG_ERROR;

	if (hfi_flags & HFI_BUF_FW_FLAG_LAST ||
	    hfi_flags & HFI_BUF_FW_FLAG_PSC_LAST)
		driver_flags |= V4L2_BUF_FLAG_LAST;

	return driver_flags;
}

static bool iris_hfi_gen2_validate_packet_payload(struct iris_hfi_packet *pkt)
{
	u32 payload_size = 0;

	switch (pkt->payload_info) {
	case HFI_PAYLOAD_U32:
	case HFI_PAYLOAD_S32:
	case HFI_PAYLOAD_Q16:
	case HFI_PAYLOAD_U32_ENUM:
	case HFI_PAYLOAD_32_PACKED:
		payload_size = 4;
		break;
	case HFI_PAYLOAD_U64:
	case HFI_PAYLOAD_S64:
	case HFI_PAYLOAD_64_PACKED:
		payload_size = 8;
		break;
	case HFI_PAYLOAD_STRUCTURE:
		if (pkt->type == HFI_CMD_BUFFER)
			payload_size = sizeof(struct iris_hfi_buffer);
		break;
	default:
		payload_size = 0;
		break;
	}

	if (pkt->size < sizeof(struct iris_hfi_packet) + payload_size)
		return false;

	return true;
}

static int iris_hfi_gen2_validate_packet(u8 *response_pkt, u8 *core_resp_pkt)
{
	u8 *response_limit = core_resp_pkt + IFACEQ_CORE_PKT_SIZE;
	u32 response_pkt_size = *(u32 *)response_pkt;

	if (!response_pkt_size)
		return -EINVAL;

	if (response_pkt_size < sizeof(struct iris_hfi_packet))
		return -EINVAL;

	if (response_pkt + response_pkt_size > response_limit)
		return -EINVAL;

	return 0;
}

static int iris_hfi_gen2_validate_hdr_packet(struct iris_core *core, struct iris_hfi_header *hdr)
{
	struct iris_hfi_packet *packet;
	int ret;
	u8 *pkt;
	u32 i;

	if (hdr->size < sizeof(*hdr) + sizeof(*packet))
		return -EINVAL;

	pkt = (u8 *)((u8 *)hdr + sizeof(*hdr));

	for (i = 0; i < hdr->num_packets; i++) {
		packet = (struct iris_hfi_packet *)pkt;
		ret = iris_hfi_gen2_validate_packet(pkt, core->response_packet);
		if (ret)
			return ret;

		pkt += packet->size;
	}

	return 0;
}

static int iris_hfi_gen2_handle_session_info(struct iris_inst *inst,
					     struct iris_hfi_packet *pkt)
{
	struct iris_inst_hfi_gen2 *inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);
	struct iris_core *core = inst->core;
	int ret = 0;
	char *info;

	switch (pkt->type) {
	case HFI_INFO_UNSUPPORTED:
		info = "unsupported";
		break;
	case HFI_INFO_DATA_CORRUPT:
		info = "data corrupt";
		inst_hfi_gen2->hfi_frame_info.data_corrupt = 1;
		break;
	case HFI_INFO_BUFFER_OVERFLOW:
		info = "buffer overflow";
		inst_hfi_gen2->hfi_frame_info.overflow = 1;
		break;
	case HFI_INFO_HFI_FLAG_DRAIN_LAST:
		info = "drain last flag";
		ret = iris_inst_sub_state_change_drain_last(inst);
		break;
	case HFI_INFO_HFI_FLAG_PSC_LAST:
		info = "drc last flag";
		ret = iris_inst_sub_state_change_drc_last(inst);
		break;
	default:
		info = "unknown";
		break;
	}

	dev_dbg(core->dev, "session info received %#x: %s\n",
		pkt->type, info);

	return ret;
}

static int iris_hfi_gen2_handle_session_error(struct iris_inst *inst,
					      struct iris_hfi_packet *pkt)
{
	struct iris_core *core = inst->core;
	char *error;

	switch (pkt->type) {
	case HFI_ERROR_MAX_SESSIONS:
		error = "exceeded max sessions";
		break;
	case HFI_ERROR_UNKNOWN_SESSION:
		error = "unknown session id";
		break;
	case HFI_ERROR_INVALID_STATE:
		error = "invalid operation for current state";
		break;
	case HFI_ERROR_INSUFFICIENT_RESOURCES:
		error = "insufficient resources";
		break;
	case HFI_ERROR_BUFFER_NOT_SET:
		error = "internal buffers not set";
		break;
	case HFI_ERROR_FATAL:
		error = "fatal error";
		break;
	case HFI_ERROR_STREAM_UNSUPPORTED:
		error = "unsupported stream";
		break;
	default:
		error = "unknown";
		break;
	}

	dev_err(core->dev, "session error received %#x: %s\n", pkt->type, error);
	iris_vb2_queue_error(inst);
	iris_inst_change_state(inst, IRIS_INST_ERROR);

	return 0;
}

/* Called with core->lock held, before recovery can release SFR memory. */
static void iris_hfi_gen2_log_sfr(struct iris_core *core)
{
	const char *reason;
	size_t length, offset, count;
	u32 size;
	char chunk[257];

	if (!core->sfr_vaddr)
		return;

	/* Coherent firmware DMA precedes its failure notification. */
	dma_rmb();
	size = READ_ONCE(*(u32 *)core->sfr_vaddr);
	if (size != SFR_SIZE) {
		dev_err(core->dev, "SFR invalid size %u (allocation %u)\n", size, SFR_SIZE);
		return;
	}

	reason = (const char *)core->sfr_vaddr + sizeof(u32);
	length = strnlen(reason, SFR_SIZE - sizeof(u32));
	dev_err(core->dev, "SFR reason bytes=%zu terminated=%u\n", length,
		(unsigned int)(length < SFR_SIZE - sizeof(u32)));
	/* Bounded chunks avoid printk truncation; never alter firmware memory. */
	for (offset = 0; offset < length; offset += count) {
		count = min_t(size_t, sizeof(chunk) - 1, length - offset);
		memcpy(chunk, reason + offset, count);
		chunk[count] = '\0';
		dev_err(core->dev, "SFR[%zu]: %s\n", offset, chunk);
	}
}

/* OEM Iris4 NoC fatal diagnostic: read only, while existing core lock is held. */
static void iris_psr_log_noc(struct iris_core *core, const char *phase)
{
	static const u32 offsets[] = {
		0x1a020, 0x1a024, 0x1a028, 0x1a02c,
		0x1a030, 0x1a034, 0x1a038, 0x1a03c,
	};
	u32 power, value;
	unsigned int i;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris"))
		return;
	/* OEM WRAPPER_CORE_POWER_STATUS: bit1 means hardware powered on. */
	power = readl(core->reg_base + 0xb0080);
	dev_err(core->dev, "PSR %s CORE_POWER_STATUS=%#x\n", phase, power);
	if (!(power & BIT(1))) {
		dev_err(core->dev, "PSR %s NoC skipped: hardware power collapsed\n", phase);
		return;
	}
	/* Native iris_vpu4x: EFUSE APV-disable bit27; CLOCK_CONFIG APV-halt bit1. */
	value = readl(core->reg_base + 0xb0008);
	dev_err(core->dev, "PSR %s WRAPPER_EFUSE_MONITOR=%#x\n", phase, value);
	value = readl(core->reg_base + 0xb0088);
	dev_err(core->dev, "PSR %s WRAPPER_CORE_CLOCK_CONFIG=%#x\n", phase, value);
	/* Read-only native/OEM reset handshake state before recovery. */
	value = readl(core->reg_base + 0x1f000);
	dev_err(core->dev, "PSR %s MVP_NOC_RESET_REQ=%#x\n", phase, value);
	value = readl(core->reg_base + 0x1f004);
	dev_err(core->dev, "PSR %s MVP_NOC_RESET_ACK=%#x\n", phase, value);
	value = readl(core->reg_base + 0x1f008);
	dev_err(core->dev, "PSR %s MVP_NOC_RESET_SYNCRST=%#x\n", phase, value);
	/* Exact OEM ERRLOG0..3 low/high reads; never ERRCLR or MAINCTL. */
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		value = readl(core->reg_base + offsets[i]);
		dev_err(core->dev, "PSR %s NOC offset=%#x value=%#x\n", phase,
			offsets[i], value);
	}
}

/* Diagnostic only: never resume, enable or write VIDEOCC to obtain evidence. */
static void iris_psr_log_videocc(struct iris_core *core, const char *phase)
{
	static const u32 offsets[] = {
		0x80e4, /* memory enable/ACK */
		0x80e0, 0x80d0, 0x80bc, 0x8134, 0x8108, 0x8090, 0x8174, 0x8164,
		0x80a8, 0x8120, 0x80f4, 0x807c, 0x814c, /* GDSCs */
		0x80ac, 0x8124, 0x80f8, 0x8080, 0x8150, /* CFG_GDSCR completion */
		0x8030, 0x8034, 0x8000, 0x8004, 0x8018, 0x801c, 0x8048, 0x804c,
		0x817c, 0x81bc, 0x81b0, 0x81ac, /* critical branches */
	};
	u32 values[ARRAY_SIZE(offsets)] = { 0 };
	int errors[ARRAY_SIZE(offsets)];
	struct device_node *node;
	struct platform_device *pdev;
	struct regmap *map;
	unsigned int i;
	int active;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris"))
		return;
	node = of_find_compatible_node(NULL, NULL, "qcom,kaanapali-videocc");
	if (!node) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: no owner node\n", phase);
		return;
	}
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: no owner device\n", phase);
		return;
	}
	/* Avoid waiting on driver bind/unbind while holding the Iris core lock. */
	if (!device_trylock(&pdev->dev)) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: owner lock busy\n", phase);
		goto put_device;
	}
	if (!pdev->dev.driver) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: owner unbound\n", phase);
		goto unlock;
	}
	active = pm_runtime_get_if_in_use(&pdev->dev);
	if (active <= 0) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: existing runtime user unavailable (%d)\n", phase,
			active);
		goto unlock;
	}
	map = dev_get_regmap(&pdev->dev, NULL);
	if (!map) {
		dev_err(core->dev, "PSR %s VIDEOCC skipped: owner regmap unavailable\n", phase);
		goto put_runtime;
	}
	/* Collect first, then print: console throughput must not space MMIO reads. */
	for (i = 0; i < ARRAY_SIZE(offsets); i++)
		errors[i] = regmap_read(map, offsets[i], &values[i]);
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		if (errors[i])
			dev_err(core->dev, "PSR %s VIDEOCC offset=%#x read_error=%d\n", phase,
				offsets[i], errors[i]);
		else
			dev_err(core->dev, "PSR %s VIDEOCC offset=%#x value=%#x\n", phase,
				offsets[i], values[i]);
	}
put_runtime:
	/* Drop only our temporary reference, without starting an idle transition. */
	pm_runtime_put_noidle(&pdev->dev);
unlock:
	device_unlock(&pdev->dev);
put_device:
	put_device(&pdev->dev);
}

/* Caller owns core->lock in PM suspend, before any hardware-off transition. */
void iris_psr_log_stall(struct iris_core *core, u32 ctrl, u32 wfi)
{
	static atomic_t taken = ATOMIC_INIT(0);
	struct iris_iface_q_info *queues[] = {
		&core->command_queue, &core->message_queue, &core->debug_queue,
	};
	struct {
		bool present;
		u32 size, rd1, wr1, rd2, wr2;
	} sample[ARRAY_SIZE(queues)] = { 0 };
	u32 irq_status, irq_mask;
	u64 begin, end;
	unsigned int i;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris") ||
	    core->state != IRIS_CORE_INIT || !wfi || (ctrl & BIT(30)))
		return;
	lockdep_assert_held(&core->lock);
	if (atomic_cmpxchg(&taken, 0, 1))
		return;

	begin = ktime_get_ns();
	/* These wrapper registers are already read by interrupt init/clear. */
	irq_status = readl(core->reg_base + WRAPPER_INTR_STATUS);
	irq_mask = readl(core->reg_base + WRAPPER_INTR_MASK);
	/* DMA-coherent queue headers only; never read packets or advance indices. */
	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct iris_hfi_queue_header *h = queues[i]->qhdr;

		if (!h)
			continue;
		sample[i].present = true;
		sample[i].size = READ_ONCE(h->q_size);
		sample[i].rd1 = READ_ONCE(h->read_idx);
		sample[i].wr1 = READ_ONCE(h->write_idx);
		dma_rmb();
		sample[i].rd2 = READ_ONCE(h->read_idx);
		sample[i].wr2 = READ_ONCE(h->write_idx);
	}
	end = ktime_get_ns();
	dev_err(core->dev, "PSR stall begin=%llu sampled=%llu ctrl=%#x wfi=%#x irq=%#x mask=%#x cached_irq=%#x\n",
		begin, end, ctrl, wfi, irq_status, irq_mask, core->intr_status);
	for (i = 0; i < ARRAY_SIZE(queues); i++)
		dev_err(core->dev, "PSR stall queue=%u present=%u words=%u rd1=%u wr1=%u rd2=%u wr2=%u\n",
			i, sample[i].present, sample[i].size, sample[i].rd1,
			sample[i].wr1, sample[i].rd2, sample[i].wr2);
	iris_psr_log_videocc(core, "stall");
	iris_psr_log_noc(core, "stall");
	dev_err(core->dev, "PSR stall end=%llu\n", ktime_get_ns());
}

static int iris_hfi_gen2_handle_system_error(struct iris_core *core,
					     struct iris_hfi_packet *pkt)
{
	struct iris_inst *instance;

	if (pkt)
		dev_err(core->dev, "received system error of type %#x\n", pkt->type);

	core->state = IRIS_CORE_ERROR;

	mutex_lock(&core->lock);
	iris_psr_log_videocc(core, "fatal");
	iris_psr_log_noc(core, "fatal");
	iris_hfi_gen2_log_sfr(core);
	list_for_each_entry(instance, &core->instances, list)
		iris_inst_change_state(instance, IRIS_INST_ERROR);
	mutex_unlock(&core->lock);

	schedule_delayed_work(&core->sys_error_handler, msecs_to_jiffies(10));

	return 0;
}

static int iris_hfi_gen2_handle_system_init(struct iris_core *core,
					    struct iris_hfi_packet *pkt)
{
	if (!(pkt->flags & HFI_FW_FLAGS_SUCCESS)) {
		core->state = IRIS_CORE_ERROR;
		return 0;
	}

	complete(&core->core_init_done);

	return 0;
}

static void iris_hfi_gen2_handle_session_close(struct iris_inst *inst,
					       struct iris_hfi_packet *pkt)
{
	if (!(pkt->flags & HFI_FW_FLAGS_SUCCESS)) {
		iris_inst_change_state(inst, IRIS_INST_ERROR);
		return;
	}

	complete(&inst->completion);
}

static int iris_hfi_gen2_handle_input_buffer(struct iris_inst *inst,
					     struct iris_hfi_buffer *buffer)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_buffer *m2m_buffer, *n;
	struct iris_buffer *buf;
	bool found = false;

	v4l2_m2m_for_each_src_buf_safe(m2m_ctx, m2m_buffer, n) {
		buf = to_iris_buffer(&m2m_buffer->vb);
		if (buf->index == buffer->index) {
			found = true;
			break;
		}
	}
	if (!found)
		return -EINVAL;

	if (!(buf->attr & BUF_ATTR_QUEUED))
		return -EINVAL;

	buf->attr &= ~BUF_ATTR_QUEUED;
	buf->attr |= BUF_ATTR_DEQUEUED;

	buf->flags = iris_hfi_gen2_get_driver_buffer_flags(inst, buffer->flags);

	return 0;
}

static int iris_hfi_gen2_handle_output_buffer(struct iris_inst *inst,
					      struct iris_hfi_buffer *hfi_buffer)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_buffer *m2m_buffer, *n;
	struct iris_buffer *buf;
	bool found = false;
	int ret;

	if (hfi_buffer->flags & HFI_BUF_FW_FLAG_LAST) {
		ret = iris_inst_sub_state_change_drain_last(inst);
		if (ret)
			return ret;
	}

	if (hfi_buffer->flags & HFI_BUF_FW_FLAG_PSC_LAST) {
		ret = iris_inst_sub_state_change_drc_last(inst);
		if (ret)
			return ret;
	}

	v4l2_m2m_for_each_dst_buf_safe(m2m_ctx, m2m_buffer, n) {
		buf = to_iris_buffer(&m2m_buffer->vb);
		if (buf->index == hfi_buffer->index &&
		    buf->device_addr == hfi_buffer->base_address &&
		    buf->data_offset == hfi_buffer->data_offset) {
			found = true;
			break;
		}
	}
	if (!found)
		return -EINVAL;

	if (!(buf->attr & BUF_ATTR_QUEUED))
		return -EINVAL;

	buf->data_offset = hfi_buffer->data_offset;
	buf->data_size = hfi_buffer->data_size;
	buf->timestamp = hfi_buffer->timestamp;

	buf->attr &= ~BUF_ATTR_QUEUED;
	buf->attr |= BUF_ATTR_DEQUEUED;

	buf->flags = iris_hfi_gen2_get_driver_buffer_flags(inst, hfi_buffer->flags);

	if (!buf->data_size && inst->state == IRIS_INST_STREAMING &&
	    !(hfi_buffer->flags & (HFI_BUF_FW_FLAG_LAST | HFI_BUF_FW_FLAG_PSC_LAST))) {
		buf->flags |= V4L2_BUF_FLAG_ERROR;
	}

	return 0;
}

static void iris_psr_hfi_vb2_done(struct iris_inst *inst, struct iris_buffer *buf)
{
	u32 type = buf->type, index = buf->index;
	u32 attr = buf->attr, data_size = buf->data_size;
	int ret;

	ret = iris_vb2_buffer_done(inst, buf);
	iris_psr_hfi_trace_done(inst->core, inst->session_id,
				type, index, attr, data_size, ret);
}

static void iris_hfi_gen2_handle_dequeue_buffers(struct iris_inst *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_buffer *buffer, *n;
	struct iris_buffer *buf = NULL;

	v4l2_m2m_for_each_src_buf_safe(m2m_ctx, buffer, n) {
		buf = to_iris_buffer(&buffer->vb);
		if (buf->attr & BUF_ATTR_DEQUEUED) {
			buf->attr &= ~BUF_ATTR_DEQUEUED;
			if (!(buf->attr & BUF_ATTR_BUFFER_DONE)) {
				buf->attr |= BUF_ATTR_BUFFER_DONE;
				iris_psr_hfi_vb2_done(inst, buf);
			}
		}
	}

	v4l2_m2m_for_each_dst_buf_safe(m2m_ctx, buffer, n) {
		buf = to_iris_buffer(&buffer->vb);
		if (buf->attr & BUF_ATTR_DEQUEUED) {
			buf->attr &= ~BUF_ATTR_DEQUEUED;
			if (!(buf->attr & BUF_ATTR_BUFFER_DONE)) {
				buf->attr |= BUF_ATTR_BUFFER_DONE;
				iris_psr_hfi_vb2_done(inst, buf);
			}
		}
	}
}

static int iris_hfi_gen2_handle_release_internal_buffer(struct iris_inst *inst,
							struct iris_hfi_buffer *buffer)
{
	u32 buf_type = iris_hfi_gen2_buf_type_to_driver(inst, buffer->type);
	struct iris_buffers *buffers = &inst->buffers[buf_type];
	struct iris_buffer *buf, *iter;
	bool found = false;

	list_for_each_entry(iter, &buffers->list, list) {
		if (iter->device_addr == buffer->base_address) {
			found = true;
			buf = iter;
			break;
		}
	}
	if (!found)
		return -EINVAL;

	buf->attr &= ~BUF_ATTR_QUEUED;

	iris_destroy_internal_buffer(inst, buf);

	return 0;
}

static int iris_hfi_gen2_handle_session_stop(struct iris_inst *inst,
					     struct iris_hfi_packet *pkt)
{
	int ret = 0;

	if (pkt->port == HFI_PORT_RAW)
		ret = iris_inst_sub_state_change_pause(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	else if (pkt->port == HFI_PORT_BITSTREAM)
		ret = iris_inst_sub_state_change_pause(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	complete(&inst->completion);

	return ret;
}

static int iris_hfi_gen2_handle_session_buffer(struct iris_inst *inst,
					       struct iris_hfi_packet *pkt)
{
	struct iris_hfi_buffer *buffer;

	if (pkt->payload_info == HFI_PAYLOAD_NONE)
		return 0;

	if (!iris_hfi_gen2_validate_packet_payload(pkt)) {
		iris_inst_change_state(inst, IRIS_INST_ERROR);
		return 0;
	}

	buffer = (struct iris_hfi_buffer *)((u8 *)pkt + sizeof(*pkt));
	if (!iris_hfi_gen2_is_valid_hfi_buffer_type(buffer->type))
		return 0;

	if (!iris_hfi_gen2_is_valid_hfi_port(pkt->port, buffer->type))
		return 0;

	if (inst->domain == DECODER) {
		if (buffer->type == HFI_BUFFER_BITSTREAM)
			return iris_hfi_gen2_handle_input_buffer(inst, buffer);
		else if (buffer->type == HFI_BUFFER_RAW)
			return iris_hfi_gen2_handle_output_buffer(inst, buffer);
		else
			return iris_hfi_gen2_handle_release_internal_buffer(inst, buffer);
	} else {
		if (buffer->type == HFI_BUFFER_RAW)
			return iris_hfi_gen2_handle_input_buffer(inst, buffer);
		else if (buffer->type == HFI_BUFFER_BITSTREAM)
			return iris_hfi_gen2_handle_output_buffer(inst, buffer);
		else
			return iris_hfi_gen2_handle_release_internal_buffer(inst, buffer);
	}
	return 0;
}

static int iris_hfi_gen2_handle_session_drain(struct iris_inst *inst,
					      struct iris_hfi_packet *pkt)
{
	int ret = 0;

	if (!(pkt->flags & HFI_FW_FLAGS_SUCCESS)) {
		iris_inst_change_state(inst, IRIS_INST_ERROR);
		return 0;
	}

	if (inst->sub_state & IRIS_INST_SUB_DRAIN)
		ret = iris_inst_change_sub_state(inst, 0, IRIS_INST_SUB_INPUT_PAUSE);

	return ret;
}

static void iris_hfi_gen2_read_input_subcr_params(struct iris_inst *inst)
{
	struct iris_inst_hfi_gen2 *inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);
	struct v4l2_pix_format_mplane *pixmp_ip = &inst->fmt_src->fmt.pix_mp;
	struct v4l2_pix_format_mplane *pixmp_op = &inst->fmt_dst->fmt.pix_mp;
	u32 primaries, matrix_coeff, transfer_char;
	struct hfi_subscription_params subsc_params;
	u32 colour_description_present_flag;
	u32 video_signal_type_present_flag;
	struct iris_core *core = inst->core;
	u32 full_range, width, height;
	struct vb2_queue *dst_q;
	struct v4l2_ctrl *ctrl;

	subsc_params = inst_hfi_gen2->src_subcr_params;
	width = (subsc_params.bitstream_resolution &
		HFI_BITMASK_BITSTREAM_WIDTH) >> 16;
	height = subsc_params.bitstream_resolution &
		HFI_BITMASK_BITSTREAM_HEIGHT;

	pixmp_ip->width = width;
	pixmp_ip->height = height;

	if (subsc_params.bit_depth == BIT_DEPTH_8 &&
	    pixmp_op->pixelformat != V4L2_PIX_FMT_NV12 &&
	    pixmp_op->pixelformat != V4L2_PIX_FMT_QC08C)
		pixmp_op->pixelformat = V4L2_PIX_FMT_NV12;
	else if (subsc_params.bit_depth == BIT_DEPTH_10 &&
		 pixmp_op->pixelformat != V4L2_PIX_FMT_P010 &&
		 pixmp_op->pixelformat != V4L2_PIX_FMT_QC10C)
		pixmp_op->pixelformat = V4L2_PIX_FMT_P010;

	switch (pixmp_op->pixelformat) {
	case V4L2_PIX_FMT_P010:
		pixmp_op->width = ALIGN(width, 128);
		pixmp_op->height = ALIGN(height, 32);
		pixmp_op->plane_fmt[0].bytesperline = ALIGN(width * 2, 256);
		break;
	case V4L2_PIX_FMT_QC10C:
		pixmp_op->width = roundup(width, 192);
		pixmp_op->height = ALIGN(height, 16);
		pixmp_op->plane_fmt[0].bytesperline = ALIGN(pixmp_op->width * 4 / 3, 256);
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_QC08C:
		pixmp_op->width = ALIGN(width, 128);
		pixmp_op->height = ALIGN(height, 32);
		pixmp_op->plane_fmt[0].bytesperline = pixmp_op->width;
		break;
	}
	pixmp_op->plane_fmt[0].sizeimage = iris_get_buffer_size(inst, BUF_OUTPUT);

	matrix_coeff = subsc_params.color_info & 0xFF;
	transfer_char = (subsc_params.color_info & 0xFF00) >> 8;
	primaries = (subsc_params.color_info & 0xFF0000) >> 16;
	colour_description_present_flag =
		(subsc_params.color_info & 0x1000000) >> 24;
	full_range = (subsc_params.color_info & 0x2000000) >> 25;
	video_signal_type_present_flag =
		(subsc_params.color_info & 0x20000000) >> 29;

	pixmp_op->colorspace = V4L2_COLORSPACE_DEFAULT;
	pixmp_op->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pixmp_op->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pixmp_op->quantization = V4L2_QUANTIZATION_DEFAULT;

	if (video_signal_type_present_flag) {
		pixmp_op->quantization =
			full_range ?
			V4L2_QUANTIZATION_FULL_RANGE :
			V4L2_QUANTIZATION_LIM_RANGE;
		if (colour_description_present_flag) {
			pixmp_op->colorspace =
				iris_hfi_get_v4l2_color_primaries(primaries);
			pixmp_op->xfer_func =
				iris_hfi_get_v4l2_transfer_char(transfer_char);
			pixmp_op->ycbcr_enc =
				iris_hfi_get_v4l2_matrix_coefficients(matrix_coeff);
		}
	}

	pixmp_ip->colorspace = pixmp_op->colorspace;
	pixmp_ip->xfer_func = pixmp_op->xfer_func;
	pixmp_ip->ycbcr_enc = pixmp_op->ycbcr_enc;
	pixmp_ip->quantization = pixmp_op->quantization;

	inst->crop.top = subsc_params.crop_offsets[0] & 0xFFFF;
	inst->crop.left = (subsc_params.crop_offsets[0] >> 16) & 0xFFFF;
	inst->crop.height = pixmp_ip->height -
		(subsc_params.crop_offsets[1] & 0xFFFF) - inst->crop.top;
	inst->crop.width = pixmp_ip->width -
		((subsc_params.crop_offsets[1] >> 16) & 0xFFFF) - inst->crop.left;

	switch (inst->codec) {
	case V4L2_PIX_FMT_HEVC:
		inst->fw_caps[PROFILE_HEVC].value = subsc_params.profile;
		inst->fw_caps[LEVEL_HEVC].value = subsc_params.level;
		break;
	case V4L2_PIX_FMT_VP9:
		inst->fw_caps[PROFILE_VP9].value = subsc_params.profile;
		inst->fw_caps[LEVEL_VP9].value = subsc_params.level;
		break;
	case V4L2_PIX_FMT_H264:
		inst->fw_caps[PROFILE_H264].value = subsc_params.profile;
		inst->fw_caps[LEVEL_H264].value = subsc_params.level;
		break;
	case V4L2_PIX_FMT_AV1:
		inst->fw_caps[PROFILE_AV1].value = subsc_params.profile;
		inst->fw_caps[LEVEL_AV1].value = subsc_params.level;
		break;
	}

	inst->fw_caps[POC].value = subsc_params.pic_order_cnt;
	inst->fw_caps[TIER].value = subsc_params.tier;

	if (subsc_params.bit_depth == BIT_DEPTH_8)
		inst->fw_caps[BIT_DEPTH].value = BIT_DEPTH_8;
	else
		inst->fw_caps[BIT_DEPTH].value = BIT_DEPTH_10;

	if ((subsc_params.bit_depth != BIT_DEPTH_8 && subsc_params.bit_depth != BIT_DEPTH_10) ||
	    !(subsc_params.coded_frames & HFI_BITMASK_FRAME_MBS_ONLY_FLAG)) {
		dev_err(core->dev, "unsupported content, bit depth: %x, pic_struct = %x\n",
			subsc_params.bit_depth, subsc_params.coded_frames);
		iris_inst_change_state(inst, IRIS_INST_ERROR);
	}

	if (inst->codec == V4L2_PIX_FMT_AV1) {
		inst->fw_caps[FILM_GRAIN].value = subsc_params.film_grain;
		inst->fw_caps[SUPER_BLOCK].value = subsc_params.super_block;
	}

	inst->fw_min_count = subsc_params.fw_min_count;
	inst->buffers[BUF_OUTPUT].min_count = iris_vpu_buf_count(inst, BUF_OUTPUT);
	inst->buffers[BUF_OUTPUT].size = pixmp_op->plane_fmt[0].sizeimage;
	ctrl = v4l2_ctrl_find(&inst->ctrl_handler, V4L2_CID_MIN_BUFFERS_FOR_CAPTURE);
	if (ctrl)
		v4l2_ctrl_s_ctrl(ctrl, inst->buffers[BUF_OUTPUT].min_count);

	dst_q = v4l2_m2m_get_dst_vq(inst->m2m_ctx);
	dst_q->min_reqbufs_allocation = inst->buffers[BUF_OUTPUT].min_count;
}

static int iris_hfi_gen2_handle_src_change(struct iris_inst *inst,
					   struct iris_hfi_packet *pkt)
{
	int ret;

	if (pkt->port != HFI_PORT_BITSTREAM)
		return 0;

	ret = iris_inst_sub_state_change_drc(inst);
	if (ret)
		return ret;

	iris_hfi_gen2_read_input_subcr_params(inst);
	iris_vdec_src_change(inst);

	return 0;
}

static int iris_hfi_gen2_handle_session_command(struct iris_inst *inst,
						struct iris_hfi_packet *pkt)
{
	int ret = 0;

	switch (pkt->type) {
	case HFI_CMD_CLOSE:
		iris_hfi_gen2_handle_session_close(inst, pkt);
		break;
	case HFI_CMD_STOP:
		iris_hfi_gen2_handle_session_stop(inst, pkt);
		break;
	case HFI_CMD_BUFFER:
		ret = iris_hfi_gen2_handle_session_buffer(inst, pkt);
		break;
	case HFI_CMD_SETTINGS_CHANGE:
		ret = iris_hfi_gen2_handle_src_change(inst, pkt);
		break;
	case HFI_CMD_DRAIN:
		ret = iris_hfi_gen2_handle_session_drain(inst, pkt);
		break;
	default:
		break;
	}

	return ret;
}

static int iris_hfi_gen2_handle_session_property(struct iris_inst *inst,
						 struct iris_hfi_packet *pkt)
{
	struct iris_inst_hfi_gen2 *inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);

	if (pkt->flags & HFI_FW_FLAGS_INFORMATION)
		return 0;

	switch (pkt->type) {
	case HFI_PROP_BITSTREAM_RESOLUTION:
		inst_hfi_gen2->src_subcr_params.bitstream_resolution = pkt->payload[0];
		break;
	case HFI_PROP_CROP_OFFSETS:
		inst_hfi_gen2->src_subcr_params.crop_offsets[0] = pkt->payload[0];
		inst_hfi_gen2->src_subcr_params.crop_offsets[1] = pkt->payload[1];
		break;
	case HFI_PROP_LUMA_CHROMA_BIT_DEPTH:
		inst_hfi_gen2->src_subcr_params.bit_depth = pkt->payload[0];
		break;
	case HFI_PROP_CODED_FRAMES:
		inst_hfi_gen2->src_subcr_params.coded_frames = pkt->payload[0];
		break;
	case HFI_PROP_BUFFER_FW_MIN_OUTPUT_COUNT:
		inst_hfi_gen2->src_subcr_params.fw_min_count = pkt->payload[0];
		break;
	case HFI_PROP_PIC_ORDER_CNT_TYPE:
		inst_hfi_gen2->src_subcr_params.pic_order_cnt = pkt->payload[0];
		break;
	case HFI_PROP_SIGNAL_COLOR_INFO:
		inst_hfi_gen2->src_subcr_params.color_info = pkt->payload[0];
		break;
	case HFI_PROP_PROFILE:
		inst_hfi_gen2->src_subcr_params.profile = pkt->payload[0];
		break;
	case HFI_PROP_LEVEL:
		inst_hfi_gen2->src_subcr_params.level = pkt->payload[0];
		break;
	case HFI_PROP_TIER:
		inst_hfi_gen2->src_subcr_params.tier = pkt->payload[0];
		break;
	case HFI_PROP_PICTURE_TYPE:
		inst_hfi_gen2->hfi_frame_info.picture_type = pkt->payload[0];
		break;
	case HFI_PROP_NO_OUTPUT:
		inst_hfi_gen2->hfi_frame_info.no_output = 1;
		break;
	case HFI_PROP_AV1_FILM_GRAIN_PRESENT:
		inst_hfi_gen2->src_subcr_params.film_grain = pkt->payload[0];
		break;
	case HFI_PROP_AV1_SUPER_BLOCK_ENABLED:
		inst_hfi_gen2->src_subcr_params.super_block = pkt->payload[0];
		break;
	case HFI_PROP_QUALITY_MODE:
	case HFI_PROP_STAGE:
	case HFI_PROP_PIPE:
	default:
		break;
	}

	return 0;
}

static int iris_hfi_gen2_handle_image_version_property(struct iris_core *core,
						       struct iris_hfi_packet *pkt)
{
	u8 *str_image_version = (u8 *)pkt + sizeof(*pkt);
	u32 req_bytes = pkt->size - sizeof(*pkt);
	char fw_version[IRIS_FW_VERSION_LENGTH];
	u32 i;

	if (req_bytes < IRIS_FW_VERSION_LENGTH - 1)
		return -EINVAL;

	for (i = 0; i < IRIS_FW_VERSION_LENGTH - 1; i++) {
		if (str_image_version[i] != '\0')
			fw_version[i] = str_image_version[i];
		else
			fw_version[i] = ' ';
	}
	fw_version[i] = '\0';
	dev_dbg(core->dev, "firmware version: %s\n", fw_version);

	return 0;
}

static int iris_hfi_gen2_handle_system_property(struct iris_core *core,
						struct iris_hfi_packet *pkt)
{
	switch (pkt->type) {
	case HFI_PROP_IMAGE_VERSION:
		return iris_hfi_gen2_handle_image_version_property(core, pkt);
	default:
		return 0;
	}
}

static int iris_hfi_gen2_handle_system_response(struct iris_core *core,
						struct iris_hfi_header *hdr)
{
	u8 *start_pkt = (u8 *)((u8 *)hdr + sizeof(*hdr));
	struct iris_hfi_packet *packet;
	u32 i, j;
	u8 *pkt;
	int ret;
	static const struct iris_hfi_gen2_core_hfi_range range[] = {
		{HFI_SYSTEM_ERROR_BEGIN, HFI_SYSTEM_ERROR_END, iris_hfi_gen2_handle_system_error },
		{HFI_PROP_BEGIN,         HFI_PROP_END, iris_hfi_gen2_handle_system_property },
		{HFI_CMD_BEGIN,          HFI_CMD_END, iris_hfi_gen2_handle_system_init },
	};

	for (i = 0; i < ARRAY_SIZE(range); i++) {
		pkt = start_pkt;
		for (j = 0; j < hdr->num_packets; j++) {
			packet = (struct iris_hfi_packet *)pkt;
			if (packet->flags & HFI_FW_FLAGS_SYSTEM_ERROR) {
				ret = iris_hfi_gen2_handle_system_error(core, packet);
				return ret;
			}

			if (packet->type > range[i].begin && packet->type < range[i].end) {
				ret = range[i].handle(core, packet);
				if (ret)
					return ret;

				if (packet->type >  HFI_SYSTEM_ERROR_BEGIN &&
				    packet->type < HFI_SYSTEM_ERROR_END)
					return 0;
			}
			pkt += packet->size;
		}
	}

	return 0;
}

static void iris_hfi_gen2_init_src_change_param(struct iris_inst *inst)
{
	struct iris_inst_hfi_gen2 *inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);
	struct v4l2_pix_format_mplane *pixmp_ip = &inst->fmt_src->fmt.pix_mp;
	struct v4l2_pix_format_mplane *pixmp_op = &inst->fmt_dst->fmt.pix_mp;
	u32 bottom_offset = (pixmp_ip->height - inst->crop.height);
	u32 right_offset = (pixmp_ip->width - inst->crop.width);
	struct hfi_subscription_params *subsc_params;
	u32 primaries, matrix_coeff, transfer_char;
	u32 colour_description_present_flag = 0;
	u32 video_signal_type_present_flag = 0;
	u32 full_range, video_format = 0;
	u32 left_offset = inst->crop.left;
	u32 top_offset = inst->crop.top;

	subsc_params = &inst_hfi_gen2->src_subcr_params;
	subsc_params->bitstream_resolution =
		pixmp_ip->width << 16 | pixmp_ip->height;
	subsc_params->crop_offsets[0] =
			left_offset << 16 | top_offset;
	subsc_params->crop_offsets[1] =
			right_offset << 16 | bottom_offset;
	subsc_params->fw_min_count = inst->buffers[BUF_OUTPUT].min_count;

	primaries = iris_hfi_gen2_get_color_primaries(pixmp_op->colorspace);
	matrix_coeff = iris_hfi_gen2_get_matrix_coefficients(pixmp_op->ycbcr_enc);
	transfer_char = iris_hfi_gen2_get_transfer_char(pixmp_op->xfer_func);
	full_range = pixmp_op->quantization == V4L2_QUANTIZATION_FULL_RANGE ? 1 : 0;
	subsc_params->color_info =
		iris_hfi_gen2_get_color_info(matrix_coeff, transfer_char, primaries,
					     colour_description_present_flag,
					     full_range, video_format,
					     video_signal_type_present_flag);

	switch (inst->codec) {
	case V4L2_PIX_FMT_HEVC:
		subsc_params->profile = inst->fw_caps[PROFILE_HEVC].value;
		subsc_params->level = inst->fw_caps[LEVEL_HEVC].value;
		break;
	case V4L2_PIX_FMT_VP9:
		subsc_params->profile = inst->fw_caps[PROFILE_VP9].value;
		subsc_params->level = inst->fw_caps[LEVEL_VP9].value;
		break;
	case V4L2_PIX_FMT_H264:
		subsc_params->profile = inst->fw_caps[PROFILE_H264].value;
		subsc_params->level = inst->fw_caps[LEVEL_H264].value;
		break;
	case V4L2_PIX_FMT_AV1:
		subsc_params->profile = inst->fw_caps[PROFILE_AV1].value;
		subsc_params->level = inst->fw_caps[LEVEL_AV1].value;
		break;
	}

	subsc_params->pic_order_cnt = inst->fw_caps[POC].value;
	subsc_params->bit_depth = inst->fw_caps[BIT_DEPTH].value;
	if (inst->fw_caps[CODED_FRAMES].value ==
			CODED_FRAMES_PROGRESSIVE)
		subsc_params->coded_frames = HFI_BITMASK_FRAME_MBS_ONLY_FLAG;
	else
		subsc_params->coded_frames = 0;
}

static int iris_hfi_gen2_handle_session_response(struct iris_core *core,
						 struct iris_hfi_header *hdr)
{
	u8 *pkt = (u8 *)((u8 *)hdr + sizeof(*hdr));
	struct iris_inst_hfi_gen2 *inst_hfi_gen2;
	struct iris_hfi_packet *packet;
	struct iris_inst *inst;
	bool dequeue = false;
	int ret = 0;
	u32 i, j;
	static const struct iris_hfi_gen2_inst_hfi_range range[] = {
		{HFI_SESSION_ERROR_BEGIN, HFI_SESSION_ERROR_END,
		 iris_hfi_gen2_handle_session_error},
		{HFI_INFORMATION_BEGIN, HFI_INFORMATION_END,
		 iris_hfi_gen2_handle_session_info},
		{HFI_PROP_BEGIN, HFI_PROP_END,
		 iris_hfi_gen2_handle_session_property},
		{HFI_CMD_BEGIN, HFI_CMD_END,
		 iris_hfi_gen2_handle_session_command },
	};

	inst = iris_get_instance(core, hdr->session_id);
	if (!inst)
		return -EINVAL;

	mutex_lock(&inst->lock);
	inst_hfi_gen2 = to_iris_inst_hfi_gen2(inst);
	memset(&inst_hfi_gen2->hfi_frame_info, 0, sizeof(struct iris_hfi_frame_info));

	for (i = 0; i < hdr->num_packets; i++) {
		packet = (struct iris_hfi_packet *)pkt;
		if (packet->type == HFI_CMD_SETTINGS_CHANGE) {
			if (packet->port == HFI_PORT_BITSTREAM) {
				iris_hfi_gen2_init_src_change_param(inst);
				break;
			}
		}
		pkt += packet->size;
	}

	pkt = (u8 *)((u8 *)hdr + sizeof(*hdr));
	for (i = 0; i < ARRAY_SIZE(range); i++) {
		pkt = (u8 *)((u8 *)hdr + sizeof(*hdr));
		for (j = 0; j < hdr->num_packets; j++) {
			packet = (struct iris_hfi_packet *)pkt;
			if (packet->flags & HFI_FW_FLAGS_SESSION_ERROR)
				iris_hfi_gen2_handle_session_error(inst, packet);

			if (packet->type > range[i].begin && packet->type < range[i].end) {
				dequeue |= (packet->type == HFI_CMD_BUFFER);
				ret = range[i].handle(inst, packet);
				iris_psr_hfi_trace_packet(core, "dispatch", hdr->session_id,
							  hdr->header_id, packet, packet->size,
							  ret, inst->state);
				if (ret)
					iris_inst_change_state(inst, IRIS_INST_ERROR);
			}
			pkt += packet->size;
		}
	}

	if (dequeue)
		iris_hfi_gen2_handle_dequeue_buffers(inst);

	mutex_unlock(&inst->lock);

	return ret;
}

static int iris_hfi_gen2_handle_response(struct iris_core *core, void *response)
{
	struct iris_hfi_header *hdr = (struct iris_hfi_header *)response;
	int ret;

	ret = iris_hfi_gen2_validate_hdr_packet(core, hdr);
	if (ret)
		return iris_hfi_gen2_handle_system_error(core, NULL);

	iris_psr_hfi_trace_header(core, "receive", hdr, hdr->size, 0);

	if (!hdr->session_id)
		return iris_hfi_gen2_handle_system_response(core, hdr);
	else
		return iris_hfi_gen2_handle_session_response(core, hdr);
}

static void iris_hfi_gen2_flush_debug_queue(struct iris_core *core, u8 *packet)
{
	struct hfi_debug_header *pkt;
	u8 *log;

	while (!iris_hfi_queue_dbg_read(core, packet)) {
		pkt = (struct hfi_debug_header *)packet;

		if (pkt->size < sizeof(*pkt))
			continue;

		if (pkt->size >= IFACEQ_CORE_PKT_SIZE)
			continue;

		packet[pkt->size] = '\0';
		log = (u8 *)packet + sizeof(*pkt) + 1;
		if (pkt->debug_level & HFI_DEBUG_LOG_ERROR_FATAL)
			dev_warn(core->dev, "firmware: %s", log);
		else
			dev_dbg(core->dev, "%s", log);
	}
}

void iris_hfi_gen2_response_handler(struct iris_core *core)
{
	if (iris_vpu_watchdog(core, core->intr_status)) {
		struct iris_hfi_packet pkt = {.type = HFI_SYS_ERROR_WD_TIMEOUT};

		dev_err(core->dev, "cpu watchdog error received\n");
		core->state = IRIS_CORE_ERROR;
		iris_hfi_gen2_handle_system_error(core, &pkt);

		return;
	}

	memset(core->response_packet, 0, sizeof(struct iris_hfi_header));
	while (!iris_hfi_queue_msg_read(core, core->response_packet)) {
		iris_hfi_gen2_handle_response(core, core->response_packet);
		memset(core->response_packet, 0, sizeof(struct iris_hfi_header));
	}

	iris_hfi_gen2_flush_debug_queue(core, core->response_packet);
}
