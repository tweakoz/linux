/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PHY_MIPI_CPHY_H_
#define __PHY_MIPI_CPHY_H_
#include <linux/types.h>
/* Experimental PSR contract, not an upstream generic timing ABI.
 * symbol_rate: symbols/s/trio (no DDR factor).
 * data_rate: equivalent bits/s/trio from captured CamX output clock.
 * settle_count: explicit Qualcomm receiver count, NOT nanoseconds.
 */
struct phy_configure_opts_mipi_cphy {
	u64 symbol_rate;
	u64 data_rate;
	u32 lanes;
	u32 settle_count;
};
#endif
