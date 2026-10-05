/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PSR_CAMERA_LEASE_H
#define _PSR_CAMERA_LEASE_H
struct mux_control;
int psr_camera_try_select(struct mux_control *mux, unsigned int state);
void psr_camera_lease_fault(struct mux_control *mux);
#endif
