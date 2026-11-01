/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * S2 PCIe link, PLL and DDR controller bring-up, from the Linux facetimehd
 * driver:
 *   Copyright (C) 2014 Patrik Jakobsson (patrik.r.jakobsson@gmail.com)
 *
 * The camera has its own DDR behind BAR 2.  Before the ISP firmware can be
 * loaded into it, the PLLs and the DDR40 PHY must be brought up by hand.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>

#include <dev/pci/pcivar.h>

#include "fthd_var.h"

#define S2_READ(off)		fthd_s2_reg_read(sc, (off))
#define S2_WRITE(val, off)	fthd_s2_reg_write(sc, (val), (off))

static void
fthd_hw_s2_pll_reset(struct fthd_softc *sc)
{
	S2_WRITE(0x40, S2_PLL_CTRL_2C);
	S2_WRITE(0x0, S2_PLL_CTRL_2C);
	S2_WRITE(0xbcbc1500, S2_PLL_CTRL_100);
	S2_WRITE(0x0, S2_PLL_CTRL_14);

	DELAY(10000);

	S2_WRITE(0x3, S2_PLL_CTRL_14);

	device_printf(sc->dev, "PLL reset finished\n");
}

static int
fthd_hw_s2_init_pcie_link(struct fthd_softc *sc)
{
	uint32_t reg;

	reg = S2_READ(S2_PCIE_LINK_D000);
	S2_WRITE(reg | 0x10, S2_PCIE_LINK_D000);

	S2_WRITE(0x1804, S2_PCIE_LINK_D120);
	S2_WRITE(0xac5800, S2_PCIE_LINK_D124);
	S2_WRITE(0x1804, S2_PCIE_LINK_D120);

	/* Check if PLL is powered down when S2 PCIe link is in L1 state */
	reg = S2_READ(S2_PCIE_LINK_D124);
	if (reg != 0xac5800) {
		device_printf(sc->dev, "Failed to init S2 PCIe link: %08x\n",
		    reg);
		return (EIO);
	}

	device_printf(sc->dev, "S2 PCIe link init succeeded\n");

	S2_WRITE(0x1f08, S2_PCIE_LINK_D128);
	S2_WRITE(0x80008610, S2_PCIE_LINK_D12C);
	S2_WRITE(0x1608, S2_PCIE_LINK_D128);
	S2_WRITE(0x8000fc00, S2_PCIE_LINK_D12C);
	S2_WRITE(0x1f08, S2_PCIE_LINK_D128);
	S2_WRITE(0x80008610, S2_PCIE_LINK_D12C);
	S2_WRITE(0x1708, S2_PCIE_LINK_D128);
	S2_WRITE(0x800005bf, S2_PCIE_LINK_D12C);

	return (0);
}

static int
fthd_hw_s2_pll_init(struct fthd_softc *sc, uint32_t ddr_speed)
{
	uint32_t reg;
	bool ref_clk_25;
	int retries = 0;

	reg = S2_READ(S2_PLL_REFCLK);
	ref_clk_25 = (reg & S2_PLL_REFCLK_25MHZ) != 0;
	device_printf(sc->dev, "Refclk: %dMHz (0x%x)\n",
	    ref_clk_25 ? 25 : 24, reg);

	switch (ddr_speed) {
	case 400:
		if (ref_clk_25) {
			S2_WRITE(0x00400078, S2_PLL_CTRL_510);
			S2_WRITE(0x19280804, S2_PLL_CTRL_24);
		} else {
			S2_WRITE(0x03200000, S2_PLL_CTRL_20);
			S2_WRITE(0x14280603, S2_PLL_CTRL_24);
		}
		break;
	case 300:
		if (ref_clk_25) {
			S2_WRITE(0x00480078, S2_PLL_CTRL_510);
			S2_WRITE(0x19280c06, S2_PLL_CTRL_24);
		} else {
			S2_WRITE(0x03200000, S2_PLL_CTRL_20);
			S2_WRITE(0x14280804, S2_PLL_CTRL_24);
		}
		break;
	case 200:
		if (ref_clk_25) {
			S2_WRITE(0x00400078, S2_PLL_CTRL_510);
			S2_WRITE(0x19281008, S2_PLL_CTRL_24);
		} else {
			S2_WRITE(0x03200000, S2_PLL_CTRL_20);
			S2_WRITE(0x14280c06, S2_PLL_CTRL_24);
		}
		break;
	default:
		if (ddr_speed != 450) {
			device_printf(sc->dev,
			    "Unsupported DDR speed %uMHz, using 450MHz\n",
			    ddr_speed);
			ddr_speed = 450;
		}
		if (ref_clk_25) {
			S2_WRITE(0x0048007d, S2_PLL_CTRL_510);
			S2_WRITE(0x19280904, S2_PLL_CTRL_24);
		} else {
			S2_WRITE(0x04b00000, S2_PLL_CTRL_20);
			S2_WRITE(0x14280904, S2_PLL_CTRL_24);
		}
		break;
	}

	fthd_hw_s2_pll_reset(sc);

	device_printf(sc->dev, "Waiting for S2 PLL to lock at %u MHz\n",
	    ddr_speed);

	do {
		reg = S2_READ(S2_PLL_CMU_STATUS);
		DELAY(10);
		retries++;
	} while (!(reg & S2_PLL_CMU_STATUS_LOCKED) && retries <= 10000);

	if (retries > 10000) {
		device_printf(sc->dev, "Failed to lock S2 PLL: 0x%x\n", reg);
		return (EIO);
	}
	device_printf(sc->dev, "S2 PLL is locked after %d us\n", retries * 10);

	reg = S2_READ(S2_PLL_STATUS_A8);
	S2_WRITE(reg | S2_PLL_BYPASS, S2_PLL_STATUS_A8);
	DELAY(10000);

	reg = S2_READ(S2_PLL_STATUS_A8);
	device_printf(sc->dev, "S2 PLL is in %sbypass mode\n",
	    (reg & S2_PLL_BYPASS) ? "" : "non-");

	return (0);
}

static void
fthd_hw_s2_preinit_ddr_controller_soc(struct fthd_softc *sc)
{
	uint32_t off;

	for (off = S2_DDR_REG_1100; off <= S2_DDR_REG_111C; off += 4)
		S2_WRITE(0x203, off);
}

static void
fthd_hw_ddr_phy_soft_reset(struct fthd_softc *sc)
{
	S2_WRITE(0x281, S2_PLL_STATUS_A8);
	S2_WRITE(0xfffff, S2_PLL_CTRL_9C);
	DELAY(10000);
	S2_WRITE(0xffbff, S2_PLL_CTRL_9C);
}

static int
fthd_hw_ddr_status_busy(struct fthd_softc *sc, int retries, int delay)
{
	uint32_t reg = 0;
	int i;

	for (i = 0; i < retries; i++) {
		reg = S2_READ(S2_DDR_STATUS_2018);
		if (!(reg & S2_DDR_STATUS_BUSY))
			return (0);
		if (delay > 0)
			DELAY(delay);
	}

	device_printf(sc->dev,
	    "S2_DDR_STATUS_2018 busy: retries=%d, delay=%d, reg=0x%08x\n",
	    retries, delay, reg);
	return (EBUSY);
}

static int
fthd_hw_ddr_rewrite_mode_regs(struct fthd_softc *sc)
{
	static const uint32_t cmds[] = { 0x02000802, 0x3, 0x1 };
	int error, i;

	for (i = 0; i < nitems(cmds); i++) {
		S2_WRITE(cmds[i], S2_DDR_2014);
		error = fthd_hw_ddr_status_busy(sc, 500, 5);
		if (error)
			return (error);
	}

	S2_WRITE(sc->ddr_speed == 450 ? 0x17003000 : 0x17002000, S2_DDR_2014);
	error = fthd_hw_ddr_status_busy(sc, 500, 5);
	if (error)
		return (error);

	device_printf(sc->dev, "Rewrite DDR mode registers succeeded\n");
	return (0);
}

/* Poll an S2 register until any bit in mask is set */
static int
fthd_hw_poll_set(struct fthd_softc *sc, uint32_t off, uint32_t mask,
    int retries, int delay, uint32_t *regp)
{
	uint32_t reg = 0;
	int i;

	for (i = 0; i <= retries; i++) {
		reg = S2_READ(off);
		if (reg & mask)
			break;
		DELAY(delay);
	}
	if (regp != NULL)
		*regp = reg;
	return (i > retries ? -1 : i);
}

static int
fthd_hw_s2_init_ddr_controller_soc(struct fthd_softc *sc)
{
	uint32_t cmd, val, reg;
	uint32_t step_size, vdl_fine, vdl_coarse;
	int error, i;

	cmd = pci_read_config(sc->dev, PCIR_COMMAND, 4);
	if ((cmd & 0x07) == 0) {
		device_printf(sc->dev,
		    "PCI link in illegal state, cfg_cmd_reg: 0x%x\n", cmd);
		return (EIO);
	}

	reg = S2_READ(S2_PLL_CTRL_9C);
	S2_WRITE(reg & 0xfffffcff, S2_PLL_CTRL_9C);
	S2_WRITE(reg | 0x300, S2_PLL_CTRL_9C);

	error = fthd_hw_s2_pll_init(sc, sc->ddr_speed);
	if (error)
		return (error);

	fthd_hw_ddr_phy_soft_reset(sc);

	S2_WRITE(0x2, S2_DDR40_WL_DRV_PAD_CTL);
	S2_WRITE(0x2, S2_DDR40_WL_CLK_PAD_DISABLE);

	/* Disable the hardware frequency change function */
	S2_WRITE(0x3f4, S2_20F8);

	/* Setup the PLL */
	S2_WRITE(0x40, S2_2434);
	S2_WRITE(0x10000000, S2_2438);
	S2_WRITE(0x4, S2_2424);
	S2_WRITE(0x1f37291, S2_2430);

	if (fthd_hw_poll_set(sc, S2_DDR_PLL_STATUS_2444,
	    S2_DDR_PLL_STATUS_2444_LOCKED, 10000, 10, NULL) < 0) {
		device_printf(sc->dev, "Failed to lock DDR PHY PLL in stage 1\n");
		return (EIO);
	}

	S2_WRITE(0x1f37205, S2_2430);

	if (fthd_hw_poll_set(sc, S2_DDR_PLL_STATUS_241C,
	    S2_DDR_PLL_STATUS_241C_LOCKED, 10000, 10, NULL) < 0) {
		device_printf(sc->dev, "Failed to lock DDR PHY PLL in stage 2\n");
		return (EIO);
	}

	DELAY(10000);

	/* WL */
	S2_WRITE(0x0c10, S2_DDR40_PHY_PLL_DIV);
	S2_WRITE(0x0010, S2_DDR40_PHY_PLL_CFG);

	if (fthd_hw_poll_set(sc, S2_DDR40_PHY_PLL_STATUS,
	    S2_DDR40_PHY_PLL_STATUS_LOCKED, 10000, 10, NULL) < 0) {
		device_printf(sc->dev, "Failed to lock DDR PHY PLL in stage 3\n");
		return (EIO);
	}

	device_printf(sc->dev, "DDR40 PHY PLL locked on safe settings\n");

	switch (sc->ddr_model) {
	case 4:
		val = 0x46a00c2;
		break;
	case 2:
		val = 0x42500c2;
		break;
	default:
		val = 0;
	}

	S2_WRITE(0x10737545, S2_DDR_20A0);
	S2_WRITE(0x12643173, S2_DDR_20A4);
	S2_WRITE(0xff3f, S2_DDR_20A8);
	S2_WRITE(val, S2_DDR_20B0);
	S2_WRITE(0x101f, S2_DDR_2118);
	S2_WRITE(0x1c0, S2_DDR40_PHY_AUX_CTL);

	S2_WRITE(sc->ddr_model == 2 ? 0x2155558 : 0x2159518,
	    S2_DDR40_STRAP_CTL);
	S2_WRITE(sc->ddr_speed == 450 ? 0x108307 : 0x108286,
	    S2_DDR40_STRAP_CTL_2);

	/* Strap control */
	S2_WRITE(0x2159559, S2_DDR40_STRAP_CTL);

	if (fthd_hw_poll_set(sc, S2_DDR40_STRAP_STATUS, 0x1, 9999, 10,
	    NULL) < 0) {
		device_printf(sc->dev, "Timeout waiting for STRAP valid\n");
		return (ENODEV);
	}
	device_printf(sc->dev, "STRAP valid\n");

	/* Manual DDR40 PHY init */
	device_printf(sc->dev, "Configuring DDR PLLs for %u MHz\n",
	    sc->ddr_speed);

	val = (sc->ddr_speed * 2 < 500) ? 0x2040 : 0x810;

	reg = S2_READ(S2_DDR40_PHY_PLL_DIV);
	S2_WRITE(val | (reg & 0xffffc700), S2_DDR40_PHY_PLL_DIV);

	reg = S2_READ(S2_DDR40_PHY_PLL_CFG);
	S2_WRITE(reg & 0xfffffffd, S2_DDR40_PHY_PLL_CFG);

	i = fthd_hw_poll_set(sc, S2_DDR40_PHY_PLL_STATUS,
	    S2_DDR40_PHY_PLL_STATUS_LOCKED, 99, 1, NULL);
	if (i < 0) {
		device_printf(sc->dev, "Failed to lock the DDR PLL\n");
		return (ENODEV);
	}
	device_printf(sc->dev, "DDR40 PLL is locked after %d us\n", i);

	/* Configure DDR40 VDL */
	S2_WRITE(0, S2_DDR40_PHY_VDL_CTL);
	S2_WRITE(0x103, S2_DDR40_PHY_VDL_CTL);

	i = fthd_hw_poll_set(sc, S2_DDR40_PHY_VDL_STATUS, 0x1, 99, 1, &reg);
	if (i >= 0) {
		device_printf(sc->dev,
		    "First DDR40 VDL calibration completed after %d us%s\n",
		    i, (reg & 0x2) ? "" : " (but failed to lock)");
	} else {
		device_printf(sc->dev, "First DDR40 VDL calibration failed\n");
	}

	S2_WRITE(0, S2_DDR40_PHY_VDL_CTL);
	S2_WRITE(0, S2_DDR40_PHY_VDL_CTL);
	S2_WRITE(0x200, S2_DDR40_PHY_VDL_CTL);	/* calib steps */

	i = fthd_hw_poll_set(sc, S2_DDR40_PHY_VDL_STATUS, 0x1, 999, 1, &reg);
	device_printf(sc->dev,
	    "Second DDR40 VDL calibration completed after %d us\n", i);

	if (reg & 0x2) {
		step_size = (reg & S2_DDR40_PHY_VDL_STEP_MASK) >>
		    S2_DDR40_PHY_VDL_STEP_SHIFT;
		device_printf(sc->dev, "Using step size %u\n", step_size);
	} else {
		val = 1000000 / sc->ddr_speed;
		/* 32-bit multiply (and overflow) as in the Linux driver */
		step_size = (val * 0x4ec4ec4fU) >> 22;
		device_printf(sc->dev, "Using default step size (%u)\n",
		    step_size);
	}
	sc->vdl_step_size = step_size;

	vdl_fine = S2_READ(S2_DDR40_PHY_VDL_CHAN_STATUS);

	/* lock = 1 and byte_sel = 1 */
	if ((vdl_fine & 2) == 0) {
		vdl_fine = ((vdl_fine >> 8) & 0x3f) | 0x10100;
		S2_WRITE(vdl_fine, S2_DDR40_PHY_VDL_OVR_FINE);

		vdl_coarse = 0x10000;
		step_size >>= 4;
		step_size += step_size * 2;
		if (step_size > 10) {
			step_size = (step_size + 118) >> 1;
			vdl_coarse = (step_size & 0x3f) | 0x10000;
		}
		S2_WRITE(vdl_coarse, S2_DDR40_PHY_VDL_OVR_COARSE);

		device_printf(sc->dev, "VDL set to: coarse=0x%x, fine=0x%x\n",
		    vdl_coarse, vdl_fine);
	}

	/* Configure Virtual VTT connections and override */
	S2_WRITE(0x1cf7fff, S2_DDR40_PHY_VTT_CONNECTIONS);
	S2_WRITE(0x77fff, S2_DDR40_PHY_VTT_OVERRIDE);
	S2_WRITE(0x4, S2_DDR40_PHY_VTT_CTL);
	device_printf(sc->dev, "Virtual VTT enabled\n");

	/* Process, Voltage and Temperature compensation */
	S2_WRITE(0xc0fff, S2_DDR40_PHY_ZQ_PVT_COMP_CTL);
	S2_WRITE(0x2, S2_DDR40_PHY_DRV_PAD_CTL);
	S2_WRITE(0x2, S2_DDR40_WL_DRV_PAD_CTL);

	val = 1000000 / sc->ddr_speed;
	reg = 4;
	if (val >= 400) {
		if (val > 900)
			reg = 1;
		reg += 5;
	}

	S2_WRITE(reg, S2_DDR40_WL_RD_DATA_DLY);
	S2_WRITE(0x2, S2_DDR40_WL_READ_CTL);		/* le_adj, te_adj */
	S2_WRITE(0x3, S2_DDR40_WL_WR_PREAMBLE_MODE);	/* mode, long */

	/* dq_oeb, dq_reb, dq_iddq, dq_rxenb */
	reg = S2_READ(S2_DDR40_WL_IDLE_PAD_CTL);
	S2_WRITE(reg & 0xff0fffff, S2_DDR40_WL_IDLE_PAD_CTL);
	DELAY(500);

	S2_WRITE(0, S2_DDR_2004);
	DELAY(10000);

	S2_WRITE(0xab0a, S2_DDR_2014);
	if (fthd_hw_ddr_status_busy(sc, 10000, 10) != 0)
		return (EBUSY);

	DELAY(10000);

	S2_WRITE(0, S2_3204);

	reg = S2_READ(S2_DDR40_STRAP_STATUS);
	device_printf(sc->dev, "S2 DRAM memory address: 0x%08x\n", reg);

	switch (sc->ddr_model) {
	case 4:
		val = 0x1fffffff;
		break;
	case 2:
		val = 0x0fffffff;
		break;
	default:
		val = sc->ddr_model;
	}

	S2_WRITE(val, S2_3208);
	S2_WRITE(0x1040, S2_3200);

	fthd_hw_ddr_rewrite_mode_regs(sc);

	S2_WRITE(0x20000, S2_DDR_2014);
	S2_WRITE(1, S2_DDR_2008);

	return (0);
}

/*
 * Write a pseudo-random pattern into S2 memory and read it back.  Returns
 * the failing bits folded into 16 bits, 0 on success.
 */
int
fthd_ddr_verify_mem(struct fthd_softc *sc, uint32_t base, int count)
{
	uint32_t state, val, failed_bits = 0;
	int i;

	state = 0x12345678;
	for (i = 0; i < count; i++) {
		state ^= state << 13; state ^= state >> 17; state ^= state << 5;
		fthd_s2_mem_write(sc, state, base + i * 4);
	}

	state = 0x12345678;
	for (i = 0; i < count; i++) {
		state ^= state << 13; state ^= state >> 17; state ^= state << 5;
		val = fthd_s2_mem_read(sc, base + i * 4);
		failed_bits |= state ^ val;
	}

	return ((failed_bits & 0xffff) | ((failed_bits >> 16) & 0xffff));
}

static void
fthd_ddr_phy_save_regs(struct fthd_softc *sc)
{
	int i;

	for (i = 0; i < DDR_PHY_NUM_REG; i++)
		sc->ddr_phy_regs[i] = fthd_isp_reg_read(sc,
		    DDR_PHY_REG_BASE + fthd_ddr_phy_reg_map[i]);
}

int
fthd_hw_init(struct fthd_softc *sc)
{
	int error, fail;

	error = fthd_hw_s2_init_pcie_link(sc);
	if (error)
		return (error);

	fthd_hw_s2_preinit_ddr_controller_soc(sc);
	error = fthd_hw_s2_init_ddr_controller_soc(sc);
	if (error)
		return (error);

	fail = fthd_ddr_verify_mem(sc, 0, 128);
	if (fail) {
		/*
		 * The Linux driver carries on here as well: shmoo calibration
		 * was never finished there.  Report it so a broken DDR setup
		 * is visible before the firmware upload.
		 */
		device_printf(sc->dev,
		    "DDR memory verification failed (bits 0x%04x)\n", fail);
	} else {
		device_printf(sc->dev, "DDR memory verification succeeded\n");
	}

	/* Save our working configuration */
	fthd_ddr_phy_save_regs(sc);

	S2_WRITE(0x8, S2_D108);
	S2_WRITE(0xc, S2_D104);

	fthd_isp_reg_write(sc, 0, ISP_REG_40004);

	error = isp_init(sc);
	if (error)
		return (error);

	device_printf(sc->dev, "Enabling interrupts\n");
	fthd_irq_enable(sc);
	return (0);
}

void
fthd_hw_deinit(struct fthd_softc *sc)
{
	fthd_isp_reg_write(sc, 0, ISP_REG_41020);
	fthd_irq_disable(sc);
}
