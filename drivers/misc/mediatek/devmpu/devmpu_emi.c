// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include <asm/page.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/arm-smccc.h>
#include <mt-plat/mtk_secure_api.h>
#include <emi.h>
#include <devmpu.h>
#include <devmpu_emi.h>

#define LOG_TAG "[DEVMPU_ACP]"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) LOG_TAG " " fmt

static irqreturn_t devmpu_irq_handler_emi(unsigned int emi_id,
	struct reg_info_t *dump, unsigned int leng)
{
	unsigned int i;
	unsigned int mpus = 0, mput = 0, mput_2nd = 0;
	unsigned long long vio_addr;
	unsigned int master_id, domain_id;
	unsigned int port_id;
	unsigned int wr_vio, wr_oo_vio;
	unsigned int hp_wr_vio;

	for (i = 0; i < leng; i++) {
		if (dump[i].offset == EMI_MPUS_OFFSET)
			mpus = dump[i].value;

		if (dump[i].offset == EMI_MPUT_OFFSET)
			mput = dump[i].value;

		if (dump[i].offset == EMI_MPUT_2ND_OFFSET)
			mput_2nd = dump[i].value;
	}

	if (!mput && !mput_2nd) {
		pr_err("%s:%d failed to get violation from emi\n",
				__func__, __LINE__);
		return IRQ_NONE;
	}

	vio_addr = ((((unsigned long long)(mput_2nd & 0xF)) << 32) + mput);
	hp_wr_vio = (mput_2nd >> 21) & 0x3;

	master_id = mpus & 0xFFFF;
	domain_id = (mpus >> 21) & 0xF;
	wr_vio = (mpus >> 29) & 0x3;
	wr_oo_vio = (mpus >> 27) & 0x3;
	port_id = master_id & 0x7;

	if (master_id == 0) {
		pr_info("%s:%d dump emi_id:%d, leng:%d\n",
			__func__, __LINE__, emi_id, leng);
		pr_info("%s:%d wr_vio:%d, wr_oo_vio:%d, port_id:%d\n",
			__func__, __LINE__, wr_vio, wr_oo_vio, port_id);
		pr_info("%s:%d vio_addr:0x%llx, hp_wr_vio:%d, master_id:0x%x, domain_id:%d\n",
			__func__, __LINE__, vio_addr, hp_wr_vio, master_id, domain_id);
		for (i = 0; i < leng; i++) {
			pr_info("%s: emi_id:%d, offset(0x%x), value(0x%x)\n",
				__func__, emi_id, dump[i].offset, dump[i].value);
		}
	}

#ifdef CONFIG_MTK_ENABLE_GENIEZONE
	if ((wr_vio == 2) && (wr_oo_vio == 0) &&
		((port_id == 0) || (port_id == 1)))
		return IRQ_HANDLED;
#endif

	/* if is hyperviosr MPU violation, deliver to DevMPU */
	if (hp_wr_vio) {
		devmpu_print_violation(vio_addr, master_id, domain_id,
				hp_wr_vio, true);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void devmpu_vio_clear_emi(unsigned int emi_id)
{
	devmpu_vio_clear(emi_id);
}

int devmpu_regist_emi(void)
{
	int ret = 0;

	ret = mtk_emimpu_prehandle_register(devmpu_irq_handler_emi);
	if (ret) {
		pr_err("%s:%d failed to register emi prehandle, %d\n",
				__func__, __LINE__, ret);
		return ret;
	}

	ret = mtk_emimpu_postclear_register(devmpu_vio_clear_emi);
	if (ret) {
		pr_err("%s:%d failed to register emi postclear, %d\n",
				__func__, __LINE__, ret);
		return ret;
	}

	return ret;
}

MODULE_DESCRIPTION("Mediatek Device MPU Driver with EMI");
MODULE_AUTHOR("Calvin Liao <calvin.liao@mediatek.com>");
MODULE_LICENSE("GPL");
