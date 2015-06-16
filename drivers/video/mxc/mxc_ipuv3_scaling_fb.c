/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2015	DATA RESPONS AS
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb.c
 *
 * @brief MXC Frame buffer driver for SDC
 *
 * @ingroup Framebuffer
 */
/*!
 * Include files
 */
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/ipu.h>
#include <linux/ipu-v3.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "mxc_dispdrv.h"

#undef SPLIT_IPU

void ipu_dump_registers(struct ipu_soc *ipu);
int ipu_reset_channel_buffer(struct ipu_soc *ipu, ipu_channel_t channel,
		ipu_buffer_t type);
/*
 * Driver name
 */
#define MXCFB_NAME      "mxc_scaling_fb"

/* Display port number */
#define MXCFB_PORT_NUM	2
/*!
 * Structure containing the MXC specific framebuffer information.
 */
struct mxcfb_info {
	int default_bpp;
	int cur_blank;
	int next_blank;
	ipu_channel_t ipu_ch_scale;
	int ipu_id;
	int ipu_di;
	u32 ipu_di_pix_fmt;
	bool ipu_int_clk;
	bool late_init;
	bool first_set_par;
	uint32_t ipu_ch_irq;
	uint32_t ipu_ch_nf_irq;
	uint32_t cur_scale_in_buf;

	u32 pseudo_palette[16];

	bool mode_found;
	struct completion flip_complete;
	struct completion vsync_complete;

	void *ipu;

	struct mxc_dispdrv_handle *dispdrv;

	struct fb_var_screeninfo cur_var;
	/* Added for scaling */
	u32 out_xres;				/* Scaled xres */
	u32 out_yres;				/* Scaled yres */
	ipu_channel_t ipu_ch_disp;	/* Channel for display */
	char __iomem *bounce_base[2];	/* Bounce buffer for the scaling */
	dma_addr_t bounce_phys[2];
	u32 bounce_len;
	uint32_t cur_scale_out_buf;
	uint32_t cur_disp_buf;
	uint32_t ipu_ch_scale_irq;
	uint32_t nr_scale_runs;
	int ipu_id_scaler;
	void *ipu_scaler;
	bool scaler_running;
	uint32_t use_db;
};

struct mxcfb_pfmt {
	u32 fb_pix_fmt;
	int bpp;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

static const struct mxcfb_pfmt mxcfb_pfmts[] = {
	/*     pixel         bpp    red         green        blue      transp */
	{IPU_PIX_FMT_RGB565, 16, {11, 5, 0}, { 5, 6, 0}, { 0, 5, 0}, { 0, 0, 0} },
	{IPU_PIX_FMT_RGB24,  24, { 0, 8, 0}, { 8, 8, 0}, {16, 8, 0}, { 0, 0, 0} },
	{IPU_PIX_FMT_BGR24,  24, {16, 8, 0}, { 8, 8, 0}, { 0, 8, 0}, { 0, 0, 0} },
	{IPU_PIX_FMT_RGB32,  32, { 0, 8, 0}, { 8, 8, 0}, {16, 8, 0}, {24, 8, 0} },
	{IPU_PIX_FMT_BGR32,  32, {16, 8, 0}, { 8, 8, 0}, { 0, 8, 0}, {24, 8, 0} },
	{IPU_PIX_FMT_ABGR32, 32, {24, 8, 0}, {16, 8, 0}, { 8, 8, 0}, { 0, 8, 0} },
};

struct mxcfb_alloc_list {
	struct list_head list;
	dma_addr_t phy_addr;
	void *cpu_addr;
	u32 size;
};

enum {
	BOTH_ON,
	SRC_ON,
	TGT_ON,
	BOTH_OFF
};

static LIST_HEAD(fb_alloc_list);

/* Return default standard(RGB) pixel format */
static uint32_t bpp_to_pixfmt(int bpp)
{
	uint32_t pixfmt = 0;

	switch (bpp) {
	case 24:
		pixfmt = IPU_PIX_FMT_BGR24;
		break;
	case 32:
		pixfmt = IPU_PIX_FMT_BGR32;
		break;
	case 16:
		pixfmt = IPU_PIX_FMT_RGB565;
		break;
	}
	return pixfmt;
}

static inline int bitfield_is_equal(struct fb_bitfield f1,
				    struct fb_bitfield f2)
{
	return !memcmp(&f1, &f2, sizeof(f1));
}

static int pixfmt_to_var(uint32_t pixfmt, struct fb_var_screeninfo *var)
{
	int i, ret = -1;

	for (i = 0; i < ARRAY_SIZE(mxcfb_pfmts); i++) {
		if (pixfmt == mxcfb_pfmts[i].fb_pix_fmt) {
			var->red    = mxcfb_pfmts[i].red;
			var->green  = mxcfb_pfmts[i].green;
			var->blue   = mxcfb_pfmts[i].blue;
			var->transp = mxcfb_pfmts[i].transp;
			var->bits_per_pixel = mxcfb_pfmts[i].bpp;
			ret = 0;
			break;
		}
	}
	return ret;
}

static int bpp_to_var(int bpp, struct fb_var_screeninfo *var)
{
	uint32_t pixfmt = 0;

	pixfmt = bpp_to_pixfmt(bpp);
	if (pixfmt)
		return pixfmt_to_var(pixfmt, var);
	else
		return -1;
}

static int check_var_pixfmt(struct fb_var_screeninfo *var)
{
	int i, ret = -1;

	for (i = 0; i < ARRAY_SIZE(mxcfb_pfmts); i++) {
		if (bitfield_is_equal(var->red, mxcfb_pfmts[i].red) &&
		    bitfield_is_equal(var->green, mxcfb_pfmts[i].green) &&
		    bitfield_is_equal(var->blue, mxcfb_pfmts[i].blue) &&
		    bitfield_is_equal(var->transp, mxcfb_pfmts[i].transp) &&
		    var->bits_per_pixel == mxcfb_pfmts[i].bpp) {
			ret = 0;
			break;
		}
	}
	return ret;
}

static uint32_t fbi_to_pixfmt(struct fb_info *fbi)
{
	int i;
	uint32_t pixfmt = 0;

	if (fbi->var.nonstd)
		return fbi->var.nonstd;

	for (i = 0; i < ARRAY_SIZE(mxcfb_pfmts); i++) {
		if (bitfield_is_equal(fbi->var.red, mxcfb_pfmts[i].red) &&
		    bitfield_is_equal(fbi->var.green, mxcfb_pfmts[i].green) &&
		    bitfield_is_equal(fbi->var.blue, mxcfb_pfmts[i].blue) &&
		    bitfield_is_equal(fbi->var.transp, mxcfb_pfmts[i].transp)) {
			pixfmt = mxcfb_pfmts[i].fb_pix_fmt;
			break;
		}
	}

	if (pixfmt == 0)
		dev_err(fbi->device, "cannot get pixel format\n");

	return pixfmt;
}

#if 0
static struct fb_info *found_registered_fb(ipu_channel_t ipu_ch, int ipu_id)
{
	int i;
	struct mxcfb_info *mxc_fbi;
	struct fb_info *fbi = NULL;

	for (i = 0; i < num_registered_fb; i++) {
		mxc_fbi =
			((struct mxcfb_info *)(registered_fb[i]->par));

		if ((mxc_fbi->ipu_ch_scale == ipu_ch) &&
			(mxc_fbi->ipu_id == ipu_id)) {
			fbi = registered_fb[i];
			break;
		}
	}
	return fbi;
}
#endif

static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id);
static irqreturn_t mxcfb_nf_irq_handler(int irq, void *dev_id);
static int mxcfb_blank(int blank, struct fb_info *info);
static int mxcfb_map_video_memory(struct fb_info *fbi);
static int mxcfb_unmap_video_memory(struct fb_info *fbi);

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ywrapstep = 1;
	fix->ypanstep = 1;

	return 0;
}

static int _start_scaler(struct fb_info *fbi) {
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	int ret;

	if (mxc_fbi->scaler_running)
		return 0;
	dev_dbg(fbi->device, "%s:\n", __func__);
	mxc_fbi->cur_disp_buf = 0;
	ipu_clear_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_OUTPUT_BUFFER, 0);
	ipu_clear_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_INPUT_BUFFER, 0);
	ipu_clear_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_OUTPUT_BUFFER, 1);
	ipu_clear_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_INPUT_BUFFER, 1);

	mxc_fbi->nr_scale_runs = 0;
	mxc_fbi->scaler_running = true;
	ret = ipu_enable_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale);

	ret = ipu_select_buffer(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_OUTPUT_BUFFER, 0);
	if (ret)
		dev_err(fbi->device, "%s: ipu_select_buffer [scale out] failed [%d]\n",
				__func__, ret);

	ret = ipu_select_buffer(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, IPU_INPUT_BUFFER, 0);
	if (ret)
		dev_err(fbi->device, "%s: ipu_select_buffer [scale in] failed [%d]\n",
				__func__, ret);

	mxc_fbi->cur_scale_in_buf = 0;
	mxc_fbi->cur_scale_out_buf = 0;
	//ipu_enable_irq(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale_irq);

	return ret;

}

static int _stop_scaler(struct fb_info *fbi) {
	int ret;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	if (!mxc_fbi->scaler_running)
		return 0;
	//ipu_disable_irq(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale_irq);

	ret = ipu_disable_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, true);
	mxc_fbi->cur_scale_in_buf = 0;
	mxc_fbi->cur_scale_out_buf = 0;
	mxc_fbi->scaler_running = false;
	dev_dbg(fbi->device, "%s: Number of scale runs %d\n", __func__, mxc_fbi->nr_scale_runs);
	return ret;
}

static int _prime_scaler(struct fb_info *fbi) {
	int status;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	uint32_t ready_buf;
	static int prev_buf=-1;
	if (!mxc_fbi->scaler_running)
		return 0;

	mxc_fbi->cur_disp_buf = ipu_get_cur_buffer_idx(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp,
			IPU_INPUT_BUFFER);

#if 0
		if (mxc_fbi->cur_disp_buf == 1)
		ipu_reset_channel_buffer(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp, IPU_INPUT_BUFFER);

	if (mxc_fbi->cur_disp_buf == prev_buf)
		dev_warn(fbi->device, "%s: disp buffers are not togglng in db mode %d\n", __func__, mxc_fbi->cur_disp_buf);
#endif

	mxc_fbi->cur_scale_out_buf = mxc_fbi->cur_disp_buf ? 0 : 1;
	ready_buf = mxc_fbi->cur_scale_out_buf;
	status = ipu_select_buffer(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
			IPU_OUTPUT_BUFFER, ready_buf);
	if (status)
		dev_err(fbi->device, "%s: ipu_select_buffer [scale out %d] failed [%d]\n",
				__func__, 0, status);

	status = ipu_select_buffer(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
			IPU_INPUT_BUFFER, ready_buf);
	if (status)
		dev_err(fbi->device, "%s: ipu_select_buffer [scale in %d] failed [%d]\n",
				__func__, 1, status);

	prev_buf = mxc_fbi->cur_disp_buf;
	return 0;
}

static int _setup_disp_channel1(struct fb_info *fbi)
{
	ipu_channel_params_t pm_disp;
	ipu_channel_params_t pm_scale;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	memset(&pm_disp, 0, sizeof(pm_disp));

	pm_disp.mem_dp_bg_sync.di = mxc_fbi->ipu_di;
	if (fbi->var.vmode & FB_VMODE_INTERLACED)
		pm_disp.mem_dp_bg_sync.interlaced = true;
	pm_disp.mem_dp_bg_sync.out_pixel_fmt = mxc_fbi->ipu_di_pix_fmt;
	pm_disp.mem_dp_bg_sync.in_pixel_fmt = fbi_to_pixfmt(fbi);

	if (ipu_init_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp, &pm_disp))
		dev_err(fbi->device, "%s: ipu_init_channel (disp) failed\n", __func__);

	memset(&pm_scale, 0, sizeof(pm_scale));
	pm_scale.mem_pp_mem.in_pixel_fmt = fbi_to_pixfmt(fbi);
	pm_scale.mem_pp_mem.out_pixel_fmt = pm_scale.mem_pp_mem.in_pixel_fmt;
	pm_scale.mem_pp_mem.in_height = fbi->var.yres;
	pm_scale.mem_pp_mem.in_width = fbi->var.xres;
	pm_scale.mem_pp_mem.out_height = mxc_fbi->out_yres;
	pm_scale.mem_pp_mem.out_width = mxc_fbi->out_xres;
	dev_dbg(fbi->device, "%s: scale from %d x %d -> %d x %d\n", __func__,
			pm_scale.mem_pp_mem.in_width, pm_scale.mem_pp_mem.in_height,
			pm_scale.mem_pp_mem.out_width, pm_scale.mem_pp_mem.out_height);

	if (ipu_init_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, &pm_scale))
		dev_err(fbi->device, "%s: ipu_init_channel (scale) failed\n", __func__);

	ipu_link_channels(mxc_fbi->ipu, mxc_fbi->ipu_ch_scale, mxc_fbi->ipu_ch_disp);
	return 0;
}


static int _setup_disp_channel2(struct fb_info *fbi)
{
	int retval = 0;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	int fb_stride;
	unsigned long base;
	unsigned int fr_xoff, fr_yoff, fr_w, fr_h;
	int bounce_stride =  1024*4;


	switch (fbi_to_pixfmt(fbi)) {
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_NV12:
	case IPU_PIX_FMT_YUV422P:
	case IPU_PIX_FMT_YVU422P:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YUV444P:
		fb_stride = fbi->var.xres_virtual;
		break;
	default:
		fb_stride = fbi->fix.line_length;
	}

	base = fbi->fix.smem_start;
	fr_xoff = fbi->var.xoffset;
	fr_w = fbi->var.xres_virtual;
	if (!(fbi->var.vmode & FB_VMODE_YWRAP)) {
		dev_dbg(fbi->device, "Y wrap disabled\n");
		fr_yoff = fbi->var.yoffset % fbi->var.yres;
		fr_h = fbi->var.yres;
		base += fbi->fix.line_length * fbi->var.yres *
			(fbi->var.yoffset / fbi->var.yres);
	} else {
		dev_dbg(fbi->device, "Y wrap enabled\n");
		fr_yoff = fbi->var.yoffset;
		fr_h = fbi->var.yres_virtual;
	}
	base += fr_yoff * fb_stride + fr_xoff;

	mxc_fbi->cur_disp_buf = 0;
	init_completion(&mxc_fbi->flip_complete);
	init_completion(&mxc_fbi->vsync_complete);
	/*
	 * We don't need to wait for vsync at the first time
	 * we do pan display after fb is initialized, as IPU will
	 * switch to the newly selected buffer automatically,
	 * so we call complete() for both mxc_fbi->flip_complete
	 * and mxc_fbi->alpha_flip_complete.
	 */
	complete(&mxc_fbi->flip_complete);

	retval = ipu_init_channel_buffer(mxc_fbi->ipu,
					 mxc_fbi->ipu_ch_disp, IPU_INPUT_BUFFER,
					 fbi_to_pixfmt(fbi),
					 mxc_fbi->out_xres, mxc_fbi->out_yres,
					 bounce_stride,
					 0,
					 mxc_fbi->bounce_phys[0],
					 mxc_fbi->bounce_phys[mxc_fbi->use_db ? 1 : 0],
					 0,
					 0, 0);
	if (retval) {
		dev_err(fbi->device,
			"ipu_init_channel_buffer error %d\n", retval);
		return retval;
	}

	/* Bounce buffer */

	retval = ipu_init_channel_buffer(mxc_fbi->ipu_scaler,
			 mxc_fbi->ipu_ch_scale, IPU_INPUT_BUFFER,
			 fbi_to_pixfmt(fbi),
			 fbi->var.xres, fbi->var.yres,
			 fb_stride,
			 0,
			 base,
			 base,
			 0,
			 0, 0);
	if (retval) {
		dev_err(fbi->device,
			"Bounce buffer in: ipu_init_channel_buffer error %d\n", retval);
		return retval;
	}

	retval = ipu_init_channel_buffer(mxc_fbi->ipu_scaler,
			 mxc_fbi->ipu_ch_scale, IPU_OUTPUT_BUFFER,
			 fbi_to_pixfmt(fbi),
			 mxc_fbi->out_xres, mxc_fbi->out_yres,
			 bounce_stride,
			 0,
			 mxc_fbi->bounce_phys[0],
			 mxc_fbi->bounce_phys[mxc_fbi->use_db ? 1 : 0],
			 0,
			 0, 0);
	if (retval) {
		dev_err(fbi->device,
			"Bounce buffer out: ipu_init_channel_buffer error %d\n", retval);
		return retval;
	}
	/* update u/v offset */
	retval = ipu_update_channel_offset(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
			IPU_INPUT_BUFFER,
			fbi_to_pixfmt(fbi),
			fr_w,
			fr_h,
			fr_w,
			0, 0,
			fr_yoff,
			fr_xoff);
		if (retval)
			dev_err(fbi->device,
					"%s: ipu_update_channel_offset error %d\n", __func__, retval);

	return retval;
}

static bool mxcfb_need_to_set_par(struct fb_info *fbi)
{
	struct mxcfb_info *mxc_fbi = fbi->par;

	if ((fbi->var.activate & FB_ACTIVATE_FORCE) &&
	    (fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		return true;

	/*
	 * Ignore xoffset and yoffset update,
	 * because pan display handles this case.
	 */
	mxc_fbi->cur_var.xoffset = fbi->var.xoffset;
	mxc_fbi->cur_var.yoffset = fbi->var.yoffset;

	return !!memcmp(&mxc_fbi->cur_var, &fbi->var,
			sizeof(struct fb_var_screeninfo));
}

/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_set_par(struct fb_info *fbi)
{
	int retval = 0;
	u32 mem_len;
	ipu_di_signal_cfg_t sig_cfg;
	uint32_t out_pixel_fmt;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	if (!mxcfb_need_to_set_par(fbi))
		return 0;

	dev_dbg(fbi->device, "Reconfiguring framebuffer\n");

	if (fbi->var.xres == 0 || fbi->var.yres == 0)
		return 0;


	ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
	ipu_disable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
	ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);
	ipu_disable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);
	ipu_disable_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp, true);
	ipu_uninit_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp);

	ipu_disable_irq(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale_irq);
	_stop_scaler(fbi);
	ipu_uninit_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale);

	/*
	 * Disable IPU hsp clock if it is enabled for an
	 * additional time in ipu common driver.
	 */
	if (mxc_fbi->first_set_par && mxc_fbi->late_init)
		ipu_disable_hsp_clk(mxc_fbi->ipu);

	mxcfb_set_fix(fbi);

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			mxcfb_unmap_video_memory(fbi);

		if (mxcfb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}

	if (mxc_fbi->first_set_par) {
		/*
		 * Clear the screen in case uboot fb pixel format is not
		 * the same to kernel fb pixel format.
		 */
		if (mxc_fbi->late_init)
			memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

		mxc_fbi->first_set_par = false;
	}



	if (mxc_fbi->next_blank != FB_BLANK_UNBLANK)
		return retval;

	if (mxc_fbi->dispdrv && mxc_fbi->dispdrv->drv->setup) {
		retval = mxc_fbi->dispdrv->drv->setup(mxc_fbi->dispdrv, fbi);
		if (retval < 0) {
			dev_err(fbi->device, "setup error, dispdrv:%s.\n",
					mxc_fbi->dispdrv->drv->name);
			return -EINVAL;
		}
	}
	_setup_disp_channel1(fbi);

	memset(&sig_cfg, 0, sizeof(sig_cfg));
	if (fbi->var.vmode & FB_VMODE_INTERLACED)
		sig_cfg.interlaced = true;
	out_pixel_fmt = mxc_fbi->ipu_di_pix_fmt;
	if (fbi->var.vmode & FB_VMODE_ODD_FLD_FIRST) /* PAL */
		sig_cfg.odd_field_first = true;
	if (mxc_fbi->ipu_int_clk)
		sig_cfg.int_clk = true;
	if (fbi->var.sync & FB_SYNC_HOR_HIGH_ACT)
		sig_cfg.Hsync_pol = true;
	if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT)
		sig_cfg.Vsync_pol = true;
	if (!(fbi->var.sync & FB_SYNC_CLK_LAT_FALL))
		sig_cfg.clk_pol = true;
	if (fbi->var.sync & FB_SYNC_DATA_INVERT)
		sig_cfg.data_pol = true;
	if (!(fbi->var.sync & FB_SYNC_OE_LOW_ACT))
		sig_cfg.enable_pol = true;
	if (fbi->var.sync & FB_SYNC_CLK_IDLE_EN)
		sig_cfg.clkidle_en = true;

	dev_dbg(fbi->device, "pixclock = %ul Hz\n",
		(u32) (PICOS2KHZ(fbi->var.pixclock) * 1000UL));

	if (ipu_init_sync_panel(mxc_fbi->ipu, mxc_fbi->ipu_di,
				(PICOS2KHZ(fbi->var.pixclock)) * 1000UL,
				mxc_fbi->out_xres, mxc_fbi->out_yres,
				out_pixel_fmt,
				fbi->var.left_margin,
				fbi->var.hsync_len,
				fbi->var.right_margin,
				fbi->var.upper_margin,
				fbi->var.vsync_len,
				fbi->var.lower_margin,
				0, sig_cfg) != 0) {
		dev_err(fbi->device,
			"mxcfb: Error initializing panel.\n");
		return -EINVAL;
	}

	fbi->mode =
		(struct fb_videomode *)fb_match_mode(&fbi->var,
						 &fbi->modelist);

	ipu_disp_set_window_pos(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale, 0, 0);

	retval = _setup_disp_channel2(fbi);
	if (retval) {
		ipu_uninit_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp);
		ipu_uninit_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale);
		return retval;
	}




	if (mxc_fbi->dispdrv && mxc_fbi->dispdrv->drv->enable) {
		retval = mxc_fbi->dispdrv->drv->enable(mxc_fbi->dispdrv, fbi);
		if (retval < 0) {
			dev_err(fbi->device, "enable error, dispdrv:%s.\n",
					mxc_fbi->dispdrv->drv->name);
			return -EINVAL;
		}
	}
	mxc_fbi->cur_var = fbi->var;

	_start_scaler(fbi);
	ipu_enable_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp);
	ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
	ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);


	return retval;
}


/*
 * Check framebuffer variable parameters and adjust to valid values.
 *
 * @param       var      framebuffer variable parameters
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 vtotal;
	u32 htotal;
	/* struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par; */


	if (var->xres == 0 || var->yres == 0)
		return 0;

	if (var->rotate > IPU_ROTATE_VERT_FLIP)
		var->rotate = IPU_ROTATE_NONE;

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres * 3;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 12) &&
	    (var->bits_per_pixel != 8))
		var->bits_per_pixel = 16;

	if (check_var_pixfmt(var))
		/* Fall back to default */
		bpp_to_var(var->bits_per_pixel, var);

	if (var->pixclock < 1000) {
		htotal = var->xres + var->right_margin + var->hsync_len +
		    var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
		    var->upper_margin;
		var->pixclock = (vtotal * htotal * 6UL) / 100UL;
		var->pixclock = KHZ2PICOS(var->pixclock);
		dev_dbg(info->device,
			"pixclock set for 60Hz refresh = %u ps\n",
			var->pixclock);
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int trans, struct fb_info *fbi)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (fbi->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->pseudo_palette;

			val = _chan_to_field(red, &fbi->var.red);
			val |= _chan_to_field(green, &fbi->var.green);
			val |= _chan_to_field(blue, &fbi->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

/*
 * Function to handle custom ioctls for MXC framebuffer.
 *
 * @param       inode   inode struct
 *
 * @param       file    file struct
 *
 * @param       cmd     Ioctl command to handle
 *
 * @param       arg     User pointer to command arguments
 *
 * @param       fbi     framebuffer information pointer
 */
static int mxcfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int __user *argp = (void __user *)arg;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	switch (cmd) {

	case MXCFB_SET_CLR_KEY:
		{
			struct mxcfb_color_key key;
			if (copy_from_user(&key, (void *)arg, sizeof(key))) {
				retval = -EFAULT;
				break;
			}
			retval = ipu_disp_set_color_key(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp,
							key.enable,
							key.color_key);
			dev_dbg(fbi->device, "Set color key to 0x%08X\n",
				key.color_key);
			break;
		}
	case MXCFB_SET_GAMMA:
		{
			struct mxcfb_gamma gamma;
			if (copy_from_user(&gamma, (void *)arg, sizeof(gamma))) {
				retval = -EFAULT;
				break;
			}
			retval = ipu_disp_set_gamma_correction(mxc_fbi->ipu,
							mxc_fbi->ipu_ch_disp,
							gamma.enable,
							gamma.constk,
							gamma.slopek);
			break;
		}
	case MXCFB_WAIT_FOR_VSYNC:
		{
			if (mxc_fbi->cur_blank != FB_BLANK_UNBLANK) {
				retval = -EINVAL;
				break;
			}

			/* init_completion(&mxc_fbi->vsync_complete);
			ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);
			ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq); */
			retval = wait_for_completion_interruptible_timeout(
				&mxc_fbi->vsync_complete, 1 * HZ);
			if (retval == 0) {
				dev_err(fbi->device,
					"MXCFB_WAIT_FOR_VSYNC: timeout %d\n",
					retval);
				retval = -ETIME;
			} else if (retval > 0) {
				retval = 0;
			}
			break;
		}
	case FBIO_ALLOC:
		{
			int size;
			struct mxcfb_alloc_list *mem;

			mem = kzalloc(sizeof(*mem), GFP_KERNEL);
			if (mem == NULL)
				return -ENOMEM;

			if (get_user(size, argp))
				return -EFAULT;

			mem->size = PAGE_ALIGN(size);

			mem->cpu_addr = dma_alloc_coherent(fbi->device, size,
							   &mem->phy_addr,
							   GFP_KERNEL);
			if (mem->cpu_addr == NULL) {
				kfree(mem);
				return -ENOMEM;
			}

			list_add(&mem->list, &fb_alloc_list);

			dev_dbg(fbi->device, "allocated %d bytes @ 0x%08X\n",
				mem->size, mem->phy_addr);

			if (put_user(mem->phy_addr, argp))
				return -EFAULT;

			break;
		}
	case FBIO_FREE:
		{
			unsigned long offset;
			struct mxcfb_alloc_list *mem;

			if (get_user(offset, argp))
				return -EFAULT;

			retval = -EINVAL;
			list_for_each_entry(mem, &fb_alloc_list, list) {
				if (mem->phy_addr == offset) {
					list_del(&mem->list);
					dma_free_coherent(fbi->device,
							  mem->size,
							  mem->cpu_addr,
							  mem->phy_addr);
					kfree(mem);
					retval = 0;
					break;
				}
			}

			break;
		}

	case MXCFB_GET_FB_IPU_CHAN:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_ch_scale, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_DIFMT:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_di_pix_fmt, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_FB_IPU_DI:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_di, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_FB_BLANK:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->cur_blank, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_SET_DIFMT:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (get_user(mxc_fbi->ipu_di_pix_fmt, argp))
				return -EFAULT;

			break;
		}
	case MXCFB_CSC_UPDATE:
		{
			struct mxcfb_csc_matrix csc;

			if (copy_from_user(&csc, (void *) arg, sizeof(csc)))
				return -EFAULT;

			if ((mxc_fbi->ipu_ch_disp != MEM_FG_SYNC) &&
				(mxc_fbi->ipu_ch_disp != MEM_BG_SYNC) &&
				(mxc_fbi->ipu_ch_disp != MEM_BG_ASYNC0))
				return -EFAULT;
			ipu_set_csc_coefficients(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp,
						csc.param);
		}
	default:
		retval = -EINVAL;
	}
	return retval;
}

/*
 * mxcfb_blank():
 *      Blank the display.
 */
static int mxcfb_blank(int blank, struct fb_info *info)
{
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;
	int ret = 0;

	dev_dbg(info->device, "blank = %d\n", blank);

	if (mxc_fbi->cur_blank == blank)
		return 0;

	mxc_fbi->next_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (mxc_fbi->dispdrv && mxc_fbi->dispdrv->drv->disable)
			mxc_fbi->dispdrv->drv->disable(mxc_fbi->dispdrv, info);
		ipu_disable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);
		ipu_disable_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp, true);
		_stop_scaler(info);
		if (mxc_fbi->ipu_di >= 0)
			ipu_uninit_sync_panel(mxc_fbi->ipu, mxc_fbi->ipu_di);
		ipu_uninit_channel(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp);
		ipu_uninit_channel(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale);
		break;
	case FB_BLANK_UNBLANK:
		info->var.activate = (info->var.activate & ~FB_ACTIVATE_MASK) |
				FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
		ret = mxcfb_set_par(info);
		break;
	}
	if (!ret)
		mxc_fbi->cur_blank = blank;
	return ret;
}

/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 *
 * @param               var     Variable screen buffer information
 * @param               info    Framebuffer information pointer
 */
static int
mxcfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;
	u_int y_bottom;
	unsigned int fr_xoff, fr_yoff, fr_w, fr_h;
	unsigned long base = 0;
	int fb_stride;
	int ret;
	int buffer_status[3];

	if (mxc_fbi->cur_blank != FB_BLANK_UNBLANK)
		return -EINVAL;

	y_bottom = var->yoffset;

	if (y_bottom > info->var.yres_virtual)
		return -EINVAL;

	switch (fbi_to_pixfmt(info)) {
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_NV12:
	case IPU_PIX_FMT_YUV422P:
	case IPU_PIX_FMT_YVU422P:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YUV444P:
		fb_stride = info->var.xres_virtual;
		break;
	default:
		fb_stride = info->fix.line_length;
	}

	base = info->fix.smem_start;
	fr_xoff = var->xoffset;
	fr_w = info->var.xres_virtual;
	if (!(var->vmode & FB_VMODE_YWRAP)) {
		dev_dbg(info->device, "Y wrap disabled\n");
		fr_yoff = var->yoffset % info->var.yres;
		fr_h = info->var.yres;
		base += info->fix.line_length * info->var.yres *
			(var->yoffset / info->var.yres);
	} else {
		dev_dbg(info->device, "Y wrap enabled\n");
		fr_yoff = var->yoffset;
		fr_h = info->var.yres_virtual;
	}
	base += fr_yoff * fb_stride + fr_xoff;

	ret = wait_for_completion_timeout(&mxc_fbi->flip_complete, HZ/2);
	if (ret == 0) {
		dev_err(info->device, "timeout when waiting for flip irq\n");
		return -ETIMEDOUT;
	}
	_stop_scaler(info);

	buffer_status[0] = ipu_update_channel_buffer(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
					IPU_INPUT_BUFFER, 0, base);

	if (buffer_status[0] == 0 ) {

		/* update u/v offset */
		ret = ipu_update_channel_offset(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
				IPU_INPUT_BUFFER,
				fbi_to_pixfmt(info),
				fr_w,
				fr_h,
				fr_w,
				0, 0,
				fr_yoff,
				fr_xoff);
		if (ret)
			dev_err(info->device, "%s: ipu_update_channel_offset ch %d failed [%d]\n",
					__func__, mxc_fbi->ipu_ch_scale, ret);

		ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
		ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);
		ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
		ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_nf_irq);

		_start_scaler(info);
	} else {
		dev_err(info->device,
			"Error [%d] updating SDC buf %d to address=0x%08lX, "
			"current buf %d, buf0 ready %d, buf1 ready %d\n",
			buffer_status[0],
			mxc_fbi->cur_scale_in_buf, base,
			ipu_get_cur_buffer_idx(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
					       IPU_INPUT_BUFFER),
			ipu_check_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
					       IPU_INPUT_BUFFER, 0),
			ipu_check_buffer_ready(mxc_fbi->ipu_scaler, mxc_fbi->ipu_ch_scale,
					       IPU_INPUT_BUFFER, 1));

		ipu_clear_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);
		ipu_enable_irq(mxc_fbi->ipu, mxc_fbi->ipu_ch_irq);

		return -EBUSY;
	}

	info->var.yoffset = var->yoffset;
	dev_dbg(info->device, "%s: Update complete\n", __func__);
	return 0;
}

/*
 * Function to handle custom mmap for MXC framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       vma     Pointer to vm_area_struct
 */
static int mxcfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	bool found = false;
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct mxcfb_alloc_list *mem;
	/* struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par; */

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory */
		len = fbi->fix.smem_len - offset;
		vma->vm_pgoff = (fbi->fix.smem_start + offset) >> PAGE_SHIFT;
	} else {
		list_for_each_entry(mem, &fb_alloc_list, list) {
			if (offset == mem->phy_addr) {
				found = true;
				len = mem->size;
				break;
			}
		}
		if (!found)
			return -EINVAL;
	}

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(fbi->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core framebuffer driver to perform operations like
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops mxcfb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = mxcfb_set_par,
	.fb_check_var = mxcfb_check_var,
	.fb_setcolreg = mxcfb_setcolreg,
	.fb_pan_display = mxcfb_pan_display,
	.fb_ioctl = mxcfb_ioctl,
	.fb_mmap = mxcfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_blank = mxcfb_blank,
};

static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mxcfb_info *mxc_fbi = fbi->par;


	complete(&mxc_fbi->flip_complete);
	return IRQ_HANDLED;
}

static irqreturn_t mxcfb_nf_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mxcfb_info *mxc_fbi = fbi->par;
#if 0
	mxc_fbi->cur_disp_buf = ipu_get_cur_buffer_idx(mxc_fbi->ipu, mxc_fbi->ipu_ch_disp, IPU_INPUT_BUFFER);
	dev_dbg(fbi->device, "%s: cur_disp_buf = %d\n", __func__,
					mxc_fbi->cur_disp_buf);
#endif
	_prime_scaler(fbi);
	complete(&mxc_fbi->vsync_complete);
	return IRQ_HANDLED;
}

static irqreturn_t mxcfb_scale_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mxcfb_info *mxc_fbi = fbi->par;
	mxc_fbi->nr_scale_runs++;
	//dev_dbg(fbi->device, "%s: IRQ %d\n", __func__, irq);
	return IRQ_HANDLED;
}

/*
 * Suspends the framebuffer and blanks the screen. Power management support
 */
static int mxcfb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	int saved_blank;
#ifdef CONFIG_FB_MXC_LOW_PWR_DISPLAY
	void *fbmem;
#endif

	console_lock();
	fb_set_suspend(fbi, 1);
	saved_blank = mxc_fbi->cur_blank;
	mxcfb_blank(FB_BLANK_POWERDOWN, fbi);
	mxc_fbi->next_blank = saved_blank;
	console_unlock();

	return 0;
}

/*
 * Resumes the framebuffer and unblanks the screen. Power management support
 */
static int mxcfb_resume(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	console_lock();
	mxcfb_blank(mxc_fbi->next_blank, fbi);
	fb_set_suspend(fbi, 0);
	console_unlock();

	return 0;
}

/*
 * Main framebuffer functions
 */

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxcfb_map_video_memory(struct fb_info *fbi)
{
	int n;

	struct mxcfb_info *mxc_fbi = fbi->par;
	int nr_buf= mxc_fbi->use_db ? 2 : 1;

	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA | GFP_KERNEL);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	/* Allocate bounce buffers */
	mxc_fbi->bounce_len = 4096 * mxc_fbi->out_yres;
	for (n=0; n < nr_buf;  n++) {

		mxc_fbi->bounce_base[n] = dma_alloc_writecombine(fbi->device,
				mxc_fbi->bounce_len,
				&mxc_fbi->bounce_phys[n],
				GFP_DMA | GFP_KERNEL);
		if (mxc_fbi->bounce_base[n] == 0) {
			dev_err(fbi->device, "Unable to allocate bounce memory\n");
			mxc_fbi->bounce_base[n] = 0;
			mxc_fbi->bounce_phys[n] = 0;
			return -ENOMEM;
		}
		memset((char *)mxc_fbi->bounce_base[n], 0, mxc_fbi->bounce_len);
		dev_dbg(fbi->device, "allocated bounce buffer @ paddr=0x%08X, size=%d.\n",
				(uint32_t) mxc_fbi->bounce_phys[n], mxc_fbi->bounce_len);
	}

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxcfb_unmap_video_memory(struct fb_info *fbi)
{
	int n;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;

	for (n=0; n < 2; n++)
		if (mxc_fbi->bounce_base[n]) {
			dma_free_writecombine(fbi->device, mxc_fbi->bounce_len,
					mxc_fbi->bounce_base[n], mxc_fbi->bounce_phys[n]);

			mxc_fbi->bounce_base[n] = 0;
			mxc_fbi->bounce_phys[n] = 0;
		}

	return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info *mxcfb_init_fbinfo(struct device *dev, struct fb_ops *ops)
{
	struct fb_info *fbi;
	struct mxcfb_info *mxcfbi;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	fbi = framebuffer_alloc(sizeof(struct mxcfb_info), dev);
	if (!fbi)
		return NULL;

	mxcfbi = (struct mxcfb_info *)fbi->par;

	fbi->var.activate = FB_ACTIVATE_NOW;

	fbi->fbops = ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = mxcfbi->pseudo_palette;

	/*
	 * Allocate colormap
	 */
	fb_alloc_cmap(&fbi->cmap, 16, 0);

	return fbi;
}

#if 0
static ssize_t show_disp_chan(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	/* struct fb_info *info = dev_get_drvdata(dev);
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)info->par; */
	return sprintf(buf, "1-layer-fb\n");
}

static ssize_t swap_disp_chan(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)info->par;
	struct mxcfb_info *fg_mxcfbi = NULL;

	console_lock();
	/* swap only happen between DP-BG and DC, while DP-FG disable */
	if (((mxcfbi->ipu_ch_scale == MEM_BG_SYNC) &&
	     (strstr(buf, "1-layer-fb") != NULL)) ||
	    ((mxcfbi->ipu_ch_scale == MEM_BG_SYNC) &&
	     (strstr(buf, "2-layer-fb-bg") != NULL))) {
		struct fb_info *fbi_fg;

		fbi_fg = found_registered_fb(MEM_FG_SYNC, mxcfbi->ipu_id);
		if (fbi_fg)
			fg_mxcfbi = (struct mxcfb_info *)fbi_fg->par;

		if (!fg_mxcfbi ||
			fg_mxcfbi->cur_blank == FB_BLANK_UNBLANK) {
			dev_err(dev,
				"Can not switch while fb2(fb-fg) is on.\n");
			console_unlock();
			return count;
		}

		if (swap_channels(info) < 0)
			dev_err(dev, "Swap display channel failed.\n");
	}

	console_unlock();
	return count;
}
static DEVICE_ATTR(fsl_disp_property, 644, show_disp_chan, swap_disp_chan);
#endif

static ssize_t show_disp_dev(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)info->par;

	return sprintf(buf, "%s\n", mxcfbi->dispdrv->drv->name);
}
static DEVICE_ATTR(fsl_disp_dev_property, S_IRUGO, show_disp_dev, NULL);

static int mxcfb_get_crtc(struct device *dev, struct mxcfb_info *mxcfbi,
			  enum crtc crtc)
{
	int i = 0;

	for (; i < ARRAY_SIZE(ipu_di_crtc_maps); i++)
		if (ipu_di_crtc_maps[i].crtc == crtc) {
			mxcfbi->ipu_id = ipu_di_crtc_maps[i].ipu_id;
			mxcfbi->ipu_di = ipu_di_crtc_maps[i].ipu_di;
			return 0;
		}

	dev_err(dev, "failed to get valid crtc\n");
	return -EINVAL;
}

static int mxcfb_dispdrv_init(struct platform_device *pdev,
		struct fb_info *fbi)
{
	struct ipuv3_fb_platform_data *plat_data = pdev->dev.platform_data;
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)fbi->par;
	struct mxc_dispdrv_setting setting;
	char disp_dev[32], *default_dev = "lcd";
	int ret = 0;

	setting.if_fmt = plat_data->interface_pix_fmt;
	setting.dft_mode_str = plat_data->mode_str;
	setting.default_bpp = plat_data->default_bpp;
	if (!setting.default_bpp)
		setting.default_bpp = 16;
	setting.fbi = fbi;
	if (!strlen(plat_data->disp_dev)) {
		memcpy(disp_dev, default_dev, strlen(default_dev));
		disp_dev[strlen(default_dev)] = '\0';
	} else {
		memcpy(disp_dev, plat_data->disp_dev,
				strlen(plat_data->disp_dev));
		disp_dev[strlen(plat_data->disp_dev)] = '\0';
	}

	dev_info(&pdev->dev, "register mxc display driver %s\n", disp_dev);

	mxcfbi->dispdrv = mxc_dispdrv_gethandle(disp_dev, &setting);
	if (IS_ERR(mxcfbi->dispdrv)) {
		ret = PTR_ERR(mxcfbi->dispdrv);
		dev_err(&pdev->dev, "NO mxc display driver found!\n");
		return ret;
	} else {
		/* fix-up  */
		mxcfbi->ipu_di_pix_fmt = setting.if_fmt;
		mxcfbi->default_bpp = setting.default_bpp;

		ret = mxcfb_get_crtc(&pdev->dev, mxcfbi, setting.crtc);
		if (ret)
			return ret;

		dev_dbg(&pdev->dev, "di_pixfmt:0x%x, bpp:0x%x, di:%d, ipu:%d\n",
				setting.if_fmt, setting.default_bpp,
				mxcfbi->ipu_di, mxcfbi->ipu_id);

	}

	return ret;
}

/*
 * Parse user specified options (`video=trident:')
 * example:
 * 	video=mxcfb0:dev=lcd,800x480M-16@55,if=RGB565,bpp=16,noaccel
 *	video=mxcfb0:dev=lcd,800x480M-16@55,if=RGB565,fbpix=RGB565
 */
static int mxcfb_option_setup(struct platform_device *pdev, struct fb_info *fbi)
{
	struct ipuv3_fb_platform_data *pdata = pdev->dev.platform_data;
	char *options, *opt, *fb_mode_str = NULL;
	char name[] = "mxcfb0";
	uint32_t fb_pix_fmt = 0;

	name[5] += pdev->id;
	if (fb_get_options(name, &options)) {
		dev_err(&pdev->dev, "Can't get fb option for %s!\n", name);
		return -ENODEV;
	}

	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "dev=", 4)) {
			memcpy(pdata->disp_dev, opt + 4, strlen(opt) - 4);
			pdata->disp_dev[strlen(opt) - 4] = '\0';
		} else if (!strncmp(opt, "if=", 3)) {
			if (!strncmp(opt+3, "RGB24", 5))
				pdata->interface_pix_fmt = IPU_PIX_FMT_RGB24;
			else if (!strncmp(opt+3, "BGR24", 5))
				pdata->interface_pix_fmt = IPU_PIX_FMT_BGR24;
			else if (!strncmp(opt+3, "GBR24", 5))
				pdata->interface_pix_fmt = IPU_PIX_FMT_GBR24;
			else if (!strncmp(opt+3, "RGB565", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_RGB565;
			else if (!strncmp(opt+3, "RGB666", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_RGB666;
			else if (!strncmp(opt+3, "YUV444", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_YUV444;
			else if (!strncmp(opt+3, "LVDS666", 7))
				pdata->interface_pix_fmt = IPU_PIX_FMT_LVDS666;
			else if (!strncmp(opt+3, "YUYV16", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_YUYV;
			else if (!strncmp(opt+3, "UYVY16", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_UYVY;
			else if (!strncmp(opt+3, "YVYU16", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_YVYU;
			else if (!strncmp(opt+3, "VYUY16", 6))
				pdata->interface_pix_fmt = IPU_PIX_FMT_VYUY;
		} else if (!strncmp(opt, "fbpix=", 6)) {
			if (!strncmp(opt+6, "RGB24", 5))
				fb_pix_fmt = IPU_PIX_FMT_RGB24;
			else if (!strncmp(opt+6, "BGR24", 5))
				fb_pix_fmt = IPU_PIX_FMT_BGR24;
			else if (!strncmp(opt+6, "RGB32", 5))
				fb_pix_fmt = IPU_PIX_FMT_RGB32;
			else if (!strncmp(opt+6, "BGR32", 5))
				fb_pix_fmt = IPU_PIX_FMT_BGR32;
			else if (!strncmp(opt+6, "ABGR32", 6))
				fb_pix_fmt = IPU_PIX_FMT_ABGR32;
			else if (!strncmp(opt+6, "RGB565", 6))
				fb_pix_fmt = IPU_PIX_FMT_RGB565;

			if (fb_pix_fmt) {
				pixfmt_to_var(fb_pix_fmt, &fbi->var);
				pdata->default_bpp =
					fbi->var.bits_per_pixel;
			}
		} else if (!strncmp(opt, "int_clk", 7)) {
			pdata->int_clk = true;
			continue;
		} else if (!strncmp(opt, "bpp=", 4)) {
			/* bpp setting cannot overwirte fbpix setting */
			if (fb_pix_fmt)
				continue;

			pdata->default_bpp =
				simple_strtoul(opt + 4, NULL, 0);

			fb_pix_fmt = bpp_to_pixfmt(pdata->default_bpp);
			if (fb_pix_fmt)
				pixfmt_to_var(fb_pix_fmt, &fbi->var);
		} else
			fb_mode_str = opt;
	}

	if (fb_mode_str)
		pdata->mode_str = fb_mode_str;

	return 0;
}

static int mxcfb_register(struct fb_info *fbi)
{
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)fbi->par;
	struct fb_videomode m;
	int ret = 0;
	char bg0_id[] = "DISP3 BG";
	char bg1_id[] = "DISP3 BG - DI1";
	char fg_id[] = "DISP3 FG";

	if (mxcfbi->ipu_di == 0) {
		bg0_id[4] += mxcfbi->ipu_id;
		strcpy(fbi->fix.id, bg0_id);
	} else if (mxcfbi->ipu_di == 1) {
		bg1_id[4] += mxcfbi->ipu_id;
		strcpy(fbi->fix.id, bg1_id);
	} else { /* Overlay */
		fg_id[4] += mxcfbi->ipu_id;
		strcpy(fbi->fix.id, fg_id);
	}

	mxcfb_check_var(&fbi->var, fbi);

	mxcfb_set_fix(fbi);

	/* Added first mode to fbi modelist. */
	if (!fbi->modelist.next || !fbi->modelist.prev)
		INIT_LIST_HEAD(&fbi->modelist);
	fb_var_to_videomode(&m, &fbi->var);
	fb_add_videomode(&m, &fbi->modelist);

	if (ipu_request_irq(mxcfbi->ipu, mxcfbi->ipu_ch_irq,
		mxcfb_irq_handler, IPU_IRQF_ONESHOT, MXCFB_NAME, fbi) != 0) {
		dev_err(fbi->device, "Error registering EOF irq handler.\n");
		ret = -EBUSY;
		goto err0;
	}
	ipu_disable_irq(mxcfbi->ipu, mxcfbi->ipu_ch_irq);
	if (ipu_request_irq(mxcfbi->ipu, mxcfbi->ipu_ch_nf_irq,
		mxcfb_nf_irq_handler, IPU_IRQF_NONE, MXCFB_NAME, fbi) != 0) {
		dev_err(fbi->device, "Error registering NFACK irq handler.\n");
		ret = -EBUSY;
		goto err1;
	}
	if (ipu_request_irq(mxcfbi->ipu_scaler, mxcfbi->ipu_ch_scale_irq,
			mxcfb_scale_irq_handler, IPU_IRQF_NONE, MXCFB_NAME, fbi) != 0) {
			dev_err(fbi->device, "Error registering SCALE irq handler.\n");
			ret = -EBUSY;
			goto err1;
	}
	ipu_disable_irq(mxcfbi->ipu, mxcfbi->ipu_ch_nf_irq);



	if (!mxcfbi->late_init) {
		fbi->var.activate |= FB_ACTIVATE_FORCE;
		console_lock();
		fbi->flags |= FBINFO_MISC_USEREVENT;
		ret = fb_set_var(fbi, &fbi->var);
		fbi->flags &= ~FBINFO_MISC_USEREVENT;
		console_unlock();
		if (ret < 0) {
			dev_err(fbi->device, "Error fb_set_var ret:%d\n", ret);
			goto err3;
		}

		if (mxcfbi->next_blank == FB_BLANK_UNBLANK) {
			console_lock();
			ret = fb_blank(fbi, FB_BLANK_UNBLANK);
			console_unlock();
			if (ret < 0) {
				dev_err(fbi->device,
					"Error fb_blank ret:%d\n", ret);
				goto err4;
			}
		}
	} else {
		/*
		 * Setup the channel again though bootloader
		 * has done this, then set_par() can stop the
		 * channel neatly and re-initialize it .
		 */
		if (mxcfbi->next_blank == FB_BLANK_UNBLANK) {
			console_lock();
			_setup_disp_channel1(fbi);
			ipu_enable_channel(mxcfbi->ipu, mxcfbi->ipu_ch_disp);
			_start_scaler(fbi);
			console_unlock();
		}
	}


	ret = register_framebuffer(fbi);
	if (ret < 0)
		goto err5;

	return ret;
err5:
	if (mxcfbi->next_blank == FB_BLANK_UNBLANK) {
		console_lock();
		if (!mxcfbi->late_init)
			fb_blank(fbi, FB_BLANK_POWERDOWN);
		else {
			_stop_scaler(fbi);
			ipu_uninit_channel(mxcfbi->ipu_scaler, mxcfbi->ipu_ch_scale);
		}
		console_unlock();
	}
err4:
err3:
	ipu_free_irq(mxcfbi->ipu, mxcfbi->ipu_ch_nf_irq, fbi);
err1:
	ipu_free_irq(mxcfbi->ipu, mxcfbi->ipu_ch_irq, fbi);
	ipu_free_irq(mxcfbi->ipu_scaler, mxcfbi->ipu_ch_scale_irq, fbi);
err0:
	return ret;
}

static void mxcfb_unregister(struct fb_info *fbi)
{
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)fbi->par;

	if (mxcfbi->ipu_ch_irq)
		ipu_free_irq(mxcfbi->ipu, mxcfbi->ipu_ch_irq, fbi);
	if (mxcfbi->ipu_ch_nf_irq)
		ipu_free_irq(mxcfbi->ipu, mxcfbi->ipu_ch_nf_irq, fbi);
	if (mxcfbi->ipu_ch_scale_irq)
			ipu_free_irq(mxcfbi->ipu_scaler, mxcfbi->ipu_ch_scale_irq, fbi);

	unregister_framebuffer(fbi);
}


static bool ipu_usage[2][2];
static int ipu_test_set_usage(int ipu, int di)
{
	if (ipu_usage[ipu][di])
		return -EBUSY;
	else
		ipu_usage[ipu][di] = true;
	return 0;
}

static void ipu_clear_usage(int ipu, int di)
{
	ipu_usage[ipu][di] = false;
}

static int mxcfb_get_of_property(struct platform_device *pdev,
				struct ipuv3_fb_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	const char *disp_dev;
	const char *mode_str;
	const char *pixfmt;
	int err;
	int len;
	u32 bpp, int_clk;
	u32 late_init;

	err = of_property_read_string(np, "disp_dev", &disp_dev);
	if (err < 0) {
		dev_dbg(&pdev->dev, "get of property disp_dev fail\n");
		return err;
	}
	err = of_property_read_string(np, "mode_str", &mode_str);
	if (err < 0) {
		dev_dbg(&pdev->dev, "get of property mode_str fail\n");
	}
	err = of_property_read_string(np, "interface_pix_fmt", &pixfmt);
	if (err) {
		dev_dbg(&pdev->dev, "get of property pix fmt fail\n");
		return err;
	}
	err = of_property_read_u32(np, "default_bpp", &bpp);
	if (err) {
		dev_dbg(&pdev->dev, "get of property bpp fail\n");
		return err;
	}
	err = of_property_read_u32(np, "int_clk", &int_clk);
	if (err) {
		dev_dbg(&pdev->dev, "get of property int_clk fail\n");
		return err;
	}
	err = of_property_read_u32(np, "late_init", &late_init);
	if (err) {
		dev_dbg(&pdev->dev, "get of property late_init fail\n");
		return err;
	}

	if (!strncmp(pixfmt, "RGB24", 5))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_RGB24;
	else if (!strncmp(pixfmt, "BGR24", 5))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_BGR24;
	else if (!strncmp(pixfmt, "GBR24", 5))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_GBR24;
	else if (!strncmp(pixfmt, "RGB565", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_RGB565;
	else if (!strncmp(pixfmt, "RGB666", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_RGB666;
	else if (!strncmp(pixfmt, "YUV444", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_YUV444;
	else if (!strncmp(pixfmt, "LVDS666", 7))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_LVDS666;
	else if (!strncmp(pixfmt, "YUYV16", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_YUYV;
	else if (!strncmp(pixfmt, "UYVY16", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_UYVY;
	else if (!strncmp(pixfmt, "YVYU16", 6))
		plat_data->interface_pix_fmt = IPU_PIX_FMT_YVYU;
	else if (!strncmp(pixfmt, "VYUY16", 6))
				plat_data->interface_pix_fmt = IPU_PIX_FMT_VYUY;
	else {
		dev_err(&pdev->dev, "err interface_pix_fmt!\n");
		return -ENOENT;
	}

	len = min(sizeof(plat_data->disp_dev) - 1, strlen(disp_dev));
	memcpy(plat_data->disp_dev, disp_dev, len);
	plat_data->disp_dev[len] = '\0';
	plat_data->mode_str = (char *)mode_str;
	plat_data->default_bpp = bpp;
	plat_data->int_clk = (bool)int_clk;
	plat_data->late_init = (bool)late_init;
	return err;
}

/*!
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process.      The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, IPU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int mxcfb_probe(struct platform_device *pdev)
{
	struct ipuv3_fb_platform_data *plat_data;
	struct fb_info *fbi;
	struct mxcfb_info *mxcfbi;
	struct resource *res;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "%s enter\n", __func__);
	pdev->id = of_alias_get_id(pdev->dev.of_node, "mxcfb");
	if (pdev->id < 0) {
		dev_err(&pdev->dev, "can not get alias id\n");
		return pdev->id;
	}

	plat_data = devm_kzalloc(&pdev->dev, sizeof(struct
					ipuv3_fb_platform_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;

	ret = mxcfb_get_of_property(pdev, plat_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "get mxcfb of property fail\n");
		return ret;
	}


	/* Initialize FB structures */
	fbi = mxcfb_init_fbinfo(&pdev->dev, &mxcfb_ops);
	if (!fbi) {
		ret = -ENOMEM;
		goto init_fbinfo_failed;
	}

	ret = mxcfb_option_setup(pdev, fbi);
	if (ret)
		goto get_fb_option_failed;

	mxcfbi = (struct mxcfb_info *)fbi->par;
	ret = of_property_read_u32(np, "display_xres", &mxcfbi->out_xres);
	if (ret)
		mxcfbi->out_xres = 1024;
	ret = of_property_read_u32(np, "display_yres", &mxcfbi->out_yres);
	if (ret)
		mxcfbi->out_yres = 768;

	mxcfbi->use_db = of_property_read_bool(np, "use_double_buffer");

	mxcfbi->ipu_int_clk = plat_data->int_clk;
	mxcfbi->late_init = plat_data->late_init;
	mxcfbi->first_set_par = true;
	mxcfbi->scaler_running = false;
	mxcfbi->bounce_base[0] = mxcfbi->bounce_base[1] = 0;
	mxcfbi->bounce_phys[0] = mxcfbi->bounce_phys[1] = 0;

	ret = mxcfb_dispdrv_init(pdev, fbi);
	if (ret < 0)
		goto init_dispdrv_failed;

	ret = ipu_test_set_usage(mxcfbi->ipu_id, mxcfbi->ipu_di);
	if (ret < 0) {
		dev_err(&pdev->dev, "ipu%d-di%d already in use\n",
				mxcfbi->ipu_id, mxcfbi->ipu_di);
		goto ipu_in_busy;
	}


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && res->start && res->end) {
		fbi->fix.smem_len = res->end - res->start + 1;
		fbi->fix.smem_start = res->start;
		dev_dbg(fbi->device, "%s: IORES start 0x%08Xl, end 0x%08Xl\n", __func__, res->start, res->end);
		fbi->screen_base = ioremap(fbi->fix.smem_start, fbi->fix.smem_len);
		/* Do not clear the fb content drawn in bootloader. */
		if (!mxcfbi->late_init)
			memset(fbi->screen_base, 0, fbi->fix.smem_len);
	}

	mxcfbi->ipu = ipu_get_soc(mxcfbi->ipu_id);
	if (IS_ERR(mxcfbi->ipu)) {
		ret = -ENODEV;
		goto get_ipu_failed;
	}
#ifdef SPLIT_IPU
	mxcfbi->ipu_id_scaler = mxcfbi->ipu_id ? 0 : 1;
	mxcfbi->ipu_scaler = ipu_get_soc(mxcfbi->ipu_id_scaler);
	if (IS_ERR(mxcfbi->ipu_scaler)) {
		dev_warn(fbi->device, "%s: Could not obtain an extra IPU\n", __func__);
		mxcfbi->ipu_scaler = mxcfbi->ipu;
		mxcfbi->ipu_id_scaler = mxcfbi->ipu_id;
	}
#else
	mxcfbi->ipu_scaler = mxcfbi->ipu;
	mxcfbi->ipu_id_scaler = mxcfbi->ipu_id;
#endif


	mxcfbi->ipu_ch_irq = IPU_IRQ_BG_SYNC_EOF;
	mxcfbi->ipu_ch_nf_irq = IPU_IRQ_BG_SYNC_NFACK;
	mxcfbi->ipu_ch_scale =  MEM_PP_MEM ;
	mxcfbi->ipu_ch_disp = MEM_BG_SYNC;
	mxcfbi->ipu_ch_scale_irq =  IPU_IRQ_PP_OUT_EOF;
	/* Unblank the primary fb only by default */
	if (pdev->id == 0)
		mxcfbi->cur_blank = mxcfbi->next_blank = FB_BLANK_UNBLANK;
	else
		mxcfbi->cur_blank = mxcfbi->next_blank = FB_BLANK_POWERDOWN;

	ret = mxcfb_register(fbi);
	if (ret < 0)
		goto mxcfb_register_failed;

	ipu_disp_set_global_alpha(mxcfbi->ipu, mxcfbi->ipu_ch_disp,
				  true, 0x80);
	ipu_disp_set_color_key(mxcfbi->ipu, mxcfbi->ipu_ch_disp, false, 0);

	platform_set_drvdata(pdev, fbi);

	if (ret)
		dev_err(&pdev->dev, "Error %d on creating file for disp "
				    "property\n", ret);

	ret = device_create_file(fbi->dev, &dev_attr_fsl_disp_dev_property);
	if (ret)
		dev_err(&pdev->dev, "Error %d on creating file for disp "
				    " device propety\n", ret);

	return 0;

mxcfb_register_failed:
get_ipu_failed:
	ipu_clear_usage(mxcfbi->ipu_id, mxcfbi->ipu_di);
ipu_in_busy:
init_dispdrv_failed:
	fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
get_fb_option_failed:
init_fbinfo_failed:
	return ret;
}

static int mxcfb_remove(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = fbi->par;

	if (!fbi)
		return 0;

	device_remove_file(fbi->dev, &dev_attr_fsl_disp_dev_property);
	mxcfb_blank(FB_BLANK_POWERDOWN, fbi);
	mxcfb_unregister(fbi);
	mxcfb_unmap_video_memory(fbi);

	ipu_clear_usage(mxc_fbi->ipu_id, mxc_fbi->ipu_di);
	if (&fbi->cmap)
		fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
	return 0;
}

static const struct of_device_id imx_mxcfb_dt_ids[] = {
	{ .compatible = "fsl,mxc_scaling_fb"},
	{ /* sentinel */ }
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcfb_driver = {
	.driver = {
		.name = MXCFB_NAME,
		.of_match_table	= imx_mxcfb_dt_ids,
	},
	.probe = mxcfb_probe,
	.remove = mxcfb_remove,
	.suspend = mxcfb_suspend,
	.resume = mxcfb_resume,
};

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MXCFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
int __init mxcfbs_init(void)
{
	return platform_driver_register(&mxcfb_driver);
}

void mxcfbs_exit(void)
{
	platform_driver_unregister(&mxcfb_driver);
}

module_init(mxcfbs_init);
module_exit(mxcfbs_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_AUTHOR("DATA RESPONS AS, www.datarespons.com");
MODULE_DESCRIPTION("MXC scaling framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
