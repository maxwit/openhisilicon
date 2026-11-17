#include <stdio.h>

#include "sensor_common.h"
#include "ot_mpi_isp.h"
#include "ot_mpi_ae.h"
#include "ot_mpi_awb.h"
#include "securec.h"

#define SC4336P_I2C_ADDR 0x60
#define SC4336P_ADDR_BYTE 2
#define SC4336P_DATA_BYTE 1
#define SC4336P_ID 4336

#define SC4336P_VMAX_VAL_LINEAR 1500
#define SC4336P_4M_30FPS_10BIT_LINEAR_MODE 0
#define SC4336P_INCREASE_LINES 0
#define SC4336P_VMAX_LINEAR (SC4336P_VMAX_VAL_LINEAR + SC4336P_INCREASE_LINES)
#define SC4336P_FPS_MIN_LINEAR 2.5
#define SC4336P_FPS_MAX_LINEAR 30
#define SC4336P_WIDTH_LINEAR 2560
#define SC4336P_HEIGHT_LINEAR 1440
#define SC4336P_MODE_LINEAR 0
#define SC4336P_BLACK_LEVEL_DEFAULT 0x410

#define SC4336P_STATIC_TEMP 5000
#define SC4336P_STATIC_WB_R_GAIN 409
#define SC4336P_STATIC_WB_GR_GAIN 256
#define SC4336P_STATIC_WB_GB_GAIN 256
#define SC4336P_STATIC_WB_B_GAIN 452

#define SC4336P_AWB_P1 (36)
#define SC4336P_AWB_P2 220
#define SC4336P_AWB_Q1 0
#define SC4336P_AWB_A1 218409
#define SC4336P_AWB_B1 128
#define SC4336P_AWB_C1 (-167686)

#define SC4336P_GOLDEN_RGAIN 0
#define SC4336P_GOLDEN_BGAIN 0

/* calc by std & min fps, 0x4650 = 18000 = Min:2.5fps */
#define SC4336P_FULL_LINES_MAX_LINEAR 0x4650

static cis_info g_sc4336p_info[OT_ISP_MAX_PIPE_NUM] = { [0 ...(
	OT_ISP_MAX_PIPE_NUM - 1)]{
	.mutex = PTHREAD_MUTEX_INITIALIZER,
	.sns_id = SC4336P_ID,
	.fswdr_mode = OT_ISP_FSWDR_NORMAL_MODE,
	.quick_start_en = TD_FALSE,
	.fast_ae_en = TD_FALSE,
	.bus_info = { .i2c_dev = -1 },
	.sns_state = TD_NULL,
	.blc_clamp_info = TD_TRUE,
	.mode_tbl = { { SC4336P_VMAX_LINEAR, SC4336P_FULL_LINES_MAX_LINEAR,
			SC4336P_FPS_MAX_LINEAR, SC4336P_FPS_MIN_LINEAR,
			SC4336P_WIDTH_LINEAR, SC4336P_HEIGHT_LINEAR,
			SC4336P_MODE_LINEAR, OT_WDR_MODE_NONE,
			"SC4336P_4M_30FPS_10BIT_LINEAR_MODE" } },
	.i2c.fd = -1,
} };

static const ot_isp_cmos_black_level g_cmos_blc = {
	TD_FALSE, /* user_black_level_en */
	{
		{ 1200, 1200, 1200, 1200 },
		{ 1200, 1200, 1200, 1200 },
		{ 1200, 1200, 1200, 1200 },
		{ 1200, 1200, 1200, 1200 },
	}, /* user_black_level */
	OT_ISP_BLACK_LEVEL_MODE_AUTO, /* ot_isp_black_level_mode */
	{
		{
			{ 0x400, 0x400, 0x400, 0x400 },
			{ 0x400, 0x400, 0x400, 0x400 },
			{ 0x400, 0x400, 0x400, 0x400 },
			{ 0x400, 0x400, 0x400, 0x400 },
		},
	}, /* manual_attr */
	{
		OT_ISP_DYNAMIC_PATTERN_RGB, /* ot_isp_black_level_dynamic_pattern */
		{ 0, 0, 2592, 14 }, /* ob_area */
		0, /* low_threshold */
		4095, /* high_threshold */
		{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
		  0x0, 0x0, 0x0, 0x0 }, /* dynablc offset */
		0x0, /* dynablc tolerance */
		0x0, /* filter_strength */
		0x0, /* separate_mode */
		{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
		  1024, 1024, 1024, 1024, 1024, 1024 },
		0,
		/* calibration black level */
	}, /* dynamic_attr */
	{
		0, /* update mode 0 */
		{
			{ 0x400, 0x400, 0x400, 0x400 }, /* black level */
			{ 0x400, 0x400, 0x400, 0x400 }, /* black level */
			{ 0x400, 0x400, 0x400, 0x400 }, /* black level */
			{ 0x400, 0x400, 0x400, 0x400 }, /* black level */
		},
	},
	{
		{ 2688, 1520 },
	},
};

static cis_reg_cfg sc4336p_linear_4m30_10bit[] = {
	{ 0x0100, 0x00 },
	{ 0x36e9, 0x80 },
	{ 0x37f9, 0x80 },
	{ 0x301f, 0x01 },
	{ 0x30b8, 0x44 },
	{ 0x3253, 0x10 },
	{ 0x3301, 0x0a },
	{ 0x3302, 0xff },
	{ 0x3305, 0x00 },
	{ 0x3306, 0x90 },
	{ 0x3308, 0x08 },
	{ 0x330a, 0x01 },
	{ 0x330b, 0xb0 },
	{ 0x330d, 0xf0 },
	{ 0x3314, 0x14 },
	{ 0x3333, 0x10 },
	{ 0x3334, 0x40 },
	{ 0x335e, 0x06 },
	{ 0x335f, 0x0a },
	{ 0x3364, 0x5e },
	{ 0x337d, 0x0e },
	{ 0x338f, 0x20 },
	{ 0x3390, 0x08 },
	{ 0x3391, 0x09 },
	{ 0x3392, 0x0f },
	{ 0x3393, 0x18 },
	{ 0x3394, 0x60 },
	{ 0x3395, 0xff },
	{ 0x3396, 0x08 },
	{ 0x3397, 0x09 },
	{ 0x3398, 0x0f },
	{ 0x3399, 0x0a },
	{ 0x339a, 0x18 },
	{ 0x339b, 0x60 },
	{ 0x339c, 0xff },
	{ 0x33a2, 0x04 },
	{ 0x33ad, 0x0c },
	{ 0x33b2, 0x40 },
	{ 0x33b3, 0x30 },
	{ 0x33f8, 0x00 },
	{ 0x33f9, 0xb0 },
	{ 0x33fa, 0x00 },
	{ 0x33fb, 0xf8 },
	{ 0x33fc, 0x09 },
	{ 0x33fd, 0x1f },
	{ 0x349f, 0x03 },
	{ 0x34a6, 0x09 },
	{ 0x34a7, 0x1f },
	{ 0x34a8, 0x28 },
	{ 0x34a9, 0x28 },
	{ 0x34aa, 0x01 },
	{ 0x34ab, 0xe0 },
	{ 0x34ac, 0x02 },
	{ 0x34ad, 0x28 },
	{ 0x34f8, 0x1f },
	{ 0x34f9, 0x20 },
	{ 0x3630, 0xc0 },
	{ 0x3631, 0x84 },
	{ 0x3632, 0x54 },
	{ 0x3633, 0x44 },
	{ 0x3637, 0x49 },
	{ 0x363f, 0xc0 },
	{ 0x3641, 0x28 },
	{ 0x3670, 0x56 },
	{ 0x3674, 0xb0 },
	{ 0x3675, 0xa0 },
	{ 0x3676, 0xa0 },
	{ 0x3677, 0x84 },
	{ 0x3678, 0x88 },
	{ 0x3679, 0x8d },
	{ 0x367c, 0x09 },
	{ 0x367d, 0x0b },
	{ 0x367e, 0x08 },
	{ 0x367f, 0x0f },
	{ 0x3696, 0x24 },
	{ 0x3697, 0x34 },
	{ 0x3698, 0x34 },
	{ 0x36a0, 0x0f },
	{ 0x36a1, 0x1f },
	{ 0x36b0, 0x81 },
	{ 0x36b1, 0x83 },
	{ 0x36b2, 0x85 },
	{ 0x36b3, 0x8b },
	{ 0x36b4, 0x09 },
	{ 0x36b5, 0x0b },
	{ 0x36b6, 0x0f },
	{ 0x370f, 0x01 },
	{ 0x3722, 0x09 },
	{ 0x3724, 0x21 },
	{ 0x3771, 0x09 },
	{ 0x3772, 0x05 },
	{ 0x3773, 0x05 },
	{ 0x377a, 0x0f },
	{ 0x377b, 0x1f },
	{ 0x3905, 0x8c },
	{ 0x391d, 0x02 },
	{ 0x391f, 0x49 },
	{ 0x3926, 0x21 },
	{ 0x3933, 0x80 },
	{ 0x3934, 0x03 },
	{ 0x3937, 0x7b },
	{ 0x3939, 0x00 },
	{ 0x393a, 0x00 },
	{ 0x39dc, 0x02 },
	{ 0x3e00, 0x00 },
	{ 0x3e01, 0x5d },
	{ 0x3e02, 0x40 },
	{ 0x440d, 0x10 },
	{ 0x440e, 0x01 },
	{ 0x4509, 0x28 },
	{ 0x450d, 0x32 },
	{ 0x5000, 0x06 },
	{ 0x5780, 0x76 },
	{ 0x5784, 0x10 },
	{ 0x5785, 0x04 },
	{ 0x5787, 0x0a },
	{ 0x5788, 0x0a },
	{ 0x5789, 0x08 },
	{ 0x578a, 0x0a },
	{ 0x578b, 0x0a },
	{ 0x578c, 0x08 },
	{ 0x578d, 0x40 },
	{ 0x5790, 0x08 },
	{ 0x5791, 0x04 },
	{ 0x5792, 0x04 },
	{ 0x5793, 0x08 },
	{ 0x5794, 0x04 },
	{ 0x5795, 0x04 },
	{ 0x5799, 0x46 },
	{ 0x579a, 0x77 },
	{ 0x57a1, 0x04 },
	{ 0x57a8, 0xd2 },
	{ 0x57aa, 0x2a },
	{ 0x57ab, 0x7f },
	{ 0x57ac, 0x00 },
	{ 0x57ad, 0x00 },
	{ 0x57d9, 0x46 },
	{ 0x57da, 0x77 },
	{ 0x59e2, 0x08 },
	{ 0x59e3, 0x03 },
	{ 0x59e4, 0x00 },
	{ 0x59e5, 0x10 },
	{ 0x59e6, 0x06 },
	{ 0x59e7, 0x00 },
	{ 0x59e8, 0x08 },
	{ 0x59e9, 0x02 },
	{ 0x59ea, 0x00 },
	{ 0x59eb, 0x10 },
	{ 0x59ec, 0x04 },
	{ 0x59ed, 0x00 },
	{ 0x5ae0, 0xfe },
	{ 0x5ae1, 0x40 },
	{ 0x5ae2, 0x38 },
	{ 0x5ae3, 0x30 },
	{ 0x5ae4, 0x28 },
	{ 0x5ae5, 0x38 },
	{ 0x5ae6, 0x30 },
	{ 0x5ae7, 0x28 },
	{ 0x5ae8, 0x3f },
	{ 0x5ae9, 0x34 },
	{ 0x5aea, 0x2c },
	{ 0x5aeb, 0x3f },
	{ 0x5aec, 0x34 },
	{ 0x5aed, 0x2c },
	{ 0x36e9, 0x44 },
	{ 0x37f9, 0x44 },
	{ 0x3221, 0x0 },
};

static cis_info *cmos_get_info(ot_vi_pipe vi_pipe)
{
	if (vi_pipe < 0 || vi_pipe >= OT_ISP_MAX_PIPE_NUM) {
		return TD_NULL;
	}
	return &g_sc4336p_info[vi_pipe];
}

static ot_isp_sns_state *cmos_get_state(ot_vi_pipe vi_pipe)
{
	if (cmos_get_info(vi_pipe) != TD_NULL)
		return cmos_get_info(vi_pipe)->sns_state;
	return TD_NULL;
}

static td_s32 sc4336p_get_ae_default(ot_vi_pipe vi_pipe,
				     ot_isp_ae_sensor_default *ae_sns_dft)
{
	return TD_SUCCESS;
}

static td_void sc4336p_inttime_update(ot_vi_pipe vi_pipe, td_u32 int_time)
{
}

static td_void sc4336p_again_calc_table(ot_vi_pipe vi_pipe, td_u32 *again_lin,
					td_u32 *again_db)
{
}

static td_void sc4336p_dgain_calc_table(ot_vi_pipe vi_pipe, td_u32 *dgain_lin,
					td_u32 *dgain_db)
{
}

static td_void sc4336p_gains_update(ot_vi_pipe vi_pipe, td_u32 again,
				    td_u32 dgain)
{
}

static td_void sc4336p_init_ae_exp_function(ot_isp_ae_sensor_exp_func *exp_func)
{
	(td_void) memset_s(exp_func, sizeof(ot_isp_ae_sensor_exp_func), 0,
			   sizeof(ot_isp_ae_sensor_exp_func));
	exp_func->pfn_cmos_get_ae_default = sc4336p_get_ae_default;
	exp_func->pfn_cmos_inttime_update = sc4336p_inttime_update;
	exp_func->pfn_cmos_gains_update = sc4336p_gains_update;
	exp_func->pfn_cmos_again_calc_table = sc4336p_again_calc_table;
	exp_func->pfn_cmos_dgain_calc_table = sc4336p_dgain_calc_table;
}

static void sc4336p_default_reg_init(cis_info *cis)
{
	td_u32 i;
	td_s32 ret = TD_SUCCESS;
	ot_isp_sns_state *past_sensor = TD_NULL;

	past_sensor = cis->sns_state;
	for (i = 0; i < past_sensor->regs_info[0].reg_num; i++) {
		ret += cis_write_reg(
			&cis->i2c,
			past_sensor->regs_info[0].i2c_data[i].reg_addr,
			past_sensor->regs_info[0].i2c_data[i].data);
	}

	if (ret != TD_SUCCESS) {
		isp_err_trace("write register failed!\n");
	}
	return;
}

static td_s32 sc4336p_reg_init(cis_info *cis, cis_reg_cfg *cfg, td_u32 len)
{
	td_u32 i;

	sns_check_return(cis_write_reg(&cis->i2c, 0x0103, 0x01));
	cis_delay_ms(1); /* 1ms */

	for (i = 0; i < len; i++) {
		sns_check_return(
			cis_write_reg(&cis->i2c, cfg->addr, cfg->data));
		cfg++;
	}

	sc4336p_default_reg_init(cis);

	sns_check_return(cis_write_reg(&cis->i2c, 0x0100, 0x01));

	return TD_SUCCESS;
}

// static td_u32 sc4336p_init_cis(cis_info *cis)
// {
// 	/* disable sleep mode */
// 	sns_check_return(cis_write_reg(&cis->i2c, 0x0100, 0x01));

// 	/* avoid completely black image */
// 	sns_check_return(cis_write_reg(&cis->i2c, 0x3e01, 0x80));

// 	return TD_SUCCESS;
// }

static td_void sc4336p_init(ot_vi_pipe vi_pipe)
{
	td_s32 ret;
	cis_info *cis = TD_NULL;
	ot_isp_sns_state *sensor_state = TD_NULL;

	cis = cmos_get_info(vi_pipe);
	sensor_state = cis->sns_state;

	cis->i2c.addr = SC4336P_I2C_ADDR;
	cis->i2c.addr_byte_num = SC4336P_ADDR_BYTE;
	cis->i2c.data_byte_num = SC4336P_DATA_BYTE;

	ret = cis_i2c_init(cis);
	if (ret != TD_SUCCESS) {
		isp_err_trace("i2c init failed!\n");
		return;
	}

	td_u32 reg_num = sizeof(sc4336p_linear_4m30_10bit) /
			 sizeof(sc4336p_linear_4m30_10bit[0]);
	if (sc4336p_reg_init(cis, sc4336p_linear_4m30_10bit, reg_num) !=
	    TD_SUCCESS) {
		isp_err_trace("cis init failed!\n");
		return;
	}

	sensor_state->init = TD_TRUE;
}

static td_void sc4336p_exit(ot_vi_pipe vi_pipe)
{
	td_s32 ret;
	cis_info *cis = TD_NULL;

	sns_check_pipe_void_return(vi_pipe);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_void_return(cis);

	ret = cis_i2c_exit(cis);
	if (ret != TD_SUCCESS)
		isp_err_trace("i2c exit failed!\n");
}

static td_void sc4336p_global_init(ot_vi_pipe vi_pipe)
{
	ot_isp_sns_state *sns_state = TD_NULL;
	cis_info *cis = TD_NULL;

	sns_check_pipe_void_return(vi_pipe);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_void_return(cis);

	sns_state = cis->sns_state;
	sns_check_pointer_void_return(sns_state);

	sns_state->img_mode = SC4336P_4M_30FPS_10BIT_LINEAR_MODE;
}

static td_s32 sc4336p_get_default(ot_vi_pipe vi_pipe,
				  ot_isp_cmos_default *isp_def)
{
	return TD_SUCCESS;
}

static td_void sc4336p_set_pixel_detect(ot_vi_pipe vi_pipe, td_bool enable)
{
}

static td_s32 sc4336p_get_sns_regs_info(ot_vi_pipe vi_pipe,
					ot_isp_sns_regs_info *sns_regs_info)
{
	return TD_SUCCESS;
}

static td_s32 sc4336p_get_black_level(ot_vi_pipe vi_pipe,
				      ot_isp_cmos_black_level *black_level)
{
	td_s32 i;
	ot_isp_sns_state *sns_state = TD_NULL;

	sns_check_pipe_return(vi_pipe);
	sns_check_pointer_return(black_level);
	sns_state = cmos_get_state(vi_pipe);
	sns_check_pointer_return(sns_state);

	(td_void) memcpy_s(black_level, sizeof(ot_isp_cmos_black_level),
			   &g_cmos_blc, sizeof(ot_isp_cmos_black_level));

	/* Don't need to update black level when iso change */
	black_level->auto_attr.update = TD_FALSE;

	/* black level of linear mode */
	for (i = 0; i < OT_ISP_BAYER_CHN_NUM; i++) {
		black_level->auto_attr.black_level[0][i] =
			SC4336P_BLACK_LEVEL_DEFAULT;
	}

	return TD_SUCCESS;
}

static td_void
sc4336p_init_sensor_exp_function(ot_isp_sns_exp_func *sensor_exp_func)
{
	(td_void) memset_s(sensor_exp_func, sizeof(ot_isp_sns_exp_func), 0,
			   sizeof(ot_isp_sns_exp_func));

	sensor_exp_func->pfn_cmos_sns_init = sc4336p_init;
	sensor_exp_func->pfn_cmos_sns_exit = sc4336p_exit;
	sensor_exp_func->pfn_cmos_sns_global_init = sc4336p_global_init;
	sensor_exp_func->pfn_cmos_get_isp_default = sc4336p_get_default;
	sensor_exp_func->pfn_cmos_get_isp_black_level = sc4336p_get_black_level;
	sensor_exp_func->pfn_cmos_set_pixel_detect = sc4336p_set_pixel_detect;
	sensor_exp_func->pfn_cmos_get_sns_reg_info = sc4336p_get_sns_regs_info;
}

static td_s32 sc4336p_awb_get_default(ot_vi_pipe vi_pipe,
				      ot_isp_awb_sensor_default *awb_sns_dft)
{
	cis_info *cis = TD_NULL;
	ot_isp_sns_state *sns_state = TD_NULL;

	sns_check_pipe_return(vi_pipe);
	sns_check_pointer_return(awb_sns_dft);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_return(cis);

	sns_state = cis->sns_state;
	sns_check_pointer_return(sns_state);

	(td_void) memset_s(awb_sns_dft, sizeof(ot_isp_awb_sensor_default), 0,
			   sizeof(ot_isp_awb_sensor_default));

	awb_sns_dft->gain_offset[0] = SC4336P_STATIC_WB_R_GAIN;
	awb_sns_dft->gain_offset[1] = SC4336P_STATIC_WB_GR_GAIN;
	awb_sns_dft->gain_offset[2] = SC4336P_STATIC_WB_GB_GAIN; /* index 2 */
	awb_sns_dft->gain_offset[3] = SC4336P_STATIC_WB_B_GAIN; /* index 3 */

	return TD_SUCCESS;
}

static td_void
sc4336p_init_awb_exp_function(ot_isp_awb_sensor_exp_func *exp_func)
{
	(td_void) memset_s(exp_func, sizeof(ot_isp_awb_sensor_exp_func), 0,
			   sizeof(ot_isp_awb_sensor_exp_func));
	exp_func->pfn_cmos_get_awb_default = sc4336p_awb_get_default;
}

static td_s32 sc4336p_register_callback(ot_vi_pipe vi_pipe,
					ot_isp_3a_alg_lib *ae_lib,
					ot_isp_3a_alg_lib *awb_lib)
{
	td_s32 ret;
	cis_register reg = { 0 };
	cis_info *cis = TD_NULL;

	sns_check_pipe_return(vi_pipe);
	sns_check_pointer_return(ae_lib);
	sns_check_pointer_return(awb_lib);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_return(cis);

	cis->pipe = vi_pipe;

	reg.ae_lib = ae_lib;
	reg.awb_lib = awb_lib;

	sc4336p_init_sensor_exp_function(&reg.isp_register.sns_exp);
	sc4336p_init_ae_exp_function(&reg.ae_register.sns_exp);
	sc4336p_init_awb_exp_function(&reg.awb_register.sns_exp);

	ret = cis_register_callback(cis, &reg);
	if (ret != TD_SUCCESS) {
		isp_err_trace("cis_register_callback failed!\n");
		return ret;
	}

	return TD_SUCCESS;
}

static td_s32 sc4336p_unregister_callback(ot_vi_pipe vi_pipe,
					  ot_isp_3a_alg_lib *ae_lib,
					  ot_isp_3a_alg_lib *awb_lib)
{
	td_s32 ret;
	cis_register reg = { 0 };
	cis_info *cis = TD_NULL;

	sns_check_pipe_return(vi_pipe);
	sns_check_pointer_return(ae_lib);
	sns_check_pointer_return(awb_lib);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_return(cis);

	reg.ae_lib = ae_lib;
	reg.awb_lib = awb_lib;
	ret = cis_unregister_callback(cis, &reg);
	if (ret != TD_SUCCESS) {
		isp_err_trace("cis_register_callback failed!\n");
		return ret;
	}

	return TD_SUCCESS;
}

static td_s32 sc4336p_set_bus_info(ot_vi_pipe vi_pipe,
				   ot_isp_sns_commbus sns_bus_info)
{
	cis_info *cis = TD_NULL;

	sns_check_pipe_return(vi_pipe);

	cis = cmos_get_info(vi_pipe);
	sns_check_pointer_return(cis);

	cis->bus_info.i2c_dev = sns_bus_info.i2c_dev;

	return TD_SUCCESS;
}

ot_isp_sns_obj g_sns_sc4336p_obj = {
	.pfn_register_callback = sc4336p_register_callback,
	.pfn_un_register_callback = sc4336p_unregister_callback,
	.pfn_set_bus_info = sc4336p_set_bus_info,
};