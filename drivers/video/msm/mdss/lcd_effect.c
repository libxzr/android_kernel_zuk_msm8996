#include "lcd_effect.h"

#define TAG "[LCD_EFFECT: ]"
//#define LCDDEBUG
#ifdef LCDDEBUG
#define lcd_effect_info(fmt, ...) printk(TAG fmt, ##__VA_ARGS__);
#else
#define lcd_effect_info(fmt, ...) do {} while (0)
#endif

static void update_effect_cmds(struct lcd_effect *effect, int level)
{
	struct lcd_effect_cmd_data *effect_cmd_data = &effect->effect_cmd_data;
	struct lcd_effect_cmds *effect_cmd = effect_cmd_data->effect_cmd;
	int cmd_cnt = effect_cmd_data->cnt;
	int code_cnt ;
	int i;

	for (i = 0; i < cmd_cnt; i++) {
		code_cnt = effect_cmd[i].effect_code.cnt;
		effect_cmd[i].lcd_cmd.cmd->payload = effect_cmd[i].effect_code.code[level >= code_cnt ? code_cnt -1 : level];
	}
}

static void inline update_level(struct lcd_effect *effect, int level)
{
	effect->level = level;
}

static inline void update_mode(struct lcd_mode_data *mode_data, int index)
{
	mode_data->current_mode = index;
}

static inline int get_level(struct lcd_effect *effect)
{
	return effect->level;
}

static inline int get_effect_cmd_cnt(struct lcd_effect *effect)
{
	return effect->effect_cmd_data.cnt;
}

static inline int get_head_cmd_cnt(struct lcd_cmds *head_cmd)
{
	return head_cmd->cnt;
}

static inline struct dsi_cmd_desc *get_head_cmd(struct lcd_cmds *head_cmd)
{
	return head_cmd->cmd;
}
static inline struct lcd_cmds *get_lcd_cmd(struct lcd_effect_cmds *effect_cmd)
{
	return &effect_cmd->lcd_cmd;
}
static inline struct lcd_effect_cmds * get_effect_cmd(struct lcd_effect *effect)
{
	return effect->effect_cmd_data.effect_cmd;
}

static inline struct dsi_cmd_desc *get_effect_cmd_desc(struct lcd_effect_cmds *effect_cmd)
{
	return effect_cmd->lcd_cmd.cmd;
}
static inline struct dsi_cmd_desc * get_mode_cmd(struct lcd_mode *mode)
{
	return mode->mode_cmd.cmd;
}
static inline int get_mode_cmd_cnt(struct lcd_mode *mode)
{
	return mode->mode_cmd.cnt;
}

static int get_mode_max_cnt(struct lcd_mode_data *mode_data)
{
	int i;
	int temp;
	int cnt = 0;

	for (i = 0; i < mode_data->supported_mode; i++) {
		temp = mode_data->mode[i].mode_cmd.cnt;
		cnt = (cnt > temp) ? cnt : temp;
		lcd_effect_info("%s cnt = %d temp = %d\n", __func__, cnt, temp);
	}

	return cnt;
}

static int get_effect_max_cnt(struct lcd_effect_data *effect_data)
{
	int cnt = 0;
	int temp;
	int i;

	for (i = 0; i < effect_data->supported_effect; i++) {
		temp = effect_data->effect[i].effect_cmd_data.cnt;
		cnt = cnt + temp;
		lcd_effect_info("%s cnt = %d temp = %d\n", __func__, cnt, temp);
	}

	return cnt;
}

static int get_init_code_max_cnt(struct panel_effect_data *panel_data, struct lcd_cmds *save_cmd)
{
	int cnt = save_cmd->cnt;

	cnt += get_mode_max_cnt(panel_data->mode_data);
	cnt += get_effect_max_cnt(panel_data->effect_data);
	lcd_effect_info("%s cnt: %d\n", __func__, cnt);
	return cnt;
}

static struct dsi_cmd_desc *copy_init_code(struct panel_effect_data *panel_data, int *cnt)
{
	int init_cnt = panel_data->save_cmd.cnt;

	memcpy(panel_data->buf, panel_data->save_cmd.cmd, (init_cnt - CMDS_LAST_CNT) * sizeof (struct dsi_cmd_desc));
	*cnt += (init_cnt - CMDS_LAST_CNT);
	lcd_effect_info("%s: line=%d\n", __func__,__LINE__);
	return (panel_data->buf + (init_cnt - CMDS_LAST_CNT));
}

static struct dsi_cmd_desc *copy_sleep_out_code(
		struct panel_effect_data *panel_data, 
		struct dsi_cmd_desc *buf, 
		int *cnt)
{
	memcpy(buf, panel_data->save_cmd.cmd + panel_data->save_cmd.cnt - CMDS_LAST_CNT, CMDS_LAST_CNT * sizeof (struct dsi_cmd_desc));
	*cnt += CMDS_LAST_CNT;
	lcd_effect_info("%s: line=%d\n", __func__,__LINE__);
	return (buf + CMDS_LAST_CNT);
}

static struct dsi_cmd_desc * copy_single_effect_code(
		struct panel_effect_data *panel_data, 
		struct dsi_cmd_desc *buf, 
		int index, 
		int level,
		int *cnt)
{
	struct lcd_effect_data *effect_data = panel_data->effect_data;
	struct lcd_effect *effect = &effect_data->effect[index];
	struct dsi_cmd_desc *temp = buf;
	struct lcd_effect_cmds *effect_cmd;
	int cmd_cnt;
	int i;

	update_effect_cmds(effect, level);
	cmd_cnt = get_effect_cmd_cnt(effect);
	effect_cmd = get_effect_cmd(effect);
	*cnt += cmd_cnt;
	for (i = 0; i < cmd_cnt; i++)
		memcpy(temp++, get_effect_cmd_desc(&effect_cmd[i]), sizeof (struct dsi_cmd_desc));

	return (buf + cmd_cnt);
}

static struct dsi_cmd_desc *copy_used_effect_code(struct panel_effect_data *panel_data, struct dsi_cmd_desc *buf, int *cnt)
{
	struct dsi_cmd_desc *temp;
	struct lcd_effect_data *effect_data = panel_data->effect_data;
	struct lcd_effect *effect = effect_data->effect;

	temp = buf;
	//protect eys mode(ct level 3) is highest priority
	if((effect[EFFECT_CE].level) && (effect[EFFECT_CT].level!=3))
		temp = copy_single_effect_code(panel_data, temp, EFFECT_CE, effect[EFFECT_CE].level, cnt);
	else
		temp = copy_single_effect_code(panel_data, temp, EFFECT_CT, effect[EFFECT_CT].level, cnt);

	temp = copy_single_effect_code(panel_data, temp, EFFECT_CABC, effect[EFFECT_CABC].level, cnt);
	temp = copy_single_effect_code(panel_data, temp, EFFECT_HBM, effect[EFFECT_HBM].level, cnt);

	lcd_effect_info("%s,EFFECT_CE level:%d,EFFECT_CT level:%d,EFFECT_CABC level:%d,EFFECT_HBM level:%d\n",__func__,
	effect[EFFECT_CE].level,effect[EFFECT_CT].level,effect[EFFECT_CABC].level,effect[EFFECT_HBM].level);

	return temp;
}

int update_init_code(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		struct panel_effect_data *panel_data,
		void (*mdss_dsi_panel_cmds_send)(struct mdss_dsi_ctrl_pdata *ctrl,struct dsi_panel_cmds *pcmds,u32 flags))
{
	struct lcd_cmds lcd_cmd;
	struct dsi_cmd_desc *temp;
	struct lcd_cmds *save_cmd = &panel_data->save_cmd;
	int cnt = 0;
	int ret = 0;
	lcd_cmd.cmd = panel_data->buf;

	temp = copy_init_code(panel_data, &cnt);

	temp = copy_used_effect_code(panel_data, temp, &cnt);
	temp = copy_sleep_out_code(panel_data, temp, &cnt);

	lcd_cmd.cnt = cnt;

	ctrl_pdata->on_cmds.cmds = lcd_cmd.cmd;
	ctrl_pdata->on_cmds.cmd_cnt = lcd_cmd.cnt;
	lcd_effect_info("%s Use system param\n", __func__);

	mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->on_cmds,CMD_REQ_COMMIT);


	ctrl_pdata->on_cmds.cmds = save_cmd->cmd;
	ctrl_pdata->on_cmds.cmd_cnt = save_cmd->cnt;
	return ret;
}


int malloc_lcd_effect_code_buf(struct panel_effect_data *panel_data)
{
	struct lcd_cmds *save_cmd = &panel_data->save_cmd;
	if (panel_data->buf == NULL) {
		panel_data->buf_size = get_init_code_max_cnt(panel_data, save_cmd);
		panel_data->buf  = kmalloc(sizeof(struct dsi_cmd_desc) * panel_data->buf_size, GFP_KERNEL);
		if ( !panel_data->buf)
			return -ENOMEM;
		return 0;
	}
	return 0;
}
