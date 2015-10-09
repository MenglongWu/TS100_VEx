#include "GLCD.h" 
#include "HzLib.h"
#include "AsciiLib.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_dac.h"
#include "prj_type.h"
#include "LCD\\gl_ui.h"
#include "LCD\\gl_type.h"
#include "project.h"
#include "key.h"
#include "lib\\base.h"
//#include <PictureData.h>
#include "..\\GLCD\\PictureData.h"
#include "USER/GLCD/TS100_Draw.h"
#include "flash.h"

#include "LCD/gl_widget.h"

struct dlg
{
	int msg;
	int sn[4];
	int licence;	
};


void OnCreate(struct gl_widget *list)
{

	while(list->id != 0) {
		gl_text(list->x, list->y, list->caption, -1);
		list++;
	}
}

void OnPaint(struct gl_widget *list, struct dlg *val)
{

	while(list->id != 0) {
		gl_text(list->x, list->y, list->caption, -1);
		list++;
	}
}
void DrawText(struct gl_widget *list, char *str)
{
	gl_text(list->x, list->y, str, -1);
}
#define GUI_NEXT_FOCUS 1
#define GUI_PARE_FOCUS 0
int NewFocus(struct gl_widget *widget_list, int len,int curIndex, int dir)
{
	int i;
	char strout [33];

	// 往下下一个
	if (dir == GUI_NEXT_FOCUS) {
		i = curIndex +1;
		// 跳过 widget_list 末端最后一个对象全 0 
		while(i < len - 1) {
			if (widget_list[i].property & GUI_EN_FOCUSE) {
				goto _End;
			}
			i++;
		}
		i = 0;
		while (i <= curIndex) {
			if (widget_list[i].property & GUI_EN_FOCUSE) {
				goto _End;
			}	
			i++;
		}
	}
	// 往上遍历
	else {
		for (i = curIndex - 1; i >= 0; i--) {
			if (widget_list[i].property & GUI_EN_FOCUSE) {
				goto _End;
			}
		}
		for (i = len - 1; i >= curIndex; i--) {
			if (widget_list[i].property & GUI_EN_FOCUSE) {
				goto _End;
			}	
		}
	}
_End:;
	sprintf(strout ,"index = %d %d", i,len);
	gl_text(0,3,strout, -1);
	return i;
}

int GetEvent()
{
	
}
void UI_LicenceDlg()
{
	struct gl_widget widget_list[] = 
	{
		{GUI_ID_STATIC,151- 20,41, "Licence"},		//licence 标题
		{GUI_ID_STATIC,55- 20,63,	"message                                    "},		//message
		{GUI_ID_STATIC,55- 20,84,	"SN"},		//SN
		{GUI_ID_STATIC,107- 20,84,	"00000000 00000000 00000000 00000000"},		//SN val
		{GUI_ID_STATIC,55- 20,108,	"licence"},		//licence
		{GUI_ID_EDIT2,107- 20,108,	"000 000 000 000",GUI_EN_FOCUSE},		//licence val
		{GUI_ID_STATIC,55- 20,129,	"month"},		//month
		{GUI_ID_EDIT3,107- 20,129,	"00",GUI_EN_FOCUSE},		//month val
		{GUI_ID_STATIC,206- 20,129,	"timeout 0000 day"},		//timeout xxx day
		{GUI_ID_OK,123- 20,150,	"ok",GUI_EN_FOCUSE},		//ok
		{GUI_ID_CANCEL,185- 20,150,	"cancle",GUI_EN_FOCUSE},		//cancle
		{0,0,0,(int8_t*)NULL},
	};
	char *pstr_msg  = (char*)widget_list[1].caption;
	char *pstr_sn_val  = (char*)widget_list[3].caption;
	char *pstr_lic_val  = (char*)widget_list[5].caption;
	char *pstr_month_val  = (char*)widget_list[7].caption;
	char *pstr_timeout  = (char*)widget_list[7].caption;
	struct dlg windlg;
	char strout[256];
	int msg;
	int16_t oldpen,oldbrush,oldbk,oldfont;	
	int focus = 0;

	oldfont = gl_ui_setfontcolor(COL_Black);
	oldpen = gl_ui_setpencolor(COL_Black);
	oldbrush =  gl_ui_setbrushcolor(COL_White);
	oldbk = gl_ui_setbkcolor(COL_White);

	msg = GUI_WM_CREATE;

	DrawFocus(-1,-1,0);
	
	while(GUI_WM_QUIT != msg) {
		if(KeyPress(GPIOA,KEY_A)) {
			focus = NewFocus(widget_list, sizeof(widget_list) / sizeof (struct gl_widget), focus,  GUI_PARE_FOCUS);
			msg = GUI_WM_FOCUS;
		}
		else if(KeyPress(GPIOA,KEY_C)) {
			focus = NewFocus(widget_list, sizeof(widget_list) / sizeof (struct gl_widget), focus, GUI_NEXT_FOCUS);
			msg = GUI_WM_FOCUS;
		}
		switch (msg) {
		case GUI_WM_CREATE:
			OnCreate(widget_list);
			DrawFocus(widget_list[0].x,widget_list[0].y, 0xff0000);
			DrawFocus(widget_list[1].x,widget_list[1].y, 0xff0000);
			DrawFocus(widget_list[2].x,widget_list[2].y, 0xff0000);
			windlg.sn[4] = 0x33445566;
			windlg.sn[2] = 0x00000000;
			windlg.licence = 1234567;
			msg = GUI_WM_PAINT;
			break;
		case GUI_WM_PAINT:
			// OnPaint(widget_list, &windlg);
			//msg = GUI_WM_PAINT;
			msg = GUI_WM_PAINT;
			break;
		case GUI_WM_KEYDOWN:
			break;
		case GUI_WM_FOCUS:
			DrawFocus(widget_list[focus].x,widget_list[focus].y, 0xff0000);
			msg = GUI_WM_PAINT;
			break;
		default:
			break;
		}
	}
_End:;
	gl_ui_setfontcolor(oldfont);
	gl_ui_setpencolor(oldpen);
	gl_ui_setbrushcolor(oldbrush);
	gl_ui_setbkcolor(oldbk);
}