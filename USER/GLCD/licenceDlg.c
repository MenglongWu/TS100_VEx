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
	unsigned int sn[4];
	unsigned int licence1;// licence分成2段
	unsigned int licence2;
	unsigned int licence;// 最终汇总到licence里，主要是为了考虑用户体验
	unsigned int month;
	unsigned int leave;
};


void OnCreate(struct gl_widget *widget_list, struct dlg *pwin)
{
	struct gl_widget *plist = widget_list;
	unsigned long lic[4];

	pwin->licence1 = 0;
	pwin->licence2 = 0;
	pwin->month = 11;
	pwin->sn[0] = 2;
	pwin->sn[1] = 2;
	pwin->sn[2] = 2;
	pwin->sn[3] = 2;
	pwin->leave = 3;
	

	lc_GetChipID(&pwin->sn[0]);
	sprintf(widget_list[3].caption, "%8.8x %8.8x %8.8x %8.8x",
			pwin->sn[0],pwin->sn[1],pwin->sn[2],pwin->sn[3]);

	lc_GetChipMonth(&pwin->month);
	sprintf(widget_list[8].caption, "%2.2d",
			pwin->month);
	

	lc_GetChipleave(&pwin->leave);
	sprintf(widget_list[9].caption, "Timeout %4.4d day",
				pwin->leave);
	switch (lc_IsLicence()) {
	case 0:
		sprintf(widget_list[1].caption, "no sn");
		break;
	case 1:
		sprintf(widget_list[1].caption, "sn success");
		break;

	}





	while(plist->id != 0) {
		gl_text(plist->x, plist->y, plist->caption, -1);
		plist++;
	}
}

void OnPaint(struct gl_widget *widget_list, struct dlg *val)
{
	struct gl_widget *plist = widget_list;

	gl_fill_rect(0,0,320,240);
	while(plist->id != 0) {
		gl_text(plist->x, plist->y, plist->caption, -1);
		plist++;
	}

}
void DrawText(struct gl_widget *widget_list, char *str)
{
	gl_text(widget_list->x, widget_list->y, str, -1);
}
#define GUI_NEXT_FOCUS 1
#define GUI_PARE_FOCUS 0
int NewFocus(struct gl_widget *widget_list,int curIndex, int dir)
{
	int i, len;
	char strout [33];

	// 遍历widget数
	len = 0;
	while(widget_list[len].id != 0) {
		len++;
	}
	if (dir == GUI_NEXT_FOCUS) {
		i = curIndex +1;
		// 跳过 widget_list 末端最后一个对象全 0 
		while(i < len ) {
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
		for (i = len ; i >= curIndex; i--) {
			if (widget_list[i].property & GUI_EN_FOCUSE) {
				goto _End;
			}	
		}
	}
_End:;

	return i;
}
#define VK_NULL 0
#define VK_A	'A'
#define VK_B	'B'
#define VK_C	'C'
#define VK_X	'X'
#define VK_Y	'Y'
#define VK_Z	'Z'

int GetKeyEvent(struct gl_msg *msg)
{
	if (KeyPress(GPIOA, KEY_A) ) {
		msg->wparam = VK_A | 0x00010000;
		return 1;
	}
	else if (KeyPress(GPIOA, KEY_B) ) {
		msg->wparam = VK_B | 0x00010000;
		return 1;
	}
	else if (KeyPress(GPIOA, KEY_C) ) {
		msg->wparam = VK_C | 0x00010000;
		return 1;
	}
	// else if (KeyPress(GPIOA, KEY_X) ) {
	// 	msg->wparam = VK_X | 0x00010000;
	// 	return 1;
	// }
	else if (KeyPress(GPIOA, KEY_Y) ) {
		msg->wparam = VK_Y | 0x00010000;
		return 1;
	}
	else if (KeyPress(GPIOA, KEY_Z) ) {
		msg->wparam = VK_Z | 0x00010000;
		return 1;
	}
	return 0;
}
int GetEvent(struct gl_widget *wdiget_list, struct gl_msg *msg)
{
	char strout[112];
	if (msg->msg_idnext != GUI_WM_UNUSE) {
		msg->msg_id = msg->msg_idnext;
		msg->msg_idnext = GUI_WM_UNUSE;
		return 1;
	}
	// Enable Key event
	if (GetKeyEvent(msg) ) {
		msg->msg_id = GUI_WM_KEYDOWN;
		return 1;
	}
	// Enable Touch event
	else if (0) {

	}
	msg->msg_id = GUI_WM_UNUSE;
	return 0;
}

int SendMsg(
	int (*cb)(struct gl_widget *, struct gl_msg *, struct dlg *),
	struct gl_widget *widget_list, struct gl_msg *msg, struct dlg *pwin)
{
	cb(widget_list,msg,pwin);
}

int PostMsg(
	struct gl_widget *widget_list, struct gl_msg *msg, struct dlg *pwin, uint32_t id)
{
	msg->msg_idnext = id;
}

#define GetDlgID(widget_list) (widget_list[msg->focus].id)

static int _cb_Window(struct gl_widget *widget_list, struct gl_msg *msg, struct dlg *pwin)
{
	char strout[200];
	int rlen;
	char input[40];
	int ret;

	switch (msg->msg_id) {
	case GUI_WM_CREATE:
		OnCreate(widget_list, pwin);
		// gl_text(0,0,widget_list[3].caption,-1);
		DrawFocus(-1,-1,0);
		msg->focus = 0;
		msg->focus = NewFocus(widget_list,  0,  GUI_NEXT_FOCUS);
		PostMsg(widget_list, msg, pwin,GUI_WM_PAINT);
		break;
	case GUI_WM_PAINT:
		OnPaint(widget_list, pwin);
		DrawFocus(widget_list[msg->focus].x,widget_list[msg->focus].y, 0xff00ff);
		//msg = GUI_WM_PAINT;
		// msg->msg_id = GUI_WM_PAINT;
		break;
	case GUI_WM_KEYDOWN:
		if( (msg->wparam & 0xffff) == VK_A) {
			msg->focus = NewFocus(widget_list, /*12,*/ msg->focus,  GUI_PARE_FOCUS);
			PostMsg(widget_list, msg, pwin,GUI_WM_FOCUS);
		}
		else if( (msg->wparam & 0xffff) == VK_C) {
			msg->focus = NewFocus(widget_list, /*12,*/ msg->focus, GUI_NEXT_FOCUS);
			PostMsg(widget_list, msg, pwin,GUI_WM_FOCUS);
		}

		// 
		if (GUI_ID_EDIT1 == GetDlgID(widget_list) && 
			(msg->wparam & 0xffff) == VK_Z) {
			InputPanel(input, 7,&rlen);

			pwin->licence1 = atof_(input);
			sprintf(widget_list[5].caption, "%6.6d", pwin->licence1);
			//sprintf(strout, "gogogo %c %c %d", msg->wparam & 0xffff, VK_A, rlen);
			// gl_text(0,20,strout,-1);
			PostMsg(widget_list, msg, pwin,GUI_WM_PAINT);
			pwin->licence = pwin->licence1 * 1000000 + pwin->licence2;
		}
		else if (GUI_ID_EDIT2 == GetDlgID(widget_list) && 
			(msg->wparam & 0xffff) == VK_Z) {
			InputPanel(input, 7,&rlen);

			pwin->licence2 = atof_(input);
			sprintf(widget_list[6].caption, "%6.6d", pwin->licence2);
			
			PostMsg(widget_list, msg, pwin,GUI_WM_PAINT);
			pwin->licence = pwin->licence1 * 1000000 + pwin->licence2;
		}
		else if (GUI_ID_EDIT3 == GetDlgID(widget_list) && 
			(msg->wparam & 0xffff) == VK_Z) {
			InputPanel(input, 3,&rlen);

			pwin->month = atof_(input);
			lc_CheckMonth(&pwin->month);
			sprintf(widget_list[8].caption, "%2.2d", pwin->month);

			PostMsg(widget_list, msg, pwin,GUI_WM_PAINT);
			pwin->licence = pwin->licence1 * 1000000 + pwin->licence2;
		}
		else if (GUI_ID_OK == GetDlgID(widget_list) && 
			(msg->wparam & 0xffff) == VK_Z) {
			pwin->licence = pwin->licence1 * 1000000 + pwin->licence2;
			ret = lc_InputLicence(&pwin->licence, &pwin->month );
			switch( ret ) {
			case 0:
				sprintf(widget_list[1].caption, "-------OK--------");
				break;
			case 1:
				sprintf(widget_list[1].caption, "Licence error    ");
				break;
			case 2:
				sprintf(widget_list[1].caption, "Have been licence");
				break;
			}
			PostMsg(widget_list, msg, pwin,GUI_WM_PAINT);
		}
		else if (GUI_ID_CANCEL == GetDlgID(widget_list) && 
			(msg->wparam & 0xffff) == VK_Z) {
			PostMsg(widget_list, msg, pwin,GUI_WM_QUIT);
		}


		// sprintf(strout, "%d", pwin->licence);
		// gl_text(0,10,strout,-1);
		break;
	case GUI_WM_FOCUS:
		DrawFocus(widget_list[msg->focus].x,widget_list[msg->focus].y, 0xff00ff);
		break;
	default:
		break;
	}
	return 0;
}

struct af
{
	char *p;
};

void UI_LicenceDlg()
{
	struct gl_widget widget_list[] = 
	{
		{GUI_ID_STATIC,151- 20,41, "LICENCE"},		//licence 标题
		{GUI_ID_STATIC,55- 20,63,	"message                                    "},		//message
		{GUI_ID_STATIC,55- 20,84,	"SN"},		//SN
		{GUI_ID_STATIC,107- 20,84,	"00000000 00000000 00000000 00000000"},		//SN val
		{GUI_ID_STATIC,55- 20,108,	"Licence"},		//licence
		{GUI_ID_EDIT1,107- 20,108,	"000000",GUI_EN_FOCUSE},		//licence val
		{GUI_ID_EDIT2,135,108,	"000000",GUI_EN_FOCUSE},		//licence val
		{GUI_ID_STATIC,55- 20,129,	"Month"},		//month
		{GUI_ID_EDIT3,107- 20,129,	"00",GUI_EN_FOCUSE},		//month val
		{GUI_ID_STATIC,206- 20,129,	"Timeout 0000 day"},		//timeout xxx day
		{GUI_ID_OK,123- 20,150,	"ok",GUI_EN_FOCUSE},		//ok
		{GUI_ID_CANCEL,185- 20,150,	"cancle",GUI_EN_FOCUSE},		//cancle
		{0,0,0,(int8_t*)NULL},
	};
	char str_msg[]       = "message                                    ";
	char str_sn_val[]    = "00000000 00000000 00000000 00000000";
	char str_lic_val1[]   = "000000";
	char str_lic_val2[]   = "000000";
	char str_month_val[] = "00";
	char str_timeout[]   = "Timeout 0000 day";

	struct dlg windlg;
	char strout[256];
	struct gl_msg msg;
	int16_t oldpen,oldbrush,oldbk,oldfont;	
	int focus = 0;
	int i = 0;



	(char*)widget_list[1].caption = str_msg;
	(char*)widget_list[3].caption = str_sn_val;
	(char*)widget_list[5].caption = str_lic_val1;
	(char*)widget_list[6].caption = str_lic_val2;
	(char*)widget_list[8].caption = str_month_val;
	(char*)widget_list[9].caption = str_timeout;

	
	
	oldfont  = gl_ui_setfontcolor(COL_Black);
	oldpen   = gl_ui_setpencolor(COL_Black);
	oldbrush = gl_ui_setbrushcolor(COL_White);
	oldbk    = gl_ui_setbkcolor(COL_White);

	msg.msg_id = GUI_WM_CREATE;

	DrawFocus(-1,-1,0);
	focus = NewFocus(widget_list, /*sizeof(widget_list) / sizeof (struct gl_widget),*/ 0,  GUI_NEXT_FOCUS);
	
	msg.msg_id = GUI_WM_CREATE;
	SendMsg(_cb_Window, widget_list,  &msg, &windlg);
	
	sprintf(widget_list[3].caption, "%8.8x %8.8x %8.8x %8.8x",
		windlg.sn[0],windlg.sn[1],windlg.sn[2],windlg.sn[3]);
	while(GUI_WM_QUIT != msg.msg_id) {
		GetEvent(widget_list, &msg);
		SendMsg(_cb_Window, widget_list,  &msg, &windlg);
		// sprintf(strout, "afss %d %8.8x %d %d %u", i, msg.wparam, msg.msg_id, msg.msg_idnext, windlg.licence);
		// i++;
		// gl_text(0,30,strout,-1);
	}
_End:;
	gl_ui_setfontcolor(oldfont);
	gl_ui_setpencolor(oldpen);
	gl_ui_setbrushcolor(oldbrush);
	gl_ui_setbkcolor(oldbk);
}