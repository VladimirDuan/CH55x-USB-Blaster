// USB-Blaster instance on CH55x MCU.
// Author: Duan
// License: MIT
// Based on USB-MIDI by Zhiyuan Wan

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>

#include "ftdi.h"

#define BTN_PIN 2 //P3.2 for bootloader entry.
SBIT(BTN, 0xB0, BTN_PIN);

SBIT(LED, 0x90, 1);
/*
SBIT(TMS, 0x90, 4);
SBIT(TCK, 0x90, 5);
SBIT(TDI, 0x90, 6);
SBIT(TDO, 0x90, 7);
*/

SBIT(TMS, 0x90, 4);
SBIT(TCK, 0x90, 7);
SBIT(TDI, 0x90, 5);
SBIT(TDO, 0x90, 6);

SBIT(P2B7, 0xA0, 7);
SBIT(P2B6, 0xA0, 6);
SBIT(P2B5, 0xA0, 5);
SBIT(P2B4, 0xA0, 4);
SBIT(P2B3, 0xA0, 3);
SBIT(P2B2, 0xA0, 2);
SBIT(P2B1, 0xA0, 1);
SBIT(P2B0, 0xA0, 0);


__xdata __at(0x0000) uint8_t transmit_buffer[128];	//fixed address for ringbuf
__xdata __at(0x0080) uint8_t receive_buffer[64];
__xdata __at(0x0100) uint8_t Ep0Buffer[0x08]; //端点0 OUT&IN缓冲区，必须是偶地址
__xdata __at(0x0140) uint8_t Ep1Buffer[0x40]; //端点1 IN缓冲区
__xdata __at(0x0180) uint8_t Ep2Buffer[0x40]; //端点2 OUT缓冲区,必须是偶地址


uint16_t SetupLen;
uint8_t SetupReq, Count, UsbConfig;
uint8_t vendor_control;
uint8_t send_dummy;

const uint8_t *pDescr;	 //USB配置标志
USB_SETUP_REQ SetupReqBuf; //暂存Setup包
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

__code uint8_t ftdi_rom[] = {
	0x00, 0x00, 0xfb, 0x09, 0x01, 0x60, 0x00, 0x04,
	0x80, 0xe1, 0x1c, 0x00, 0x00, 0x02, 0x94, 0x0e,
	0xa2, 0x18, 0xba, 0x12, 0x0e, 0x03, 0x41, 0x00,
	0x6c, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00,
	0x61, 0x00, 0x18, 0x03, 0x55, 0x00, 0x53, 0x00,
	0x42, 0x00, 0x2d, 0x00, 0x42, 0x00, 0x6c, 0x00,
	0x61, 0x00, 0x73, 0x00, 0x74, 0x00, 0x65, 0x00,
	0x72, 0x00, 0x12, 0x03, 0x43, 0x00, 0x30, 0x00,
	0x42, 0x00, 0x46, 0x00, 0x41, 0x00, 0x36, 0x00,
	0x44, 0x00, 0x37, 0x00, 0x02, 0x03, 0x01, 0x00,
	0x52, 0x45, 0x56, 0x42, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb5, 0xb2};

__code uint8_t DevDesc[] = {
	0x12, 0x01, 0x00, 0x02,
	0x00, 0x00, 0x00, 0x08,
	0xFB, 0x09, 0x01, 0x60, 0x00, 0x04, 0x01, 0x02, 0x03, /* VID PID bString */
	0x01};

__code uint8_t CfgDesc[] = {
	0x09, 0x02, sizeof(CfgDesc) & 0xff, sizeof(CfgDesc) >> 8,
	0x01, 0x01, 0x00, 0x80, 0xe1,
	/* Interface Descriptor */
	0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,
	/* Endpoint Descriptor */
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x01, //EP1_IN
	0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x01, //EP2_OUT
};

/* USB String Descriptors (optional) */
unsigned char __code LangDes[] = {0x04, 0x03, 0x09, 0x04}; // EN_US
unsigned char __code SerDes[] = {
	//TODO: variable SN.
	sizeof(SerDes), 0x03,
	'C', 0, '0', 0, 'B', 0, 'F', 0, 'A', 0, '6', 0, 'D', 0, '7', 0 /* "C0BFA6D7" */
};
unsigned char __code Prod_Des[] = {
	sizeof(Prod_Des),
	0x03,
	'U', 0, 'S', 0, 'B', 0, '-', 0, 'B', 0, 'l', 0, 'a', 0, 's', 0, 't', 0, 'e', 0, 'r', 0 /* "USB-Blaster" */
};
unsigned char __code Manuf_Des[] = {
	sizeof(Manuf_Des),
	0x03,
	'A', 0, 'l', 0, 't', 0, 'e', 0, 'r', 0, 'a', 0 /* Manufacturer: "Altera" */
};

volatile __idata uint8_t USBByteCount = 0;   //代表USB端点接收到的数据
volatile __idata uint8_t USBBufOutPoint = 0; //取数据指针
volatile __idata uint16_t sof_count = 0;
volatile __idata uint8_t ep1_in_busy = 0; //上传端点是否忙标志
volatile __idata uint8_t latency_timer = 4;

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description	: USB设备模式配置
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceCfg()
{
	USB_CTRL = 0x00;									   //清空USB控制寄存器
	USB_CTRL &= ~bUC_HOST_MODE;							   //该位为选择设备模式
	USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; //USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK
	USB_DEV_AD = 0x00;									   //设备地址初始化
	//	 USB_CTRL |= bUC_LOW_SPEED;
	//	 UDEV_CTRL |= bUD_LOW_SPEED;												//选择低速1.5M模式
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED; //选择全速12M模式，默认方式
	UDEV_CTRL = bUD_PD_DIS;		 // 禁止DP/DM下拉电阻
	UDEV_CTRL |= bUD_PORT_EN;	//使能物理端口
}
/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description	: USB设备模式中断初始化
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceIntCfg()
{
	USB_INT_EN |= bUIE_SUSPEND;  //使能设备挂起中断
	USB_INT_EN |= bUIE_TRANSFER; //使能USB传输完成中断
	USB_INT_EN |= bUIE_BUS_RST;  //使能设备模式USB总线复位中断
	USB_INT_EN |= bUIE_DEV_SOF;	 //For timeout count.
	USB_INT_FG |= 0x1F;			 //清中断标志
	IE_USB = 1;					 //使能USB中断
	EA = 1;						 //允许单片机中断
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description	: USB设备模式端点配置，模拟兼容HID设备，除了端点0的控制传输，还包括端点2批量上下传
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceEndPointCfg()
{
	UEP1_DMA = (uint16_t)Ep1Buffer; //端点1 IN数据传输地址
	UEP2_DMA = (uint16_t)Ep2Buffer; //端点2 OUT数据传输地址
	UEP2_3_MOD = 0x08;
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK; //端点2自动翻转同步标志位，OUT返回ACK
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //端点1自动翻转同步标志位，IN事务返回NAK
	UEP0_DMA = (uint16_t)Ep0Buffer;			   //端点0数据传输地址
	UEP4_1_MOD = 0x40;						   //端点1上传缓冲区；端点0单64字节收发缓冲区
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; //手动翻转，OUT事务返回ACK，IN事务返回NAK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description	: CH55xUSB中断处理函数
*******************************************************************************/
void DeviceInterrupt(void) __interrupt(INT_NO_USB) //USB中断服务程序,使用寄存器组1
{
	uint16_t len;
	if ((USB_INT_ST & MASK_UIS_TOKEN) == UIS_TOKEN_SOF)
	{
		sof_count++;
	}
	if (UIF_TRANSFER) //USB传输完成标志
	{
		switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
		{
		case UIS_TOKEN_IN | 1: //endpoint 1
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
			ep1_in_busy = 0;
			break;
		case UIS_TOKEN_OUT | 2: //endpoint 2
		{
			if (U_TOG_OK) // 不同步的数据包将丢弃
			{
				USBByteCount = USB_RX_LEN;
				USBBufOutPoint = 0;										 //取数据指针复位
				UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
			}
			break;
		}
		break;
		case UIS_TOKEN_SETUP | 0: //SETUP事务
			len = USB_RX_LEN;
			if (len == (sizeof(USB_SETUP_REQ)))
			{
				uint8_t addr;
				SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
				len = 0;			// 默认为成功并且上传0长度
				vendor_control = 0; //默认非vendor
				SetupReq = UsbSetupBuf->bRequest;
				if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_VENDOR)
				{
					vendor_control = 1;
					if (SetupLen == 0)
					{
						//No Data
						switch (SetupReq)
						{
						case FTDI_VEN_REQ_RESET:
							break;
						case FTDI_VEN_REQ_SET_BAUDRATE:
							break;
						case FTDI_VEN_REQ_SET_DATA_CHAR:
							break;
						case FTDI_VEN_REQ_SET_FLOW_CTRL:
							break;
						case FTDI_VEN_REQ_SET_MODEM_CTRL:
							break;
						default:
							break;
						}
					}
					else
					{
						//Data
						switch (SetupReq)
						{
						case FTDI_VEN_REQ_RD_EEPROM:
							addr = UsbSetupBuf->wIndexL << 1; //((req->wIndex >> 8) & 0x3F) << 1;
							Ep0Buffer[0] = ftdi_rom[addr];
							Ep0Buffer[1] = ftdi_rom[addr + 1];
							len = 2;
							break;
						case FTDI_VEN_REQ_GET_MODEM_STA:
							// return fixed modem status
							Ep0Buffer[0] = FTDI_MODEM_STA_DUMMY0;
							Ep0Buffer[1] = FTDI_MODEM_STA_DUMMY1;
							len = 2;
							break;
						case FTDI_VEN_REQ_SET_LAT_TIMER:
							latency_timer = UsbSetupBuf->wValueL;
							len = 0;
							break;
						default:
							// return dummy data
							Ep0Buffer[0] = 0x0;
							Ep0Buffer[1] = 0x0;
							len = 2;
							break;
						}
					}
				}
				else if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
				{
					switch (SetupReq) //请求码
					{
					case USB_GET_DESCRIPTOR:
						switch (UsbSetupBuf->wValueH)
						{
						case 1:				  //设备描述符
							pDescr = DevDesc; //把设备描述符送到要发送的缓冲区
							len = sizeof(DevDesc);
							break;
						case 2:				  //配置描述符
							pDescr = CfgDesc; //把设备描述符送到要发送的缓冲区
							len = sizeof(CfgDesc);
							break;
						case 3:
							if (UsbSetupBuf->wValueL == 0)
							{
								pDescr = LangDes;
								len = sizeof(LangDes);
							}
							else if (UsbSetupBuf->wValueL == 1)
							{
								pDescr = Manuf_Des;
								len = sizeof(Manuf_Des);
							}
							else if (UsbSetupBuf->wValueL == 2)
							{
								pDescr = Prod_Des;
								len = sizeof(Prod_Des);
							}
							else
							{
								pDescr = SerDes;
								len = sizeof(SerDes);
							}
							break;
						default:
							len = 0xff; //不支持的命令或者出错
							break;
						}
						if (SetupLen > len)
						{
							SetupLen = len; //限制总长度
						}
						len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen; //本次传输长度
						memcpy(Ep0Buffer, pDescr, len);										  //加载上传数据
						SetupLen -= len;
						pDescr += len;
						break;
					case USB_SET_ADDRESS:
						SetupLen = UsbSetupBuf->wValueL; //暂存USB设备地址
						break;
					case USB_GET_CONFIGURATION:
						Ep0Buffer[0] = UsbConfig;
						if (SetupLen >= 1)
						{
							len = 1;
						}
						break;
					case USB_SET_CONFIGURATION:
						UsbConfig = UsbSetupBuf->wValueL;
						break;
					case USB_GET_INTERFACE:
						break;
					case USB_CLEAR_FEATURE:												//Clear Feature
						if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_DEVICE) /* 清除设备 */
						{
							if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
							{
								if (CfgDesc[7] & 0x20)
								{
									/* 唤醒 */
								}
								else
								{
									len = 0xFF; /* 操作失败 */
								}
							}
							else
							{
								len = 0xFF; /* 操作失败 */
							}
						}
						else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // 端点
						{
							switch (UsbSetupBuf->wIndexL)
							{
							case 0x02:
								UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
								break;
							case 0x81:
								UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
								break;
							default:
								len = 0xFF; // 不支持的端点
								break;
							}
							ep1_in_busy = 0;
						}
						else
						{
							len = 0xFF; // 不是端点不支持
						}
						break;
					case USB_SET_FEATURE:												/* Set Feature */
						if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_DEVICE) /* 设置设备 */
						{
							if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
							{
								if (CfgDesc[7] & 0x20)
								{
									/* 休眠 */
#ifdef DE_PRINTF
									printf("suspend\r\n"); //睡眠状态
#endif
									while (XBUS_AUX & bUART0_TX)
									{
										; //等待发送完成
									}
									SAFE_MOD = 0x55;
									SAFE_MOD = 0xAA;
									WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; //USB或者RXD0/1有信号时可被唤醒
									PCON |= PD;											   //睡眠
									SAFE_MOD = 0x55;
									SAFE_MOD = 0xAA;
									WAKE_CTRL = 0x00;
								}
								else
								{
									len = 0xFF; /* 操作失败 */
								}
							}
							else
							{
								len = 0xFF; /* 操作失败 */
							}
						}
						else if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_ENDP) /* 设置端点 */
						{
							if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
							{
								switch (((uint16_t)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
								{
								case 0x82:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
									break;
								case 0x02:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
									break;
								case 0x81:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
									break;
								case 0x01:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点1 OUT Stall */
								default:
									len = 0xFF; /* 操作失败 */
									break;
								}
							}
							else
							{
								len = 0xFF; /* 操作失败 */
							}
						}
						else
						{
							len = 0xFF; /* 操作失败 */
						}
						break;
					case USB_GET_STATUS:
						Ep0Buffer[0] = 0x00;
						Ep0Buffer[1] = 0x00;
						if (SetupLen >= 2)
						{
							len = 2;
						}
						else
						{
							len = SetupLen;
						}
						break;
					default:
						len = 0xff; //操作失败
						break;
					}
				}
				else
				{
					switch (SetupReq)
					{

					default:
						len = 0xFF; /*命令不支持*/
						break;
					}
				}
			}
			else
			{
				len = 0xff; //包长度错误
			}
			if (len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
			}
			else if (len <= DEFAULT_ENDP0_SIZE) //上传数据或者状态阶段返回0长度包
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1，返回应答ACK
			}
			else
			{
				UEP0_T_LEN = 0;														 //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1,返回应答ACK
			}
			break;
		case UIS_TOKEN_IN | 0: //endpoint0 IN
			switch (SetupReq)
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen; //本次传输长度
				memcpy(Ep0Buffer, pDescr, len);										  //加载上传数据
				SetupLen -= len;
				pDescr += len;
				UEP0_T_LEN = len;
				UEP0_CTRL ^= bUEP_T_TOG; //同步标志位翻转
				break;
			case USB_SET_ADDRESS:
				if (!vendor_control)
				{
					USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				}
				break;
			default:
				UEP0_T_LEN = 0; //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;
		case UIS_TOKEN_OUT | 0: // endpoint0 OUT
								/*if(SetupReq ==SET_LINE_CODING)  //设置串口属性
			{
				if( U_TOG_OK )
				{
				//	memcpy(LineCoding,UsbSetupBuf,USB_RX_LEN);
				//	Config_Uart1(LineCoding);
					UEP0_T_LEN = 0;
					UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // 准备上传0包
				}
			}
			else
			{*/
			UEP0_T_LEN = 0;
			UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK; //状态阶段，对IN响应NAK
			//}
			break;

		default:
			break;
		}
		UIF_TRANSFER = 0; //写0清空中断
	}
	if (UIF_BUS_RST) //设备模式USB总线复位中断
	{
#ifdef DE_PRINTF
		printf("reset\r\n"); //睡眠状态
#endif
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0; //清中断标志

		USBByteCount = 0; //USB端点收到的长度
		UsbConfig = 0;	//清除配置值
		ep1_in_busy = 0;
	}
	if (UIF_SUSPEND) //USB总线挂起/唤醒完成
	{
		UIF_SUSPEND = 0;
		if (USB_MIS_ST & bUMS_SUSPEND) //挂起
		{
#ifdef DE_PRINTF
			printf("suspend\r\n"); //睡眠状态
#endif
			while (XBUS_AUX & bUART0_TX)
			{
				; //等待发送完成
			}
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; //USB或者RXD0/1有信号时可被唤醒
			PCON |= PD;											   //睡眠
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
	}
	else
	{					   //意外的中断,不可能发生的情况
		USB_INT_FG = 0xFF; //清中断标志
	}
}

__idata uint8_t transmit_buffer_in_offset;
__idata uint8_t transmit_buffer_out_offset;
__idata uint8_t send_len;

//主函数
void main()
{
	uint8_t length = 0;
	uint8_t read_buffer_index = 0;
	uint8_t shift_count = 0;
	uint8_t operand = 0;
	uint8_t shift_en = 0;
	uint8_t read_en = 0;
	uint16_t timeout_count = 0;

	CfgFsys();   //CH559时钟选择配置
	//mDelaymS(5); //修改主频等待内部晶振稳定,必加
	BTN = 1;
	mDelaymS(50);
	if (BTN == 0)
	{
		mDelaymS(50);
		if (BTN == 0)
		{
			EA = 0;
			mDelaymS(100);
			(*(void (*)(void))0x3800)(); // goto bootloader.
			while (1)
				;
		}
	}

	USBDeviceCfg();
	USBDeviceEndPointCfg(); //端点配置
	USBDeviceIntCfg();		//中断初始化

	//P1.1 P1.4 P1.5 P1.6 output push-pull.
	//P1.7 input.
	P1_MOD_OC = 0x40;
	P1_DIR_PU = 0xf2;
	TDO = 1;

	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0; //预使用发送长度一定要清空

	Ep1Buffer[0] = FTDI_MODEM_STA_DUMMY0;
	Ep1Buffer[1] = FTDI_MODEM_STA_DUMMY1;

	transmit_buffer_in_offset = 0;
	transmit_buffer_out_offset = 0;

	length = 0;
	send_dummy = 1;
	LED = 0;

	while (1)
	{
		if (UsbConfig)
		{
			length = 0;
			if (USBByteCount) //USB接收端点有数据
			{
				//memcpy(receive_buffer, Ep2Buffer, USBByteCount);
				
				__asm
					push ar7
					push a
					inc _XBUS_AUX	//dptr1
					mov	dptr, #_receive_buffer	//target receive_buffer
					dec _XBUS_AUX	//dptr0
					mov	dptr, #_Ep2Buffer	//source Ep2Buffer
					mov ar7, _USBByteCount
				1$:	
					movx a, @dptr
					inc dptr
					.db #0xA5	//WCH 0xA5 instruction
					djnz ar7, 1$
					pop a
					pop ar7
				__endasm;
				
				UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
				length = USBByteCount;
				USBByteCount = 0;
			}

			read_buffer_index = 0;
			while (read_buffer_index < length)
			{
				P2 = receive_buffer[read_buffer_index];
				read_buffer_index++;
				//TODO: Assembly implementation for IO control. 
				//TODO: Use hardware spi for shift control.
				if (shift_count == 0)
				{
					shift_en = P2B7;
					read_en = P2B6;
					if (shift_en)
					{
						shift_count = P2 & 0x3f;
					}
					else if (read_en)
					{
						LED = P2B5;
						TDI = P2B4;
						TMS = P2B1;
						TCK = P2B0;
						transmit_buffer[transmit_buffer_in_offset] = TDO;
						transmit_buffer_in_offset++;
						transmit_buffer_in_offset &= 0x7f;// %= sizeof(transmit_buffer);
					}
					else
					{
						LED = P2B5;
						TDI = P2B4;
						TMS = P2B1;
						TCK = P2B0;
					}
				}
				else
				{
					shift_count--;
					if (read_en)
					{
						TDI = P2B0;
						P2B0 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B1;
						P2B1 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B2;
						P2B2 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B3;
						P2B3 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B4;
						P2B4 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B5;
						P2B5 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B6;
						P2B6 = TDO;
						TCK = 1;
						TCK = 0;

						TDI = P2B7;
						P2B7 = TDO;
						TCK = 1;
						TCK = 0;

						transmit_buffer[transmit_buffer_in_offset] = P2;
						transmit_buffer_in_offset++;
						transmit_buffer_in_offset &= 0x7f;
					}
					else
					{
						TDI = P2B0;
						TCK = 1;
						TCK = 0;

						TDI = P2B1;
						TCK = 1;
						TCK = 0;

						TDI = P2B2;
						TCK = 1;
						TCK = 0;

						TDI = P2B3;
						TCK = 1;
						TCK = 0;

						TDI = P2B4;
						TCK = 1;
						TCK = 0;

						TDI = P2B5;
						TCK = 1;
						TCK = 0;

						TDI = P2B6;
						TCK = 1;
						TCK = 0;

						TDI = P2B7;
						TCK = 1;
						TCK = 0;
					}
				}
			}

			if (ep1_in_busy == 0) //端点不繁忙（空闲后的第一包数据，只用作触发上传）
			{
				int8_t data_len = transmit_buffer_in_offset - transmit_buffer_out_offset;
				data_len = data_len < 0 ? 128 + data_len : data_len;
				if (data_len > 0) // 2 for modem bytes.
				{
					uint8_t i;
					send_len = (data_len >= 62) ? 62 : data_len;
					
					for (i = 0; i < send_len; i++)
					{
						Ep1Buffer[i + 2] = transmit_buffer[transmit_buffer_out_offset];
						transmit_buffer_out_offset++;
						transmit_buffer_out_offset &= 0x7f;// %= sizeof(transmit_buffer);
					}
					/*
					__asm
					push ar7
					push a
					inc _XBUS_AUX	//dptr1
					mov	dptr, #(_Ep1Buffer + 0x0002)	//target receive_buffer
					dec _XBUS_AUX	//dptr0
					mov dph, #(_transmit_buffer >> 8)	//fixed address 0x00XX
					mov dpl, _transmit_buffer_out_offset
					mov ar7, _send_len
				2$:	
					movx a, @dptr
					.db #0xA5	//WCH 0xA5 instruction
					inc _transmit_buffer_out_offset	//idata
					anl _transmit_buffer_out_offset, #0x7f	//ring operation
					mov dph, #(_transmit_buffer >> 8)	//fixed address 0x00XX
					mov dpl, _transmit_buffer_out_offset
					djnz ar7, 2$
					pop a
					pop ar7
					__endasm;
					*/
					
					ep1_in_busy = 1;
					UEP1_T_LEN = send_len + 2;
					UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //应答ACK
				}
				else if ((sof_count - timeout_count) > latency_timer)
				{
					timeout_count = sof_count;
					ep1_in_busy = 1;
					UEP1_T_LEN = 2;											 //预使用发送长度一定要清空
					UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //应答ACK
				}
				else if(send_dummy)
				{
					send_dummy--;
					ep1_in_busy = 1;
					UEP1_T_LEN = 2;											 //预使用发送长度一定要清空
					UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //应答ACK
				}
			}
		}
	}
}
