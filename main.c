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

#define BTN_PIN 4	//for bootloader entry.
SBIT(BTN, 0x90, BTN_PIN);

__xdata __at(0x0100) uint8_t Ep0Buffer[0x08]; //端点0 OUT&IN缓冲区，必须是偶地址
__xdata __at(0x0140) uint8_t Ep1Buffer[0x40]; //端点1 IN缓冲区
__xdata __at(0x0180) uint8_t Ep2Buffer[0x40]; //端点2 OUT缓冲区,必须是偶地址

uint16_t SetupLen;
uint8_t SetupReq, Count, UsbConfig;
uint8_t vendor_control;
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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb5, 0xb2
};

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
unsigned char __code SerDes[] = {	//TODO: variable SN.
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

volatile __idata uint8_t ep1_in_busy = 0; //上传端点是否忙标志

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
				len = 0; // 默认为成功并且上传0长度
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
							break;
						case FTDI_VEN_REQ_GET_MODEM_STA:
							// return fixed modem status
							Ep0Buffer[0] = FTDI_MODEM_STA_DUMMY0;
							Ep0Buffer[1] = FTDI_MODEM_STA_DUMMY1;
							break;
						default:
							// return dummy data
							Ep0Buffer[0] = 0x0;
							Ep0Buffer[1] = 0x0;
							break;
						}
						len = 2;
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
				if(!vendor_control)
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
		UIF_BUS_RST = 0;	   //清中断标志
		
		USBByteCount = 0;	  //USB端点收到的长度
		UsbConfig = 0;		   //清除配置值
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

__idata uint8_t receive_buffer[64];
//主函数
void main()
{
	uint8_t length = 0;
	CfgFsys();   //CH559时钟选择配置
	mDelaymS(5); //修改主频等待内部晶振稳定,必加

	mDelaymS(50);
	if (BTN == 0)
	{
		mDelaymS(50);
		if (BTN == 0)
		{
			EA = 0;
			mDelaymS(100);
			(*(void (*)(void))0x3800)();	// goto bootloader.
			while (1)
				;
		}
	}

	USBDeviceCfg();
	USBDeviceEndPointCfg(); //端点配置
	USBDeviceIntCfg();		//中断初始化
	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0; //预使用发送长度一定要清空
	UEP2_T_LEN = 0;
	Ep1Buffer[0] = FTDI_MODEM_STA_DUMMY0;
	Ep1Buffer[1] = FTDI_MODEM_STA_DUMMY1;

	while (1)
	{
		if(UsbConfig)
		{
			if (USBByteCount) //USB接收端点有数据
			{
				memcpy(receive_buffer, Ep2Buffer, USBByteCount);
				UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
				length = USBByteCount;
				USBByteCount = 0;
			}

			if (ep1_in_busy == 0) //端点不繁忙（空闲后的第一包数据，只用作触发上传）
			{
				length = 2;
				if (length > 0)
				{
					Ep1Buffer[0] = FTDI_MODEM_STA_DUMMY0;
					Ep1Buffer[1] = FTDI_MODEM_STA_DUMMY1;

					ep1_in_busy = 1;
					UEP1_T_LEN = length;									 //预使用发送长度一定要清空
					UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //应答ACK
					//}
				}
			}
		}
	}
}
