#include "SPI_Receive.h"
#include "spi.h"
void W25Q128_Init(void)
{
	MX_SPI2_Init();
}
extern void W25Q128_ReadID(uint8_t *MID,uint16_t *DID)//mid是厂商ID（0xEF），DID是芯片ID（0x4017）
{
	SPI_START();
	MySPI_SwapByte(W25Q128_JEDEC_ID);//询问ID号（0x9F）判断是否正常使用
	*MID=MySPI_SwapByte(W25Q128_DUMMY_BYTE);//垃圾byte（0xFF）换取设备ID
	*DID=MySPI_SwapByte(W25Q128_DUMMY_BYTE);//垃圾byte（0xFF）换取厂家ID（高八位）
	*DID <<= 8;//低位移到高位
	*DID |= MySPI_SwapByte(W25Q128_DUMMY_BYTE);// |= 防止高位被覆盖
	SPI_STOP();
}
extern void W25Q128_WriteEnable(void)//使能寄存器
{
	SPI_START();
	MySPI_SwapByte(W25Q128_WRITE_ENABLE);
	SPI_STOP();
}
extern void W25Q128_WaitBusy(void)//读取状态寄存器，忙或不忙，等待Busy为0
{
	uint32_t Timeout=100000;
	SPI_START();
	MySPI_SwapByte(W25Q128_READ_STATUS_REGISTER_1);
	while((MySPI_SwapByte(W25Q128_DUMMY_BYTE) & 0x01)==0x01)//等待忙状态结束
	{
		Timeout--;
		if(Timeout==0)
		{
			break;
		}
	}
	SPI_STOP();
}
extern void W25Q128_PageProgram(uint32_t Address,uint8_t*DataArray,uint16_t Count)//一次最多写256个byte,写不能跨页，读可以跨页
{
	W25Q128_WriteEnable();
	
	uint16_t i;
	SPI_START();
	MySPI_SwapByte(W25Q128_PAGE_PROGRAM);
	//24位的指定地址
	MySPI_SwapByte((uint8_t)(Address >> 16));//最高位
	MySPI_SwapByte((uint8_t)(Address >> 8));//中间位
	MySPI_SwapByte((uint8_t)Address);//最低位
	//发送byte
	for(i=0;i<Count;i++)
	{
		MySPI_SwapByte(DataArray[i]);
	}
	SPI_STOP();
	
	W25Q128_WaitBusy();//事后等待,事前等待效率更高
}
extern void W25Q128_SectorErase_32KB(uint32_t Address)//擦除所在扇区
{
	W25Q128_WriteEnable();
	
	SPI_START();
	MySPI_SwapByte(W25Q128_SECTOR_ERASE_4KB);
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8);
	MySPI_SwapByte(Address);
	SPI_STOP();
	
	W25Q128_WaitBusy();
}
extern void W25Q128_ReadDate(uint32_t Address,uint8_t*DataArray,uint32_t Count)//读数据没有限制
{
	uint32_t i;
	SPI_START();
	MySPI_SwapByte(W25Q128_READ_DATA);
	//24位的指定地址
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8);
	MySPI_SwapByte(Address);
	//接收byte
	for(i=0;i<Count;i++)
	{
		DataArray[i]=MySPI_SwapByte(W25Q128_DUMMY_BYTE);
	}
	SPI_STOP();
}