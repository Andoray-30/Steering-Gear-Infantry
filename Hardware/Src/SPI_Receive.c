#include "SPI_Receive.h"
#include "spi.h"
void W25Q128_Init(void)
{
	MX_SPI2_Init();
}
extern void W25Q128_ReadID(uint8_t *MID,uint16_t *DID)//mid�ǳ���ID��0xEF����DID��оƬID��0x4017��
{
	SPI_START();
	MySPI_SwapByte(W25Q128_JEDEC_ID);//ѯ��ID�ţ�0x9F���ж��Ƿ�����ʹ��
	*MID=MySPI_SwapByte(W25Q128_DUMMY_BYTE);//����byte��0xFF����ȡ�豸ID
	*DID=MySPI_SwapByte(W25Q128_DUMMY_BYTE);//����byte��0xFF����ȡ����ID���߰�λ��
	*DID <<= 8;//��λ�Ƶ���λ
	*DID |= MySPI_SwapByte(W25Q128_DUMMY_BYTE);// |= ��ֹ��λ������
	SPI_STOP();
}
extern void W25Q128_WriteEnable(void)//ʹ�ܼĴ���
{
	SPI_START();
	MySPI_SwapByte(W25Q128_WRITE_ENABLE);
	SPI_STOP();
}
extern void W25Q128_WaitBusy(void)//��ȡ״̬�Ĵ�����æ��æ���ȴ�BusyΪ0
{
	uint32_t Timeout=100000;
	SPI_START();
	MySPI_SwapByte(W25Q128_READ_STATUS_REGISTER_1);
	while((MySPI_SwapByte(W25Q128_DUMMY_BYTE) & 0x01)==0x01)//�ȴ�æ״̬����
	{
		Timeout--;
		if(Timeout==0)
		{
			break;
		}
	}
	SPI_STOP();
}
extern void W25Q128_PageProgram(uint32_t Address,uint8_t*DataArray,uint16_t Count)//һ�����д256��byte,д���ܿ�ҳ�������Կ�ҳ
{
	W25Q128_WriteEnable();
	
	uint16_t i;
	SPI_START();
	MySPI_SwapByte(W25Q128_PAGE_PROGRAM);
	//24λ��ָ����ַ
	MySPI_SwapByte((uint8_t)(Address >> 16));//���λ
	MySPI_SwapByte((uint8_t)(Address >> 8));//�м�λ
	MySPI_SwapByte((uint8_t)Address);//���λ
	//����byte
	for(i=0;i<Count;i++)
	{
		MySPI_SwapByte(DataArray[i]);
	}
	SPI_STOP();
	
	W25Q128_WaitBusy();//�º�ȴ�,��ǰ�ȴ�Ч�ʸ���
}
extern void W25Q128_SectorErase_32KB(uint32_t Address)//������������
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
extern void W25Q128_ReadDate(uint32_t Address,uint8_t*DataArray,uint32_t Count)//������û������
{
	uint32_t i;
	SPI_START();
	MySPI_SwapByte(W25Q128_READ_DATA);
	//24λ��ָ����ַ
	MySPI_SwapByte(Address >> 16);
	MySPI_SwapByte(Address >> 8);
	MySPI_SwapByte(Address);
	//����byte
	for(i=0;i<Count;i++)
	{
		DataArray[i]=MySPI_SwapByte(W25Q128_DUMMY_BYTE);
	}
	SPI_STOP();
}