#include "mdio.h"
#include "stm32f1xx_hal.h"

/* ����MDIO����Ϊ���ģʽ */
static void mdio_output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MDIO_GPIO_Port, &GPIO_InitStruct);
}

/* ����MDIO����Ϊ����ģʽ */
static void mdio_input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
  GPIO_InitStruct.Pin = MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MDIO_GPIO_Port, &GPIO_InitStruct);
}

/* MDIO ����clk ���� */
void mdio_clk(void)
{
	uint8_t i;
	
	HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	
	
	for(i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
	}
}

/* ��MDIOдһ���ֽ� */
void mdio_write_byte(uint8_t dat)
{
	uint8_t i;
	
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 8; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
}

/* MDIO read start */
void mdio_read_start(void)
{
	uint8_t dat = 0x06;
	uint8_t i;
	
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 4; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
}

/* PHY address */
void phyad(void)
{
	uint8_t dat = 0x00;
	uint8_t i;
	
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
}

/* REG address */
void regad(void)
{
	uint8_t dat = 0x00;
	uint8_t i;
	
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
}


/* MDIO turn around */
void turn_around(void)
{
	uint8_t dat = 0x01;
	uint8_t i;
	
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 2; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
}

/* ��MDIO��ȡһ���ֽ� */
uint8_t mdio_read_byte(void)
{
	uint8_t i;
	uint8_t dat = 0;
	
	/* ��io�л�Ϊ����ģ�� */
	mdio_input();
	
	for(i = 0; i < 8; i++)
	{
		dat >>= 1;
		
		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
			dat |= 0x80;

		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
	}
	
	return dat;
}

uint16_t all(uint8_t regad);

uint16_t read_data(uint8_t phyad, uint8_t regad)
{
//	mdio_clk();
//	mdio_read_start();
////	mdio_write_byte(0x55);
//	phyad();
//	regad();
//	turn_around();
//	mdio_read_byte();
//	mdio_read_byte();
	
//	 return all(regad);
	
/* 0. ���� IDLE mdio_clk */
	uint8_t i;
	
	HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	
	for(i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
	}
	
/* 1. ��ʼλ01 + 2. ������10 mdio_read_start */
	uint8_t dat = 0x06;
	
	/* ��io�л�Ϊ���ģʽ */
	//  mdio_output();	
	
	for(i = 0; i < 4; i++)
	{
		if(dat & 0x08)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
	
/* 3. PHY��ַ phyad */
		/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	dat = phyad;
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x10)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
/* 4. �Ĵ�����ַ regad */
	dat = regad;
//	for(i = 0; i < 5; i++)
//	{
//		if(dat & 0x01)
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
//		else
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
//		
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
//		dat >>= 1;
//	}
	
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x10)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}

/* 5. ת��10 turn around */
	dat = 0x01;
	
	/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	
	for(i = 0; i < 2; i++)
	{
		if(dat & 0x02)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
/* 7. ��ȡ16bits���� */	
		/* ��io�л�Ϊ����ģ�� */
	mdio_input();
	uint16_t out_data = 0x0000;
	
//	for(i = 0; i < 16; i++)
//	{
//		out_data >>= 1;
//		
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
//			out_data |= 0x8000;

//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
//	}
	
	for(i = 0; i < 16; i++)
	{
		out_data <<= 1;
		
		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
			out_data |= 0x0001;

		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
	}
	
	for(i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
	}
	
	return out_data;
	
}


void write_data(uint8_t phyad, uint8_t regad, uint16_t out_data)
{

/* 0. ���� IDLE mdio_clk */
	uint8_t i;
	
	HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	for(i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
	}
	
	
/* 1. ��ʼλ01 + 2. д����01 mdio_read_start */
	uint8_t dat = 0x05;
	
	/* ��io�л�Ϊ���ģʽ */
	//  mdio_output();	
	
	for(i = 0; i < 4; i++)
	{
		if(dat & 0x08)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
	
/* 3. PHY��ַ phyad */
		/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	dat = phyad;
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x10)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
/* 4. �Ĵ�����ַ regad */
	dat = regad;
//	for(i = 0; i < 5; i++)
//	{
//		if(dat & 0x01)
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
//		else
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
//		
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
//		dat >>= 1;
//	}
	
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x10)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}

/* 5. ת��10 turn around */
	dat = 0x10;
	
	/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	
	for(i = 0; i < 2; i++)
	{
		if(dat & 0x02)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}
	
/* 7. д16bits���� */	
	
//	for(i = 0; i < 16; i++)
//	{
//		out_data >>= 1;
//		
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
//			out_data |= 0x8000;

//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
//	}
	
	for(i = 0; i < 16; i++)
	{
		if(out_data & 0x8000)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		out_data <<= 1;
	}
	
	return;
}









#if 0

uint16_t all(uint8_t regad)
{
/* mdio_clk */
	uint8_t i;
	
	HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
	/* ��io�л�Ϊ���ģʽ */
  mdio_output();	
	
	
	for(i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
	}
	
/* mdio_read_start */
	uint8_t dat = 0x06;
	
	/* ��io�л�Ϊ���ģʽ */
	//  mdio_output();	
	
	for(i = 0; i < 4; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
	
	
/* phyad */
		/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	dat = 0x00;
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
	
/* regad */
	dat = regad;
//	for(i = 0; i < 5; i++)
//	{
//		if(dat & 0x01)
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
//		else
//			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
//		
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
//		dat >>= 1;
//	}
	
	for(i = 0; i < 5; i++)
	{
		if(dat & 0x10)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat <<= 1;
	}

	
	dat = 0x01;
	
	/* ��io�л�Ϊ���ģʽ */
//  mdio_output();	
	
	for(i = 0; i < 2; i++)
	{
		if(dat & 0x01)
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MDIO_GPIO_Port, MDIO_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);
		dat >>= 1;
	}
	
	
		/* ��io�л�Ϊ����ģ�� */
	mdio_input();
	uint16_t out_data = 0x0000;
	
//	for(i = 0; i < 16; i++)
//	{
//		out_data >>= 1;
//		
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
//			out_data |= 0x8000;

//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
//	}
	
	for(i = 0; i < 16; i++)
	{
		out_data <<= 1;
		
		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(MDIO_GPIO_Port, MDIO_Pin))
			out_data |= 0x0001;

		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MDC_GPIO_Port, MDC_Pin, GPIO_PIN_RESET);	
	}
	
	return out_data;
}

#endif