#include "stm32f1xx.h"
#include "./BSP/NRF24L01/NRF24L01_define.h"
#include "./SYSTEM/delay/delay.h"
#include <stdio.h>

#define NOP 0xFF

uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};

void W_MOSI(uint8_t Value)
{
    HAL_GPIO_WritePin(MOSI_Port, MOSI_Pin, (GPIO_PinState)Value);
}

void W_SCK(uint8_t Value)
{
    HAL_GPIO_WritePin(SCK_Port, SCK_Pin, (GPIO_PinState)Value);
}

void W_CSN(uint8_t Value)
{
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, (GPIO_PinState)Value);
}

void W_CE(uint8_t Value)
{
    HAL_GPIO_WritePin(CE_Port, CE_Pin, (GPIO_PinState)Value);
}

uint8_t R_IRQ(void)
{
    return HAL_GPIO_ReadPin(IRQ_Port, IRQ_Pin);
}

uint8_t R_MISO(void)
{
    return HAL_GPIO_ReadPin(MISO_Port, MISO_Pin);
}

void NRF24L01_Pin_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 配置成推挽输出
    GPIO_InitStructure.Pin = MOSI_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOSI_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SCK_Pin;
    HAL_GPIO_Init(SCK_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CSN_Pin;
    HAL_GPIO_Init(CSN_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CE_Pin;
    HAL_GPIO_Init(CE_Port, &GPIO_InitStructure);

    // 配置成上拉输入
    GPIO_InitStructure.Pin = IRQ_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IRQ_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = MISO_Pin;
    HAL_GPIO_Init(MISO_Port, &GPIO_InitStructure);
}

//SPI交换一个字节
uint8_t SPI_SwapByte(uint8_t Byte)
{
    uint8_t i,ByteReceive=0x00;
    for(i=0;i<8;i++)
    {
        W_MOSI(Byte&(0x80>>i));
        W_SCK(1);
        if(R_MISO()==1)
        {
            ByteReceive=ByteReceive|(0x80>>i);
        }
        W_SCK(0);
    }
    return ByteReceive;
}

void W_Reg(uint8_t Reg,uint8_t Value)
{
    W_CSN(0);//表示选中NRF24L01
    SPI_SwapByte(Reg);//交换的第一个字节就是指令
    SPI_SwapByte(Value);//交换的第二个字节就是交换的数据
    W_CSN(1);//停止选中NRF24L01
}

uint8_t R_Reg(uint8_t Reg)
{
    uint8_t Value;
    W_CSN(0);//表示选中NRF24L01
    SPI_SwapByte(Reg);//交换的第一个字节就是指令
    Value=SPI_SwapByte(NOP);//交换的第二个字节就是交换的数据
    W_CSN(1);//停止选中NRF24L01
    return Value;
}

void W_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len)
{
    uint8_t i;
    W_CSN(0);//选中NRF24L01
    SPI_SwapByte(Reg);
    for(i=0;i<Len;i++)
    {
        SPI_SwapByte(Buf[i]);
    }
    W_CSN(1);//停止选中NRF24L01
}

void R_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len)
{
    uint8_t i;
    W_CSN(0);//选中NRF24L01
    SPI_SwapByte(Reg);
    for(i=0;i<Len;i++)
    {
        Buf[i]=SPI_SwapByte(NOP);
    }
    W_CSN(1);//停止选中NRF24L01
}

void NRF24L01_Init(void)
{
    NRF24L01_Pin_Init();

    W_CE(0);

    W_Buf(W_REGISTER+TX_ADDR, T_ADDR, 5);//配置发送地址
    W_Buf(W_REGISTER+RX_ADDR_P0, R_ADDR, 5);//配置接收通道0
    W_Reg(W_REGISTER+CONFIG,0x0F);//配置成接收模式
    W_Reg(W_REGISTER+EN_AA,0x01);//通道0开启自动应答
    W_Reg(W_REGISTER+RF_CH,0x00);//配置通信频率2.4G
    W_Reg(W_REGISTER+RX_PW_P0,0x20);//配置接收通道0接收的数据宽度32字节
    W_Reg(W_REGISTER+EN_RXADDR,0x01);//接收通道0使能
    W_Reg(W_REGISTER+SETUP_RETR,0x1A);//配置580us重发时间间隔,重发10次
    W_Reg(FLUSH_RX,NOP);

    W_CE(1);
}

void Receive(uint8_t* Buf)
{
    uint8_t Status;
    Status =R_Reg(R_REGISTER+STATUS);
    if(Status & RX_OK)
    {
        R_Buf(R_RX_PAYLOAD, Buf, 32);
        W_Reg(FLUSH_RX,NOP);
        W_Reg(W_REGISTER+STATUS, Status);
        HAL_Delay(150);
    }
}

uint8_t Send(uint8_t* Buf)
{
    uint8_t Status;
    W_Buf(W_TX_PAYLOAD, Buf, 32);//在发送数据缓存器发送要发送的数据

    W_CE(0);
    W_Reg(W_REGISTER+CONFIG,0x0E);
    W_CE(1);

    while(R_IRQ()==1);//等待中断
    Status= R_Reg(R_REGISTER+STATUS);

    if(Status & MAX_TX)//如果发送达到最大次数
    {
        W_Reg(FLUSH_TX,NOP);//清除发送数据缓存器
        W_Reg(W_REGISTER+STATUS,Status);//中断位写1清除中断
        return MAX_TX;
    }
    if(Status & TX_OK)//如果发送成功,接收到应答信号
    {
        W_Reg(W_REGISTER+STATUS,Status);//清除中断
        return TX_OK;
    }
    return 0;
}

/**
  * @brief  检查NRF24L01初始化是否正确
  * @retval 0:成功  其他:错误标志（具体见位定义）
  *         位0: CONFIG寄存器错误
  *         位1: EN_AA寄存器错误
  *         位2: RF_CH寄存器错误
  *         位3: RX_PW_P0寄存器错误
  *         位4: EN_RXADDR寄存器错误
  *         位5: SETUP_RETR寄存器错误
  *         位6: TX_ADDR地址不匹配
  *         位7: RX_ADDR_P0地址不匹配
  */
uint8_t NRF24L01_Check(void) 
{
    uint8_t error = 0;
    uint8_t reg_val;
    uint8_t addr_buf[5];
    
    // 1. 检查CONFIG寄存器（预期值0x0F）
    reg_val = R_Reg(R_REGISTER + CONFIG);
    if (reg_val != 0x0F) {
        error |= 1 << 0; // 标记位0错误
    }
    
    // 2. 检查EN_AA寄存器（预期值0x01）
    reg_val = R_Reg(R_REGISTER + EN_AA);
    if (reg_val != 0x01) {
        error |= 1 << 1; // 标记位1错误
    }
    
    // 3. 检查RF_CH频率（预期值0x00）
    reg_val = R_Reg(R_REGISTER + RF_CH);
    if (reg_val != 0x00) {
        error |= 1 << 2; // 标记位2错误
    }
    
    // 4. 检查RX_PW_P0数据宽度（预期值0x20=32字节）
    reg_val = R_Reg(R_REGISTER + RX_PW_P0);
    if (reg_val != 0x20) {
        error |= 1 << 3; // 标记位3错误
    }
    
    // 5. 检查EN_RXADDR使能状态（预期值0x01）
    reg_val = R_Reg(R_REGISTER + EN_RXADDR);
    if (reg_val != 0x01) {
        error |= 1 << 4; // 标记位4错误
    }
    
    // 6. 检查SETUP_RETR重发配置（预期值0x1A）
    reg_val = R_Reg(R_REGISTER + SETUP_RETR);
    if (reg_val != 0x1A) {
        error |= 1 << 5; // 标记位5错误
    }
    
    // 7. 验证TX_ADDR发送地址
    R_Buf(R_REGISTER + TX_ADDR, addr_buf, 5);
    for (uint8_t i = 0; i < 5; i++) {
        if (addr_buf[i] != T_ADDR[i]) {
            error |= 1 << 6; // 标记位6错误
            break;
        }
    }
    
    // 8. 验证RX_ADDR_P0接收地址
    R_Buf(R_REGISTER + RX_ADDR_P0, addr_buf, 5);
    for (uint8_t i = 0; i < 5; i++) {
        if (addr_buf[i] != R_ADDR[i]) {
            error |= 1 << 7; // 标记位7错误
            break;
        }
    }
    
    return error; // 返回错误标志
}

