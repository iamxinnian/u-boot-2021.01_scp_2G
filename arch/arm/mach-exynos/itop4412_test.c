#include <asm/arch/itop4412_test.h>
#include <asm/io.h>

void led2on(void)
{
	unsigned int *led2_con_addr=(unsigned int *)0x11000100;
	unsigned int *led2_data_addr=(unsigned int *)0x11000104;
	unsigned int value=readl(led2_con_addr);
	value |=0x1;
	writel(value,led2_con_addr);

	value=readl(led2_data_addr);
	value |=0x1;
	writel(value,led2_data_addr);

	return;
}

void led2off(void)
{
	unsigned int *led2_con_addr=(unsigned int *)0x11000100;
	unsigned int *led2_data_addr=(unsigned int *)0x11000104;
	unsigned int value=readl(led2_con_addr);
	value |=0x1;
	writel(value,led2_con_addr);

	value=readl(led2_data_addr);
	value &=0xfffffffe;
	writel(value,led2_data_addr);

	return;
}

optimize0 void delay(unsigned int n)
{
	unsigned int i=0;
	while(n--)
	{
		for(i=0;i<0xfff;i++);
	}
}

optimize0 void blink_led2(unsigned int Times,unsigned int DelayTime)
{
	unsigned int t=0;
	for(t=0;t<Times;t++)
	{
		led2on();
		delay(DelayTime);
		led2off();
		delay(DelayTime);
	}
	return;
}

optimize0 void UartInit(void)
{
/* 1.设置相应的GPIO用于串口功能 */
unsigned long tmp = 0; 
tmp = GPA1CON;
tmp &= ~(0xff); //设置UART0对应的GPIO为UART功能
tmp |= 0x22;
GPA1CON = tmp;
 
/* 3.设置串口0相关 */
/* 设置FIFO中断触发阈值
* 使能FIFO
*/
UFCON3 = 0x111;
 
/* 设置数据格式: 8n1, 即8个数据位,没有较验位,1个停止位 */
ULCON3 = 0x3;
 
/* 工作于中断/查询模式
* 另一种是DMA模式,本章不使用
*/
UCON3 = 0x5;
 
/* SCLK_UART0=100MHz, 波特率设置为115200
* 寄存器的值如下计算:
* DIV_VAL = 100,000,000 / (115200 * 16) - 1 = 53.25
* UBRDIVn0 = 整数部分 = 53
* UFRACVAL3 = 小数部分 x 16 = 0.25 * 16 = 4
*/
UBRDIV3 = 53;
UFRACVAL3 = 4;
 
}


optimize0 char mygetc(void)
{
char c;
/* 查询状态寄存器，直到有有效数据 */
while (!(UTRSTAT3 & (1<<0)));
 
c = URXH3; /* 读取接收寄存器的值 */
 
return c;
}
 
optimize0 void myputc(char c)
{
/* 查询状态寄存器，直到发送缓存为空 */
while (!(UTRSTAT3 & (1<<2)));
 
UTXH3 = c; /* 写入发送寄存器 */
 
return;
}
 
optimize0 void myputs(char *s)
{
while (*s)
{
myputc(*s);
s++;
}
}


optimize0 char mem_write(unsigned int *addr,unsigned int data)
{
	*addr=data;
	return 0;
}

optimize0 char mem_read(unsigned int *addr,unsigned int *data)
{
	*data=*addr;
	return 0;
}

optimize0 char mem_test(unsigned int *start_addr,unsigned size)
{
	unsigned int i=0;
	unsigned int *start_write_addr=start_addr;
	unsigned int *start_read_addr=start_addr;
	UartInit();
	if(size==0)
	{
		return -2;
	}

	//write test data
	for(i = 0;i < size; i++)
	{
		*start_write_addr=(unsigned int)start_write_addr+i;
		start_write_addr++;
		if(i%0x100000==0)
			myputs("write\n");
	}

	//read and check data
	for(i = 0;i < size; i++)
	{
		if(*start_read_addr != ((unsigned int)start_read_addr+i))
		{
			return -1;
		}
		start_read_addr++;
		if(i%0x100000==0)
			myputs("check\n");
	}
	return 0;
}
