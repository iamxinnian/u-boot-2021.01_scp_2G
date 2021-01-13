#ifndef _ITOP4412_TEST_H_
#define _ITOP4412_TEST_H_

/************代码优化定义******************/
#define optimize(grade)	__attribute__((__optimize__("O"#grade)))
#define optimize0	optimize(0)
#define optimize1	optimize(1)
#define optimize2	optimize(2)
#define optimize3	optimize(3)

// UART
#define 	GPA1CON		(*(volatile unsigned int *)0x11400020)
#define		UFCON3		(*(volatile unsigned int *)0x13820008)
#define		ULCON3		(*(volatile unsigned int *)0x13820000)
#define 	UCON3		(*(volatile unsigned int *)0x13820004)
#define 	UBRDIV3		(*(volatile unsigned int *)0x13820028)
#define 	UFRACVAL3	(*(volatile unsigned int *)0x1382002c)
#define 	UTXH3		(*(volatile unsigned int *)0x13820020)
#define 	URXH3		(*(volatile unsigned int *)0x13820024)
#define 	UTRSTAT3	(*(volatile unsigned int *)0x13820010)
/***********点灯测试********************/
void led2on(void);
void led2off(void);
optimize0 void blink_led2(unsigned int Times,unsigned int DelayTime);
optimize0 void delay(unsigned int n);

optimize0 void UartInit(void);
optimize0 void myputs(char *s);
optimize0 char mem_write(unsigned int *addr,unsigned int data);
optimize0 char mem_read(unsigned int *addr,unsigned int *data);
optimize0 char mem_test(unsigned int *start_addr,unsigned size);

#endif

