/************代码优化定义******************/
#define optimize(grade)	__attribute__((__optimize__("O"#grade)))
#define optimize0	optimize(0)
#define optimize1	optimize(1)
#define optimize2	optimize(2)
#define optimize3	optimize(3)

#define SlaveAddress 0x66 //s5m8767地址
#define RegAddress 0x33 //s5m8767地址

typedef struct {
	unsigned int CON;
	unsigned int DAT;
	unsigned int PUD;
	unsigned int DRV;
	unsigned int CONPDN;
	unsigned int PUDPDN;
}gpd1;
#define GPD1 (* (volatile gpd1 *)0x114000C0)

typedef struct {
        unsigned int I2CCON;
        unsigned int I2CSTAT;
        unsigned int I2CADD;
        unsigned int I2CDS;
        unsigned int I2CLC;
}I2C1;

#define  I2C1 (* (volatile I2C1 *)0x13870000 )

optimize0 void iic1_init(void)
{
	GPD1.CON = (GPD1.CON & ~(0xff<<8)) | 0x22<<8;
	
	I2C1.I2CSTAT = 0xD0;
	I2C1.I2CCON &= ~(1<<4);  /*clean interrupt pending bit  */
}
optimize0 void iic1_read(unsigned char slave_addr, unsigned char addr, unsigned char *data)
{
  I2C1.I2CDS = (slave_addr << 1); //将从机地址写入I2CDS寄存器中
  I2C1.I2CCON = (1 << 7)|(1 << 6)|(1 << 5); //设置时钟并使能中断
  I2C1.I2CSTAT = 0xf0;    //[7:6]设置为0b11，主机发送模式；
  //往[5：4]位写0b11，即产生启动信号,发出IICDS寄存器中的地址
  
  while(!(I2C1.I2CCON & (1 << 4))); // 等待传输结束，传输结束后，I2CCON [4]位为1，标识有中断发生；   
  
  // 此位为1时，SCL线被拉低，此时I2C传输停止；
  I2C1.I2CDS = addr;       //写命令值
  I2C1.I2CCON = I2C1.I2CCON & (~(1 << 4));// I2CCON [4]位清0，继续传输
  while(!(I2C1.I2CCON & (1 << 4)));// 等待传输结束
  
  I2C1.I2CSTAT = 0xD0; // I2CSTAT[5:4]位写0b01,发出停止信号
  I2C1.I2CDS = (slave_addr << 1) | 1;  //表示要读出数据
  
  I2C1.I2CCON = (1 << 7)|(1 << 6) |(1 << 5) ; //设置时钟并使能中断
  I2C1.I2CSTAT = 0xb0;//[7:6]位0b10,主机接收模式；
  
  //往[5：4]位写0b11，即产生启动信号,发出IICDS寄存器中的地址
  //    I2C1.I2CCON = I2C1.I2CCON & (~(1 << 4));    如果强行关闭，将读取不到数据
  
  while(!(I2C1.I2CCON & (1 << 4)));//等待传输结束，接收数据
  
  I2C1.I2CCON &= ~((1<<7)|(1 << 4));/* Resume the operation  & no ack*/
   // I2CCON [4]位清0，继续传输，接收数据，   
   // 主机接收器接收到最后一字节数据后，不发出应答信号 no ack   
  
  // 从机发送器释放SDA线，以允许主机发出P信号，停止传输；
  while(!(I2C1.I2CCON & (1 << 4)));// 等待传输结束
  
  I2C1.I2CSTAT = 0x90;
  *data = I2C1.I2CDS;
  I2C1.I2CCON &= ~(1<<4);  /*clean interrupt pending bit  */
}
/**************************************************************
 * @brief            iic write a byte program body
 * @param[in]    slave_addr, addr, data
 * @return         None
 *************************************************************/

optimize0 void iic1_write (unsigned char slave_addr, unsigned char addr, unsigned char data)
{
  I2C1.I2CDS = (slave_addr << 1);
  I2C1.I2CCON = (1 << 7)|(1 << 6)|(1 << 5) ;
  I2C1.I2CSTAT = 0xf0;

  while(!(I2C1.I2CCON & (1 << 4)));
  I2C1.I2CDS = addr;
  I2C1.I2CCON = I2C1.I2CCON & (~(1 << 4));
  while(!(I2C1.I2CCON & (1 << 4)));
  
  I2C1.I2CDS = data;
  I2C1.I2CCON = I2C1.I2CCON & (~(1 << 4));
  
  while(!(I2C1.I2CCON & (1 << 4)));
  
  I2C1.I2CSTAT = 0xd0;
  I2C1.I2CCON = I2C1.I2CCON & (~(1 << 4));
}


optimize0 void S5M8767A_spl_init(void)
{
	iic1_init();
	
	for(int i=0;i<10;i++){
		if(RegAddress+i != 0x34)
		{
			iic1_write(SlaveAddress,RegAddress+i,0x70);
		}
	}
}

