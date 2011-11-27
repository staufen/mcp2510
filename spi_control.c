/*
 *	Module Name:	SPI_CONCTROL.C
 *	Abstract:	Basic operation to SPI controller of S3C6410	
 *	Notes:		Presently, only the SPI Channel 0 is supported.
 *	Date: 		2011/09/16
 *	By: 		Staufen Yang
 */ 

#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/irqs.h>
#include <asm/mach/irq.h>
#include <asm/arch-s3c2410/regs-s3c-clock.h>		//needed by the clock setting
#include <asm/arch/regs-gpio.h>				//defined the gpio address	

#define _SPI_CTL_SOURCE_

//#define DEBUG
#ifdef DEBUG
#define DbgPrintk printk
#else
#define DbgPrintk
#endif

#include 	"spi_control.h"
#include	"can.h"
#include 	"mcpcan.h"

volatile void  __iomem *spiregs;			//global variable for mapping spi register

/*	print_reg(void) - Print the spi controller register	*/
void print_reg(void)
{
	DbgPrintk("\nCH_CFG 	= 0x%08x\n",readl(spiregs + S3C_CH_CFG));
	DbgPrintk("CLK_CFG = 0x%08x\n",readl(spiregs + S3C_CLK_CFG));
	DbgPrintk("MODE_CFG = 0x%08x\n",readl(spiregs + S3C_MODE_CFG));
	DbgPrintk("SLAVE_CFG = 0x%08x\n",readl(spiregs + S3C_SLAVE_SEL));
	DbgPrintk("INT_EN 	= 0x%08x\n",readl(spiregs + S3C_SPI_INT_EN));
	DbgPrintk("SPI_STATUS = 0x%08x\n",readl(spiregs + S3C_SPI_STATUS));
	DbgPrintk("PACKET_CNT = 0x%08x\n",readl(spiregs + S3C_PACKET_CNT));
	DbgPrintk("PEND_CLR = 0x%08x\n",readl(spiregs + S3C_PENDING_CLR));
	DbgPrintk("SWAP_CFG = 0x%08x\n",readl(spiregs + S3C_SWAP_CFG));
	DbgPrintk("FB_CLK 	= 0x%08x\n",readl(spiregs + S3C_FB_CLK));
	DbgPrintk("PCLK_GATE 	= 0x%08x\n",readl(S3C_PCLK_GATE));
	DbgPrintk("CLK_SRC 	= 0x%08x\n",readl(S3C_CLK_SRC));
	DbgPrintk("CLK_DIV2 	= 0x%08x\n",readl(S3C_CLK_DIV2));

}

/*	Port_Init() - Config the pins might be used	*/
void Port_Init(void)
{
	DbgPrintk("\nIn port_init\n");
	
	/*	config the spi0 ports	*/
	s3c_gpio_cfgpin(S3C_GPC0, S3C_GPC0_SPI_MISO0);
	s3c_gpio_cfgpin(S3C_GPC1, S3C_GPC1_SPI_CLK0);
	s3c_gpio_cfgpin(S3C_GPC2, S3C_GPC2_SPI_MOSI0);
	s3c_gpio_cfgpin(S3C_GPC3, S3C_GPC3_SPI_CS0);
	s3c_gpio_pullup(S3C_GPC0,1);
	s3c_gpio_pullup(S3C_GPC1,1);
	s3c_gpio_pullup(S3C_GPC2,1);
	s3c_gpio_pullup(S3C_GPC3,1);

	DbgPrintk("\nS3C_GPNCON:%08x\n",readl(S3C_GPNCON));	
	DbgPrintk("\nS3C_GPNPU:%08x\n",readl(S3C_GPNPU));	
	DbgPrintk("\nS3C_EINTCON0:%08x\n",readl(S3C_EINTCON0));	
	DbgPrintk("\nS3C_EINTMASK:%x08\n",readl(S3C_EINTMASK));	
	
	/*	config the eint9	*/
 	writel((readl(S3C_GPNCON)&(~(0x3<<18)))|(0x2<<18),S3C_GPNCON);	
 	writel((readl(S3C_GPNPU)&(~(0x3<<18)))|(0<<18),S3C_GPNPU);
 	DbgPrintk("\nS3C_GPNCON:%08x\n",readl(S3C_GPNCON));
 	DbgPrintk("\nS3C_GPNPU:%08x\n",readl(S3C_GPNPU));	

	/*	set the hardware reset port	*/
	s3c_gpio_cfgpin(S3C_GPK0, S3C_GPK0_OUTP);
	s3c_gpio_pullup(S3C_GPK0,1);

	DbgPrintk("\nOut port_init\n");
}

/*	SPI_Init() - Map and initiate the registers might be used	*/
BOOL SPI_Init(VOID)
{
	spiregs = (volatile)ioremap(0x7F00B000, 0x30);  //just request for the spi0
	U32 spi_chcfg, spi_slavecfg, spi_inten, spi_packet;
	U32 spi_clkcfg, spi_modecfg;
	spi_chcfg = 0;
	spi_slavecfg = 0;
	spi_inten = 0;
	spi_packet=0;
	spi_clkcfg = 0;
	spi_modecfg = 0 ;
	U8 prescaler = 1;		// 22.2175 Mhz

	/*	initialise the spi controller	*/
	DbgPrintk("\nbefor port_init");
	Port_Init(); 		//select pins' functions ,deselect mcp2510
	DbgPrintk("\nafter port_init");

	DbgPrintk("\nbefore spiregs init");
	print_reg();

	/*	start PCLK = 66Mhz	*/
	writel((readl(S3C_PCLK_GATE)|S3C_CLKCON_PCLK_SPI0),S3C_PCLK_GATE);
	DbgPrintk("\nS3C_PCLK_GATE:%x\n",*(unsigned int *)S3C_PCLK_GATE);	

	/*	1. Set transfer type (CPOL & CPHA set)	*/
	spi_chcfg = SPI_CH_RISING | SPI_CH_FORMAT_A;
	spi_chcfg |= SPI_CH_MASTER;
	writel( spi_chcfg , spiregs + S3C_CH_CFG);

	/*	2. Set clock configuration register	
 	*	SPI clockout = clock source / (2 * (prescaler +1))	
	*	PCLK=66Mhz, SPI clockout = clock source / (2 * (prescaler +1))=66/4=16.5Mhz	*/
	spi_clkcfg = SPI_ENCLK_ENABLE;
	spi_clkcfg |= SPI_CLKSEL_PCLK;
	writel( spi_clkcfg , spiregs + S3C_CLK_CFG);
	spi_clkcfg = readl( spiregs + S3C_CLK_CFG);

	spi_clkcfg |= 49;	 // the least spi speed = 660Khz
	writel( spi_clkcfg , spiregs + S3C_CLK_CFG);
	
	/*	3. Set SPI MODE configuration register	*/
	spi_modecfg = SPI_MODE_CH_TSZ_BYTE| SPI_MODE_BUS_TSZ_BYTE;
	spi_modecfg |= SPI_MODE_TXDMA_OFF| SPI_MODE_SINGLE| SPI_MODE_RXDMA_OFF;
	spi_modecfg &= ~( 0x3f << 5);
	spi_modecfg |= ( 0x1 << 5); 	// Tx FIFO trigger level in INT mode 
	spi_modecfg &= ~( 0x3f << 11);
	spi_modecfg |= ( 0x1 << 11); 	// Rx FIFO trigger level in INT mode 
	spi_modecfg &= ~( 0x3ff << 19);	
	spi_modecfg |= ( 0x1 << 19);	// Counting of Tailing Bytes

	writel(spi_modecfg, spiregs + S3C_MODE_CFG);

	/*	4. Set SPI INT_EN register	*/
	writel(spi_inten, spiregs + S3C_SPI_INT_EN);
	writel(0x1f, spiregs + S3C_PENDING_CLR);

	/*	5. Set Packet Count configuration register	*/
	spi_packet = SPI_PACKET_CNT_EN;
	spi_packet |= 0xffff;
	writel(spi_packet, spiregs + S3C_PACKET_CNT);

	/*	6. Set Tx or Rx Channel on	*/
	spi_chcfg = readl(spiregs + S3C_CH_CFG);
	spi_chcfg |= SPI_CH_TXCH_OFF | SPI_CH_RXCH_OFF;
	spi_chcfg |= SPI_CH_TXCH_ON;
	spi_chcfg |= SPI_CH_RXCH_ON;
	writel(spi_chcfg, spiregs + S3C_CH_CFG);

	DbgPrintk("\thard reset success\n");
	DbgPrintk("\nafter spiregs init");
	print_reg();
	return TRUE;
}

/*	SPI_Deinit() - Unmap the register and stop the clock source	*/
BOOL SPI_Deinit(VOID)
{
	//----- 1. Stop the SPI clocks -----
	writel((readl(S3C_PCLK_GATE)|S3C_CLKCON_PCLK_SPI0),S3C_PCLK_GATE);
	iounmap(spiregs);
	return TRUE;
}

/*	spi_sendbyte() - Transmitt one byte through SPI0
 *	@data: the byte to be sent	*/
BOOL spi_sendbyte( BYTE data)
{
	BYTE chr;
	U32 spi_chcfg = spiregs + S3C_CH_CFG;

	if(!spi_wait_TX_ready())
	{
		printk("%s: failed to get tx channel.\n");
		return FALSE;
	}
	writel( data, spiregs + S3C_SPI_TX_DATA);
	while(!spi_wait_RX_ready());
	readl( spiregs + S3C_SPI_RX_DATA);
	return TRUE;
}

/*	spi_flush_fifo() - Clear the TxFIFO , RxFIFO and TX/RX shift register
 *	@spiregs: the SPI register address*/
VOID spi_flush_fifo(void *spiregs)
{
	/*	soft rest the spi controller, flush the FIFO	*/
	if(spi_wait_TX_done())
	{
		writel(readl(spiregs + S3C_CH_CFG) | SPI_CH_SW_RST, spiregs + S3C_CH_CFG);
		writel(readl(spiregs + S3C_CH_CFG) & ~SPI_CH_SW_RST, spiregs + S3C_CH_CFG);
	}
}

/*	spi_readbyte() - Read a byte received on SPI0	*/
BYTE spi_readbyte( void)
{
	U32 tmp;
	U32 spi_chcfg = spiregs + S3C_CH_CFG;
	BYTE ret;

	if(!spi_wait_TX_ready())
		return FALSE;
	spi_flush_fifo(spiregs);
	writel( 0xFF, spiregs + S3C_SPI_TX_DATA);
	
	if( spi_wait_RX_ready())
	{
		tmp = readl(spiregs + S3C_SPI_RX_DATA);
		ret = tmp & 0xff;
	}
	return ret;
}
	
/*	spi_wait_TX_ready() - wait for TX_READY and TX_DONE	*/
BOOL spi_wait_TX_ready( void)
{
	unsigned long loops = msecs_to_loops(10);
	U32 val = 0;
	do{
		val = readl(spiregs + S3C_SPI_STATUS);
	}while( !((val & SPI_STUS_TX_DONE) && (val & SPI_STUS_TX_FIFORDY)) && loops--);
	
	if( loops == 0)
		return FALSE;
	else
		return TRUE;
}

/*	spi_wait_TX_done() - wait for TX_DONE	*/
BOOL spi_wait_TX_done( void)
{
	unsigned long loops = msecs_to_loops(10);
	U32 val = 0;
	do{
		val = readl(spiregs + S3C_SPI_STATUS);
	}while( !(val & SPI_STUS_TX_DONE)  && loops--);
	
	if( loops == 0)
		return FALSE;
	else
		return TRUE;
}

/*	spi_wait_RX_ready() - wait for RX_READY	*/
BOOL spi_wait_RX_ready( void)
{
	unsigned long loops = msecs_to_loops(10);
	U32 val = 0;
	do{
		val = readl(spiregs + S3C_SPI_STATUS);
	}while( !(val & SPI_STUS_TRAILCNT_ZERO) && loops--);
	
	if( loops == 0)
		return FALSE;
	else
		return TRUE;
}
