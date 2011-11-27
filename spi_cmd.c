/*
 *	Module Name:	SPI_CMD.C
 *	Abstract:	Basic Operation to mcp2510 using spi bus
 *	Notes:		Presently, only the SPI Channel 0 is supported.
 *	Environment:	MCP2510
 *	Date: 		2011/09/16
 *	By: 		Staufen Yang
 */ 


#include	<asm-arm/plat-s3c64xx/s3c6410.h>
#include 	<linux/kernel.h>
#include	<asm/io.h>
#include 	"spi_control.h"
#include	"spi_cmd.h"
#include 	"mcpcan.h"

#define _SPI_CMD_SOURCE_

//#define DEBUG
#ifdef DEBUG
#define DbgPrintk printk
#else
#define DbgPrintk
#endif

extern volatile void  __iomem *spiregs;

/*	mcp2510_read_reg() - Read a specified register of mcp2510
 *	@chip: the device structure of mcp2510
 *	@addr: address of register	*/
U8 mcp2510_read_reg( struct mcp251x *chip, U8 addr)
{
	U8 ret;
	
	down(&chip->lock);			//try to get the spi bus lock
	S3C_SPI_ACT(spiregs);			//set the CS pin low
	
	spi_sendbyte(SPI_CMD_READ);
	spi_sendbyte(addr);
	ret = spi_readbyte();

	S3C_SPI_DEACT(spiregs);			//disselect

	/*	As SPI is working full duplex.
 	*	RxFIFO will get dummy bytes while we are sending bytes.
 	*	In order to prevent the FIFO overflow,
 	*	we flush the FIFOs after we complete a operation to mcp2510*/
	spi_flush_fifo(spiregs);
	up(&chip->lock);			//release the spi bus lock
	return ret;
	
}

/*	mcp2510_write_reg() - Write to a specified register of mcp2510
 *	@chip: the device structure of mcp2510
 *	@addr: address of register
 *	@data: byte to be written	*/
VOID mcp2510_write_reg( struct mcp251x *chip, U8 addr, BYTE data)
{
	int ret = 0;
	down(&chip->lock);
	S3C_SPI_ACT(spiregs);

	ret = spi_sendbyte(SPI_CMD_WRITE);
	ret = spi_sendbyte(addr);
	ret = spi_sendbyte(data);

	S3C_SPI_DEACT(spiregs);
	spi_flush_fifo(spiregs);
	up(&chip->lock);
}

/*	mcp2510_modify_bit() - Set the specified bit of mcp2510's register
 *	@chip: the device structrue of mcp2510
 *	@addr: address of register
 *	@mask: register mask indicating the bit to be modified
 *	@data: bit to be written	*/
VOID mcp2510_modify_bit( struct mcp251x *chip, U8 addr, U8 mask, U8 data)
{
	down(&chip->lock);
	S3C_SPI_ACT(spiregs);

	spi_sendbyte(SPI_CMD_BITMOD);
	spi_sendbyte(addr);
	spi_sendbyte(mask);
	spi_sendbyte(data);

	S3C_SPI_DEACT(spiregs);
	spi_flush_fifo(spiregs);
	up(&chip->lock);
}

/* 	mcp2510_reset() - Send the reset command to mcp2510
 *  	@chip: the device structure of mcp2510, for getting its spi bus lock	*/
VOID mcp2510_reset( struct mcp251x *chip)
{
	down(&chip->lock);
	S3C_SPI_ACT(spiregs);

	spi_sendbyte(SPI_CMD_RESET);

	S3C_SPI_DEACT(spiregs);
	spi_flush_fifo(spiregs);
	up(&chip->lock);
}

/* 	mcp2510_hw_reset() - Set the reset pin of mcp2510 low to make a hareware reset.
 *	@chip: the device structure of mcp2510	*/
VOID mcp2510_hw_reset( struct mcp251x *chip)
{
	writel((readl(S3C_GPKDAT)&0xfffe), S3C_GPKDAT);	//set reset port low
	mdelay(1);
	writel((readl(S3C_GPKDAT)|0x0001), S3C_GPKDAT);	//set reset port high
}

/*	mcp2510_rts() - Request the specified transmit buffer to send CAN frame
 *	@chip: the device structure of mcp2510
 *	@buf_idx: indicate the tranmit buffer, which is between [0...2]	*/
VOID mcp2510_rts(struct mcp251x *chip, int buf_idx)
{
	down(&chip->lock);
	S3C_SPI_ACT(spiregs);

	spi_sendbyte(SPI_CMD_RTS(buf_idx));

	S3C_SPI_DEACT(spiregs);
	spi_flush_fifo(spiregs);
	up(&chip->lock);
}

/*	mcp2510_set_bit_rate() - set the CAN boud rate
 *	Note: JUST a interface function for more expansibility. NOT be used in this version of driver
 *	@chip: the device structure of mcp2510
 *	@bit_rate: the boud rate need to be set	*/
BOOL mcp2510_set_bit_rate(struct mcp251x *chip, int bit_rate)
{
	int f_osc = 8000000;	//oscillator frequence is 8Mhz
	int ps1 = 3;		//phase segment1
	int ps2 = 3;		//phase segment2
	int propseg = 1;	//propagation time segment
	int sjw = 1;		//sync segment
	int brp = f_osc / (2 * bit_rate * (ps1 + ps2 + propseg + sjw)) - 1;
	BYTE cnf1, cnf2, cnf3;
	BYTE canctrl;

	/* The CAN bus bit time (tbit) is determined by:
	*   tbit = (SyncSeg + PropSeg + PS1 + PS2) * TQ
	*   TQ is  determined by :
	*   TQ = 2 * (BRP + 1) * Tosc
	*   with sequence chart as follow:
	*	__________________________________________________________________________
	*     |  sync   | propagation time segment  |  phase segment1  |  phase segment2  |
	*     |_________|___________________________|__________________|__________________|
	*
	*	sampling point is just between phase segment1 and phase segment2
	*   
	*   where:
	*	SyncSeg = 1
	*	sample point (between PS1 and PS2) must be at 60%-70% of the bit time
	*	PropSeg + PS1 >= PS2
	*	PropSeg + PS1 >= Tdelay
	*	PS2 > SJW
	*	1 <= PropSeg <= 8, 1 <= PS1 <=8, 2 <= PS2 <= 8
	* 	SJW = 1 is sufficient in most cases.
	* 	Tdelay is usually 1 or 2 TQ.
	*/

	cnf1 = (((sjw - 1) << 6) && 0xc0 ) | (brp && 0x3f);
	cnf2 = ((propseg - 1) && 0x07) | (((ps1 - 1) && 0x07) << 3) | ( 0x10 << 6);
	cnf3 = ( ps2 - 1) && 0x03;

	/*  save the state and set mcp2510 to config mode */
	canctrl = mcp2510_read_reg(chip, CANCTRL);
	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

	mcp2510_write_reg(chip, CNF1, cnf1);
	mcp2510_write_reg(chip, CNF2, cnf2);
	mcp2510_write_reg(chip, CNF3, cnf3);

	/* restore the state */
	mcp2510_write_reg(chip, CANCTRL, canctrl);

	chip->bit_rate = bit_rate;

	/* set mcp2510 to normal mode */
	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_NORMAL);

	return 0;
}

/*	mcp2510_set_filter() - set the CAN receive filter
 *	Note: JUST a interface function for more expansibility. NEED to be completed
 *	@chip: the device structure of mcp2510
 *	@can_filter: the can_filter need to be set	*/
BOOL mcp2510_set_filter( struct mcp251x *chip, struct can_filter *filter)
{
	
}

/*	mcp2510_send_frame() - Load the CAN frame to be sent
 *	Note: JUST load the CAN frame. The transmit request will be set by mcp2510_rts()
 *	@chip: the device structure of mcp2510
 *	@buf_idx: indicate which transmit buffer to be used	*/
BOOL mcp2510_send_frame( struct mcp251x *chip, int buf_idx)
{
	BYTE *buf = chip->spi_transfer_buf;
	struct can_frame *frame;
	int count = 0;
	int ret;

	frame = &(chip->tx);

	down(&chip->lock);
	S3C_SPI_ACT(spiregs);
	if(frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
		frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
	spi_sendbyte(SPI_CMD_WRITE);
	spi_sendbyte(TXBnSIDH(buf_idx));
	spi_sendbyte(frame->info_id);
	spi_sendbyte((frame->src_addr & 0xe0) | TXB_EXIDE_ENABLE | ((frame->src_addr & 0x18) >> 3));
	spi_sendbyte((frame->src_addr << 5) | (frame->dest_addr >> 3));
	spi_sendbyte((frame->dest_addr << 5) | frame->info_order);
	spi_sendbyte(frame->can_dlc & TXB_RTR_DATA);
	for(count = 0; count < frame->can_dlc; ++count)
	{
		spi_sendbyte(frame->data[count]);
	}
	S3C_SPI_DEACT(spiregs);
	up(&chip->lock);
		
	spi_flush_fifo(spiregs);

	return 0;
}

/*	mcp2510_read_frame() - Read the CAN frame received by mcp2510
 *	@chip: the device structure of mcp2510
 *	@buf_idx: indicate which receive buffer to be read	*/
BOOL mcp2510_read_frame( struct mcp251x *chip, int buf_idx)
{
	BYTE *buf = chip->spi_transfer_buf;
	int count = 0;
	struct can_frame *frame;

	if (down_interruptible(&chip->rxblock))
		return -ERESTARTSYS;
	
	frame = &chip->rxb[chip->rxbin];
	memset(buf, 0, SPI_TRANSFER_BUF_LEN);

	down(&chip->lock);
	S3C_SPI_ACT(spiregs);
	
	spi_sendbyte(SPI_CMD_READ);
	spi_sendbyte(RXBnSIDH(buf_idx));
	for(; count < 13; ++count)
	{
		buf[count] = spi_readbyte();
	}

	frame->info_id = buf[0];
	frame->src_addr = (buf[1] & 0xe0) | ((buf[1] & 0x03) << 3) | ((buf[2] & 0xe0) >> 5);
	frame->dest_addr = ((buf[2] & 0x1f) << 3) | ((buf[3] & 0xe0) >> 5);
	frame->info_order = buf[3] & 0x1f;
	frame->can_dlc = buf[4] & 0x0f;
	memcpy(frame->data, buf + 5, CAN_FRAME_MAX_DATA_LEN);
	
	S3C_SPI_DEACT(spiregs);
	spi_flush_fifo(spiregs);
	up(&chip->lock);

    /* update pos of ring buffer */
	chip->rxbin++;
	if (chip->rxbin >= MCP251X_BUF_LEN)
		chip->rxbin = 0;
			
	up(&chip->rxblock);

	return 0;
}


