/*
 *	Module Name:	MCPCAN.H
 *	Abstract:	Define the constants for mcp2510 operation and the device structure
 *	Environment:	MCP2510
 *	Date: 		2011/09/17
 *	By: 		Staufen Yang
 */ 

#ifndef	MCPCAN_H
#define	MCPCAN_H
#include <linux/init.h>		/* for module_init */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/errno.h> 	/* for -EBUSY */
#include <linux/slab.h>
#include <linux/ioport.h>	/* for verify_area */
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/types.h>

#include <linux/module.h>
#include <linux/fcntl.h>
#include <linux/poll.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/arch/irqs.h>
#include <asm/mach/irq.h>
#include <asm/signal.h>
#include <asm/siginfo.h>
#include <asm/uaccess.h>  	/* for get_user and put_user */
#include <asm/fcntl.h>
#include <asm/cache.h>
#include <asm/atomic.h>
#include <linux/syscalls.h>		//2.6 kernel
#include <linux/timer.h>
#include <asm/arch/regs-gpio.h>

#include <linux/cdev.h>
#include "can.h"

/* 	Buffer size required for the largest SPI transfer 	*/
#define SPI_TRANSFER_BUF_LEN	(2*(7 + CAN_FRAME_MAX_DATA_LEN))

/* Buffer size required for ring buffer of receive and send */
#define MCP251X_BUF_LEN		20

#define CAN_MAJOR   98
#define CAN_MINOR   0

#define DRIVER_NAME "mcp2510"

/*	mcp2510 device structure	*/
struct mcp251x {
	struct cdev cdev;
	struct semaphore lock;		 /* semaphore for spi bus share. */
	struct semaphore rxblock;	 /* semaphore for ring buffer of receive. */
	struct semaphore txblock;	 /* semaphore for ring buffer of send. */
	
	BYTE *spi_transfer_buf;	 /* temp buffer for spi bus transfer. */

	struct can_frame rxb[MCP251X_BUF_LEN]; /* ring buffer for receive. */
	struct can_frame tx;
//	struct can_frame txb[MCP251X_BUF_LEN]; /* ring buffer for send. */
	
//	int txbin;			 /* pos of in for ring buffer of sned. */
//	int txbout;			 /* pos of out for ring buffer of send. */
	int rxbin;			 /* pos of in for ring buffer of receive. */
	int rxbout;			 /* pos of out for ring buffer of receive. */
	
	int bit_rate;		 /* save bit rate of current set. */
//	int count;			 /* count of the device opened. */
    
	wait_queue_head_t wq;	 /* queue for read process. */
    
	struct work_struct irq_work; /* bottom half of interrupt task. */
	struct timer_list timer;	/* timer to do resetting*/
//    struct can_filter filter;	 /* save the filter data of current set. */
};

/*	Register address of mcp2510	*/
#define CANSTAT           0x0e
#define CANSTAT_IRQ_NONE	0x00
#define CANSTAT_IRQ_ERROR	0x02
#define CANSTAT_IRQ_WAKE	0x04
#define CANSTAT_IRQ_TXB1	0x06
#define CANSTAT_IRQ_TXB2	0x08
#define CANSTAT_IRQ_TXB3	0x0a
#define CANSTAT_IRQ_RXB1	0x0c
#define CANSTAT_IRQ_RXB2	0x0e

#if 0
#define CAN_STATE_TX2IF		0x01
#define CAN_STATE_TX2REQ	0x02
#define CAN_STATE_TX1IF		0x04
#define CAN_STATE_TX1REQ	0x08
#define CAN_STATE_TX0IF       0x10
#define CAN_STATE_TX0REQ	0x20
#define CAN_STATE_RX1IF	0x40
#define CAN_STATE_RX0IF	0x80
#endif

#define CANCTRL           0x0f
#define CANCTRL_REQOP_MASK        0xe0
#define CANCTRL_REQOP_CONF        0x80
#define CANCTRL_REQOP_LISTEN_ONLY 0x60
#define CANCTRL_REQOP_LOOPBACK    0x40
#define CANCTRL_REQOP_SLEEP       0x20
#define CANCTRL_REQOP_NORMAL      0x00
#define CANCTRL_OSM               0x08
#define CANCTRL_ABAT              0x10

#define TEC           0x1c
#define REC           0x1d

#define CNF1          0x2a
#define CNF2          0x29
#define CNF2_BTLMODE  0x80
#define CNF3          0x28
#define CNF3_SOF      0x08
#define CNF3_WAKFIL   0x04
#define CNF3_PHSEG2_MASK 0x07

#define BFPCTRL	0x0c
#define TXRTSCTRL	0x0d

#define CANINTE       0x2b
#define CANINTE_MERRE 0x80
#define CANINTE_WAKIE 0x40
#define CANINTE_ERRIE 0x20
#define CANINTE_TX2IE 0x10
#define CANINTE_TX1IE 0x08
#define CANINTE_TX0IE 0x04
#define CANINTE_RX1IE 0x02
#define CANINTE_RX0IE 0x01

#define CANINTF       0x2c
#define CANINTF_MERRF 0x80
#define CANINTF_WAKIF 0x40
#define CANINTF_ERRIF 0x20
#define CANINTF_TX2IF 0x10
#define CANINTF_TX1IF 0x08
#define CANINTF_TX0IF 0x04
#define CANINTF_RX1IF 0x02
#define CANINTF_RX0IF 0x01

#define EFLG          0x2d
#define EFLG_RX1OVR   0x80
#define EFLG_RX0OVR   0x40

#define TXBCTRL(n)	((n * 0x10) + 0x30)
#define TXBCTRL_TXREQ	0x08
#define TXBCTRL_TXPRI(n) (n)
#define TXBCTRL_TXERR	 (1 << 4)
#define TXBCTRL_MLOA     (1 << 5)
#define TXBCTRL_ABTF     (1 << 6)
#define TXBnSIDH(n)		(0x31 + (n * 0x10))
#define TXB_EXIDE_ENABLE	0x08
#define TXB_RTR_DATA	0xbf

#define RXBCTRL(n)  ((n * 0x10) + 0x60)
#define RXBCTRL_MASK   0x60
#define RXBCTRL_RXRTR  0x08
#define RXBCTRL_BULK	(1 << 2)
#define RXBCTRL_RXM_MACH_ALL	(0 << 6)
#define RXBCTRL_RXM_MACH_STD	(1 << 6)
#define RXBCTRL_RXM_MACH_EXT	(2 << 6)
#define RXBCTRL_TXM_MACH_OFF	(3 << 6)
#define RXBCTRL_FILHIT_MASK    0x07

#define RXM_BASE(n)   (0x20 + (n *  4))
#define RXF_BASE(n)   ((n>2)?(0x10 + (n-3)*4):(0x00 + (n*4)))
#define RXBnSIDH(n)	(0x61 + (n * 0x10))

/*	ioctl handle param	*/
//#define IOCTL_SET_LOOP _IO(CAN_MAJOR, 0)
//#define IOCTL_SET_NORMAL _IO(CAN_MAJOR, 1)
#define IOCTL_SET_LOOP 		0
#define IOCTL_SET_NORMAL	1
#define IOCTL_RESET		2

/*	wait time to reset	*/
#define RESET_DELAY		HZ*15

/*	Define the initial value for the mcp2510's configuration register	*/

#define	BFPCTRL_INIT_VAL	(0x00)		// Buffer Pin Full Control : disabled
#define	TXRTSCTRL_INIT_VAL	(0x00)	// Transmitter Request to Send pin : Digital Input

#define	CNF1_INIT_VAL		(0x04)
#define	CNF2_INIT_VAL		(0x90)
#define	CNF3_INIT_VAL		(0x02)		// Configure the bit timing : 100kHZ

#define	CANINTE_INIT_VAL	(0x23)    	// Enable the error interrupt and receive interrupt
#define	CANINTF_INIT_VAL	(0x00)		// Clear up the interrupt flags

#define	EFLG_INIT_VAL		(0x00)		// Reset Error Flag Register
#define	TXBnCTRL_INIT_VAL	(0x00)		// Transmitter Buffer Control

#define	RXB0CTRL_INIT_VAL	(0x64)		// Receiver Buffer 0 Control Register : Turn mask/filter off, receiver all message, Rollover enable
#define	RXB1CTRL_INIT_VAL	(0x60)		// Receiver Buffer 1 Control Register : Turn mask/filter off, receiver all message


/*	function declaration	*/
static int mcp2510_device_init(void);
static void mcp2510_hw_Init(struct mcp251x *chip);
static irqreturn_t mcp2510_irq(int irq, void *dev_id, struct pt_regs *regs);
static void mcp2510_irq_handler(void *dev_id);
static int mcp2510_open(struct inode *inode,struct file *filp);
static int mcp2510_release(struct inode *inode, struct file *filp);
static ssize_t mcp2510_read(struct file *file, char __user *buf, size_t count, loff_t *ofs);
static ssize_t mcp2510_write (struct file *filp, const char __user *buf, size_t count, loff_t *ofs);
static int mcp2510_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);


#endif
