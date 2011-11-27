/*
 *	Module Name:	MCPCAN.C
 *	Abstract:	mcp2510 driver 
 *	Date: 		2011/09/17
 *	By: 		Staufen Yang
 */ 

#include "mcpcan.h"
#include "spi_cmd.h"
#include "spi_control.h"
#include "can.h"

//#define DEBUG
#ifdef DEBUG
#define DbgPrintk printk
#else
#define DbgPrintk
#endif

/*	Variable define	*/
extern volatile void  __iomem *spiregs;
static dev_t devid;
static int can_major = CAN_MAJOR;
static int can_minor = CAN_MINOR;
unsigned int MCP_major = CAN_MAJOR;
struct mcp251x *device;
unsigned int MCP_irq = IRQ_EINT9; 
static int irqcount0 = 0;


/*	create and initiate the device structure	*/
static int mcp2510_device_init(void)
{
	int ret;
	device = kmalloc(sizeof(struct mcp251x),GFP_KERNEL);
	if(!device)
	{
		printk("%s: failed to kmalloc device struct\n",__FUNCTION__);
		ret = -ENOMEM;
		goto error_alloc;
	}
	memset(device, 0, sizeof(struct mcp251x));

	device->rxbin = device->rxbout = 0;
	init_MUTEX(&device->lock);
	init_MUTEX(&device->txblock);
	init_MUTEX(&device->rxblock);
	init_waitqueue_head(&device->wq);

	INIT_WORK(&device->irq_work, mcp2510_irq_handler);		
	device->spi_transfer_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);	
	if(!device->spi_transfer_buf)
	{
		ret = -ENOMEM;
		goto error_buf;
	}

	return 0;
	
error_buf:
	kfree(device);
error_alloc:
	return ret;	
}


/*	Initialize the mcp2510	*/
static void mcp2510_hw_Init(struct mcp251x *chip)
{
	unsigned char canstat = 0;
	mcp2510_hw_reset(chip);
	mcp2510_reset(chip);
	DbgPrintk("reset over\n");

	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

	/*	make sure we are in configuration mode	*/
	while( (mcp2510_read_reg( chip, CANSTAT)>>5) != 0x04 );

	/*	start configuration	*/
	mcp2510_write_reg(chip, BFPCTRL, BFPCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXRTSCTRL, TXRTSCTRL_INIT_VAL);
	mcp2510_write_reg(chip, CNF3, CNF3_INIT_VAL);
	mcp2510_write_reg(chip, CNF2, CNF2_INIT_VAL);
	mcp2510_write_reg(chip, CNF1, CNF1_INIT_VAL);
	mcp2510_write_reg(chip, CANINTE, CANINTE_INIT_VAL);
	mcp2510_write_reg(chip, CANINTF, CANINTF_INIT_VAL);
	mcp2510_write_reg(chip, EFLG, EFLG_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(0), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(1), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(2), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, RXBCTRL(0), RXB0CTRL_INIT_VAL);
	mcp2510_write_reg(chip, RXBCTRL(1), RXB1CTRL_INIT_VAL);

	/*	set mcp2510 into normal mode	*/
	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_NORMAL);

	printk("%s: Finished initiating MCP2510\n", __FUNCTION__);
}

/*	the upper level of interrupt handle function
 *	To handling the interrupt event faster,
 *	it just schedule the work structure and reset the timer then return.
 *	All the operations dealing with interrupt event will be done in mcp2510_irq_handler().	*/
static irqreturn_t mcp2510_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mcp251x *chip = dev_id;

	mod_timer(&(chip->timer), jiffies + RESET_DELAY);
	schedule_work(&chip->irq_work);
	return IRQ_HANDLED;
}

/*	The bottom level of interrupt handle function
 *	It actually does the dirty work dealing the interrupt event.:)	*/
static void mcp2510_irq_handler(void *dev_id)
{
	struct mcp251x *chip = device;
	BYTE intf, rxs;

	while(1)
	{
		intf = mcp2510_read_reg(chip, CANINTF);
		if (intf == 0x00)
			break;
		if (intf & CANINTF_ERRIF)
		{
			BYTE eflg = mcp2510_read_reg(chip, EFLG);
			printk("%s: EFLG == 0x%02x.\n", __FUNCTION__, eflg);

			if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR))
			{
				mcp2510_write_reg(chip, EFLG, 0x00);
			}
		}

/*	We haven't use the transmitt interrupt yet
 *	So the three if-statement fellow was ignored
 *	For more extensibility, we just comment it*/
#if 0
		if (intf & CANINTF_TX0IF)
			mcp2510_send_frame(chip, 0);
		if (intf & CANINTF_TX1IF)
			mcp2510_send_frame(chip, 1);
		if (intf & CANINTF_TX2IF)
			mcp2510_send_frame(chip, 2);
#endif

		if (intf & CANINTF_RX0IF)
			mcp2510_read_frame(chip, 0);
		if (intf & CANINTF_RX1IF)
			mcp2510_read_frame(chip, 1);

		mcp2510_modify_bit(chip, CANINTF, intf, 0x00);

		/* If ring buffer of receive is not empty, wake up the read queue. */
		if (chip->rxbin != chip->rxbout)
		    wake_up_interruptible(&chip->wq);
	}
}

/*	When timer is triggered, this function will be called.
 *	Remember to modify timer at the last	*/
void mcp2510_timer_callback(unsigned int data)
{
	struct mcp251x *chip =(struct mcp251x *)data;
	mcp2510_hw_Init(chip);
#if 0
	mcp2510_hw_reset(chip);
	mcp2510_reset(chip);
	DbgPrintk("reset over\n");

	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

	/*	make sure we are in configuration mode	*/
	while( (mcp2510_read_reg( chip, CANSTAT)>>5) != 0x04 );

	/*	start configuration	*/
	mcp2510_write_reg(chip, BFPCTRL, BFPCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXRTSCTRL, TXRTSCTRL_INIT_VAL);
	mcp2510_write_reg(chip, CNF3, CNF3_INIT_VAL);
	mcp2510_write_reg(chip, CNF2, CNF2_INIT_VAL);
	mcp2510_write_reg(chip, CNF1, CNF1_INIT_VAL);
	mcp2510_write_reg(chip, CANINTE, CANINTE_INIT_VAL);
	mcp2510_write_reg(chip, CANINTF, CANINTF_INIT_VAL);
	mcp2510_write_reg(chip, EFLG, EFLG_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(0), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(1), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, TXBCTRL(2), TXBnCTRL_INIT_VAL);
	mcp2510_write_reg(chip, RXBCTRL(0), RXB0CTRL_INIT_VAL);
	mcp2510_write_reg(chip, RXBCTRL(1), RXB1CTRL_INIT_VAL);

	/*	set mcp2510 into normal mode	*/
	mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_NORMAL);

	printk("%s: Finished initiating MCP2510\n", __FUNCTION__);
#endif
	mod_timer(&(chip->timer), jiffies + RESET_DELAY);
}

static int mcp2510_open(struct inode *inode,struct file *filp)
{
	int minor_num;
	int eint_irq;
	int ret;
	struct mcp251x *chip = device;
	filp->private_data = chip;
	SPI_Init();
	mcp2510_hw_Init(chip);
	DbgPrintk("%s: Init mcp2510 over\n", __FUNCTION__);
#if 1
	/*	register irq	*/
	if(irqcount0 == 0)
	{
		eint_irq = IRQ_EINT9;
		set_irq_type(eint_irq, IRQT_FALLING);
		ret = request_irq(eint_irq, mcp2510_irq, IRQF_DISABLED, DRIVER_NAME, chip);
		if (ret < 0)
		{
			DbgPrintk("%s: request EINT9 failed.\n", __FUNCTION__);
			goto error_irq;
		}
		
		writel(readl(S3C_EINTMASK)&(~(1<<12)),S3C_EINTMASK);
		writel(readl(S3C_EINTPEND)|(1<<12),S3C_EINTPEND);
	}

	irqcount0++;
#endif

	/*	init the timer	*/
	init_timer(&(chip->timer));
	(chip->timer).data = chip;
	(chip->timer).function = &mcp2510_timer_callback;
	(chip->timer).expires = jiffies + RESET_DELAY;
	add_timer(&(chip->timer));

	return 0;
error_irq:
	return ret;
}
static int mcp2510_release(struct inode *inode, struct file *filp)
{
	struct mcp251x *chip = filp->private_data;

	del_timer(&(chip->timer));
#if 1
	irqcount0--;
	if (irqcount0 == 0)
	{
		free_irq(IRQ_EINT9, chip);
	}
#endif
	return 0;
}

static ssize_t mcp2510_read(struct file *file, char __user *buf, size_t count, loff_t *ofs)
{
	struct mcp251x *chip = file->private_data;
	struct can_frame *frame;

	if (count != sizeof(struct can_frame))
		return -EINVAL;

	if (down_interruptible(&chip->rxblock))
		return -ERESTARTSYS;

	/*	if the rx ring buffer is empty, wait	*/
	while (chip->rxbin == chip->rxbout)
	{
		up(&chip->rxblock);
		if (file->f_flags & O_NONBLOCK)
	    		return -EAGAIN;
		if (wait_event_interruptible(chip->wq, (chip->rxbin != chip->rxbout)))
			return -ERESTARTSYS;
		if (down_interruptible(&chip->rxblock))
			return -ERESTARTSYS;
	}

	frame = &chip->rxb[chip->rxbout];
	if (copy_to_user(buf, frame, sizeof(struct can_frame))) 
	{
		up(&chip->rxblock);
		return -EFAULT;
	}

	/*	update the read pointer of rx ring buffer	*/
	chip->rxbout++;
	if(chip->rxbout >= MCP251X_BUF_LEN)
	{
		chip->rxbout = 0;
	}
	
	up(&chip->rxblock);
	return count;
}

static ssize_t mcp2510_write (struct file *filp, const char __user *buf, size_t count, loff_t *ofs)
{
	struct mcp251x *chip = filp->private_data;
	struct can_frame *frame = &chip->tx;
	int ret;
	int i;
	BYTE txreq;

	if (count < sizeof(struct can_frame))
	{
		printk("%s: write byte is too short.\n", __FUNCTION__);
		return -EINVAL;
	}
	if (down_interruptible(&chip->txblock))
	{
		printk("%s: cannot get txblock.\n", __FUNCTION__);
		return -ERESTARTSYS;
	}
	ret = copy_from_user(frame, buf, sizeof(struct can_frame));

	/*	Obviously, I just use the TX0 buffer.
 	*	Forgive my laziness	*/
	mcp2510_send_frame(chip, 0);
	mcp2510_rts(chip, 0);
	up(&chip->txblock);
	return count;
}

static int mcp2510_ioctl(struct inode *inode, struct file *file, 
			 unsigned int cmd, unsigned long arg)
{
	struct mcp251x *chip = file->private_data;
	int ret;
	
	switch(cmd)
	{
		case IOCTL_SET_LOOP:
			mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_LOOPBACK);
			ret = mcp2510_read_reg(chip, CANSTAT);
			if(ret & 0x40)
				printk("%s: have set mcp2510 into loopback mode.\n", __FUNCTION__);
			break;

		case IOCTL_SET_NORMAL:
			mcp2510_modify_bit(chip, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_NORMAL);
			break;

		case IOCTL_RESET:
			mcp2510_hw_Init(chip);
			break;
		default:
			printk("%s: Incorrect command.\n", __FUNCTION__);
			return -EINVAL;

	}
	return 0;
}



/*	File operations define	*/
struct file_operations Fops = {
	.open = mcp2510_open,
	.release = mcp2510_release,
	.read = mcp2510_read,
	.write = mcp2510_write,
	.ioctl = mcp2510_ioctl,
};

static int __init mcpcan_init_module(void)
{
	int ret;
	int eint_irq;
	/*	Register the character device (at least try) 	*/
	ret = register_chrdev(MCP_major,"mcp2510",&Fops);
	if(ret < 0) 
	{
		DbgPrintk("%s: device register failed with %d.\n",__FUNCTION__, ret);
		goto err_reg_chrdev;
        }

	if(MCP_major == 0)
		MCP_major = ret;

    	mcp2510_device_init();
#if 0
	/*	register irq	*/
	if(irqcount0 == 0)
	{
		eint_irq = IRQ_EINT9;
		set_irq_type(eint_irq, IRQT_FALLING);
		ret = request_irq(eint_irq, mcp2510_irq, IRQF_DISABLED, DRIVER_NAME, NULL);
		if (ret < 0)
		{
			DbgPrintk("%s: request EINT9 failed.\n", __FUNCTION__);
			goto error_irq;
		}
		
		writel(readl(S3C_EINTMASK)&(~(1<<12)),S3C_EINTMASK);
		writel(readl(S3C_EINTPEND)|(1<<12),S3C_EINTPEND);
	}

	irqcount0++;
#endif
	printk ("%s The major device number is %d.\n",
				"Registeration is a success", 
				MCP_major);
		          
	printk ("If you want to talk to the device driver,\n");
	printk ("you'll have to create a device file. \n");
	printk ("We suggest you use:\n");
	printk ("mknod /dev/%s c %d 0\n", "can",MCP_major);
	printk ("The device file name is important, because\n");
	printk ("the ioctl program assumes that's the\n");
	printk ("file you'll use.\n");

	return 0;

//error_irq:
//	unregister_chrdev( MCP_major,"mcp2510");
err_reg_chrdev:
	return ret;
}

static void __exit mcp2510_cleanup_module(void)
{
	struct mcp251x *chip = device;
#if 0
	irqcount0--;
	if (irqcount0 == 0)
	{
		free_irq(IRQ_EINT9, chip);
	}
#endif
	unregister_chrdev( MCP_major,"mcp2510");

	kfree(chip->spi_transfer_buf);
	kfree(chip);
	printk("MCPCAN release success.\n");
}

module_init(mcpcan_init_module);
module_exit(mcp2510_cleanup_module);
MODULE_LICENSE("Dual BSD/GPL");  //should always exist or you’ll get a warning
MODULE_AUTHOR("StaufenYANG"); //optional
MODULE_DESCRIPTION("MCP2510_DRIVER"); //optional
