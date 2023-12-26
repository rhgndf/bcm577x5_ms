#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/freezer.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/highmem.h>
#include <asm/byteorder.h>
#include <linux/swab.h>
#include <linux/memstick.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/ctype.h>
#include <linux/random.h>
#include "bcm577x5_ms.h"

#define DRV_NAME "bcm577x5_ms"

static int enable_dma = 2;
module_param(enable_dma, int, S_IRUGO);
MODULE_PARM_DESC(enable_dma,
		 "Enable usage of the DMA (0 = no, 1 = yes, 2 = auto,default)");
//#define dev_dbg dev_info

static const struct pci_device_id bcm577x5_pci_id_tbl[] = {
	{
		PCI_VDEVICE(BROADCOM, 0x16be),
	},
	{},
};

static const char *tpc_names[] = {
	"MS_TPC_READ_MG_STATUS",  "MS_TPC_READ_LONG_DATA",
	"MS_TPC_READ_SHORT_DATA", "MS_TPC_READ_REG",
	"MS_TPC_READ_QUAD_DATA",  "INVALID",
	"MS_TPC_GET_INT",	  "MS_TPC_SET_RW_REG_ADRS",
	"MS_TPC_EX_SET_CMD",	  "MS_TPC_WRITE_QUAD_DATA",
	"MS_TPC_WRITE_REG",	  "MS_TPC_WRITE_SHORT_DATA",
	"MS_TPC_WRITE_LONG_DATA", "MS_TPC_SET_CMD",
};

const char *memstick_debug_get_tpc_name(int tpc)
{
	return tpc_names[tpc - 1];
}

static inline u32 bcm577x5_reg_readl(struct bcm577x5_device *dev, int address)
{
	u32 value = readl(dev->mmio + address);
	return value;
}
static inline u16 bcm577x5_reg_readw(struct bcm577x5_device *dev, int address)
{
	u32 value = readw(dev->mmio + address);
	return value;
}
static inline u8 bcm577x5_reg_readb(struct bcm577x5_device *dev, int address)
{
	u32 value = readb(dev->mmio + address);
	return value;
}

static inline void bcm577x5_reg_writel(struct bcm577x5_device *dev, int address,
				       u32 value)
{
	writel(value, dev->mmio + address);
}
static inline void bcm577x5_reg_writew(struct bcm577x5_device *dev, int address,
				       u16 value)
{
	writew(value, dev->mmio + address);
}
static inline void bcm577x5_reg_writeb(struct bcm577x5_device *dev, int address,
				       u8 value)
{
	writeb(value, dev->mmio + address);
}

static void dump_regs(struct bcm577x5_device *dev)
{
	int i;
	u32 val;

	for (i = 0; i <= 0x4C; i += 4) {
		val = bcm577x5_reg_readl(dev, i);
		dev_dbg(&dev->pci_dev->dev, "Rval at %02X: %08X\n", i, val);
	}

	for (i = 0x180; i <= 0x1B0; i += 4) {
		val = bcm577x5_reg_readl(dev, i);
		dev_dbg(&dev->pci_dev->dev, "Rval at %02X: %08X\n", i, val);
	}
}

static int bcm577x5_reg_waitb(struct bcm577x5_device *dev, int address, u8 mask,
			      u8 value, int timeout)
{
	unsigned long wait_time = jiffies + msecs_to_jiffies(timeout);
	u8 reg;

	do {
		reg = bcm577x5_reg_readb(dev, address);
		if ((reg & mask) == value)
			return 0;

		cpu_relax();

	} while (time_before(jiffies, wait_time));

	return -ETIME;
}

static int bcm577x5_reg_waitl(struct bcm577x5_device *dev, int address,
			      u32 mask, u32 value, int timeout)
{
	unsigned long wait_time = jiffies + msecs_to_jiffies(timeout);
	u32 reg;

	do {
		reg = bcm577x5_reg_readl(dev, address);
		if ((reg & mask) == value)
			return 0;

		cpu_relax();

	} while (time_before(jiffies, wait_time));

	return -ETIME;
}

static int bcm577x5_reg_waitl_different(struct bcm577x5_device *dev,
					int address, u32 value, int timeout)
{
	unsigned long wait_time = jiffies + msecs_to_jiffies(timeout);
	u32 reg;

	do {
		reg = bcm577x5_reg_readl(dev, address);
		if (reg != value)
			return 0;

		cpu_relax();

	} while (time_before(jiffies, wait_time));

	return -ETIME;
}

static int bcm577x5_ms_reset(struct bcm577x5_device *dev, u8 reset)
{
	bcm577x5_reg_writeb(dev, BCM577x5_MS_SOFTWARE_RESET, reset);

	return bcm577x5_reg_waitb(dev, BCM577x5_MS_SOFTWARE_RESET, reset, 0,
				  1000);
}

static int bcm577x5_ms_power(struct bcm577x5_device *dev, int power)
{
	switch (power) {
	case MEMSTICK_POWER_ON:
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL,
				    BCM577x5_MS_POWER_330);
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL,
				    BCM577x5_MS_POWER_330 |
					    BCM577x5_MS_POWER_ON);
		return bcm577x5_reg_waitb(dev, BCM577x5_MS_POWER_CONTROL,
					  BCM577x5_MS_POWER_ON,
					  BCM577x5_MS_POWER_ON, 1000);
	case MEMSTICK_POWER_OFF:
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL, 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static int bcm577x5_ms_clock(struct bcm577x5_device *dev, int clock_hz)
{
	//Fixed divisor (2) for now, does 24MHz
	int err;
	u16 divisor, divisor_lo, divisor_hi, clock_reg;

	divisor = dev->base_clock / clock_hz / 2;
	divisor_lo = divisor & BCM577x5_MS_DIV_MASK;
	divisor_hi = (divisor & BCM577x5_MS_DIV_HI_MASK) >> 8;
	clock_reg = (divisor_lo << BCM577x5_MS_DIVIDER_SHIFT) |
		    (divisor_hi << BCM577x5_MS_DIVIDER_HI_SHIFT);

	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL, clock_reg);
	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL,
			    clock_reg | BCM577x5_MS_CLOCK_INT_EN);
	err = bcm577x5_reg_waitb(dev, BCM577x5_MS_CLOCK_CONTROL,
				 BCM577x5_MS_CLOCK_INT_STABLE,
				 BCM577x5_MS_CLOCK_INT_STABLE, 1000);
	if (err)
		return err;
	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL,
			    clock_reg | BCM577x5_MS_CLOCK_INT_EN |
				    BCM577x5_MS_CLOCK_CARD_EN);
	return 0;
}
static int bcm577x5_ms_init(struct bcm577x5_device *dev)
{
	/* Reset the controller */
	bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_ALL);
	
	/* Power cycle the controller */
	bcm577x5_ms_power(dev, MEMSTICK_POWER_OFF);
	bcm577x5_ms_power(dev, MEMSTICK_POWER_ON);
	
	/* Set clock divider to 2 */
	bcm577x5_ms_clock(dev, 24000000);

	/* Set timeout */
	bcm577x5_reg_writeb(dev, BCM577x5_MS_TIMEOUT_CONTROL, 0xE);

	bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_CMD);
	bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA);

	/* Re-enable interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE,
			    BCM577x5_MS_INT_DEFAULT);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE,
			    BCM577x5_MS_INT_DEFAULT);

	return 0;
}

static irqreturn_t bcm577x5_irq(int irq, void *data)
{
	struct bcm577x5_device *dev = (struct bcm577x5_device *)data;
	irqreturn_t ret = IRQ_NONE;
	u32 val, status;
	int i;

	val = bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS);
	if (val & BCM577x5_MS_STATUS_INT_ASSERTED) {
		status = bcm577x5_reg_readl(dev, BCM577x5_MS_INT_STATUS);

		if (status & 0x100) {
			bcm577x5_reg_writel(
				dev, BCM577x5_MS_INT_ENABLE,
				bcm577x5_reg_readl(dev,
						   BCM577x5_MS_INT_ENABLE) &
					~0x100);
		}

		/* Each individual bit needs to be acknowledged separately */
		for (i = 0; i < 32; i++) {
			if (status & BIT(i)) {
				bcm577x5_reg_writel(dev, BCM577x5_MS_INT_STATUS,
						    BIT(i));
			}
		}

		/* Signal the command or TPC is done */
		if (status & BCM577x5_MS_INT_RESPONSE) {
			complete(&dev->cmd_done);
		}

		/* Signal the FIFO is ready */
		if (status & BCM577x5_MS_INT_DATA_AVAIL) {
			complete(&dev->fifo_done);
		}

		if (status & BCM577x5_MS_INT_DATA_TIMEOUT) {
			dev_warn(&dev->pci_dev->dev, "Timeout!");
		}

		if (status &
		    (BCM577x5_MS_INT_DATA_END | BCM577x5_MS_INT_DMA_END)) {
			complete(&dev->dma_done);
		}

		if (status & (BCM577x5_MS_INT_CARD_INSERT |
			      BCM577x5_MS_INT_CARD_REMOVE)) {
			if (status & BCM577x5_MS_INT_CARD_INSERT)
				dev_info(&dev->pci_dev->dev, "Card inserted\n");
			else
				dev_info(&dev->pci_dev->dev, "Card removed\n");
			memstick_detect_change(dev->host);
		}

		ret = IRQ_HANDLED;
	}
	return ret;
}

static int bcm577x5_ms_mode(struct bcm577x5_device *dev, int mode)
{
	u8 host_control = bcm577x5_reg_readb(dev, BCM577x5_MS_HOST_CONTROL);
	switch (mode) {
	case MEMSTICK_SERIAL:
		dev_dbg(&dev->pci_dev->dev, "Switching to serial");
		bcm577x5_reg_writeb(dev, BCM577x5_MS_HOST_CONTROL,
				    host_control & ~BCM577x5_MS_CTRL_4BITBUS);
		return 0;
	case MEMSTICK_PAR4:
		dev_dbg(&dev->pci_dev->dev, "Switching to PAR4");
		bcm577x5_reg_writeb(dev, BCM577x5_MS_HOST_CONTROL,
				    host_control | BCM577x5_MS_CTRL_4BITBUS);
		return 0;
	default:
		return -EINVAL;
	}
}
/* External inteface: set settings */
static int bcm577x5_ms_set_param(struct memstick_host *host,
				 enum memstick_param param, int value)
{
	struct bcm577x5_device *dev = memstick_priv(host);

	switch (param) {
	case MEMSTICK_POWER:
		return bcm577x5_ms_power(dev, value);
	case MEMSTICK_INTERFACE:
		return bcm577x5_ms_mode(dev, value);
	default:
		return -EINVAL;
	}
}
static int bcm577x5_ms_prepare_dma(struct bcm577x5_device *dev)
{
	int len = dev->req->sg.length;
	void *dma_copy_buffer;
	dma_addr_t dma_address;

	if (!dev->enable_dma)
		return -ENOSYS;

	reinit_completion(&dev->dma_done);

	if (dev->req->data_dir == WRITE) {
		dma_copy_buffer = dev->dma_buffer_aligned + 4096 - len;
		sg_copy_to_buffer(&dev->req->sg,
				  sg_nents_for_len(&dev->req->sg, len),
				  dma_copy_buffer, len);
		dma_sync_single_for_device(&dev->pci_dev->dev, dev->dma_handle,
					   4096, DMA_TO_DEVICE);
	}
	/* The last byte should cause an overflow interrupt on the 4096 byte boundary */
	dma_address = dev->dma_handle + 4096 - len;

	bcm577x5_reg_writel(dev, BCM577x5_MS_DMA_ADDRESS, dma_address);
	return 0;
}
static int bcm577x5_ms_finish_dma(struct bcm577x5_device *dev)
{
	int len = dev->req->sg.length;
	void *dma_copy_buffer;

	if (!dev->enable_dma)
		return -EINVAL;

	if (!wait_for_completion_timeout(&dev->dma_done,
					 msecs_to_jiffies(1000)))
		return -ETIME;

	/*if(bcm577x5_reg_waitl(dev, BCM577x5_MS_DMA_ADDRESS,
	 0xFFFFFFFF,
	 dev->dma_handle_aligned + 4096, 1000))
	 {
		 return -ETIME;
	 }*/

	if (dev->req->data_dir == READ) {
		dma_copy_buffer = dev->dma_buffer_aligned + 4096 - len;
		dma_sync_single_for_cpu(&dev->pci_dev->dev, dev->dma_handle,
					4096, DMA_FROM_DEVICE);
		sg_copy_from_buffer(&dev->req->sg,
				    sg_nents_for_len(&dev->req->sg, len),
				    dma_copy_buffer, len);

		/*struct sg_mapping_iter miter;

			sg_miter_start(&miter, &dev->req->sg, 1,
				       SG_MITER_ATOMIC | SG_MITER_TO_SG);

			while (sg_miter_next(&miter)) {
				memcpy(miter.addr, dma_copy_buffer, miter.length);
				dma_copy_buffer += miter.length;
			}

			sg_miter_stop(&miter);*/
	}

	return 0;
}

static int bcm577x5_ms_prepare_fifo(struct bcm577x5_device *dev)
{
	struct sg_mapping_iter miter;
	int i;

	reinit_completion(&dev->fifo_done);

	if (dev->req->data_dir == WRITE) {
		sg_miter_start(&miter, &dev->req->sg, 1,
			       SG_MITER_ATOMIC | SG_MITER_FROM_SG);
		while (sg_miter_next(&miter)) {
			u32 *buf = miter.addr;
			for (i = 0; i < miter.length / 4; i++) {
				bcm577x5_reg_writel(dev, BCM577x5_MS_BUFFER,
						    buf[i]);
			}
		}
		sg_miter_stop(&miter);
	}

	return 0;
}
static int bcm577x5_ms_finish_fifo(struct bcm577x5_device *dev)
{
	struct sg_mapping_iter miter;
	int i;

	if (!wait_for_completion_timeout(&dev->fifo_done, msecs_to_jiffies(10)))
		return -ETIME;

	if (dev->req->data_dir == READ) {
		sg_miter_start(&miter, &dev->req->sg, 1,
			       SG_MITER_ATOMIC | SG_MITER_TO_SG);
		while (sg_miter_next(&miter)) {
			u32 *buf = miter.addr;
			for (i = 0; i < miter.length / 4; i++) {
				buf[i] = bcm577x5_reg_readl(dev,
							    BCM577x5_MS_BUFFER);
			}
		}
		sg_miter_stop(&miter);
	}

	return 0;
}
static int bcm577x5_ms_execute_cmd_short(struct bcm577x5_device *dev, u32 cmd)
{
	int len = dev->req->data_len;
	u32 *data = (u32 *)dev->req->data;
	u32 response;

	/* Write the data into the controller */
	if (dev->req->data_dir == WRITE) {
		bcm577x5_reg_writel(dev, BCM577x5_MS_ARGUMENT, data[0]);
		if (len > 4)
			bcm577x5_reg_writel(dev, BCM577x5_MS_ARGUMENT2,
					    data[1]);
	}

	reinit_completion(&dev->cmd_done);

	response = bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE);
	bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG, cmd);

	if (!wait_for_completion_timeout(&dev->cmd_done,
					 msecs_to_jiffies(100))) {
		dev_dbg(&dev->pci_dev->dev, "Present: %08X\n",
			bcm577x5_reg_readl(dev, BCM577x5_MS_PRESENT_STATE));
		dev_dbg(&dev->pci_dev->dev, "Interrupt: %08X\n",
			bcm577x5_reg_readl(dev, BCM577x5_MS_INT_STATUS));
		dev_dbg(&dev->pci_dev->dev, "Response: %08X\n",
			bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE));
		return -ETIME;
	}

	if (dev->req->tpc == MS_TPC_SET_CMD) {
		/*
		 * The controller will execute a GET_INT command, wait for it.
		 * Sometimes, the second interrupt with coalesce with the first,
		 * so detecting when it is completed is a bit tricky.
		 * 
		 * The GET_INT command changes the response register,
		 * so we can detect when it is completed by checking whether
		 * the response register has changed.
		 */
		bcm577x5_reg_waitl_different(dev, BCM577x5_MS_RESPONSE,
					     response, 10);

		/*dev_info(&dev->pci_dev->dev, "GET_INT response: %d %08X %08X %08X %08X\n",
			 len, data[0], response,
			 bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE), bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE2));*/

		/* The int reg is placed in the response register */
		dev->req->int_reg =
			bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE) & 0xFF;

		/*
		 * Reset is needed because the controller will expect certain commands
		 * to follow the SET_CMD command, which the block driver may not
		 * necessarily issue.
		 */
		if (bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA))
			return -ETIME;
	}

	/* Read the data back */
	if (dev->req->data_dir == READ) {
		data[0] = bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE);
		if (len > 4)
			data[1] =
				bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE2);
	}

	return 0;
}
static int bcm577x5_ms_execute_cmd_long(struct bcm577x5_device *dev, u32 cmd)
{
	int err;
	bool use_dma = dev->enable_dma;

	if (dev->req->tpc != MS_TPC_READ_LONG_DATA) {
		return 0;
	}

	err = bcm577x5_ms_prepare_dma(dev);
	if (err) {
		use_dma = false;
		err = bcm577x5_ms_prepare_fifo(dev);
	}

	if (use_dma)
		cmd |= BCM577x5_MS_TRNS_DMA;

	/* Execute the TPC */
	bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG, cmd);

	if (use_dma)
		err = bcm577x5_ms_finish_dma(dev);
	else
		err = bcm577x5_ms_finish_fifo(dev);

	/* A reset is needed or else the next command will not execute */
	if (bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA))
		return -ETIME;
	return err;
}

static int bcm577x5_ms_execute_cmd(struct bcm577x5_device *dev)
{
	int len =
		dev->req->long_data ? dev->req->sg.length : dev->req->data_len;
	int err;
	u32 cmd;

	/* Immediately return if there is no card inserted */
	if (!(bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS) &
	      BCM577x5_MS_STATUS_MS_DETECT))
		return -ENODEV;

	dev_dbg(&dev->pci_dev->dev,
		"Executing %s %x long: %d len: %d data: %08X %08X\n",
		memstick_debug_get_tpc_name(dev->req->tpc), dev->req->tpc,
		dev->req->long_data, len, *((int *)dev->req->data),
		*((int *)(&dev->req->data[4])));

	/* Write the length of the transfer */
	bcm577x5_reg_writel(dev, BCM577x5_MS_BLOCK_SIZE, len);

	/* Create the execution command */
	cmd = BCM577x5_MS_MAKE_TPC(dev->req->tpc);
	if (dev->req->data_dir == READ)
		cmd |= BCM577x5_MS_TRNS_READ;

	/* Two types of transfers have very different flow,
	 * so split them into functions
	 */
	if (dev->req->long_data)
		err = bcm577x5_ms_execute_cmd_long(dev, cmd);
	else
		err = bcm577x5_ms_execute_cmd_short(dev, cmd);

	return err;
}
static void bcm577x5_ms_req_process(struct work_struct *data)
{
	struct bcm577x5_device *dev =
		container_of(data, struct bcm577x5_device, req_work);
	int err, cmderr;

	/* Already processing requests */
	if (dev->req)
		return;

	mutex_lock(&dev->req_lock);
	do {
		err = memstick_next_req(dev->host, &dev->req);
		if (!err) {
			cmderr = bcm577x5_ms_execute_cmd(dev);
			dev->req->error = cmderr;
		}
	} while (!err && !cmderr);

	if (cmderr) {
		/* Controller will not respond to any more commands anyway */
		while (!err && dev->req) {
			dev->req->error = -ETIME;
			err = memstick_next_req(dev->host, &dev->req);
		}

		/* Reset the controller so it accepts new commands */
		bcm577x5_ms_init(dev);
	}
	mutex_unlock(&dev->req_lock);
}

static void bcm577x5_ms_submit_req(struct memstick_host *host)
{
	struct bcm577x5_device *dev = memstick_priv(host);

	schedule_work(&dev->req_work);
}
static int bcm577x5_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int error = -ENOMEM;
	struct memstick_host *host;
	struct bcm577x5_device *dev;
	u32 val;

	host = memstick_alloc_host(sizeof(struct bcm577x5_device), &pdev->dev);
	if (!host)
		goto error1;

	dev = memstick_priv(host);
	dev->host = host;
	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	error = pci_enable_device(pdev);
	if (error)
		goto error2;

	pci_set_master(pdev);

	error = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (error)
		goto error3;

	error = pci_request_regions(pdev, DRV_NAME);
	if (error)
		goto error3;

	dev->mmio = pci_ioremap_bar(pdev, 0);
	if (!dev->mmio)
		goto error4;

	INIT_WORK(&dev->req_work, bcm577x5_ms_req_process);

	host->caps = MEMSTICK_CAP_PAR4;
	host->request = bcm577x5_ms_submit_req;
	host->set_param = bcm577x5_ms_set_param;

	val = bcm577x5_reg_readl(dev, BCM577x5_MS_CAPABILITY_SLOT2);

	dev->base_clock = ((val & BCM577x5_MS_CAPABILITY_BASE_CLOCK_MASK) >>
			   BCM577x5_MS_CAPABILITY_BASE_CLOCK_SHIFT) *
			  1000000;

	if (enable_dma < 2) {
		dev_info(&pdev->dev, "DMA set by module parameter\n");
		dev->enable_dma = enable_dma;
	} else if (val & BCM577x5_MS_CAPABILITY_SDMA) {
		dev->enable_dma = 1;
		if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
			dev_warn(&pdev->dev, "No suitable DMA available\n");
			dev->enable_dma = 0;
		}
	} else {
		dev->enable_dma = 0;
	}

	if (dev->enable_dma) {
		dev->dma_buffer = kzalloc(8192, GFP_DMA | GFP_KERNEL);

		/* This needs to be 4096 aligned since the hardware does not seem to
		 * issue a dma complete interrupt, so we have to rely on
		 * address line overflows to detect completion.
		 */
		dev->dma_buffer_aligned = PTR_ALIGN(dev->dma_buffer, 4096);

		dev->dma_handle =
			dma_map_single(&pdev->dev, dev->dma_buffer_aligned,
				       4096, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(&pdev->dev, dev->dma_handle)) {
			dev_warn(
				&pdev->dev,
				"bcm577x5_ms: DMA Mapping Failed, disabling\n");
			dev->enable_dma = 0;
		}
	}

	/* Disable interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE, 0);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE, 0);

	pci_read_config_dword(pdev, 0x8, &val);
	dev->irq = pdev->irq;
	dev_info(&pdev->dev, "Rev %c%c dma=%d clock=%dHz irq=%d\n",
		 ((val & 0xFF) >> 3) + 'A', ((val & 0xFF) & 0b111) + '0',
		 dev->enable_dma, dev->base_clock, dev->irq);

	if (request_irq(dev->irq, &bcm577x5_irq, IRQF_SHARED, DRV_NAME, dev))
		goto error5;

	//bcm577x5_reg_writel(dev, 0x1E4, 0x9E9E9E9E);
	//bcm577x5_reg_writel(dev, 0x198, (bcm577x5_reg_readl(dev, 0x198) & ~0x00F000)| 0x3000);

	error = bcm577x5_ms_init(dev);
	if (error)
		goto error6;

	mutex_init(&dev->req_lock);
	init_completion(&dev->cmd_done);
	init_completion(&dev->fifo_done);
	init_completion(&dev->dma_done);

	if (memstick_add_host(host))
		goto error6;

	return 0;
error6:
	free_irq(dev->irq, dev);
error5:
	iounmap(dev->mmio);
error4:
	pci_release_regions(pdev);
error3:
	pci_disable_device(pdev);
error2:
	memstick_free_host(host);
error1:
	return error;
}

static void bcm577x5_remove(struct pci_dev *pdev)
{
	int error = 0;
	struct bcm577x5_device *dev = pci_get_drvdata(pdev);

	/* Cancel the work before interrupts are disabled */
	cancel_work_sync(&dev->req_work);

	/* Clear interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE, 0);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE, 0);

	/* Invalidate all outstanding requests */
	while (!error && dev->req) {
		dev->req->error = -ENODEV;
		error = memstick_next_req(dev->host, &dev->req);
	}

	memstick_remove_host(dev->host);

	if (dev->dma_handle) {
		dma_unmap_single(&dev->pci_dev->dev, dev->dma_handle, 8192,
				 DMA_BIDIRECTIONAL);
		kfree(dev->dma_buffer);
	}

	free_irq(dev->irq, dev);
	iounmap(dev->mmio);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	memstick_free_host(dev->host);
}

static int __maybe_unused bcm577x5_ms_suspend(struct device *pdev)
{
	struct bcm577x5_device *dev = dev_get_drvdata(pdev);

	memstick_suspend_host(dev->host);
	return 0;
}

static int __maybe_unused bcm577x5_ms_resume(struct device *pdev)
{
	struct bcm577x5_device *dev = dev_get_drvdata(pdev);

	memstick_resume_host(dev->host);
	return 0;
}
static SIMPLE_DEV_PM_OPS(bcm577x5_ms_pm_ops, bcm577x5_ms_suspend,
			 bcm577x5_ms_resume);

MODULE_DEVICE_TABLE(pci, bcm577x5_pci_id_tbl);

static struct pci_driver bcm577x5_pci_driver = {
	.name = DRV_NAME,
	.id_table = bcm577x5_pci_id_tbl,
	.probe = bcm577x5_probe,
	.remove = bcm577x5_remove,
	.driver.pm = &bcm577x5_ms_pm_ops,
};

module_pci_driver(bcm577x5_pci_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM577x5 MS/MSPro card reader driver");
