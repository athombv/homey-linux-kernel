/*
 *	Sigma Designs ZM5304 Kernel Driver
 *
 *	This driver implements the lowest level protocol of the ZM5304
 *	Z-Wave stack. By implementing the protocol-level flow-control in
 *	Kernel-space, the dependencies between the TX and RX flow in userspace
 *	get reduced. This leads to a significant performance increase.
 *
 *	Copyright (C) 2018 Athom B.V.
 *
 *	Written by Jeroen Vollenbrock <jeroen@athom.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2
 *	as published by the Free Software Foundation
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/kfifo.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/of.h>
#include <linux/serdev.h>

#include <linux/gpio/consumer.h>
#include "zm5304_ioctls.h"


#define ZWAVE_SOF 0x01
#define ZWAVE_ACK 0x06
#define ZWAVE_NAK 0x15
#define ZWAVE_CAN 0x18

#define ZWAVE_FIELD_TYPE 0x00
#define ZWAVE_FIELD_LENGTH 0x01

#define MIN_ZWAVE_FRAME_SIZE 3
#define MAX_ZWAVE_FRAME_SIZE 258

#define WRITE_TIMEOUT_MS 1000
#define ACK_TIMEOUT_MS 1600

#define RESET_HOLD_MS 10
#define RESET_BACKOFF_MS 100

#define ZWAVE_PROG_TIMEOUT_MS (1600)
#define ZWAVE_PROG_CMD_MASK (0xFF000000)

#define ZM5304_MINORS 256

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, ZM5304_MINORS);
/* The main reason to have this class is to make mdev/udev create the
 * /dev/apa102-* character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *zm5304_class;
static unsigned int zm5304_major = 0;


enum zm5304_mode {
	MODE_CLOSED = 0,
	MODE_OPEN_REGULAR,
	MODE_OPEN_PROG,
};

/**
 * struct zm5304 - per device instance data structure
 * @serdev - serdev pointer
 * @rx_queue - received but undelivered messages
 * @rx_tx_status_queue - received but unhandled acknowledgements
 * @tx_got_ack_wq - waitqueue for acknowledgements
 * @enable_gpio - chip reset gpio
 * @mode - programming or regular mode
 */
struct zm5304_data {
	struct list_head		device_entry;
	struct serdev_device	*serdev;
	struct gpio_desc		*enable_gpio;
	dev_t					devt;
	unsigned int			users;
	enum zm5304_mode		mode;
	unsigned int			rx_invalid_crc_ctr;

	wait_queue_head_t		wq;
	struct mutex			write_lock;
	struct mutex			read_lock;

	DECLARE_KFIFO( rx_queue, u8, 2048 );
	DECLARE_KFIFO( rx_tx_status_queue, u8, 8 );

	u8                      partial_rx_pkt[MAX_ZWAVE_FRAME_SIZE];
	size_t                  partial_rx_pkt_len;
};

/*-------------------------------------------------------------------------*/

static inline size_t
zm5304_frame_size(const u8 *pckt)
{
	return pckt[ZWAVE_FIELD_LENGTH]+2;
}

static inline bool
zm5304_is_complete_frame(const unsigned char *buf, size_t count)
{
	return	(count >= MIN_ZWAVE_FRAME_SIZE
				&& count >= zm5304_frame_size(buf)
			);
}

static u8 zm5304_calc_checksum(const unsigned char *buf)
{
	u8 result = 0xFF;
	size_t length = zm5304_frame_size(buf);
	int i = ZWAVE_FIELD_LENGTH;
	while(i < length-1) {
		result ^= buf[i];
		i++;
	}
	return result;
}

static u8 zm5304_get_checksum(const unsigned char *buf)
{
	return buf[zm5304_frame_size(buf)-1];
}

static bool zm5304_is_valid_frame(const unsigned char *buf, size_t count)
{
	return (zm5304_is_complete_frame(buf, count)
			&& buf[0] == ZWAVE_SOF
			&& count == zm5304_frame_size(buf)
			&& zm5304_calc_checksum(buf) == zm5304_get_checksum(buf)
			);
}

static int
zm5304_send_data(struct zm5304_data *zm5304dev, const void *data, size_t len) {
    int res;
	dev_dbg(&zm5304dev->serdev->dev, "TX: prepare to send %zu bytes by serdev.\n", len);
	res = serdev_device_write(zm5304dev->serdev, (const u8*)data, len, msecs_to_jiffies(WRITE_TIMEOUT_MS));
	wake_up(&zm5304dev->wq);
	dev_dbg(&zm5304dev->serdev->dev, "TX: serdev write done.\n");
	return res;
}

static int zm5304_send_nak(struct zm5304_data *zm5304dev) {
	static const u8 ZWAVE_NAK_FRAME[] = { ZWAVE_NAK };
	return zm5304_send_data(zm5304dev, ZWAVE_NAK_FRAME, sizeof(ZWAVE_NAK_FRAME));
}

static int zm5304_send_ack(struct zm5304_data *zm5304dev) {
	static const u8 ZWAVE_ACK_FRAME[] = { ZWAVE_ACK };
	return zm5304_send_data(zm5304dev, ZWAVE_ACK_FRAME, sizeof(ZWAVE_ACK_FRAME));
}

static int zm5304_hard_reset(struct zm5304_data *zm5304dev) {
	if(zm5304dev->mode != MODE_OPEN_REGULAR) return -EINVAL;

	gpiod_set_value_cansleep(zm5304dev->enable_gpio, false);
	msleep(20);

	serdev_device_write_flush(zm5304dev->serdev);
    mutex_lock(&zm5304dev->write_lock);
	kfifo_reset_out(&zm5304dev->rx_tx_status_queue);
	mutex_unlock(&zm5304dev->write_lock);

	mutex_lock(&zm5304dev->read_lock);
	kfifo_reset_out(&zm5304dev->rx_queue);
	mutex_unlock(&zm5304dev->read_lock);

	gpiod_set_value_cansleep(zm5304dev->enable_gpio, true);
	msleep(500);

	return 0;
}


static int zm5304_exit_flash_mode(struct zm5304_data *zm5304dev) {
	u32 exit_flash_cmd = cpu_to_be32(0xFFFFFFFF);

	if(zm5304dev->mode != MODE_OPEN_REGULAR) return 0;
	if(zm5304dev->mode != MODE_OPEN_PROG) return -EINVAL;

	zm5304_send_data(zm5304dev, &exit_flash_cmd, sizeof(exit_flash_cmd));
	msleep(RESET_BACKOFF_MS);
	gpiod_set_value_cansleep(zm5304dev->enable_gpio, false);
	msleep(RESET_HOLD_MS);
	gpiod_set_value_cansleep(zm5304dev->enable_gpio, true);
	msleep(RESET_HOLD_MS);
	zm5304dev->mode = MODE_OPEN_REGULAR;
	return 0;
}

static int zm5304_enter_flash_mode(struct zm5304_data *zm5304dev) {
	u32 flash_cmd = cpu_to_be32(0xAC53AA55);
	u32 expected_result = cpu_to_be32(0xAA550000);
	u32 result = 0;

	if(zm5304dev->mode == MODE_OPEN_PROG) return 0;
	if(zm5304dev->mode != MODE_OPEN_REGULAR) return -EINVAL;

	zm5304dev->mode = MODE_OPEN_PROG;

	gpiod_set_value_cansleep(zm5304dev->enable_gpio, true);
	msleep(RESET_HOLD_MS);
	gpiod_set_value_cansleep(zm5304dev->enable_gpio, false);
	msleep(RESET_BACKOFF_MS);

	serdev_device_write_flush(zm5304dev->serdev);
	kfifo_reset_out(&zm5304dev->rx_tx_status_queue);
	kfifo_reset_out(&zm5304dev->rx_queue);


	zm5304_send_data(zm5304dev, &flash_cmd, sizeof(flash_cmd));
	if(wait_event_timeout(zm5304dev->wq,
		kfifo_peek_len(&zm5304dev->rx_tx_status_queue) >= 2,
		msecs_to_jiffies(ZWAVE_PROG_TIMEOUT_MS)))
	{
		int res = kfifo_out(&zm5304dev->rx_tx_status_queue,
				  (u8*)&result, sizeof(result));
		BUG_ON(res < 2);
		if(result == flash_cmd || result == expected_result)
			  return 0;
		return -EIO;
	}

	zm5304_exit_flash_mode(zm5304dev);

	return -ETIMEDOUT;
}


static long zm5304_send_flash_cmd(struct zm5304_data *zm5304dev, u32 cmd) {
	u32 data = 0;
	u32 dataBE = 0;
	u32 cmdBE = cpu_to_be32(cmd);

	if(zm5304dev->mode != MODE_OPEN_PROG) return -EINVAL;

	zm5304_send_data(zm5304dev, &cmdBE, sizeof(cmdBE));


	if(wait_event_timeout(zm5304dev->wq,
		kfifo_peek_len(&zm5304dev->rx_tx_status_queue) >= sizeof(dataBE),
		msecs_to_jiffies(ZWAVE_PROG_TIMEOUT_MS)))
	{
		int res = kfifo_out(&zm5304dev->rx_tx_status_queue,
						  (u8*)&dataBE, sizeof(dataBE));

		BUG_ON(res != sizeof(dataBE));

		data = be32_to_cpu(dataBE);

		if((data & ZWAVE_PROG_CMD_MASK) != (cmd & ZWAVE_PROG_CMD_MASK))
			return -EIO;
		return data & ~ZWAVE_PROG_CMD_MASK;
	}

	return -ETIMEDOUT;
}


/*-------------------------------------------------------------------------*/

static size_t zm5304_handle_rx_packet(struct zm5304_data *zm5304dev, const u8 *pkt, size_t len) {
    size_t pkt_size = zm5304_frame_size(pkt);

    BUG_ON(pkt_size > len);

    if(kfifo_avail(&zm5304dev->rx_queue) < pkt_size) {
        //drop packet, don't send a nak, just let the chip time out to create some breathing room
        dev_err(&zm5304dev->serdev->dev, "Not Enough buffer space, dropping packet!\n");
    } else if(zm5304_is_valid_frame(pkt, pkt_size)) {
        zm5304dev->rx_invalid_crc_ctr = 0;
        kfifo_in(&zm5304dev->rx_queue, pkt, pkt_size);
        zm5304_send_ack(zm5304dev);
    } else {
        dev_err(&zm5304dev->serdev->dev, "Got malformed packet, sending NAK!\n");
        zm5304_send_nak(zm5304dev);
        zm5304dev->rx_invalid_crc_ctr++;

        //INS12350-4C, p6.4.2
        if(zm5304dev->rx_invalid_crc_ctr > 3) {
            dev_err(&zm5304dev->serdev->dev, "Received 3 consecutive malformed packets, resetting chip.\n");
            zm5304_hard_reset(zm5304dev);
        }
    }
    return pkt_size;
}

static int zm5304_add_partial_rx(struct zm5304_data *zm5304dev, const u8*buf, size_t length) {
    size_t remaining = 2;

    if(zm5304dev->partial_rx_pkt_len >= MIN_ZWAVE_FRAME_SIZE) {
        remaining = zm5304_frame_size(zm5304dev->partial_rx_pkt) - zm5304dev->partial_rx_pkt_len;
    }

    if(remaining > length) remaining = length;

    BUG_ON(zm5304dev->partial_rx_pkt_len+remaining > sizeof(zm5304dev->partial_rx_pkt));

    memcpy(&zm5304dev->partial_rx_pkt[zm5304dev->partial_rx_pkt_len], buf, remaining);
    zm5304dev->partial_rx_pkt_len += remaining;

    if(zm5304_is_complete_frame(zm5304dev->partial_rx_pkt, zm5304dev->partial_rx_pkt_len)) {
        zm5304_handle_rx_packet(zm5304dev, zm5304dev->partial_rx_pkt, zm5304dev->partial_rx_pkt_len);
        zm5304dev->partial_rx_pkt_len = 0;
    }

    return remaining;
}

static int zm5304_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t length)
{
	size_t i = 0;
	struct zm5304_data *zm5304dev = serdev_device_get_drvdata(serdev);

	dev_dbg(&serdev->dev, "RX: Got %zu bytes\n", length);

	if(zm5304dev->mode == MODE_OPEN_PROG) {
	    dev_dbg(&serdev->dev, "RX: Sending %zu PROG bytes\n", length);
		length = kfifo_in(&zm5304dev->rx_tx_status_queue, buf, length);
		wake_up(&zm5304dev->wq); //Wake up writers
		return length;
	}

	/* get current ZWAVE rx buffer */
	while(i < length) {
		const u8 *pkt = &buf[i];
		size_t remaining = length - i;

		if(zm5304dev->partial_rx_pkt_len > 0) {
    		//slow flow, buffer partials
    		i += zm5304_add_partial_rx(zm5304dev, pkt, remaining);
    		continue;
		}

		switch(pkt[0]) {
    		dev_dbg(&serdev->dev, "Evaluating byte 0x%02X\n", pkt[0]);
			case ZWAVE_SOF:
                //INS12350-4C, (p6.2.1/6.4.1 is not implemented)
                if(!zm5304_is_complete_frame(pkt, remaining)) {
                    dev_dbg(&serdev->dev, "RX: Got partial packet %zu\n", remaining);
                    i += zm5304_add_partial_rx(zm5304dev, pkt, remaining);
                } else {
                    //fast flow, skip buffer
                    dev_dbg(&serdev->dev, "RX: Got full packet %zu\n", zm5304_frame_size(pkt));
                    i += zm5304_handle_rx_packet(zm5304dev, pkt, remaining);
                }
			break;
			case ZWAVE_ACK:
			case ZWAVE_NAK:
			case ZWAVE_CAN:
			    i++;
				kfifo_in(&zm5304dev->rx_tx_status_queue, pkt, 1);
				wake_up(&zm5304dev->wq); //Wake up writers
			break;
			default:
				dev_err(&serdev->dev, "Unknown frame type from tty, data discarded (%02X)\n", pkt[0]);
                i++;
		}
	}

	return length;
}

static struct serdev_device_ops zm5304_serdev_client_ops = {
	.receive_buf = zm5304_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};


/*-------------------------------------------------------------------------*/
static int zm5304_probe(struct serdev_device *serdev)
{
	int				 status;
	struct zm5304_data	 *zm5304dev;
	struct device 		*dev;
	unsigned int		minor;

	struct gpio_desc	 *enable_gpio;

	enable_gpio = devm_gpiod_get(&serdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(enable_gpio))
		return PTR_ERR_OR_ZERO(enable_gpio);


	zm5304dev = devm_kzalloc(&serdev->dev, sizeof(struct zm5304_data), GFP_KERNEL);
	if (!zm5304dev)
		return -ENOMEM;

	INIT_KFIFO(zm5304dev->rx_queue);
	INIT_KFIFO(zm5304dev->rx_tx_status_queue);
	init_waitqueue_head(&zm5304dev->wq);
	mutex_init(&zm5304dev->read_lock);
	mutex_init(&zm5304dev->write_lock);

	zm5304dev->serdev = serdev;
	zm5304dev->enable_gpio = enable_gpio;

	serdev_device_set_client_ops(serdev, &zm5304_serdev_client_ops);
	serdev_device_set_drvdata(serdev, zm5304dev);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
		minor = find_first_zero_bit(minors, ZM5304_MINORS);
		zm5304dev->devt = MKDEV(zm5304_major, minor);

		if(minor < ZM5304_MINORS) {
			set_bit(minor, minors);
			list_add(&zm5304dev->device_entry, &device_list);
		}
	mutex_unlock(&device_list_lock);

	if(minor >= ZM5304_MINORS) {
		dev_err(&serdev->dev, "no minor number available!\n");
		status = -ENODEV;
		goto fail_device_create;
	}

	dev = device_create(zm5304_class, &serdev->dev, zm5304dev->devt,
				   zm5304dev, "zwave%u", minor);
	status = PTR_ERR_OR_ZERO(dev);

fail_device_create:
	return status;
}

static void zm5304_remove(struct serdev_device *serdev)
{
	struct zm5304_data *zm5304dev = serdev_device_get_drvdata(serdev);
	mutex_destroy(&zm5304dev->read_lock);
	mutex_destroy(&zm5304dev->write_lock);
}

/*-------------------------------------------------------------------------*/

static int zm5304dev_devt_get(dev_t devt, struct zm5304_data **result) {
	struct zm5304_data	*zm5304dev;
	int status = -ENODEV;

	*result = NULL;

	mutex_lock(&device_list_lock);

	list_for_each_entry(zm5304dev, &device_list, device_entry) {
		if (zm5304dev->devt == devt) {
			*result = zm5304dev;
			break;
		}
	}

	if(*result) {
		(*result)->users++;
		status = (*result)->users;
	}

	mutex_unlock(&device_list_lock);

	return status;
}

static int zm5304dev_devt_put(dev_t devt) {
	struct zm5304_data	*zm5304dev;
	struct zm5304_data	*result = NULL;
	int status = -ENODEV;

	mutex_lock(&device_list_lock);

	list_for_each_entry(zm5304dev, &device_list, device_entry) {
		if (zm5304dev->devt == devt) {
			result = zm5304dev;
			break;
		}
	}

	if(result) {
		result->users--;
		status = result->users;
	}

	mutex_unlock(&device_list_lock);

	return status;
}


/* Write-only message with current device setup */
static ssize_t
zm5304_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct zm5304_data	*zm5304dev;
	unsigned char		packet[MAX_ZWAVE_FRAME_SIZE];
	u8					 ack = 0;
	u8					 attempt = 0;
	ssize_t				status = 0;

	if (count > MAX_ZWAVE_FRAME_SIZE || count < 3)
		return -EMSGSIZE;

	if (copy_from_user(packet, buf, count) != 0) {
		return -EFAULT;
	}

	if(!zm5304_is_valid_frame(packet, count)) {
		return -EINVAL;
	}

	zm5304dev = filp->private_data;
	mutex_lock(&zm5304dev->write_lock);

	if(zm5304dev->mode != MODE_OPEN_REGULAR) {
		status = -EBADFD;
		goto end;
	}

	for(attempt = 0, ack = 0; attempt < 3 && ack != ZWAVE_ACK; attempt++) {
		status = zm5304_send_data(zm5304dev, packet, count);

		//p6.2.2, INS12350-4C
		wait_event_timeout(zm5304dev->wq, !kfifo_is_empty(&zm5304dev->rx_tx_status_queue), msecs_to_jiffies(ACK_TIMEOUT_MS));

		if(kfifo_out(&zm5304dev->rx_tx_status_queue, &ack, 1) == 1)
			switch(ack) {
				case ZWAVE_ACK:
					break;
				case ZWAVE_NAK:
				case ZWAVE_CAN:
				    dev_err(&zm5304dev->serdev->dev, "Transmission failed, retrying %u/3 (0x%02X)\n", attempt, ack);
					msleep(100+attempt*1000); //p6.3, INS12350-4C
				default:
					status = -EIO;
			}
		else
			status = -ETIMEDOUT;

		// JV: If a timeout actually did cause an ACK to be transmitted,
		// The status queue gets out of sync. After each transmission is handled,
		// it should be empty. Let's clear it here to be sure.
		kfifo_reset_out(&zm5304dev->rx_tx_status_queue);
	}

end:

	mutex_unlock(&zm5304dev->write_lock);
	if(attempt >= 3 && status) zm5304_hard_reset(zm5304dev);
	return status;
}



/* Read-only message with current device setup */
static ssize_t
zm5304_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct zm5304_data	*zm5304dev;
	u8					 hdr[2];
	ssize_t				status = 0;

	if(count < MIN_ZWAVE_FRAME_SIZE) return -EMSGSIZE;

	zm5304dev = filp->private_data;
	mutex_lock(&zm5304dev->read_lock);

	if(zm5304dev->mode != MODE_OPEN_REGULAR) {
		status = -EBADFD;
		goto end;
	}

	//read header
	status = wait_event_interruptible(zm5304dev->wq, (kfifo_peek_len(&zm5304dev->rx_queue) >= sizeof(hdr)));
	if(status != 0) goto end;

	status = kfifo_out_peek(&zm5304dev->rx_queue, hdr, sizeof(hdr));
	BUG_ON(status != 2);

	status = wait_event_interruptible(zm5304dev->wq, (kfifo_peek_len(&zm5304dev->rx_queue) >= zm5304_frame_size(hdr)));
	if(status != 0) goto end;

	status = -EFAULT;
	BUG_ON(kfifo_to_user(&zm5304dev->rx_queue, buf, zm5304_frame_size(hdr), &status) != 0);

end:
	mutex_unlock(&zm5304dev->read_lock);

	return status;
}

static unsigned int
zm5304_poll(struct file *file, poll_table *wait)
{
	unsigned int res = 0;
	struct zm5304_data	*zm5304dev = file->private_data;

	poll_wait(file, &zm5304dev->wq, wait);

	if (!kfifo_is_empty(&zm5304dev->rx_queue))
		res |= POLLIN;

	if (!mutex_is_locked(&zm5304dev->write_lock))
		res |= POLLOUT;

	return res;
}

static long
zm5304_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long status = -EINVAL;
	struct zm5304_data	*zm5304dev = file->private_data;
	if(cmd == ZM5304_RESET) return zm5304_hard_reset(zm5304dev);


	mutex_lock(&zm5304dev->write_lock);
	switch(cmd) {
		case ZM5304_MODE_FLASH:
			status = zm5304_enter_flash_mode(zm5304dev);
			break;
		case ZM5304_MODE_REGULAR:
			status = zm5304_exit_flash_mode(zm5304dev);
			break;
		case ZM5304_FLASH32:
			status = zm5304_send_flash_cmd(zm5304dev, arg);
			break;
		default:
			break;
	}

	mutex_unlock(&zm5304dev->write_lock);

	return status;
}


static int zm5304_open(struct inode *inode, struct file *filp)
{
	struct zm5304_data	*zm5304dev;
	int	status = zm5304dev_devt_get(inode->i_rdev, &zm5304dev);

	if (status != 1) {
		BUG_ON(status == 0);
		zm5304dev_devt_put(inode->i_rdev);
		return -EBUSY;
	}

	filp->private_data = zm5304dev;
	status = nonseekable_open(inode, filp);

	serdev_device_open(zm5304dev->serdev);
	serdev_device_set_baudrate(zm5304dev->serdev, 115200);

	zm5304dev->mode = MODE_OPEN_REGULAR;
	gpiod_set_value_cansleep(zm5304dev->enable_gpio, true);
	msleep(RESET_BACKOFF_MS);

	return status;
}

static int zm5304_release(struct inode *inode, struct file *filp)
{
	struct zm5304_data	*zm5304dev = filp->private_data;
	int status = zm5304dev_devt_put(inode->i_rdev);

	if(!status) {
		mutex_lock(&zm5304dev->write_lock);
		if(zm5304dev->mode == MODE_OPEN_PROG) {
			zm5304_exit_flash_mode(zm5304dev);
		}
		zm5304dev->mode = MODE_CLOSED;
		gpiod_set_value_cansleep(zm5304dev->enable_gpio, false);
		mutex_unlock(&zm5304dev->write_lock);
		serdev_device_close(zm5304dev->serdev);
	}

	filp->private_data = NULL;

	return 0;
}

static const struct file_operations zm5304_fops = {
	.owner =			  THIS_MODULE,

	.read =				 zm5304_read,
	.write =			  zm5304_write,

	.poll =				 zm5304_poll,
	.unlocked_ioctl =	 zm5304_ioctl,

	.open =				  zm5304_open,
	.release =			zm5304_release,
};



/*-------------------------------------------------------------------------*/

static const struct of_device_id zm5304_of_match[] = {
	{ .compatible = "sigma,zm5304" },
	{},
};
MODULE_DEVICE_TABLE(of, zm5304_of_match);

static struct serdev_device_driver zm5304_serdev_driver = {
	.driver		= {
		.name	= "sigma-zm5304",
		.of_match_table = of_match_ptr(zm5304_of_match),
	},
	.probe	= zm5304_probe,
	.remove	= zm5304_remove,
};


static int __init zm5304_init(void)
{
	int status;

	printk(KERN_INFO "[%s] %s (%d) \n", __FILE__, __FUNCTION__, __LINE__);

	/* Request a major device number and claim the minor numbers.
	 * Then register a class that will key udev/mdev to add /dev nodes.
	 * Finally, register the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(ZM5304_MINORS > 256);
	zm5304_major = register_chrdev(0, zm5304_serdev_driver.driver.name, &zm5304_fops);
	if (zm5304_major < 0)
		return zm5304_major;

	zm5304_class = class_create(THIS_MODULE, zm5304_serdev_driver.driver.name);
	if (IS_ERR(zm5304_class)) {
		unregister_chrdev(zm5304_major, zm5304_serdev_driver.driver.name);
		return PTR_ERR(zm5304_class);
	}

	status = serdev_device_driver_register(&zm5304_serdev_driver);
	if (status < 0) {
		class_destroy(zm5304_class);
		unregister_chrdev(zm5304_major, zm5304_serdev_driver.driver.name);
	}

	return status;
}
module_init(zm5304_init);

static void __exit zm5304_exit(void)
{
	serdev_device_driver_unregister(&zm5304_serdev_driver);
	class_destroy(zm5304_class);
	unregister_chrdev(zm5304_major, zm5304_serdev_driver.driver.name);
}
module_exit(zm5304_exit);

MODULE_AUTHOR("Jeroen Vollenbrock, <jeroen@athom.com>");
MODULE_DESCRIPTION("ZM5304 Z-Wave Interface");
MODULE_LICENSE("GPL");
