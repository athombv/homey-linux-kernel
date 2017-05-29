/* generic ZWave line discipline for Linux
 *
 * Written by Jeroen Vollenbrock jeroen@athom.com
 * for Athom B.V.
 *
 * This module implements the tty line discipline N_ZWAVE for use with
 * ZM Modules from sigma designs.
 *
 * All ZWave data is frame oriented which means:
 *
 * 1. tty write calls should represent one complete transmit frame of data
 *    The device driver should accept the complete frame or none of 
 *    the frame (busy) in the write method. Each write call should have
 *    a byte count in the range of 3-258 bytes (3 is min frame
 *    with 1 sof byte, 1 length byte, and crc).
 *
 *
 * 2. receive callbacks from the line discipline represent
 *    one completely received frame.
 *
 *    The ZWave line discipline queues the receive frames in separate
 *    buffers so complete receive frames will be returned by the
 *    tty read calls.
 *
 * 3. tty read calls returns an entire frame of data or nothing.
 *    
 * 4. all send and receive data is considered raw. No processing
 *    or translation is performed by the line discipline, regardless
 *    of the tty flags
 *
 * 5. When line discipline is queried for the amount of receive
 *    data available (FIOC), 0 is returned if no data available,
 *    otherwise the size of the next available frame is returned.
 *    (instead of the sum of all received frame counts).
 *
 * These conventions allow the standard tty programming interface
 * to be used for synchronous ZWave applications when used with
 * this line discipline.
 *
 * This implementation is very basic and does not maintain
 * any statistics. The main point is to enforce the raw data
 * and frame orientation of ZWave communications.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define ZWAVE_MAGIC 0x2b3e

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/jiffies.h>
#include <linux/wait.h>

#include <linux/poll.h>
#include <linux/in.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>	/* used in new tty drivers */
#include <linux/signal.h>	/* used in new tty drivers */
#include <linux/if.h>
#include <linux/bitops.h>

#include <asm/termios.h>
#include <asm/uaccess.h>

/*
 * Buffers for individual ZWAVE frames
 */
#define MIN_ZWAVE_FRAME_SIZE 3
#define MAX_ZWAVE_FRAME_SIZE 258  
#define DEFAULT_RX_BUF_COUNT 10
#define MAX_RX_BUF_COUNT 60
#define DEFAULT_TX_BUF_COUNT 3

/* 
 * ZWave Serial Protocol values
 */
#define ZWAVE_SOF 0x01
#define ZWAVE_ACK 0x06
#define ZWAVE_NAK 0x15
#define ZWAVE_CAN 0x18
#define ZWAVE_FIELD_TYPE 0x00
#define ZWAVE_FIELD_LENGTH 0x01

#define ZWAVE_FRAME_STATE_QUEUED  0x00
#define ZWAVE_FRAME_STATE_WAITING 0x01
#define ZWAVE_FRAME_STATE_GOT_ACK 0x02
#define ZWAVE_FRAME_STATE_GOT_NAK 0x03
#define ZWAVE_FRAME_STATE_GOT_CAN 0x04

/* Docs specify minimal amount of 1600, 
 * but the timeout is started when the cmd gets queued,
 * so we're going to be a bit less strict here
 */
#define ZWAVE_ACK_TIMEOUT_MILLISECS 3000 

static const __u8 ZWAVE_ACK_FRAME[] = { ZWAVE_ACK };
static const __u8 ZWAVE_NAK_FRAME[] = { ZWAVE_NAK };

struct n_zwave_buf {
	struct n_zwave_buf *link;
	int       state;
	int		  count;
	unsigned char buf[1];
};

#define	N_ZWAVE_BUF_SIZE	(sizeof(struct n_zwave_buf) + MAX_ZWAVE_FRAME_SIZE)

struct n_zwave_buf_list {
	struct n_zwave_buf *head;
	struct n_zwave_buf *tail;
	size_t		  count;
	spinlock_t	  spinlock;
};

/**
 * struct n_zwave - per device instance data structure
 * @magic - magic value for structure
 * @flags - miscellaneous control flags
 * @tty - ptr to TTY structure
 * @tbuf - currently transmitting tx buffer
 * @tx_buf_list - list of pending transmit frame buffers
 * @rx_buf_list - list of received frame buffers
 * @tx_free_buf_list - list unused transmit frame buffers
 * @rx_free_buf_list - list unused received frame buffers
 * @send_ack - indicates if an ACK frame should be sent ASAP
 * @send_nak - indicates if a NAK frame should be sent ASAP
 * @is_busy - indicates if a thread is writing data
 */
struct n_zwave {
	int			magic;
	__u32			flags;
	struct tty_struct	*tty;
	struct n_zwave_buf	*tbuf;
	struct n_zwave_buf_list	tx_buf_list;
	struct n_zwave_buf_list	rx_buf_list;
	struct n_zwave_buf_list	tx_free_buf_list;
	struct n_zwave_buf_list	rx_free_buf_list;
	int send_ack;
	int send_nak;
	int is_busy;
};

/*
 * ZWAVE buffer list manipulation functions
 */
static void n_zwave_buf_put(struct n_zwave_buf_list *list,
			   struct n_zwave_buf *buf);
static struct n_zwave_buf *n_zwave_buf_get(struct n_zwave_buf_list *list);

/* Local functions */

static struct n_zwave *n_zwave_alloc (void);

/* debug level can be set by insmod for debugging purposes */
#define DEBUG_LEVEL_INFO	1
static int debuglevel;


/* TTY callbacks */

static ssize_t n_zwave_tty_read(struct tty_struct *tty, struct file *file,
			   __u8 __user *buf, size_t nr);
static ssize_t n_zwave_tty_write(struct tty_struct *tty, struct file *file,
			    const unsigned char *buf, size_t nr);
static int n_zwave_tty_ioctl(struct tty_struct *tty, struct file *file,
			    unsigned int cmd, unsigned long arg);
static unsigned int n_zwave_tty_poll(struct tty_struct *tty, struct file *filp,
				    poll_table *wait);
static int n_zwave_tty_open(struct tty_struct *tty);
static void n_zwave_tty_close(struct tty_struct *tty);
static int n_zwave_tty_receive(struct tty_struct *tty, const __u8 *cp,
			       char *fp, int count);
static void n_zwave_tty_wakeup(struct tty_struct *tty);

#define bset(p,b)	((p)[(b) >> 5] |= (1 << ((b) & 0x1f)))

#define tty2n_zwave(tty)	((struct n_zwave *) ((tty)->disc_data))
#define n_zwave2tty(n_zwave)	((n_zwave)->tty)

static void flush_rx_queue(struct tty_struct *tty)
{
	struct n_zwave *n_zwave = tty2n_zwave(tty);
	struct n_zwave_buf *buf;

	while ((buf = n_zwave_buf_get(&n_zwave->rx_buf_list)))
		n_zwave_buf_put(&n_zwave->rx_free_buf_list, buf);
}

static void flush_tx_queue(struct tty_struct *tty)
{
	struct n_zwave *n_zwave = tty2n_zwave(tty);
	struct n_zwave_buf *buf;
	unsigned long flags;

	while ((buf = n_zwave_buf_get(&n_zwave->tx_buf_list)))
		n_zwave_buf_put(&n_zwave->tx_free_buf_list, buf);
 	spin_lock_irqsave(&n_zwave->tx_buf_list.spinlock, flags);
	if (n_zwave->tbuf) {
		n_zwave_buf_put(&n_zwave->tx_free_buf_list, n_zwave->tbuf);
		n_zwave->tbuf = NULL;
	}
	spin_unlock_irqrestore(&n_zwave->tx_buf_list.spinlock, flags);
}

static struct tty_ldisc_ops n_zwave_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "zwave",
	.open		= n_zwave_tty_open,
	.close		= n_zwave_tty_close,
	.read		= n_zwave_tty_read,
	.write		= n_zwave_tty_write,
	.ioctl		= n_zwave_tty_ioctl,
	.poll		= n_zwave_tty_poll,
	.receive_buf2	= n_zwave_tty_receive,
	.write_wakeup	= n_zwave_tty_wakeup,
	.flush_buffer   = flush_rx_queue,
};

/**
 * ZWAVE PROTOCOL FUNCTIONS
 */
static void n_zwave_send_ack(struct n_zwave *n_zwave)
{
    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_send_ack()\n",
                __FILE__,__LINE__ );
    if(n_zwave->tty->ops->write_room(n_zwave->tty) >= sizeof(ZWAVE_ACK_FRAME)) {
        n_zwave->tty->ops->write(n_zwave->tty, ZWAVE_ACK_FRAME, sizeof(ZWAVE_ACK_FRAME));
        n_zwave->send_ack = 0;
    } else {
        n_zwave->send_ack = 1;

        set_bit(TTY_DO_WRITE_WAKEUP, &n_zwave->tty->flags);
        if (debuglevel >= DEBUG_LEVEL_INFO)
            printk("%s(%d)n_zwave_send_ack() no room available, requesting wakeup\n",
                __FILE__,__LINE__ );
	}
}

static void n_zwave_send_nak(struct n_zwave *n_zwave)
{
    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_send_nak()\n",
                __FILE__,__LINE__ );
    if(n_zwave->tty->ops->write_room(n_zwave->tty) >= sizeof(ZWAVE_NAK_FRAME)) {
        n_zwave->tty->ops->write(n_zwave->tty, ZWAVE_NAK_FRAME, sizeof(ZWAVE_NAK_FRAME));
        n_zwave->send_nak = 0;
    } else {
        n_zwave->send_nak = 1;

        set_bit(TTY_DO_WRITE_WAKEUP, &n_zwave->tty->flags);
        if (debuglevel >= DEBUG_LEVEL_INFO)
            printk("%s(%d)n_zwave_send_nak() no room available, requesting wakeup\n",
                __FILE__,__LINE__ );
	}
}

static void n_zwave_got_ack(struct n_zwave *n_zwave)
{
    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_got_ack() for %p\n",
                __FILE__,__LINE__, n_zwave->tbuf );
    if(n_zwave->tbuf) n_zwave->tbuf->state = ZWAVE_FRAME_STATE_GOT_ACK;
    n_zwave->tbuf = NULL;

	/* wait up sleeping writers */
	wake_up_interruptible(&n_zwave->tty->write_wait);
}

static void n_zwave_got_nak(struct n_zwave *n_zwave)
{
    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_got_nak() for %p\n",
                __FILE__,__LINE__, n_zwave->tbuf);
    if(n_zwave->tbuf) n_zwave->tbuf->state = ZWAVE_FRAME_STATE_GOT_NAK;
    n_zwave->tbuf = NULL;
	/* wait up sleeping writers */
	wake_up_interruptible(&n_zwave->tty->write_wait);
}

static void n_zwave_got_can(struct n_zwave *n_zwave)
{
    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_got_can() for %p\n",
                __FILE__,__LINE__, n_zwave->tbuf);
    if(n_zwave->tbuf) n_zwave->tbuf->state = ZWAVE_FRAME_STATE_GOT_CAN;
    n_zwave->tbuf = NULL;
	/* wait up sleeping writers */
	wake_up_interruptible(&n_zwave->tty->write_wait);
}

static int n_zwave_is_complete_frame(const unsigned char *buf, size_t count)
{
    return (   count >= MIN_ZWAVE_FRAME_SIZE
            && count >= buf[ZWAVE_FIELD_LENGTH]+2
           );
}

static __u8 n_zwave_calc_checksum(const unsigned char *buf)
{
    __u8 result = 0xFF;
    size_t length = buf[ZWAVE_FIELD_LENGTH];
    int i = ZWAVE_FIELD_LENGTH;
    while(i <= length) {
        result ^= buf[i];
        i++;
    }
    return result;
}

static void n_zwave_print_frame(const unsigned char *buf, size_t count, const char *name)
{
    int i;
    if (debuglevel < DEBUG_LEVEL_INFO) return;
    printk("======== START %s FRAME =========", name);
    for(i = 0; i < count && buf; i++) {
        if(i % 16 == 0) printk("\n");
        printk("%2x ",buf[i]);
    }
    printk("\n");
    printk("======== END %s FRAME =========\n", name);
}

static __u8 n_zwave_get_checksum(const unsigned char *buf)
{
    return buf[buf[ZWAVE_FIELD_LENGTH]+1];
}

static int n_zwave_is_valid_frame(const unsigned char *buf, size_t count)
{
    int res = (   n_zwave_is_complete_frame(buf, count)
            && buf[ZWAVE_FIELD_TYPE] == ZWAVE_SOF
            && buf[ZWAVE_FIELD_LENGTH]+2 == count
            && n_zwave_calc_checksum(buf) == n_zwave_get_checksum(buf)
           );
    if(!res) {
        printk("n_zwave_tty_receive Invalid frame, %d+%d+%d+%d => %d (checksums expected=%u got=%u)\n",
                n_zwave_is_complete_frame(buf, count),
                buf[ZWAVE_FIELD_TYPE] == ZWAVE_SOF,
                buf[ZWAVE_FIELD_LENGTH]+2 == count,
                n_zwave_calc_checksum(buf) == n_zwave_get_checksum(buf),
                res, n_zwave_calc_checksum(buf), n_zwave_get_checksum(buf)
            );
    }
    return res;
}

/**
 * n_zwave_release - release an n_zwave per device line discipline info structure
 * @n_zwave - per device line discipline info structure
 */
static void n_zwave_release(struct n_zwave *n_zwave)
{
	struct tty_struct *tty = n_zwave2tty (n_zwave);
	struct n_zwave_buf *buf;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_release() called\n",__FILE__,__LINE__);

	/* Ensure that the n_zwaved process is not hanging on select()/poll() */
	wake_up_interruptible (&tty->read_wait);
	wake_up_interruptible (&tty->write_wait);

	if (tty->disc_data == n_zwave)
		tty->disc_data = NULL;	/* Break the tty->n_zwave link */

	/* Release transmit and receive buffers */
	for(;;) {
		buf = n_zwave_buf_get(&n_zwave->rx_free_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_zwave_buf_get(&n_zwave->tx_free_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_zwave_buf_get(&n_zwave->rx_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_zwave_buf_get(&n_zwave->tx_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	kfree(n_zwave->tbuf);
	kfree(n_zwave);

}	/* end of n_zwave_release() */

/**
 * n_zwave_tty_close - line discipline close
 * @tty - pointer to tty info structure
 *
 * Called when the line discipline is changed to something
 * else, the tty is closed, or the tty detects a hangup.
 */
static void n_zwave_tty_close(struct tty_struct *tty)
{
	struct n_zwave *n_zwave = tty2n_zwave (tty);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_close() called\n",__FILE__,__LINE__);

	if (n_zwave != NULL) {
		if (n_zwave->magic != ZWAVE_MAGIC) {
			printk (KERN_WARNING"n_zwave: trying to close unopened tty!\n");
			return;
		}

		tty->disc_data = NULL;
		if (tty != n_zwave->tty)
			return;
		n_zwave_release (n_zwave);
	}

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_close() success\n",__FILE__,__LINE__);

}	/* end of n_zwave_tty_close() */

/**
 * n_zwave_tty_open - called when line discipline changed to n_zwave
 * @tty - pointer to tty info structure
 *
 * Returns 0 if success, otherwise error code
 */
static int n_zwave_tty_open (struct tty_struct *tty)
{
	struct n_zwave *n_zwave = tty2n_zwave (tty);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_open() called (device=%s)\n",
		__FILE__,__LINE__,
		tty->name);

	/* There should not be an existing table for this slot. */
	if (n_zwave) {
		printk (KERN_ERR"n_zwave_tty_open:tty already associated!\n" );
		return -EEXIST;
	}

	n_zwave = n_zwave_alloc();
	if (!n_zwave) {
		printk (KERN_ERR "n_zwave_alloc failed\n");
		return -ENFILE;
	}

	tty->disc_data = n_zwave;
	n_zwave->tty    = tty;
	tty->receive_room = 65536;


	/* flush receive data from driver */
	tty_driver_flush_buffer(tty);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_open() success\n",__FILE__,__LINE__);

    n_zwave_send_nak(n_zwave);

	return 0;

}	/* end of n_tty_zwave_open() */

/**
 * n_zwave_send_frames - send frames on pending send buffer list
 * @n_zwave - pointer to ldisc instance data
 * @tty - pointer to tty instance data
 *
 * Send frames on pending send buffer list until the driver does not accept a
 * frame (busy) this function is called after adding a frame to the send buffer
 * list and by the tty wakeup callback.
 */
static void n_zwave_send_frames(struct n_zwave *n_zwave, struct tty_struct *tty)
{
	register int actual;
	struct n_zwave_buf *tbuf;
    unsigned long flags;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_send_frames() called\n",__FILE__,__LINE__);

	spin_lock_irqsave(&n_zwave->tx_buf_list.spinlock,flags);
	if(n_zwave->is_busy) {
    	spin_unlock_irqrestore(&n_zwave->tx_buf_list.spinlock,flags);
    	return;
	}
	n_zwave->is_busy = 1;
	spin_unlock_irqrestore(&n_zwave->tx_buf_list.spinlock,flags);

	/* get current transmit buffer or get new transmit */
	/* buffer from list of pending transmit buffers */
	tbuf = n_zwave->tbuf;
	if (!tbuf)
		tbuf = n_zwave_buf_get(&n_zwave->tx_buf_list);

	if (tbuf && tbuf->state == ZWAVE_FRAME_STATE_QUEUED) {
		if (debuglevel >= DEBUG_LEVEL_INFO)
			printk("%s(%d)sending frame %p, count=%d\n",
				__FILE__,__LINE__,tbuf,tbuf->count);

		/* Send the next block of data to device */
		actual = 0;
		if(tty->ops->write_room(n_zwave->tty) >= tbuf->count) {
		    actual = tty->ops->write(tty, tbuf->buf, tbuf->count);
        }


		if (actual == tbuf->count) {
			if (debuglevel >= DEBUG_LEVEL_INFO)
				printk("%s(%d)frame %p completed\n",
					__FILE__,__LINE__,tbuf);
		    tbuf->state = ZWAVE_FRAME_STATE_WAITING;
		} else {
			if (debuglevel >= DEBUG_LEVEL_INFO)
				printk("%s(%d)frame %p pending\n",
					__FILE__,__LINE__,tbuf);

			/* buffer not accepted by driver */
		}
        /* set this buffer as pending buffer */
        n_zwave->tbuf = tbuf;
	}
    if(!n_zwave->tbuf)
	    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

    spin_lock_irqsave(&n_zwave->tx_buf_list.spinlock,flags);
	n_zwave->is_busy = 0;
	spin_unlock_irqrestore(&n_zwave->tx_buf_list.spinlock,flags);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_send_frames() exit\n",__FILE__,__LINE__);

}	/* end of n_zwave_send_frames() */

/**
 * n_zwave_tty_wakeup - Callback for transmit wakeup
 * @tty	- pointer to associated tty instance data
 *
 * Called when low level device driver can accept more send data.
 */
static void n_zwave_tty_wakeup(struct tty_struct *tty)
{
	struct n_zwave *n_zwave = tty2n_zwave(tty);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_wakeup() called\n",__FILE__,__LINE__);

	if (!n_zwave)
		return;

	if (tty != n_zwave->tty) {
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		return;
	}
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
    if(n_zwave->send_ack) {
        n_zwave_send_ack(n_zwave);
    } else if(n_zwave->send_nak) {
        n_zwave_send_nak(n_zwave);
    } else {
	    n_zwave_send_frames(n_zwave, tty);
	}

}	/* end of n_zwave_tty_wakeup() */

/**
 * n_zwave_tty_receive - Called by tty driver when receive data is available
 * @tty	- pointer to tty instance data
 * @data - pointer to received data
 * @flags - pointer to flags for data
 * @count - count of received data in bytes
 *
 * Called by tty low level driver when receive data is available. Data is
 * interpreted as one ZWAVE frame.
 */
static int n_zwave_tty_receive(struct tty_struct *tty, const __u8 *data,
			       char *flags, int count)
{
	register struct n_zwave *n_zwave = tty2n_zwave (tty);
	register struct n_zwave_buf *buf;
    size_t i, j;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_receive() called count=%d\n",
			__FILE__,__LINE__, count);
	n_zwave_print_frame(data, count, "rx data");
	n_zwave_print_frame(flags, count, "rx flags");

	/* This can happen if stuff comes in on the backup tty */
	if (!n_zwave || tty != n_zwave->tty)
		return 0;

	/* verify line is using ZWAVE discipline */
	if (n_zwave->magic != ZWAVE_MAGIC) {
		printk("%s(%d) line not using ZWAVE discipline\n",
			__FILE__,__LINE__);
		return 0;
	}


    /* get current ZWAVE rx buffer */
    buf = NULL;
	for(i = 0, j = 0; i < count; i++) {
    	if(unlikely(flags && flags[i] != TTY_NORMAL)) continue;

    	if(!buf) {
        	switch(data[i]) {
            	case ZWAVE_SOF:
            		if (debuglevel >= DEBUG_LEVEL_INFO)
                        printk("%s(%d)n_zwave_tty_receive() got SOF\n",
                                __FILE__,__LINE__);
                    /* get a free ZWAVE buffer */
            	    buf = n_zwave_buf_get(&n_zwave->rx_free_buf_list);
                	if (!buf) {
                		/* no buffers in free list, attempt to allocate another rx buffer */
                		/* unless the maximum count has been reached */
                		if (n_zwave->rx_buf_list.count < MAX_RX_BUF_COUNT)
                			buf = kmalloc(N_ZWAVE_BUF_SIZE, GFP_ATOMIC);
                	}

                	if (!buf) {
                        printk(KERN_ERR"%s(%d)n_zwave_tty_receive() no more rx buffers, data discarded\n",
                			       __FILE__,__LINE__);
                		return j;
                	}
                	buf->count = 0; //reset count
                	buf->state = ZWAVE_FRAME_STATE_WAITING;
                break;
            	case ZWAVE_ACK:
            	    if (debuglevel >= DEBUG_LEVEL_INFO)
                        printk("%s(%d)n_zwave_tty_receive() got ACK\n",
                                __FILE__,__LINE__);
            	    n_zwave_got_ack(n_zwave);
            	    j++;
                break;
            	case ZWAVE_NAK:
            	    printk(KERN_ERR"%s(%d)n_zwave_tty_receive() got NAK\n",
                			       __FILE__,__LINE__);
            	    n_zwave_got_nak(n_zwave);
            	    j++;
                break;
            	case ZWAVE_CAN:
            	    printk(KERN_ERR"%s(%d)n_zwave_tty_receive() got CAN\n",
                			       __FILE__,__LINE__);
            	    n_zwave_got_can(n_zwave);
            	    j++;
                break;
            	default:
            	    printk(KERN_WARNING"%s(%d)n_zwave_tty_receive() unknown frame type from tty, data discarded\n",
                    			       __FILE__,__LINE__);
            	    j++;
        	}
    	}
        if(buf) { //We are receiving
            buf->buf[buf->count++] = data[i];
        	if(n_zwave_is_complete_frame(buf->buf, buf->count)) {

                if (debuglevel >= DEBUG_LEVEL_INFO)
                    printk("%s(%d)n_zwave_tty_receive() got complete frame, count=%d\n",
                        __FILE__,__LINE__, buf->count);

                n_zwave_print_frame(buf->buf, buf->count, "zwave");

            	if(!n_zwave_is_valid_frame(buf->buf, buf->count)) {
                    printk("%s(%d)n_zwave_tty_receive() got invalid frame, sending NAK\n",
                    			       __FILE__,__LINE__);
                	n_zwave_send_nak(n_zwave);
                	n_zwave_buf_put(&n_zwave->rx_free_buf_list, buf);
            	} else {
                	n_zwave_send_ack(n_zwave);

                	/* add ZWAVE buffer to list of received frames */
                	n_zwave_buf_put(&n_zwave->rx_buf_list, buf);

                	/* wake up any blocked reads and perform async signalling */
                	wake_up_interruptible (&tty->read_wait);
                	if (n_zwave->tty->fasync != NULL)
                		kill_fasync (&n_zwave->tty->fasync, SIGIO, POLL_IN);
            	}
            	j += buf->count;
                buf = NULL;
            }
        }
    }

    if(buf) {
        //TODO: Figure out why double parts of packets are received
        n_zwave_buf_put(&n_zwave->rx_free_buf_list, buf);
    }

    if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_receive() exit processed=%d\n",
			__FILE__,__LINE__, i);

    return i;
}	/* end of n_zwave_tty_receive() */

/**
 * n_zwave_tty_read - Called to retrieve one frame of data (if available)
 * @tty - pointer to tty instance data
 * @file - pointer to open file object
 * @buf - pointer to returned data buffer
 * @nr - size of returned data buffer
 * 	
 * Returns the number of bytes returned or error code.
 */
static ssize_t n_zwave_tty_read(struct tty_struct *tty, struct file *file,
			   __u8 __user *buf, size_t nr)
{
	struct n_zwave *n_zwave = tty2n_zwave(tty);
	int ret = 0;
	struct n_zwave_buf *rbuf;
	DECLARE_WAITQUEUE(wait, current);

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_read() called\n",__FILE__,__LINE__);

	/* Validate the pointers */
	if (!n_zwave)
		return -EIO;

	/* verify user access to buffer */
	if (!access_ok(VERIFY_WRITE, buf, nr)) {
		printk(KERN_WARNING "%s(%d) n_zwave_tty_read() can't verify user "
		"buffer\n", __FILE__, __LINE__);
		return -EFAULT;
	}

	add_wait_queue(&tty->read_wait, &wait);

	for (;;) {
		if (test_bit(TTY_OTHER_CLOSED, &tty->flags)) {
			ret = -EIO;
			break;
		}
		if (tty_hung_up_p(file))
			break;

		set_current_state(TASK_INTERRUPTIBLE);

		rbuf = n_zwave_buf_get(&n_zwave->rx_buf_list);
		if (rbuf) {
            if (debuglevel >= DEBUG_LEVEL_INFO)
                printk("%s(%d)n_zwave_tty_read() got buffer %p, count=%Zd\n",
                        __FILE__,__LINE__,rbuf, rbuf->count);
			if (rbuf->count > nr) {
				/* too large for caller's buffer */
				ret = -EOVERFLOW;
			} else {
				if (copy_to_user(buf, rbuf->buf, rbuf->count))
					ret = -EFAULT;
				else
					ret = rbuf->count;
			}

			if (n_zwave->rx_free_buf_list.count >
			    DEFAULT_RX_BUF_COUNT)
				kfree(rbuf);
			else
				n_zwave_buf_put(&n_zwave->rx_free_buf_list, rbuf);
			break;
		}

		/* no data */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		schedule();

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
	}

	remove_wait_queue(&tty->read_wait, &wait);
	__set_current_state(TASK_RUNNING);

	return ret;

}	/* end of n_zwave_tty_read() */

/**
 * n_zwave_tty_write - write a single frame of data to device
 * @tty	- pointer to associated tty device instance data
 * @file - pointer to file object data
 * @data - pointer to transmit data (one frame)
 * @count - size of transmit frame in bytes
 * 		
 * Returns the number of bytes written (or error code).
 */
static ssize_t n_zwave_tty_write(struct tty_struct *tty, struct file *file,
			    const unsigned char *data, size_t count)
{
	struct n_zwave *n_zwave = tty2n_zwave (tty);
	int error = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct n_zwave_buf *tbuf;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_write() called count=%Zd\n",
			__FILE__,__LINE__,count);

	/* Verify pointers */
	if (!n_zwave)
		return -EIO;

	if (n_zwave->magic != ZWAVE_MAGIC)
		return -EIO;

	/* verify frame size */
	if (count > MAX_ZWAVE_FRAME_SIZE ) {
        if (debuglevel >= DEBUG_LEVEL_INFO)
		    printk("%s(%d)n_zwave_tty_write() frame too large\n",
			        __FILE__,__LINE__);
        return -EINVAL;
	}
	if( !n_zwave_is_valid_frame(data, count)) {
        if (debuglevel >= DEBUG_LEVEL_INFO)
		    printk("%s(%d)n_zwave_tty_write() invalid packet\n",
			        __FILE__,__LINE__);
    	return -EINVAL;
	}

	add_wait_queue(&tty->write_wait, &wait);

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);

		tbuf = n_zwave_buf_get(&n_zwave->tx_free_buf_list);
		if (tbuf)
			break;

		if (file->f_flags & O_NONBLOCK) {
			error = -EAGAIN;
			break;
		}
		schedule();

		n_zwave = tty2n_zwave (tty);
		if (!n_zwave || n_zwave->magic != ZWAVE_MAGIC ||
		    tty != n_zwave->tty) {
			printk("n_zwave_tty_write: %p invalid after wait!\n", n_zwave);
			error = -EIO;
			break;
		}

		if (signal_pending(current)) {
			error = -EINTR;
			break;
		}
	}

	__set_current_state(TASK_RUNNING);

    if (debuglevel >= DEBUG_LEVEL_INFO)
        printk("%s(%d)n_zwave_tty_write() got buffer or error: %d\n",
			        __FILE__,__LINE__, error);

	if (!error) {
		/* Retrieve the user's buffer */
		memcpy(tbuf->buf, data, count);

		/* Send the data */
		tbuf->count = error = count;
		tbuf->state = ZWAVE_FRAME_STATE_QUEUED;
		n_zwave_buf_put(&n_zwave->tx_buf_list,tbuf);
		/* Tell device to request data */

        if (debuglevel >= DEBUG_LEVEL_INFO)
            printk("%s(%d)n_zwave_tty_write() requesting wakeup.\n",
			        __FILE__,__LINE__);

		n_zwave_tty_wakeup(tty);

    	if(!wait_event_interruptible_timeout(tty->write_wait,
    					     (tbuf->state != ZWAVE_FRAME_STATE_WAITING && tbuf->state != ZWAVE_FRAME_STATE_QUEUED),
    					     msecs_to_jiffies(ZWAVE_ACK_TIMEOUT_MILLISECS)))
            error = -ETIMEDOUT;
    	else if(tbuf->state == ZWAVE_FRAME_STATE_GOT_NAK) error = -EBADE;
    	else if(tbuf->state == ZWAVE_FRAME_STATE_GOT_CAN) error = -ECANCELED;

        if (debuglevel >= DEBUG_LEVEL_INFO)
            printk("%s(%d)n_zwave_tty_write() state changed to: %d, error:%d\n",
			        __FILE__,__LINE__, tbuf->state, error);

	}
	remove_wait_queue(&tty->write_wait, &wait);

    /* free current transmit buffer */
	n_zwave_buf_put(&n_zwave->tx_free_buf_list, tbuf);

	return error;

}	/* end of n_zwave_tty_write() */

/**
 * n_zwave_tty_ioctl - process IOCTL system call for the tty device.
 * @tty - pointer to tty instance data
 * @file - pointer to open file object for device
 * @cmd - IOCTL command code
 * @arg - argument for IOCTL call (cmd dependent)
 *
 * Returns command dependent result.
 */
static int n_zwave_tty_ioctl(struct tty_struct *tty, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct n_zwave *n_zwave = tty2n_zwave (tty);
	int error = 0;
	int count;
	unsigned long flags;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_ioctl() called %d\n",
			__FILE__,__LINE__,cmd);

	/* Verify the status of the device */
	if (!n_zwave || n_zwave->magic != ZWAVE_MAGIC)
		return -EBADF;

	switch (cmd) {
	case FIONREAD:
		/* report count of read data available */
		/* in next available frame (if any) */
		spin_lock_irqsave(&n_zwave->rx_buf_list.spinlock,flags);
		if (n_zwave->rx_buf_list.head)
			count = n_zwave->rx_buf_list.head->count;
		else
			count = 0;
		spin_unlock_irqrestore(&n_zwave->rx_buf_list.spinlock,flags);
		error = put_user(count, (int __user *)arg);
		break;

	case TIOCOUTQ:
		/* get the pending tx byte count in the driver */
		count = tty_chars_in_buffer(tty);
		/* add size of next output frame in queue */
		spin_lock_irqsave(&n_zwave->tx_buf_list.spinlock,flags);
		if (n_zwave->tx_buf_list.head)
			count += n_zwave->tx_buf_list.head->count;
		spin_unlock_irqrestore(&n_zwave->tx_buf_list.spinlock,flags);
		error = put_user(count, (int __user *)arg);
		break;

	case TCFLSH:
		switch (arg) {
		case TCIOFLUSH:
		case TCOFLUSH:
			flush_tx_queue(tty);
		}
		/* fall through to default */

	default:
		error = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;
	}
	return error;

}	/* end of n_zwave_tty_ioctl() */

/**
 * n_zwave_tty_poll - TTY callback for poll system call
 * @tty - pointer to tty instance data
 * @filp - pointer to open file object for device
 * @poll_table - wait queue for operations
 * 
 * Determine which operations (read/write) will not block and return info
 * to caller.
 * Returns a bit mask containing info on which ops will not block.
 */
static unsigned int n_zwave_tty_poll(struct tty_struct *tty, struct file *filp,
				    poll_table *wait)
{
	struct n_zwave *n_zwave = tty2n_zwave (tty);
	unsigned int mask = 0;

	if (debuglevel >= DEBUG_LEVEL_INFO)
		printk("%s(%d)n_zwave_tty_poll() called\n",__FILE__,__LINE__);

	if (n_zwave && n_zwave->magic == ZWAVE_MAGIC && tty == n_zwave->tty) {
		/* queue current process into any wait queue that */
		/* may awaken in the future (read and write) */

		poll_wait(filp, &tty->read_wait, wait);
		poll_wait(filp, &tty->write_wait, wait);

		/* set bits for operations that won't block */
		if (n_zwave->rx_buf_list.head)
			mask |= POLLIN | POLLRDNORM;	/* readable */
		if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
			mask |= POLLHUP;
		if (tty_hung_up_p(filp))
			mask |= POLLHUP;
		if (!tty_is_writelocked(tty) &&
				n_zwave->tx_free_buf_list.head)
			mask |= POLLOUT | POLLWRNORM;	/* writable */
	}
	return mask;
}	/* end of n_zwave_tty_poll() */

/**
 * n_zwave_alloc - allocate an n_zwave instance data structure
 *
 * Returns a pointer to newly created structure if success, otherwise %NULL
 */
static struct n_zwave *n_zwave_alloc(void)
{
	struct n_zwave_buf *buf;
	int i;
	struct n_zwave *n_zwave = kzalloc(sizeof(*n_zwave), GFP_KERNEL);

	if (!n_zwave)
		return NULL;

	spin_lock_init(&n_zwave->rx_free_buf_list.spinlock);
	spin_lock_init(&n_zwave->tx_free_buf_list.spinlock);
	spin_lock_init(&n_zwave->rx_buf_list.spinlock);
	spin_lock_init(&n_zwave->tx_buf_list.spinlock);

	/* allocate free rx buffer list */
	for(i=0;i<DEFAULT_RX_BUF_COUNT;i++) {
		buf = kmalloc(N_ZWAVE_BUF_SIZE, GFP_KERNEL);
		if (buf)
			n_zwave_buf_put(&n_zwave->rx_free_buf_list,buf);
		else if (debuglevel >= DEBUG_LEVEL_INFO)
			printk("%s(%d)n_zwave_alloc(), kalloc() failed for rx buffer %d\n",__FILE__,__LINE__, i);
	}

	/* allocate free tx buffer list */
	for(i=0;i<DEFAULT_TX_BUF_COUNT;i++) {
		buf = kmalloc(N_ZWAVE_BUF_SIZE, GFP_KERNEL);
		if (buf)
			n_zwave_buf_put(&n_zwave->tx_free_buf_list,buf);
		else if (debuglevel >= DEBUG_LEVEL_INFO)
			printk("%s(%d)n_zwave_alloc(), kalloc() failed for tx buffer %d\n",__FILE__,__LINE__, i);
	}

	/* Initialize the control block */
	n_zwave->magic  = ZWAVE_MAGIC;
	n_zwave->flags  = 0;

	return n_zwave;

}	/* end of n_zwave_alloc() */

/**
 * n_zwave_buf_put - add specified ZWAVE buffer to tail of specified list
 * @list - pointer to buffer list
 * @buf	- pointer to buffer
 */
static void n_zwave_buf_put(struct n_zwave_buf_list *list,
			   struct n_zwave_buf *buf)
{
	unsigned long flags;
	spin_lock_irqsave(&list->spinlock,flags);

	buf->link=NULL;
	if (list->tail)
		list->tail->link = buf;
	else
		list->head = buf;
	list->tail = buf;
	(list->count)++;

	spin_unlock_irqrestore(&list->spinlock,flags);

}	/* end of n_zwave_buf_put() */

/**
 * n_zwave_buf_get - remove and return an ZWAVE buffer from list
 * @list - pointer to ZWAVE buffer list
 * 
 * Remove and return an ZWAVE buffer from the head of the specified ZWAVE buffer
 * list.
 * Returns a pointer to ZWAVE buffer if available, otherwise %NULL.
 */
static struct n_zwave_buf* n_zwave_buf_get(struct n_zwave_buf_list *list)
{
	unsigned long flags;
	struct n_zwave_buf *buf;
	spin_lock_irqsave(&list->spinlock,flags);

	buf = list->head;
	if (buf) {
		list->head = buf->link;
		(list->count)--;
	}
	if (!list->head)
		list->tail = NULL;

	spin_unlock_irqrestore(&list->spinlock,flags);
	return buf;

}	/* end of n_zwave_buf_get() */

static char zwave_banner[] __initdata =
	KERN_INFO "N_ZWAVE line discipline.\n";
static char zwave_register_ok[] __initdata =
	KERN_INFO "N_ZWAVE line discipline registered.\n";
static char zwave_register_fail[] __initdata =
	KERN_ERR "error registering line discipline: %d\n";

static int __init n_zwave_init(void)
{
	int status;

	printk(zwave_banner);

	status = tty_register_ldisc(N_ZWAVE, &n_zwave_ldisc);
	if (!status)
		printk(zwave_register_ok);
	else
		printk(zwave_register_fail, status);

	return status;

}	/* end of init_module() */

static char zwave_unregister_ok[] __exitdata =
	KERN_INFO "N_ZWAVE: line discipline unregistered\n";
static char zwave_unregister_fail[] __exitdata =
	KERN_ERR "N_ZWAVE: can't unregister line discipline (err = %d)\n";

static void __exit n_zwave_exit(void)
{
	/* Release tty registration of line discipline */
	int status = tty_unregister_ldisc(N_ZWAVE);

	if (status)
		printk(zwave_unregister_fail, status);
	else
		printk(zwave_unregister_ok);
}

module_init(n_zwave_init);
module_exit(n_zwave_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeroen Vollenbrock jeroen@athom.com");
module_param(debuglevel, int, 0);
MODULE_ALIAS_LDISC(N_ZWAVE);
