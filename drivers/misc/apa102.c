/*
 * Simple userspace interface for APA102 Ledring animations
 *
 * Copyright (C) 2017 Athom B.V.
 *	Jeroen Vollenbrock <jeroen@athom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/gpio/consumer.h>
#include <linux/poll.h>

#include "apa102.h"

#define N_APA102_MINORS			32	/* ... up to 256 */


#define FRAMETIME_STATIC 50
#define FRAMETIME_TRANSITION 16
#define MAX_ANIM_FRAMES 200
#define MAX_FPS 100

#define BRIGHTNESS 255

#define PCT_FCT 100000

#define APA102_MAX_CMD_SIZE 16384


#pragma pack(push, 1)
struct apa102_spi_color {
	u8	  brightness;
	struct apa102_color color;
};

struct apa102_spi_frame {
	struct apa102_spi_color start;
	struct apa102_spi_color leds[0];
	struct apa102_spi_color end;
};
#pragma pack(pop)


struct apa102_ledring_args {
	void *priv;

	u32 num_leds;
	wait_queue_head_t cmd_queue;
	struct apa102_command *next_cmd;
};

struct apa102_ledring {
	struct apa102_ledring_args *args;

	int next_transition_steps; //amount of steps after next animation or -1
	int next_frame_time; //duration (ms) of a frame after next animation or -1
	int next_rotation_per_ms;
	int next_rotation_count;
	struct apa102_frame *next_frame; //circular frame list

	int frametime; //milliseconds per frame.

	struct apa102_spi_frame *current_frame; //current frame data
	size_t spi_frame_size;

	int transition_progress; //progress of interpolation
	int transition_steps; //amount of steps needed for the interpolation between last and next

	struct apa102_frame *temp_frame; //used to store last frame of previous animation during transition period.

	int changed; //flag to see if we are in a transition between animations

	int rotation_per_ms; //rotation per second
	int rotation_count; //amount of rotations

	u8 led_brightness;

	ktime_t last_poll;
};

struct apa102_data {
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* for future use */
	wait_queue_head_t	poll_wait;

	struct apa102_ledring_args args;
	struct task_struct *animate_tsk;

	unsigned			   users;
	struct gpio_desc	 *enable_gpio;
};

static DECLARE_BITMAP(minors, N_APA102_MINORS);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*-------------------------------------------------------------------------*/

static ssize_t
apa102_sync(struct apa102_data *apa102dev, struct spi_message *message)
{
	int status;
	struct spi_device *spi;

	spin_lock_irq(&apa102dev->spi_lock);
	spi = apa102dev->spi;
	spin_unlock_irq(&apa102dev->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

static inline ssize_t
apa102_sync_write(void *buf, size_t len, struct apa102_ledring_args *args)
{
	struct apa102_data *apa102dev = (struct apa102_data *)args->priv;
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
			.rx_buf		  = NULL,
			.speed_hz	= 0,
			.bits_per_word = 8,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return apa102_sync(apa102dev, &m);
}


/*-------------------------------------------------------------------------*/

static inline int apa102_ledring_ceilpct(int in, int div) {
	if(in % div > 0) in += div;
	return in/div;
}

static inline int apa102_ledring_mod(int a, int b) {
	int res = a%b;
	if(res < 0) res += b;
	return res;
}

static inline void apa102_ledring_interpolate_led_bright(struct apa102_color *dest, struct apa102_color *src1, struct apa102_color *src2, int a_factor, u8 brightness) {
	int b_factor = PCT_FCT-a_factor;
	int bright_factor = brightness*PCT_FCT/BRIGHTNESS;

	int r = (src1->r*a_factor + src2->r*b_factor)/PCT_FCT;
	int g = (src1->g*a_factor + src2->g*b_factor)/PCT_FCT;
	int b = (src1->b*a_factor + src2->b*b_factor)/PCT_FCT;

	dest->r = apa102_ledring_ceilpct(r*bright_factor, PCT_FCT);
	dest->g = apa102_ledring_ceilpct(g*bright_factor, PCT_FCT);
	dest->b = apa102_ledring_ceilpct(b*bright_factor, PCT_FCT);
}

static inline void apa102_ledring_interpolate_led(struct apa102_color *dest, struct apa102_color *src1, struct apa102_color * src2, int a_factor) {
	apa102_ledring_interpolate_led_bright(dest, src1, src2, a_factor, BRIGHTNESS);
}

struct apa102_frame *apa102_ledring_frame_alloc(struct apa102_ledring *ledring, struct apa102_color *leds) {
	size_t size = sizeof(struct apa102_frame)+sizeof(struct apa102_color)*ledring->args->num_leds;
	struct apa102_frame *result;

	result = (struct apa102_frame*)kzalloc(size, GFP_KERNEL);
	if(!result) return result;

	INIT_LIST_HEAD(&result->frame_entry);
	if(leds) {
		memcpy(&result->leds, leds, size);
	}
	return result;
}

static void apa102_ledring_add_frame(struct apa102_ledring *ledring, struct apa102_color *leds) {
	struct apa102_frame *next;

	next = apa102_ledring_frame_alloc(ledring, leds);
	if(!next) return;

	if(ledring->next_frame)
		list_add_tail(&next->frame_entry, &ledring->next_frame->frame_entry);
	else
		ledring->next_frame = next;
}

static void apa102_ledring_clear_temp_frame(struct apa102_ledring *ledring) {

	if(ledring->temp_frame != NULL) {
		//this should only occur when a new animation was received when transitioning to new animation
		kfree(ledring->temp_frame);
		ledring->temp_frame = NULL;
	}
}

static void apa102_ledring_store_last_frame(struct apa102_ledring *ledring) {
	struct apa102_frame *a, *b;
	int i;

	a = ledring->temp_frame ? ledring->temp_frame :
		 list_entry(ledring->next_frame->frame_entry.prev, struct apa102_frame, frame_entry);
	b = ledring->next_frame;

	if(ledring->temp_frame == NULL)
		 ledring->temp_frame = apa102_ledring_frame_alloc(ledring, NULL);
    if(!ledring->temp_frame) return;

	for(i = 0; i < ledring->args->num_leds; i++) {
		apa102_ledring_interpolate_led(
			&ledring->temp_frame->leds[i],
			&a->leds[i],
			&b->leds[i],
			(PCT_FCT*ledring->transition_progress)/ledring->transition_steps
		);
	}
}

static void apa102_ledring_clear_queue(struct apa102_ledring *ledring) {
	struct apa102_frame *cursor;
	struct apa102_frame *next;


	if(!ledring || !(ledring->next_frame)) {
		return;
	}

	apa102_ledring_store_last_frame(ledring);

	list_for_each_entry_safe(cursor, next, &ledring->next_frame->frame_entry, frame_entry) {
		kfree(cursor);
	}

	kfree(ledring->next_frame);
	ledring->next_frame = NULL;
}

//pragma mark - Commands
static void apa102_ledring_animation(struct apa102_ledring *ledring, struct apa102_cmd_animation *animation) {
	int i;

	//clear queue
	apa102_ledring_clear_queue(ledring);

	//set step size
	if(animation->target_rate % animation->frame_rate != 0) {
		animation->target_rate += animation->frame_rate - (animation->target_rate % animation->frame_rate);
	}
	ledring->next_transition_steps = (animation->target_rate / animation->frame_rate);
	ledring->next_frame_time = 1000/animation->target_rate; //fps to miliseconds
	ledring->next_rotation_per_ms = -1*(((int)animation->rotation_per_minute)*((int)ledring->args->num_leds*PCT_FCT)/60000); //led shift per milisecond

	if(ledring->next_rotation_per_ms == 0) {
		ledring->next_rotation_count = 0;
	} else {
		ledring->next_rotation_count = ledring->rotation_count;
	}

	//add animation frames to queue
	for(i = 0; i < animation->frame_count; i++) {
		apa102_ledring_add_frame(ledring, &animation->frames[i*ledring->args->num_leds]);
	}
}

static void apa102_ledring_progress(struct apa102_ledring *ledring, struct apa102_cmd_progress *progress) {
	int i;
	struct apa102_color leds[ledring->args->num_leds];
	struct apa102_color c_black = {.r=0, .g=0, .b=0};
	struct apa102_color c_partial;
	struct apa102_cmd_animation animation = {
		.command = {
			.command = CMD_ANIMATION,
			.transition_time = progress->command.transition_time
		},
		.frame_rate = 1,
		.target_rate = 60,
		.rotation_per_minute = 30,
		.frame_count = 0
	};

	int leds_on = ((int)progress->progress*ledring->args->num_leds)/255;
	int remainder = ((int)progress->progress*ledring->args->num_leds)%255;
	apa102_ledring_interpolate_led_bright(&c_partial, &c_black, &progress->color, 0, remainder);

	for(i = 0; i < ledring->args->num_leds; i++) {
		if(i == ledring->args->num_leds - leds_on - 1) {
			leds[i] = progress->color;
		} else if(i < ledring->args->num_leds - leds_on) {
			leds[i] = c_black;
		} else {
			leds[i] = progress->color;
		}
	}

	apa102_ledring_animation(ledring, &animation);
	apa102_ledring_add_frame(ledring, leds);
}

static void apa102_ledring_brightness(struct apa102_ledring *ledring, struct apa102_cmd_brightness *command) {
	ledring->led_brightness = command->brightness;
}

static void apa102_ledring_solid(struct apa102_ledring *ledring, struct apa102_cmd_solid *command) {
	int i;
	struct apa102_color leds[ledring->args->num_leds];

	apa102_ledring_clear_queue(ledring);
	//generate frame
	for(i = 0; i < ledring->args->num_leds; i++) {
		leds[i] = command->solid;
	}
	//add frame to queue
	apa102_ledring_add_frame(ledring, leds);

	ledring->next_frame_time = FRAMETIME_STATIC;
	ledring->next_rotation_per_ms = 0;
	ledring->next_transition_steps = 1;
}

static void apa102_ledring_off(struct apa102_ledring *ledring) {
	struct apa102_cmd_solid animation = {
		.command = {
			.command = CMD_SOLID,
			.transition_time = 0
		},
		.solid = {
			.r = 0,
			.g = 0,
			.b = 0
		}
	};
	apa102_ledring_solid(ledring, &animation);
}

//pragma mark - Frame magic

static void apa102_ledring_rotate(struct apa102_ledring *ledring) {

	int bound = ledring->args->num_leds*PCT_FCT;

	if(ledring->rotation_per_ms == 0 && ledring->next_rotation_count == 0) return;

	ledring->rotation_count = apa102_ledring_mod(ledring->rotation_count + ledring->rotation_per_ms * ledring->frametime, bound);

	ledring->next_rotation_count = apa102_ledring_mod(ledring->next_rotation_count + ledring->next_rotation_per_ms * ledring->frametime, bound);

}

static void apa102_ledring_switch_next_frame(struct apa102_ledring *ledring) {

	ledring->rotation_count = ledring->next_rotation_count;
	ledring->rotation_per_ms = ledring->next_rotation_per_ms;
	ledring->transition_steps = ledring->next_transition_steps;
	ledring->frametime = ledring->next_frame_time;
	ledring->transition_progress = ledring->transition_steps;
	ledring->next_frame = list_entry(ledring->next_frame->frame_entry.next, struct apa102_frame, frame_entry);

	ledring->changed = 0;
	apa102_ledring_clear_temp_frame(ledring);
}

static void apa102_ledring_interpolate_frame(struct apa102_ledring *ledring) {
	struct apa102_frame *a, *b;
	int a_sign, b_sign;
	struct apa102_color led_a, led_b;
	int i,j,k;

	if(ledring->transition_steps <= 1 || ledring->transition_progress <= 1) {
		apa102_ledring_switch_next_frame(ledring);
	}

	apa102_ledring_rotate(ledring);
	a = ledring->temp_frame ? ledring->temp_frame :
				list_entry(ledring->next_frame->frame_entry.prev, struct apa102_frame, frame_entry);
	b = ledring->next_frame;

	a_sign = ledring->rotation_count >= 0 ? 1 : -1;
	b_sign = ledring->next_rotation_count >= 0 ? 1 : -1;

	for(i = 0; i < ledring->args->num_leds; i++) {
		j = apa102_ledring_mod(i + (ledring->rotation_count/PCT_FCT), ledring->args->num_leds);
		k = apa102_ledring_mod(i + (ledring->next_rotation_count/PCT_FCT), ledring->args->num_leds);

		apa102_ledring_interpolate_led(
			&led_a,
			&a->leds[apa102_ledring_mod(j+a_sign,ledring->args->num_leds)],
			&a->leds[j],
			apa102_ledring_mod(ledring->rotation_count, PCT_FCT)
		);
		apa102_ledring_interpolate_led(
			&led_b,
			&b->leds[apa102_ledring_mod(k+b_sign,ledring->args->num_leds)],
			&b->leds[k],
			apa102_ledring_mod(ledring->next_rotation_count, PCT_FCT)
		);
		apa102_ledring_interpolate_led_bright(
			&ledring->current_frame->leds[i].color,
			&led_a,
			&led_b,
			(PCT_FCT*ledring->transition_progress)/ledring->transition_steps,
			ledring->led_brightness
		);
		ledring->current_frame->leds[i].brightness = BRIGHTNESS;
	}
	ledring->transition_progress--;
	apa102_sync_write(ledring->current_frame, ledring->spi_frame_size, ledring->args);
}

//pragma mark - Communication
static void apa102_ledring_handle_command(struct apa102_ledring *ledring, struct apa102_command *command) {
	switch(command->command) {
		case CMD_ANIMATION:
			apa102_ledring_animation(ledring, (struct apa102_cmd_animation*)command);
			break;
		case CMD_BRIGHTNESS:
			apa102_ledring_brightness(ledring, (struct apa102_cmd_brightness*)command);
			break;
		case CMD_SOLID:
			apa102_ledring_solid(ledring, (struct apa102_cmd_solid*)command);
			break;
		case CMD_PROGRESS:
			apa102_ledring_progress(ledring, (struct apa102_cmd_progress*)command);
			break;
	}
	if(command->command != CMD_BRIGHTNESS) {
		ledring->frametime = FRAMETIME_TRANSITION;
		ledring->transition_progress = ledring->transition_steps = (command->transition_time / ledring->frametime);
	}
	ledring->changed = 1;
}

static void apa102_ledring_parse_command(struct apa102_ledring *ledring) {
	struct apa102_command *cmd;

	//ask for new commands with given timeout
	ledring->last_poll = ktime_add_ms(ledring->last_poll, ledring->frametime);
	if(ktime_before(ledring->last_poll, ktime_get())) {
		ledring->last_poll = ktime_add_ms(ktime_get(), ledring->frametime);
	}

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_hrtimeout_range(&ledring->last_poll, NSEC_PER_MSEC*ledring->frametime/10, HRTIMER_MODE_ABS);
	set_current_state(TASK_RUNNING);

	spin_lock(&ledring->args->cmd_queue.lock);
	cmd = ledring->args->next_cmd;
	ledring->args->next_cmd = NULL;
	wake_up_locked(&ledring->args->cmd_queue);
	spin_unlock(&ledring->args->cmd_queue.lock);

	if(cmd) {
		apa102_ledring_handle_command(ledring, cmd);
		kfree(cmd);
	}
}

static void apa102_init_spiframe(struct apa102_ledring *ledring) {
	ledring->spi_frame_size = sizeof(struct apa102_spi_frame)+sizeof(struct apa102_spi_color) * ledring->args->num_leds;
	ledring->current_frame = kzalloc(ledring->spi_frame_size, GFP_KERNEL);
	if(!ledring->current_frame) return;

	ledring->current_frame->start.brightness = 0;
	ledring->current_frame->start.color.r = 0;
	ledring->current_frame->start.color.g = 0;
	ledring->current_frame->start.color.b = 0;

	ledring->current_frame->leds[ledring->args->num_leds+1].brightness = 0;
	ledring->current_frame->leds[ledring->args->num_leds+1].color.r = 0;
	ledring->current_frame->leds[ledring->args->num_leds+1].color.g = 0;
	ledring->current_frame->leds[ledring->args->num_leds+1].color.b = 0;
}

static int apa102_ledring_run(void *data) {
	struct apa102_ledring ledring = {};
	struct apa102_ledring_args *args = (struct apa102_ledring_args *)data;
	ledring.args = args; //user data
	ledring.next_transition_steps = 1; //amount of steps after next animation
	ledring.next_frame_time = FRAMETIME_STATIC; //duration (ms) of a frame after next animation
	ledring.transition_steps = 1; //amount of steps for this animation
	ledring.frametime = FRAMETIME_STATIC; //miliseconds per frame.
	ledring.led_brightness = BRIGHTNESS;
	ledring.last_poll = ktime_get();

	apa102_init_spiframe(&ledring);
	if(!ledring.current_frame) return -ENOMEM;

	ledring.temp_frame = apa102_ledring_frame_alloc(&ledring, NULL);
	if(!ledring.temp_frame) return -ENOMEM;
	apa102_ledring_clear_queue(&ledring);

	while(!kthread_should_stop()) {
		apa102_ledring_parse_command(&ledring); //miliseconds
		if(ledring.next_frame == NULL || (list_empty(&ledring.next_frame->frame_entry) && !ledring.changed && ledring.rotation_per_ms == 0) ) {
			continue; //animation of 1 frame
		}

		BUG_ON(ledring.next_frame == NULL);

		apa102_ledring_interpolate_frame(&ledring);
	}
	apa102_ledring_off(&ledring);

	apa102_ledring_clear_queue(&ledring);
	apa102_ledring_clear_temp_frame(&ledring);

	kfree(ledring.current_frame);
	return 0;
}

static int apa102_validate_cmd(struct apa102_ledring_args *args, struct apa102_command *cmd, size_t size) {
	struct apa102_cmd_animation *animation;

	if(size < sizeof(struct apa102_command)) return -EINVAL;
	switch(cmd->command) {
		case CMD_ANIMATION:
			if(size < sizeof(struct apa102_cmd_animation)) return -EINVAL;
			animation = (struct apa102_cmd_animation *)cmd;
			if(animation->frame_count < 1 || animation->frame_count > MAX_ANIM_FRAMES) {
				return -EINVAL;
			}
			if(animation->target_rate < animation->frame_rate || animation->target_rate > MAX_FPS) {
				return -EINVAL;
			}
			if(animation->frame_rate < 1 || animation->frame_rate > MAX_FPS) {
				return -EINVAL;
			}
			if(size < sizeof(struct apa102_cmd_animation)+sizeof(struct apa102_color)*animation->frame_count*args->num_leds)
			return -EINVAL;
			break;
		case CMD_BRIGHTNESS:
			if(size < sizeof(struct apa102_cmd_brightness)) return -EINVAL;
			break;
		case CMD_SOLID:
			if(size < sizeof(struct apa102_cmd_solid)) return -EINVAL;
			break;
		case CMD_PROGRESS:
			if(size < sizeof(struct apa102_cmd_progress)) return -EINVAL;
			break;
		default:
			return -EINVAL;
	}

	return size;
}


/*-------------------------------------------------------------------------*/

/* Write-only message with current device setup */
static ssize_t
apa102_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct apa102_data	*apa102dev;
	struct apa102_command *command;
	ssize_t				status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > APA102_MAX_CMD_SIZE)
		return -EMSGSIZE;

	apa102dev = filp->private_data;

	command = (struct apa102_command *)kzalloc(count, GFP_KERNEL);
	if(!command) goto fail_alloc;

	if (copy_from_user(command, buf, count) != 0) {
		status = -EFAULT;
		goto fail_copy;
	}

	status = apa102_validate_cmd(&apa102dev->args, command, count);
	if(status < 0) goto fail_copy;

	spin_lock(&apa102dev->args.cmd_queue.lock);
	if(wait_event_interruptible_locked(apa102dev->args.cmd_queue, apa102dev->args.next_cmd == NULL) < 0) {
		status = -EINTR;
		goto fail_wait_event;
	}
	apa102dev->args.next_cmd = command;
	spin_unlock(&apa102dev->args.cmd_queue.lock);

	if(apa102dev->enable_gpio && command->command != CMD_BRIGHTNESS)
		gpiod_set_value_cansleep(apa102dev->enable_gpio, true);

	return status;
fail_wait_event:
	spin_unlock(&apa102dev->args.cmd_queue.lock);
fail_copy:
	kfree(command);
fail_alloc:

	printk(KERN_INFO "[%s] %s (%d) Failed: %d \n", __FILE__, __FUNCTION__, __LINE__, status);
	return status;
}

/* Read-only message with current device setup */
static ssize_t
apa102_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct apa102_data	*apa102dev;
	char				data[10];
	ssize_t				status = sizeof(data);

	apa102dev = filp->private_data;

	if(*f_pos > 0) return 0;

	status = snprintf(data, sizeof(data), "%d\n", apa102dev->args.num_leds);
	if(status > count) status = count;

	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, data, status);
		if (missing == status)
			status = -EFAULT;
		else {
			status = status - missing;
			*f_pos += status;
		}
	}

	return status;
}

static unsigned int
apa102_poll(struct file *file, poll_table *wait)
{
	struct apa102_data	*apa102dev;
	apa102dev = file->private_data;

	poll_wait(file, &apa102dev->poll_wait, wait);
	if (file->f_pos == 0)
		return POLLIN | POLLRDNORM;
	return 0;
}


static int apa102_open(struct inode *inode, struct file *filp)
{
	struct apa102_data	*apa102dev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(apa102dev, &device_list, device_entry) {
		if (apa102dev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("apa102dev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}


	if(!apa102dev->animate_tsk) {
		apa102dev->animate_tsk = kthread_run(apa102_ledring_run,
					&apa102dev->args, "apa102_animate/apa102-%d",
					iminor(inode));
	}


	apa102dev->users++;
	filp->private_data = apa102dev;
	nonseekable_open(inode, filp);

err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int apa102_release(struct inode *inode, struct file *filp)
{
	struct apa102_data	*apa102dev;

	mutex_lock(&device_list_lock);
	apa102dev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	apa102dev->users--;
	if (!apa102dev->users && apa102dev->animate_tsk) {
		kthread_stop(apa102dev->animate_tsk);
		apa102dev->animate_tsk = NULL;
		if(apa102dev->enable_gpio)
			gpiod_set_value_cansleep(apa102dev->enable_gpio, false);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations apa102_fops = {
	.owner =	THIS_MODULE,

	.poll =	 apa102_poll,

	.read =	 apa102_read,
	.write =	apa102_write,

	.open =		apa102_open,
	.release =	apa102_release,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/apa102-* character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *apa102_class;
static unsigned int apa102_major = 0;

#ifdef CONFIG_OF
static const struct of_device_id apa102_dt_ids[] = {
	{ .compatible = "apa,apa102" },
	{},
};
MODULE_DEVICE_TABLE(of, apa102_dt_ids);
#endif


/*-------------------------------------------------------------------------*/

static int apa102_probe(struct spi_device *spi)
{
	struct apa102_data	*apa102dev;
	struct device 		*dev;
	int					status;
	unsigned int		minor;

	/* Allocate driver data */
	apa102dev = devm_kzalloc(&spi->dev, sizeof(struct apa102_data), GFP_KERNEL);
	if (!apa102dev) {
		status = -ENOMEM;
		goto fail_init;
	}

	apa102dev->args.priv = apa102dev;
	init_waitqueue_head(&apa102dev->args.cmd_queue);

	apa102dev->enable_gpio = devm_gpiod_get(&spi->dev, "enable", GPIOD_OUT_LOW);
	if(IS_ERR(apa102dev->enable_gpio)) {
		apa102dev->enable_gpio = NULL;
		printk(KERN_ERR "[%s] %s: No enable GPIO specified.\n", __FILE__, __FUNCTION__);
	}

	status = of_property_read_u32(spi->dev.of_node, "num-leds", &apa102dev->args.num_leds);
	if(status != 0 || apa102dev->args.num_leds < 1) goto fail_init;

	printk(KERN_INFO "[%s] %s: Running with %u leds\n", __FILE__, __FUNCTION__, apa102dev->args.num_leds);

	/* Initialize the driver data */
	apa102dev->spi = spi;
	spin_lock_init(&apa102dev->spi_lock);

	init_waitqueue_head(&apa102dev->poll_wait);

	INIT_LIST_HEAD(&apa102dev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_APA102_MINORS);

	if(minor >= N_APA102_MINORS) {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
		goto fail_device_create;
	}

	apa102dev->devt = MKDEV(apa102_major, minor);
	dev = device_create(apa102_class, &spi->dev, apa102dev->devt,
				   apa102dev, "apa102-%u", minor);
	status = PTR_ERR_OR_ZERO(dev);

	if(status != 0)
		goto fail_device_create;

	set_bit(minor, minors);
	list_add(&apa102dev->device_entry, &device_list);

	mutex_unlock(&device_list_lock);

	spi_set_drvdata(spi, apa102dev);

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0 | SPI_NO_CS;
	status = spi_setup(spi);
	if(status != 0)
		goto fail_spi_setup;

	return 0;

fail_spi_setup:
	mutex_lock(&device_list_lock);
	list_del(&apa102dev->device_entry);
	device_destroy(apa102_class, apa102dev->devt);
	clear_bit(minor, minors);
fail_device_create:
	mutex_unlock(&device_list_lock);
fail_init:


	printk(KERN_ERR "[%s] %s: Probe failed (%d).\n", __FILE__, __FUNCTION__, status);

	return status;
}

static int apa102_remove(struct spi_device *spi)
{
	struct apa102_data	*apa102dev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&apa102dev->spi_lock);
	apa102dev->spi = NULL;
	spin_unlock_irq(&apa102dev->spi_lock);

	if(apa102dev->args.next_cmd) {
		kfree(apa102dev->args.next_cmd);
		apa102dev->args.next_cmd = NULL;
	}

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&apa102dev->device_entry);
	device_destroy(apa102_class, apa102dev->devt);
	clear_bit(MINOR(apa102dev->devt), minors);

	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver apa102_spi_driver = {
	.driver = {
		.name =		"apa102-ledring",
		.of_match_table = of_match_ptr(apa102_dt_ids),
	},
	.probe =	apa102_probe,
	.remove =	apa102_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.	The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init apa102_init(void)
{
	int status;

	printk(KERN_INFO "[%s] %s (%d) \n", __FILE__, __FUNCTION__, __LINE__);

	/* Request a major device number and claim the minor numbers.
	 * Then register a class that will key udev/mdev to add /dev nodes.
	 * Finally, register the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_APA102_MINORS > 256);
	apa102_major = register_chrdev(0, apa102_spi_driver.driver.name, &apa102_fops);
	if (apa102_major < 0)
		return apa102_major;

	apa102_class = class_create(THIS_MODULE, "apa102");
	if (IS_ERR(apa102_class)) {
		unregister_chrdev(apa102_major, apa102_spi_driver.driver.name);
		return PTR_ERR(apa102_class);
	}

	status = spi_register_driver(&apa102_spi_driver);
	if (status < 0) {
		class_destroy(apa102_class);
		unregister_chrdev(apa102_major, apa102_spi_driver.driver.name);
	}

	return status;
}
module_init(apa102_init);

static void __exit apa102_exit(void)
{

	spi_unregister_driver(&apa102_spi_driver);
	class_destroy(apa102_class);
	unregister_chrdev(apa102_major, apa102_spi_driver.driver.name);
}
module_exit(apa102_exit);

MODULE_AUTHOR("Jeroen Vollenbrock, <jeroen@athom.com>");
MODULE_DESCRIPTION("APA102 Ledring Interface");
MODULE_LICENSE("GPL");
