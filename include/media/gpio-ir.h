/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GPIO_IR_H__
#define __GPIO_IR_H__

struct gpio_ir_platform_data {
	int		rx_gpio_nr;
	bool	rx_active_low;
	int		tx_gpio_nr;
	bool	tx_active_low;
	u64		allowed_protos;
	const char	*map_name;
};

#endif /* __GPIO_IR_H__ */

