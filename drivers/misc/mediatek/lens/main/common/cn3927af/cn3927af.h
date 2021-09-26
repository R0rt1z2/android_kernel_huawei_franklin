/*
 * cn3927af.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * cn3927af image sensor config define
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef CN3927AF_H
#define CN3927AF_H

struct af_i2c_reg {
	unsigned short addr;
	unsigned short data;
	unsigned short delay;
};

#endif