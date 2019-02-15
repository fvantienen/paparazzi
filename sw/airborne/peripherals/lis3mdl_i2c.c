/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/lis3mdl_i2c.c
 *
 * Driver for ST LIS3MDL 3D magnetometer.
 */

#include "peripherals/lis3mdl_i2c.h"
#include "std.h"

/**
 * Initialize Lis3mdl struct and set default config options.
 * @param lis   Lis3mdl struct
 * @param i2c_p I2C peripheral to use
 * @param addr  I2C address of Lis3mdl
 */
void lis3mdl_i2c_init(struct lis3mdl_t *lis, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  lis->i2c_p = i2c_p;
  /* set i2c address */
  lis->i2c_trans.slave_addr = addr;
  lis->i2c_trans.status = I2CTransDone;
  /* set default config options */
  lis3mdl_mag_set_default_config(&(lis->config));
  
  lis->status = LIS_STATE_UNINIT;
  lis->initialized = false;
}

static void lis3mdl_i2c_tx_reg(struct lis3mdl_t *lis, uint8_t reg, uint8_t val)
{
  lis->i2c_trans.type = I2CTransTx;
  lis->i2c_trans.buf[0] = reg;
  lis->i2c_trans.buf[1] = val;
  lis->i2c_trans.len_r = 0;
  lis->i2c_trans.len_w = 2;
  i2c_submit(lis->i2c_p, &(lis->i2c_trans));
}

/// Configuration function called once before normal use
static void lis3mdl_i2c_send_config(struct lis3mdl_t *lis)
{
  switch (lis->status) {
    case LIS_STATE_RESET:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_AD_CTRL_REG2, LIS3MDL_CTRL_REG2_SOFT_RST);
      lis->status++;
      break;
    case LIS_STATE_CTRL_REG1:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_AD_CTRL_REG1, lis->config.rate);
      lis->status++;
      break;
    case LIS_STATE_CTRL_REG2:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_AD_CTRL_REG2, lis->config.scale);
      lis->status++;
      break;
    case LIS_STATE_CTRL_REG3:
      lis3mdl_i2c_tx_reg(lis, LIS3MDL_AD_CTRL_REG3, lis->config.mode);
      lis->status++;
      break;
    case LIS_STATE_IDLE:
      lis->initialized = true;
      lis->i2c_trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

// Configure
void lis3mdl_i2c_start_configure(struct lis3mdl_t *lis)
{
  if (lis->status == LIS_STATE_UNINIT) {
    lis->status++;
    if (lis->i2c_trans.status == I2CTransSuccess || lis->i2c_trans.status == I2CTransDone) {
      lis3mdl_i2c_send_config(lis);
    }
  }
}

// Normal reading
void lis3mdl_i2c_read(struct lis3mdl_t *lis)
{
  if (lis->initialized && lis->i2c_trans.status == I2CTransDone) {
    if(lis->status == LIS_STATE_IDLE) {
      lis->i2c_trans.buf[0] = LIS3MDL_AD_STATUS_REG;
      lis->i2c_trans.type = I2CTransTxRx;
      lis->i2c_trans.len_r = 1;
      lis->i2c_trans.len_w = 1;
      i2c_submit(lis->i2c_p, &(lis->i2c_trans));
    } else {
      lis->i2c_trans.buf[0] = LIS3MDL_AD_OUT_X_L;
      lis->i2c_trans.type = I2CTransTxRx;
      lis->i2c_trans.len_r = 6;
      lis->i2c_trans.len_w = 1;
      i2c_submit(lis->i2c_p, &(lis->i2c_trans));
    }
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void lis3mdl_i2c_event(struct lis3mdl_t *lis)
{
  if (lis->initialized) {

    // Is initialized so read values
    if (lis->i2c_trans.status == I2CTransFailed) {
      lis->status = LIS_STATE_IDLE;
      lis->i2c_trans.status = I2CTransDone;
    } else if (lis->i2c_trans.status == I2CTransSuccess) {
      // Check for new values
      if(lis->status == LIS_STATE_IDLE) {
        if(lis->i2c_trans.buf[0] & LIS3MDL_STATUS_REG_XYZDA)
          lis->status = LIS_STATE_READ;
        
        lis->i2c_trans.status = I2CTransDone;
      }
      else {
        // Read new values
        lis->data[0] = Int16FromBuf(lis->i2c_trans.buf, 0);
        lis->data[1] = Int16FromBuf(lis->i2c_trans.buf, 2);
        lis->data[2] = Int16FromBuf(lis->i2c_trans.buf, 4);
        lis->data_available = true;
        lis->status = LIS_STATE_IDLE;
        lis->i2c_trans.status = I2CTransDone;
      }
    }

  } else {

    // Configuring but not yet initialized
    if (lis->status != LIS_STATE_UNINIT) { 
      if (lis->i2c_trans.status == I2CTransSuccess || lis->i2c_trans.status == I2CTransDone) {
        lis->i2c_trans.status = I2CTransDone;
        lis3mdl_i2c_send_config(lis);
      }
      if (lis->i2c_trans.status == I2CTransFailed) {
        //lis->status--;
        lis->i2c_trans.status = I2CTransDone;
        lis3mdl_i2c_send_config(lis); // Retry config (TODO max retry)
      }
    }
  }
}
