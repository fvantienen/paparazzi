/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/demosaic.h"
#include "udp_socket.h"

// Threaded computer vision
#include <pthread.h>


#define MT9F002_RAW_ROWS 2816
#define MT9F002_RAW_COLS 2112
#define MT9F002_ROWS 364
#define MT9F002_COLS 274
#define MT9F002_RAW_IMAGE_SIZE 2*274*364
#define MT9F002_GRAY_IMAGE_SIZE 274*364


// The place where the shots are saved (without slash on the end)
#ifndef VIEWVIDEO_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH /data/ftp/internal_000/images
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Main thread
static void *viewvideo_thread(void *data);
static void viewvideo_save_shot(struct image_t *img);
void viewvideo_periodic(void) { }

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .shot_number = 0
};


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *viewvideo_thread(void *data __attribute__((unused)))
{
  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(viewvideo.dev)) {
    printf("[viewvideo-thread] Could not start capture of %s.\n", viewvideo.dev->name);
    return 0;
  }

  // Start streaming
  viewvideo.is_streaming = TRUE;
  while (viewvideo.is_streaming) {
    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(viewvideo.dev, &img);

    // Save the image
    viewvideo_save_shot(&img);

    // Free the image
    v4l2_image_free(viewvideo.dev, &img);
  }

  return 0;
}

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  // Initialize the V4L2 subdevice (TODO: fix hardcoded path, which and code)
  if (!v4l2_init_subdev("/dev/v4l-subdev1", 0, 0, V4L2_MBUS_FMT_SGBRG10_1X10, 1408, 2112)) {
    printf("[viewvideo] Could not initialize the v4l-subdev1 subdevice.\n");
    return;
  }

  // Initialize the V4L2 device
  viewvideo.dev = v4l2_init("/dev/video1", 1408, 2112, 10);
  if (viewvideo.dev == NULL) {
    printf("[viewvideo] Could not initialize the /dev/video1 V4L2 device.\n");
    return;
  }

  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[viewvideo] Could not create shot directory %s.\n", STRINGIFY(VIEWVIDEO_SHOT_PATH));
    return;
  }
}

/**
 * Start with streaming
 */
void viewvideo_start(void)
{
  // Check if we are already running
  if (viewvideo.is_streaming) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  if (pthread_create(&tid, NULL, viewvideo_thread, NULL) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void viewvideo_stop(void)
{
  // Check if not already stopped streaming
  if (!viewvideo.is_streaming) {
    return;
  }

  // Stop the streaming thread
  viewvideo.is_streaming = FALSE;

  // Stop the capturing
  if (!v4l2_stop_capture(viewvideo.dev)) {
    printf("[viewvideo] Could not stop capture of %s.\n", viewvideo.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}

static void viewvideo_save_shot(struct image_t *img)
{
  uint8_t img_buf[MT9F002_RAW_IMAGE_SIZE];
  for (int i = 0; i < MT9F002_COLS; i++)
    {
      memcpy(img_buf + i * 2 * MT9F002_ROWS, img->buf + i * MT9F002_RAW_ROWS, MT9F002_ROWS * 2);
    }

  uint8_t gray_img[MT9F002_ROWS * MT9F002_COLS];
  uint16_t *img_buf_u16 = (uint16_t*)img_buf;
  BilinearDemosaicGray(gray_img, img_buf_u16, MT9F002_ROWS, MT9F002_COLS, 0, 1);

  // Search for a file where we can write to
  char save_name[128];
  for (; viewvideo.shot_number < 99999; viewvideo.shot_number++) {
    sprintf(save_name, "%s/img_%05d.pgm", STRINGIFY(VIEWVIDEO_SHOT_PATH), viewvideo.shot_number);
    // Check if file exists or not
    if (access(save_name, F_OK) == -1) {
      FILE *fp = fopen(save_name, "w");
      if (fp == NULL) {
        printf("[viewvideo-thread] Could not write shot %s.\n", save_name);
      } else {
        // Save it to the file and close it
        char pgm_header[] = "P5\n364 264\n255\n";
        fwrite(pgm_header, sizeof(char), 15, fp);
        fwrite(gray_img, sizeof(uint8_t), MT9F002_GRAY_IMAGE_SIZE, fp);
        fclose(fp);
      }

      // We don't need to seek for a next index anymore
      break;
    }
  }
}
