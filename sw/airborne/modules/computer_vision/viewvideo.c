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
#include "viewvideo.h"
#include "cv.h"
#include "lib/encoding/P7_H264.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

#include BOARD_CONFIG


// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 4
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Default image folder
#ifndef VIEWVIDEO_SHOT_PATH
#ifdef VIDEO_THREAD_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH VIDEO_THREAD_SHOT_PATH
#else
#define VIEWVIDEO_SHOT_PATH /data/video/images
#endif
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 30
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Define stream priority
#ifndef VIEWVIDEO_NICE_LEVEL
#define VIEWVIDEO_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

static P7_H264_context_t videoEncoder;
static FILE *video_file;

/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
struct image_t *viewvideo_function(struct image_t *img);
struct image_t *viewvideo_function(struct image_t *img)
{
  int32_t h264BufferIndex, size;
  uint8_t* h264Buffer;
  struct image_t releaseImg;

  if (viewvideo.is_streaming) {

    //jpeg_encode_image(img, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);

    /*if (viewvideo.use_rtp) {

      // Send image with RTP
      rtp_frame_send(
        &video_sock,              // UDP socket
        &img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        VIEWVIDEO_RTP_TIME_INC    // 90kHz time increment
      );
      // Extra note: when the time increment is set to 0,
      // it is automaticaly calculated by the send_rtp_frame function
      // based on gettimeofday value. This seems to introduce some lag or jitter.
      // An other way is to compute the time increment and set the correct value.
      // It seems that a lower value is also working (when the frame is received
      // the timestamp is always "late" so the frame is displayed immediately).
      // Here, we set the time increment to the lowest possible value
      // (1 = 1/90000 s) which is probably stupid but is actually working.
    }*/

    P7_H264_releaseInputBuffer(&videoEncoder, img->buf_idx);

    while ((h264BufferIndex = P7_H264_find_FreeBuffer(videoEncoder.inputBuffers, BUFFER_TOBE_RELEASED, videoEncoder.numInputBuffers)) != -1){
      releaseImg.buf_idx = h264BufferIndex;
      videoEncoder.inputBuffers[h264BufferIndex].status = BUFFER_FREE;
      v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, &releaseImg);
    }

    while (!P7_H264_getOutputBuffer(&videoEncoder, &h264BufferIndex))
    {

      h264Buffer = P7_H264_bufferIndex2OutputPointer(&videoEncoder, h264BufferIndex);
      size = P7_H264_bufferIndex2OutputSize(&videoEncoder, h264BufferIndex);

      if (size == 0)
        fprintf(stderr, "%s:%d warning, no data to write\n",__FILE__,__LINE__);
      else {
        printf("Got frame of size: %d\r\n", size);
        fwrite(h264Buffer, size, 1, video_file);
      }
      P7_H264_releaseOutputBuffer(&videoEncoder, h264BufferIndex);
    }

  } else {
    v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, img);
  }

  return NULL; // No new images were created
}

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  char save_name[512];
  video_file = fopen("test.h264", "wb");


  struct video_listener *listener = cv_add_to_device(&VIEWVIDEO_CAMERA, viewvideo_function);
  listener->maximum_fps = 0;

  viewvideo.is_streaming = true;
  videoEncoder.inputType = H264ENC_YUV422_INTERLEAVED_UYVY;
  videoEncoder.bitRate = 10*1000*1000; // 10 000 kbps
  videoEncoder.frameRate = 30;
  P7_H264_open(&videoEncoder, VIEWVIDEO_CAMERA.thread.dev);

  // Open udp socket
  udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);

  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create SDP file.\n");
  }
}

