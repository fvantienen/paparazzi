/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/lib/opengl/opengl.c
 *
 * Handles all the OpenGL ES and EGL setup and shared
 * functions. This is developped for the Mali GPU
 * especially for vision processing.
 */

#include "opengl.h"
#include <stdio.h>
#include <stdlib.h>

#define STRINGIFY(s) #s

/* The main OpenGL structure */
struct opengl_t opengl;

/* OpenGL Config attributes */
static EGLint opengl_config_att[] = {
  EGL_SAMPLES,             4,
  EGL_RED_SIZE,            8,                   /* For UYVY this is U */
  EGL_GREEN_SIZE,          8,                   /* For UYVY this is Y */
  EGL_BLUE_SIZE,           8,                   /* For UYVY this is V */
  EGL_ALPHA_SIZE,          8,                   /* For UYVY this is Y */
  EGL_BUFFER_SIZE,         32,                  /* The total of RGBA 8*4=32 */
  EGL_STENCIL_SIZE,        0,
  EGL_RENDERABLE_TYPE,     EGL_OPENGL_ES2_BIT,  /* Request version 2 of opengl */
  EGL_SURFACE_TYPE,        EGL_PBUFFER_BIT,     /* We are creating a pixel buffer */
  EGL_DEPTH_SIZE,          16,
  EGL_NONE
};

/* OpenGL Context attributes */
static EGLint opengl_context_att[] = {
  EGL_CONTEXT_CLIENT_VERSION, 2, /* Request opengl version 2 */
  EGL_NONE
};

/* OpenGL Pixel Buffer attributes */
static EGLint opengl_pbuffer_att[] = {
  EGL_WIDTH, 320,
  EGL_HEIGHT, 240,
/*  EGL_COLORSPACE, GL_RGB, */
  EGL_TEXTURE_FORMAT, EGL_TEXTURE_RGBA,
  EGL_TEXTURE_TARGET, EGL_TEXTURE_2D,
  EGL_NONE
};

/**
 * Initialize OpenGL and EGL for vision processing
 */
bool_t opengl_init(void)
{
  EGLBoolean ret;
  EGLint config_nb = 0;

  /* Get the current display */
  opengl.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (opengl.display == EGL_NO_DISPLAY) {
    printf("[opengl] Could not get the default display.\n");
    return FALSE;
  }

  /* Initialize the display (ignore the version number) */
  ret = eglInitialize(opengl.display, NULL, NULL);
  if (!ret) {
    printf("[opengl] Could not initialize the EGL display.\n");
    return FALSE;
  }

  /* Choose an EGL config */
  ret = eglChooseConfig(opengl.display, opengl_config_att, &opengl.config, 1, &config_nb);
  if (!ret || config_nb != 1) {
    printf("[opengl] Could not choose a config.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Create a window surface */
  opengl.surface = eglCreatePbufferSurface(opengl.display, opengl.config, opengl_pbuffer_att);
  if (opengl.surface == EGL_NO_SURFACE) {
    printf("[opengl] Could not create a pixel buffer surface.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Create a new context */
  opengl.context = eglCreateContext(opengl.display, opengl.config, EGL_NO_CONTEXT, opengl_context_att);
  if (opengl.context == EGL_NO_CONTEXT) {
    printf("[opengl] Could not create context.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Set the display, surface and context as current */
  ret = eglMakeCurrent(opengl.display, opengl.surface, opengl.surface, opengl.context);
  if(!ret) {
    printf("[opengl] Could not set the display, surface and context as current.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  char vShaderStr[] = STRINGIFY(
    attribute vec4 inputTextureCoordinate;
    attribute vec4 vPosition;
    varying vec2 textureCoordinate;
    void main() {
      gl_Position = vPosition;
      textureCoordinate = inputTextureCoordinate.xy;
    }
  );

  printf("Shader string:\n");
  char fShaderStr[] = STRINGIFY(
    precision highp float;
    uniform sampler2D videoFrame;
    uniform float texel_width;
    uniform float texel_height;
    varying highp vec2 textureCoordinate;
    void main() {
      mediump float p[16];
      int pixel[16];

      for(int i=0; i < 16; i++) {
          pixel[i] = i;
      }

      p[8]  = texture2D(videoFrame, vec2(textureCoordinate.x,               textureCoordinate.y-(texel_height*3.0))).g; //  0  -3
      p[7]  = texture2D(videoFrame, vec2(textureCoordinate.x+ texel_width     , textureCoordinate.y-(texel_height*3.0))).g; //  1  -3
      p[6]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*2.0), textureCoordinate.y-(texel_height*2.0))).g; //  2  -2

      p[5]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*3.0), textureCoordinate.y- texel_height     )).g; //  3  -1
      p[4]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*3.0), textureCoordinate.y              )).g; //  3   0
      p[3]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*3.0), textureCoordinate.y+ texel_height     )).g; //  3   1

      p[2]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*2.0), textureCoordinate.y+(texel_height*2.0))).g; //  2   2
      p[1]  = texture2D(videoFrame, vec2(textureCoordinate.x+(texel_width*1.0), textureCoordinate.y+(texel_height*3.0))).g; //  1   3
      p[0]  = texture2D(videoFrame, vec2(textureCoordinate.x              , textureCoordinate.y+(texel_height*3.0))).g; //  0   3

      p[15]  = texture2D(videoFrame, vec2(textureCoordinate.x- texel_width     , textureCoordinate.y+(texel_height*3.0))).g; // -1   3
      p[14] = texture2D(videoFrame, vec2(textureCoordinate.x-(texel_width*2.0), textureCoordinate.y+(texel_height*2.0))).g; // -2   2
      p[13] = texture2D(videoFrame, vec2(textureCoordinate.x-(texel_width*3.0), textureCoordinate.y+ texel_height     )).g; // -3   1

      p[12] = texture2D(videoFrame, vec2(textureCoordinate.x-(texel_width*3.0), textureCoordinate.y              )).g; // -3  0
      p[11] = texture2D(videoFrame, vec2(textureCoordinate.x-(texel_width*3.0), textureCoordinate.y- texel_height     )).g; // -3  -1
      p[10] = texture2D(videoFrame, vec2(textureCoordinate.x-(texel_width*2.0), textureCoordinate.y-(texel_height*2.0))).g; // -2  -2

      p[9] = texture2D(videoFrame, vec2(textureCoordinate.x- texel_width     , textureCoordinate.y-(texel_height*3.0))).g; // -1  -3

      gl_FragColor = texture2D(videoFrame, textureCoordinate);

      mediump float cb = texture2D(videoFrame, textureCoordinate).g + 0.2;
      mediump float c_b= texture2D(videoFrame, textureCoordinate).g - 0.2;

      for(int x=0; x < 1; x++) {

//*
        if(p[pixel[0]] > cb) {
         if(p[pixel[1]] > cb)
          if(p[pixel[2]] > cb)
           if(p[pixel[3]] > cb)
            if(p[pixel[4]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 {}
                else
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[15]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[12]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[14]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[14]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[13]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[6]] < c_b)
                  if(p[pixel[5]] < c_b)
                   if(p[pixel[4]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[5]] < c_b)
                    if(p[pixel[4]] < c_b)
                     {}
                    else
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[5]] < c_b)
                 if(p[pixel[4]] < c_b)
                  if(p[pixel[3]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[4]] < c_b)
                   if(p[pixel[3]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[6]] < c_b)
              if(p[pixel[5]] < c_b)
               if(p[pixel[4]] < c_b)
                if(p[pixel[3]] < c_b)
                 if(p[pixel[2]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[10]] < c_b)
              if(p[pixel[6]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[3]] < c_b)
                  if(p[pixel[2]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        }
        else if(p[pixel[0]] < c_b) {
         if(p[pixel[1]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[6]] > cb)
              if(p[pixel[5]] > cb)
               if(p[pixel[4]] > cb)
                if(p[pixel[3]] > cb)
                 if(p[pixel[2]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[2]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[5]] > cb)
                 if(p[pixel[4]] > cb)
                  if(p[pixel[3]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[3]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[6]] > cb)
                  if(p[pixel[5]] > cb)
                   if(p[pixel[4]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[4]] > cb)
             if(p[pixel[13]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[11]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[5]] > cb)
              if(p[pixel[14]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[12]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[6]] > cb)
               if(p[pixel[15]] < c_b)
                if(p[pixel[13]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[12]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[7]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 {}
                else
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
               else
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[14]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[5]] > cb)
                    if(p[pixel[4]] > cb)
                     {}
                    else
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[4]] > cb)
                   if(p[pixel[3]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[10]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[3]] > cb)
                  if(p[pixel[2]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        }
        else {
         if(p[pixel[7]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[6]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[4]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[2]] > cb)
                 if(p[pixel[1]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  if(p[pixel[15]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[7]] < c_b)
          if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[6]] < c_b)
             if(p[pixel[5]] < c_b)
              if(p[pixel[4]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[2]] < c_b)
                 if(p[pixel[1]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] < c_b)
              if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  if(p[pixel[15]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else
          continue;
        }
        // */
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
      }
    }
  );

  GLuint vertexShader = opengl_shader_load(vShaderStr, GL_VERTEX_SHADER);
  GLuint fragmentShader = opengl_shader_load(fShaderStr, GL_FRAGMENT_SHADER);
  opengl.programObject = glCreateProgram();

  printf("Linking!\n");
  glAttachShader(opengl.programObject, vertexShader);
  glAttachShader(opengl.programObject, fragmentShader);
  glBindAttribLocation(opengl.programObject, 0, "vPosition");
  glBindAttribLocation(opengl.programObject, 1, "inputTextureCoordinate");
  glLinkProgram(opengl.programObject);

  GLint linked;
  glGetProgramiv(opengl.programObject, GL_LINK_STATUS, &linked);
  if (!linked)
  {
    printf("[opengl] Could not link program\n");
  }
  printf("Linking done!\n");


  // Clear the color buffer (UYVY)
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  //Create a texture
  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  GLint texUnitLoc = glGetUniformLocation(opengl.programObject, "videoFrame");
  glUniform1i(texUnitLoc, 1);
  glBindTexture(GL_TEXTURE_2D, 1);

  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

  // Set the viewport
  glViewport(0, 0, 320/2, 240);
  glClear(GL_COLOR_BUFFER_BIT);

  return TRUE;
}

/**
 * Clean up all the initialized OpenGL stuff
 */
void opengl_free(void) {
  eglTerminate(opengl.display);
}

/**
 * Create a new shader of a certain type based on the shader source.
 */
GLuint opengl_shader_load(const char *shaderSrc, GLenum type)
{
  GLuint shader;
  GLint compiled;

  // Create the shader object
  shader = glCreateShader(type);
  if (shader == 0) {
    printf("[opengl] Failed to create shader of type %d\n", type);
    return 0;
  }

  // Load the shader source
  glShaderSource(shader, 1, &shaderSrc, NULL);

  // Compile the shader
  glCompileShader(shader);

  // Check the compile status
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (!compiled) {
    GLint infoLen = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);

    if (infoLen > 1) {
      char *infoLog = malloc(sizeof(char) * infoLen);
      glGetShaderInfoLog(shader, infoLen, NULL, infoLog);
      printf("[opengl] Error compiling shader:\n%s\n", infoLog);
      free(infoLog);
    }
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}
