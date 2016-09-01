/*
 * P7_H264.h
 *
 *  Created on: 4 févr. 2013
 *      Author: peline
 */

#ifndef P7_H264_H_
#define P7_H264_H_


#include <stdint.h>

/* For Hantro H.264 encoder */
#include <h264encapi.h>
#include <ewl.h>

#include <pthread.h>
#include <asm/types.h>          /* for videodev2.h */
#include <stdint.h>

#include <linux/videodev2.h>
#include "lib/v4l/v4l2.h"

#define P7_H264_INTRA_RATE_DEFAULT 30

typedef enum P7_H264_res_t_
{
  P7_H264_INVALID_ARGUMENTS = -5,
  P7_H264_NO_OUTPUT_BUFFER_AVAILABLE=-4,
  P7_H264_NO_INPUT_BUFFER_AVAILABLE=-3,
  P7_H264_INVALID_BUFFER_INDEX=-2,
  P7_H264_FAILED=-1,
  P7_H264_OK=0,
}P7_H264_res_t;

typedef enum P7_H264_bufferStatus_t_
{
  BUFFER_FREE = 0,          // buffer not in used
  BUFFER_CLIENT_LOCKED, // buffer is locked by client
  BUFFER_READY,         // input buffer, it means frame is ready to be encoded. ouput buffer, frame is ready to be used by client
  BUFFER_TOBE_RELEASED,
}P7_H264_bufferStatus_t;

typedef enum P7_H264_serverStatus_t_
{
  SERVER_INIT,
  SERVER_IDLE,
  SERVER_WORK,
}P7_H264_serverStatus_t;

typedef enum P7_H264_frameType_t_
{
  P7_H264_INVALID_FRAME=-1,
  P7_H264_I_FRAME,
  P7_H264_P_FRAME,
}P7_H264_frameType_t;

typedef struct P7_H264_buffer_t_
{
  uint32_t size;
  P7_H264_frameType_t frameType;
  EWLLinearMem_t vencMem;
  P7_H264_bufferStatus_t status;
  uint32_t frameIndex;
}P7_H264_buffer_t;

typedef struct P7_H264_context_t_
{
  // public
  uint32_t width;              /* width of the video source */
  uint32_t height;             /* height of the video source */
  uint32_t frameRate;          /* number of frames per second of the source */
  uint32_t bitRate;            /* number of bit per second (target) */
  uint32_t numInputBuffers;    /* number of input buffers for the encoder */
  uint32_t numOutputBuffers;   /* number of output buffers for the encoder */
  uint32_t intraRate;          /* every intraRate frames, an Intra frame is generated, set this parameter to 0 for default value */
  H264EncPictureType inputType;

  // private
  H264EncInst encoder;
  P7_H264_buffer_t* inputBuffers;
  P7_H264_buffer_t* outputBuffers;

  EWLLinearMem_t startBuffer;
  uint32_t startBufferSize;

  uint32_t inputFrameCounter;
  uint32_t outputFrameCounter;
  uint32_t encoderFrameCounter;

  H264EncIn encIn;
  H264EncOut encOut;

  uint32_t intraCounter;

  // thread & sync
  pthread_t* thread;                // list of thread
  pthread_attr_t attr;               // thread options
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  P7_H264_serverStatus_t serverStatus;
  uint32_t threadRun;
  struct v4l2_device *v4l2_dev;
}P7_H264_context_t;

/*
 * @brief Open a encoder instance
 * @param context       a context with public fields set
 * @return              P7_H264_OK if succeed, an error otherwise
 */
int P7_H264_open(P7_H264_context_t* context, struct v4l2_device *v4l2_dev);


/*
 * @brief Get input buffer to write a frame
 * @param context       an opened context
 * @param bufferIndex   return a valid bufferIndex, or -1 if an error happened
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_getInputBuffer(P7_H264_context_t* context, int32_t* bufferIndex);


/*
 * Release input buffer and start to encode it
 * @param context       an opened context
 * @param bufferIndex   a valid bufferIndex obtained with P7_H264_getInputBuffer()
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_releaseInputBuffer(P7_H264_context_t* context, int32_t bufferIndex);

/*
 * Get ouput buffer (encoded frame)
 * @param context       an opened context
 * @param bufferIndex   return a valid bufferIndex, or -1 if an error happened
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_getOutputBuffer(P7_H264_context_t* context, int32_t* bufferIndex);

/*
 * Release ouput buffer, so that it can be used to encoded another frame
 * @param context       an opened context
 * @param bufferIndex   a valid bufferIndex obtained with P7_H264_getOutputBuffer()
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_releaseOutputBuffer(P7_H264_context_t* context, int32_t bufferIndex);

/*
 * Finalyse stream and close encoder instance
 * @param context       context to destroy
 */
void P7_H264_close(P7_H264_context_t* context);


/*
 *  buffer utilities functions
 */

/*
 * @brief Get input frame pointer from bufferIndex
 *
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              input frame data pointer or NULL if the index is not valid
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getInputBuffer();
 */
uint8_t* P7_H264_bufferIndex2InputPointer(P7_H264_context_t* context, int32_t bufferIndex);

/*
 * @brief Get encoded frame pointer from bufferIndex
 *
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              output data pointer or NULL if the index is not valid
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
uint8_t* P7_H264_bufferIndex2OutputPointer(P7_H264_context_t* context, int32_t bufferIndex);

/*
 * @brief Get buffer index from encoded frame pointer
 *
 * @param context       encoder context
 * @param outputPointer pointer to convert
 * @return              a valid buffer index or -1 if an error occured
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
int32_t P7_H264_OutputPointer2bufferIndex(P7_H264_context_t* context, uint8_t* outputPointer);

/*
 * @brief Get encoded frame size from bufferIndex
 *
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              output data pointer or 0 if the index is not valid
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
uint32_t P7_H264_bufferIndex2OutputSize(P7_H264_context_t* context, int32_t bufferIndex);

/*
 * @brief Get encoded frame type from bufferIndex
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              frame type or P7_H264_INVALID_FRAME if an error occured
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
P7_H264_frameType_t P7_H264_bufferIndex2OutputType(P7_H264_context_t* context, int32_t bufferIndex);

int P7_H264_find_FreeBuffer(P7_H264_buffer_t *, int, uint32_t);
#endif /* P7_H264_H_ */
