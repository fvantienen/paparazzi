/*
 * P7_H264.c
 *
 *  Created on: 4 févr. 2013
 *      Author: peline
 */

#define P7R2

#include "P7_H264.h"

/* For SW/HW shared memory allocation */
#include "ewl.h"

/* For accessing the EWL instance inside the encoder */
#include "H264Instance.h"

/* For compiler flags, test data, debug and tracing */
#include "enccommon.h"

/* For Hantro H.264 encoder */
#include "h264encapi.h"

/* For printing and file IO */
#include <stdio.h>

/* For dynamic memory allocation */
#include <stdlib.h>

/* For memset, strcpy and strlen */
#include <string.h>

#include <sys/time.h>

// be careful with next macro, some prints are done under locked mutex
//#define VERBOSE

#define PRINT(string,...) fprintf(stderr, "[P7_venc->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#ifdef VERBOSE
#define VERBOSE_PRINT PRINT
static char* streamType[2] = { "BYTE_STREAM", "NAL_UNITS" };
#else
#define VERBOSE_PRINT(...)
#endif

#define MIN_OUTPUT_BUF_SIZE 128*1024

/*------------------------------------------------------------------------------

    CloseEncoder
       Release an encoder insatnce.

   Params:
        encoder - the instance to be released
------------------------------------------------------------------------------*/
static void CloseEncoder(P7_H264_context_t* context)
{
    H264EncRet ret;

    if (context->encoder != NULL)
    {
      if((ret = H264EncRelease(context->encoder)) != H264ENC_OK)
      {
        PRINT("Failed to release the encoder. Error code: %8i\n", ret);
      }
      context->encoder=NULL;
    }
}

/*------------------------------------------------------------------------------

    OpenEncoder
        Create and configure an encoder instance.

    Params:
        context
    Return:
        0   - for success
        -1  - error

------------------------------------------------------------------------------*/
static int OpenEncoder(P7_H264_context_t* context)
{
    H264EncRet ret;
    H264EncConfig cfg;
    H264EncCodingCtrl codingCfg;
    H264EncRateCtrl rcCfg;
    H264EncPreProcessingCfg preProcCfg;
    H264EncBuild encBuild;
    H264EncInst encoder;

    encBuild = H264EncGetBuild();
    if (encBuild.hwBuild == EWL_ERROR){
      PRINT( "No device found\n");
      return -1;
    }

    cfg.width = context->width;
    cfg.height = context->height;

    cfg.frameRateDenom = 1;
    cfg.frameRateNum = context->frameRate;

    cfg.streamType = H264ENC_BYTE_STREAM;

    cfg.level = H264ENC_LEVEL_2_2; // level 4 minimum for 1080p
    cfg.viewMode = H264ENC_BASE_VIEW_DOUBLE_BUFFER; // maybe H264ENC_BASE_VIEW_SINGLE_BUFFER
    cfg.scaledWidth = 0;
    cfg.scaledHeight = 0;
    VERBOSE_PRINT("Init config: size %dx%d   %d/%d fps  %s L %d\n",
         cfg.width, cfg.height, cfg.frameRateNum,
         cfg.frameRateDenom, streamType[cfg.streamType], cfg.level);

    if((ret = H264EncInit(&cfg, &context->encoder)) != H264ENC_OK)
    {
        PRINT(
                "Failed to initialize the encoder. Error code: %8i\n", ret);
        return -1;
    }

    encoder = context->encoder;

    /* Encoder setup: rate control */
    if((ret = H264EncGetRateCtrl(encoder, &rcCfg)) != H264ENC_OK)
    {
        PRINT( "Failed to get RC info. Error code: %8i\n",
                ret);
        CloseEncoder(context);
        return -1;
    }
    else
    {
        VERBOSE_PRINT("Get rate control: qp %2d [%2d, %2d]  %8d bps  "
               "pic %d mb %d skip %d  hrd %d\n  cpbSize %d gopLen %d\n",
               rcCfg.qpHdr, rcCfg.qpMin, rcCfg.qpMax, rcCfg.bitPerSecond,
               rcCfg.pictureRc, rcCfg.mbRc, rcCfg.pictureSkip, rcCfg.hrd,
               rcCfg.hrdCpbSize, rcCfg.gopLen);

        rcCfg.bitPerSecond = context->bitRate;
        rcCfg.gopLen = context->intraRate; // user guide says to set goLopen to I frame rate
        rcCfg.hrd = 0; // disable Hrd conformance
#if 0
        if(cml->qpHdr != DEFAULT)
              rcCfg.qpHdr = cml->qpHdr;
          if(cml->qpMin != DEFAULT)
              rcCfg.qpMin = cml->qpMin;
          if(cml->qpMax != DEFAULT)
              rcCfg.qpMax = cml->qpMax;
          if(cml->vopSkip != DEFAULT)
              rcCfg.pictureSkip = cml->vopSkip;
          if(cml->vopRc != DEFAULT)
              rcCfg.pictureRc = cml->vopRc;
          if(cml->mbRc != DEFAULT)
              rcCfg.mbRc = cml->mbRc != 0 ? 1 : 0;
          if(cml->bitPerSecond != DEFAULT)
              rcCfg.bitPerSecond = cml->bitPerSecond;

          if(cml->hrdConformance != DEFAULT)
              rcCfg.hrd = cml->hrdConformance;

          if(cml->cpbSize != DEFAULT)
              rcCfg.hrdCpbSize = cml->cpbSize;

          if(cml->intraVopRate != 0)
              rcCfg.gopLen = cml->intraVopRate;
#endif

        VERBOSE_PRINT("Set rate control: qp %2d [%2d, %2d]  %8d bps  "
               "pic %d mb %d skip %d  hrd %d\n  cpbSize %d gopLen %d\n",
               rcCfg.qpHdr, rcCfg.qpMin, rcCfg.qpMax, rcCfg.bitPerSecond,
               rcCfg.pictureRc, rcCfg.mbRc, rcCfg.pictureSkip, rcCfg.hrd,
               rcCfg.hrdCpbSize, rcCfg.gopLen);

        if((ret = H264EncSetRateCtrl(encoder, &rcCfg)) != H264ENC_OK)
        {
            PRINT( "Failed to set RC info. Error code: %8i\n",
                    ret);
            CloseEncoder(context);
            return -1;
        }
    }

    /* Encoder setup: coding control */
    if((ret = H264EncGetCodingCtrl(encoder, &codingCfg)) != H264ENC_OK)
    {
        PRINT( "Failed to get CODING info. Error code: %8i\n",
                ret);
        CloseEncoder(context);
        return -1;
    }
    else
    {
        VERBOSE_PRINT
            ("Set coding control: SEI %d  Slice %5d  deblocking %d "
             "constIntra %d  videoRange %d\n",
             codingCfg.seiMessages, codingCfg.sliceSize,
             codingCfg.disableDeblockingFilter,
             codingCfg.constrainedIntraPrediction, codingCfg.videoFullRange);

        if((ret = H264EncSetCodingCtrl(encoder, &codingCfg)) != H264ENC_OK)
        {
            PRINT(
                    "Failed to set CODING info. Error code: %8i\n", ret);
            CloseEncoder(context);
            return -1;
        }
    }

    /* PreP setup */
    if((ret = H264EncGetPreProcessing(encoder, &preProcCfg)) != H264ENC_OK)
    {
        PRINT( "Failed to get PreP info. Error code: %8i\n",
                ret);
        CloseEncoder(context);
        return -1;
    }
    VERBOSE_PRINT
        ("Get PreP: input %4dx%d : offset %4dx%d : format %d : rotation %d "
           ": stab %d : cc %d\n",
         preProcCfg.origWidth, preProcCfg.origHeight, preProcCfg.xOffset,
         preProcCfg.yOffset, preProcCfg.inputType, preProcCfg.rotation,
         preProcCfg.videoStabilization, preProcCfg.colorConversion.type);

    preProcCfg.inputType = context->inputType;
    preProcCfg.rotation = H264ENC_ROTATE_0;
    preProcCfg.origWidth = context->width;
    preProcCfg.origHeight = context->height;

    VERBOSE_PRINT
        ("Set PreP: input %4dx%d : offset %4dx%d : format %d : rotation %d "
           ": stab %d : cc %d\n",
         preProcCfg.origWidth, preProcCfg.origHeight, preProcCfg.xOffset,
         preProcCfg.yOffset, preProcCfg.inputType, preProcCfg.rotation,
         preProcCfg.videoStabilization, preProcCfg.colorConversion.type);

    if((ret = H264EncSetPreProcessing(encoder, &preProcCfg)) != H264ENC_OK)
    {
        PRINT( "Failed to set PreP info. Error code: %8i\n",
                ret);
        CloseEncoder(context);
        return -1;
    }
    return 0;
}

/*------------------------------------------------------------------------------

    AllocRes

    Allocation of the physical memories used by both SW and HW:
    the input pictures and the output stream buffer.

    NOTE! The implementation uses the EWL instance from the encoder
          for OS independence. This is not recommended in final environment
          because the encoder will release the EWL instance in case of error.
          Instead, the memories should be allocated from the OS the same way
          as inside EWLMallocLinear().

------------------------------------------------------------------------------*/
static int AllocRes(P7_H264_context_t* context)
{
    i32 ret = EWL_OK;
    u32 pictureSize;
    u32 outbufSize;
    u32 i;

    /*
     *  alloc input buffer arrays
     */
    context->inputBuffers = malloc(context->numInputBuffers*sizeof(context->inputBuffers[0]));
    if (!context->inputBuffers)
    {
      PRINT("error, failed to allocate input buffer array\n");
      return -1;
    }

    /*
     *  alloc input buffers
     */
    pictureSize = context->width*context->height*3/2;

    for (i=0; i<context->numInputBuffers; i++)
    {
      /* Here we use the EWL instance directly from the encoder
       * because it is the easiest way to allocate the linear memories */
     /* ret = EWLMallocLinear(((h264Instance_s *)context->encoder)->asic.ewl, pictureSize,
          &context->inputBuffers[i].vencMem);*/
      context->inputBuffers[i].vencMem.busAddress = context->v4l2_dev->buffers[i].physp;
      context->inputBuffers[i].vencMem.virtualAddress = context->v4l2_dev->buffers[i].buf;
      context->inputBuffers[i].vencMem.size = context->v4l2_dev->buffers[i].length;
      context->inputBuffers[i].size = pictureSize;
      if (ret != EWL_OK)
      {
        PRINT("error, failed to allocate input buffer %d/%d\n",i,context->numInputBuffers);
        return -1;
      }
    }

    /*
     *  alloc output buffer array
     */
    context->outputBuffers = malloc(context->numOutputBuffers*sizeof(context->outputBuffers[0]));
    if (!context->outputBuffers)
    {
      PRINT("error, failed to allocate output buffer array\n");
      return -1;
    }

    /*
     *  alloc output buffers
     */
    /*outbufSize = context->bitRate/context->frameRate;*/
    outbufSize = context->width * context->height;
    PRINT("%s %d, outbufsize 0x%x\n", __func__, __LINE__, outbufSize);
    if (outbufSize < MIN_OUTPUT_BUF_SIZE)
    {
      outbufSize = MIN_OUTPUT_BUF_SIZE;
    }

    for (i=0; i<context->numOutputBuffers; i++)
    {
      printf("((h264Instance_s *)context->encoder)->asic.ewl: %p   -> %d\r\n", ((h264Instance_s *)context->encoder), offsetof(h264Instance_s, rateControl.virtualBuffer.timeScale));
      ret = EWLMallocLinear(((h264Instance_s *)context->encoder)->asic.ewl, outbufSize,
          &context->outputBuffers[i].vencMem);
      if (ret != EWL_OK)
      {
        PRINT("error, failed to allocate output buffer %d/%d\n",i,context->numOutputBuffers);
        return -1;
      }
    }


    /*
     * alloc start buffer
     */
    outbufSize = 128;
    ret = EWLMallocLinear(((h264Instance_s *)context->encoder)->asic.ewl, outbufSize,
        &context->startBuffer);
    if (ret != EWL_OK)
    {
      PRINT("error allocating start buffer\n");
      return -1;
    }


    VERBOSE_PRINT("Input buffer 0 size:          %d bytes\n", context->inputBuffers[0].vencMem.size);
    VERBOSE_PRINT("Input buffer 0 bus address:   0x%08x\n", context->inputBuffers[0].vencMem.busAddress);
    VERBOSE_PRINT("Input buffer 0 user address:  0x%08x\n", (u32) context->inputBuffers[0].vencMem.virtualAddress);
    VERBOSE_PRINT("Output buffer 0 size:         %d bytes\n", context->outputBuffers[0].vencMem.size);
    VERBOSE_PRINT("Output buffer 0 bus address:  0x%08x\n", context->outputBuffers[0].vencMem.busAddress);
    VERBOSE_PRINT("Output buffer 0 user address: 0x%08x\n", (u32) context->outputBuffers[0].vencMem.virtualAddress);

    return 0;
}

/*------------------------------------------------------------------------------

    FreeRes

    Release all resources allcoated byt AllocRes()

------------------------------------------------------------------------------*/
static void FreeRes(P7_H264_context_t* context)
{
  if (context->encoder)
  {
    if (context->inputBuffers)
    {
      uint32_t i;
      // free input buffers
      for (i=0; i<context->numInputBuffers; i++)
      {
        EWLFreeLinear(((h264Instance_s *)context->encoder)->asic.ewl, &context->inputBuffers[i].vencMem);
      }
      free(context->inputBuffers);
      context->inputBuffers = NULL;
    }
    if (context->outputBuffers)
    {
      uint32_t i;
      // free ouput buffers
      for (i=0; i<context->numOutputBuffers; i++)
      {
        EWLFreeLinear(((h264Instance_s *)context->encoder)->asic.ewl, &context->outputBuffers[i].vencMem);
      }
      free(context->outputBuffers);
      context->outputBuffers = NULL;
    }
  }
}

static int Encode(P7_H264_context_t* context, uint32_t inputBufferIndex, uint32_t outBufferIndex)
{
  int res=-1;
  H264EncRet ret=-1;
  H264EncRateCtrl rc;

  /* set in/out buffer */
#ifndef P7R2
  context->encIn.pNaluSizeBuf = NULL;
  context->encIn.naluSizeBufSize = 0;
#endif

  context->encIn.pOutBuf = context->outputBuffers[outBufferIndex].vencMem.virtualAddress;
  context->encIn.busOutBuf = context->outputBuffers[outBufferIndex].vencMem.busAddress;
  context->encIn.outBufSize = context->outputBuffers[outBufferIndex].vencMem.size;

  context->encIn.busLuma = context->inputBuffers[inputBufferIndex].vencMem.busAddress;
  context->encIn.busChromaU = context->encIn.busLuma + (context->width * context->height);
  context->encIn.timeIncrement=1;

  switch(context->inputBuffers[inputBufferIndex].frameType)
  {
    case P7_H264_I_FRAME:
      context->encIn.codingType = H264ENC_INTRA_FRAME;
      break;
    case P7_H264_P_FRAME:
      context->encIn.codingType = H264ENC_PREDICTED_FRAME;
      break;
    default:
      PRINT("unknown frame type %d\n",context->inputBuffers[inputBufferIndex].frameType);
      return -1;
  }

  /* encode frame */
#ifdef P7R2
  ret = H264EncStrmEncode(context->encoder, &context->encIn, &context->encOut, NULL, NULL);
#else
  ret = H264EncStrmEncode(context->encoder, &context->encIn, &context->encOut);
#endif
  H264EncGetRateCtrl(context->encoder, &rc);

  switch (ret)
  {
  case H264ENC_FRAME_READY:
    res = 0;
    VERBOSE_PRINT("frame #%5i | type %s | size %7i\n", context->inputBuffers[inputBufferIndex].frameIndex,
                                                       context->encOut.codingType == H264ENC_INTRA_FRAME ? " I  " :
                                                       context->encOut.codingType == H264ENC_PREDICTED_FRAME ? " P  " : "skip",
                                                       context->encOut.streamSize);
    context->outputBuffers[outBufferIndex].size = context->encOut.streamSize;
    break;

  case H264ENC_OUTPUT_BUFFER_OVERFLOW:
    VERBOSE_PRINT("frame #%5i | %2i | %s | size %7i\n", context->inputBuffers[inputBufferIndex].frameIndex, rc.qpHdr, "lost",
                                                context->encOut.streamSize);
    context->outputBuffers[outBufferIndex].size = 0;
    break;

  default:
    PRINT("FAILED. Error code: %i\n", ret);
    context->outputBuffers[outBufferIndex].size = 0;
    break;
  }

  return res;
}

static int P7_H264_initStartBuffer(P7_H264_context_t* context)
{
  H264EncRet ret;
#ifndef P7R2
  context->encIn.pNaluSizeBuf = NULL;
  context->encIn.naluSizeBufSize = 0;
#endif
  context->startBufferSize = 0;

  // generate start frame in last buffer
  context->encIn.pOutBuf = context->startBuffer.virtualAddress;
  context->encIn.busOutBuf = context->startBuffer.busAddress;
  context->encIn.outBufSize = context->startBuffer.size;

  /* Start stream */
  ret = H264EncStrmStart(context->encoder, &context->encIn, &context->encOut);
  if(ret != H264ENC_OK)
  {
      PRINT( "Failed to start the stream. Error code: %8i\n",
              ret);
      return -1;
  }

  context->startBufferSize = context->encOut.streamSize;
  printf("Start stream: %d\n", context->startBufferSize);

  return 0;
}

/*
 * thread buffer managment functions
 */
static int32_t P7_H264_serverWaitInputBuffer(P7_H264_context_t* context)
{
  int32_t inputBufferIndex=-1;

  pthread_mutex_lock(&context->mutex);
  {
    /* find an input buffer to process */
    while (context->threadRun)
    {
      uint32_t i;
      for (i=0; i<context->numInputBuffers; i++)
      {
        if (context->inputBuffers[i].status == BUFFER_READY &&
            context->inputBuffers[i].frameIndex == (context->encoderFrameCounter+1))
        {
          inputBufferIndex = i;
          break;
        }
      }
      if (inputBufferIndex == -1)
      {
        /* no pending buffer, wait for signal */
        pthread_cond_wait(&context->cond,&context->mutex);
      }
      else
      {
        break;
      }
    }
  }
  pthread_mutex_unlock(&context->mutex);

  return inputBufferIndex;
}

static int32_t P7_H264_serverWaitOutputBuffer(P7_H264_context_t* context)
{
  int32_t outputBufferIndex=-1;

  pthread_mutex_lock(&context->mutex);
  {
    /* find an input buffer to process */
    while (context->threadRun)
    {
      uint32_t i;
      for (i=0; i<context->numOutputBuffers; i++)
      {
        if (context->outputBuffers[i].status == BUFFER_FREE)
        {
          outputBufferIndex = i;
          break;
        }
      }
      if (outputBufferIndex == -1)
      {
        /* no pending buffer, wait for signal */
        pthread_cond_wait(&context->cond,&context->mutex);
      }
      else
      {
        break;
      }
    }
  }
  pthread_mutex_unlock(&context->mutex);

  return outputBufferIndex;
}


/*
 * Encoder thread
 */
static void *P7_H264_encoderThread(void* param)
{
  /* retrieve context */
  P7_H264_context_t* context = (P7_H264_context_t*) param;
  int ret = 0;

  /* next state is idle */
  pthread_mutex_lock(&context->mutex);
  {
    context->serverStatus = SERVER_IDLE;
    /* notify client, server is ready */
    pthread_cond_signal(&context->cond);
  }
  pthread_mutex_unlock(&context->mutex);

  /* let's work ! */
  while(context->threadRun)
  {
    int32_t inputBufferIndex;
    int32_t outputBufferIndex;

    /* find an input buffer to encode */
    VERBOSE_PRINT("server looks for an input buffer\n");
    inputBufferIndex = P7_H264_serverWaitInputBuffer(context);
    VERBOSE_PRINT("server took input buffer %d\n",inputBufferIndex);
    // check we didn't receive thread end signal
    if (!context->threadRun)
    {
      break;
    }
    /* find an available output buffer for encoded frame */
    VERBOSE_PRINT("server looks for an output buffer\n");
    outputBufferIndex = P7_H264_serverWaitOutputBuffer(context);
    VERBOSE_PRINT("server took output buffer %d\n",outputBufferIndex);
    // check we didn't receive thread end signal
    if (!context->threadRun)
    {
      break;
    }

    VERBOSE_PRINT("vencThread: encode buffer %d -> buffer %d\n",inputBufferIndex,outputBufferIndex);

    /* encode buffer */
    if (!(ret = Encode(context,inputBufferIndex,outputBufferIndex)))
    {
      /* bind input output buffer */
      context->outputBuffers[outputBufferIndex].frameType = context->inputBuffers[inputBufferIndex].frameType;
      context->outputBuffers[outputBufferIndex].frameIndex = context->inputBuffers[inputBufferIndex].frameIndex;

      /* insert SPS PPS at the beginning of an I frame */
      if (context->inputBuffers[inputBufferIndex].frameType == P7_H264_I_FRAME)
      {
        if ((context->outputBuffers[outputBufferIndex].size+context->startBufferSize) < context->outputBuffers[outputBufferIndex].vencMem.size)
        {
          // move encoded frame
          memmove((uint8_t*)context->outputBuffers[outputBufferIndex].vencMem.virtualAddress+context->startBufferSize, (uint8_t*)context->outputBuffers[outputBufferIndex].vencMem.virtualAddress, context->outputBuffers[outputBufferIndex].size);
          // insert SPS/PPS at the beginning
          memcpy(context->outputBuffers[outputBufferIndex].vencMem.virtualAddress,context->startBuffer.virtualAddress,context->startBufferSize);
          // update buffer size
          context->outputBuffers[outputBufferIndex].size += context->startBufferSize;
          PRINT("INSERT SPS/PPS\r\n");
        }
        else
        {
          PRINT("server error, failed to insert SPS/PPS\n");
        }
      }
      VERBOSE_PRINT("vencThread: encode buffer %d -> buffer %d, done\n",inputBufferIndex,outputBufferIndex);

      /* commit results */
      pthread_mutex_lock(&context->mutex);
      {
        context->outputBuffers[outputBufferIndex].status = BUFFER_READY;
        context->inputBuffers[inputBufferIndex].status = BUFFER_TOBE_RELEASED;
        context->encoderFrameCounter++;
        context->serverStatus = SERVER_IDLE;
      }
      pthread_mutex_unlock(&context->mutex);
    }
    else
    {
      PRINT("Ecoder ERROR %d\n",ret);
      /* an error occured, frame has not been encoded */
      pthread_mutex_lock(&context->mutex);
      {
        /* don't release output buffer */
        context->outputBuffers[outputBufferIndex].status = BUFFER_FREE;
        /* free input buffer */
        context->inputBuffers[inputBufferIndex].status = BUFFER_TOBE_RELEASED;
        context->encoderFrameCounter++;
        context->serverStatus = SERVER_IDLE;
      }
      pthread_mutex_unlock(&context->mutex);
    }

  }

  pthread_exit(NULL);
}


/**************************************************************************************************************************/
/*
 * @brief Open a encoder instance
 * @param context       a context with public fields set
 * @return              P7_H264_OK if succeed, an error otherwise
 */
int P7_H264_open(P7_H264_context_t* context, struct v4l2_device *v4l2_dev)
{
  P7_H264_res_t res = P7_H264_OK;

   context->numInputBuffers = v4l2_dev->buffers_cnt;
   context->numOutputBuffers = v4l2_dev->buffers_cnt;
   context->width = v4l2_dev->w;
   context->height = v4l2_dev->h;
   context->v4l2_dev = v4l2_dev;


  if (!context)
  {
    PRINT("error, context is null\n");
    return ((int) P7_H264_FAILED);
  }

  if (!context->intraRate )
  {
    context->intraRate = P7_H264_INTRA_RATE_DEFAULT;
  }

  // check number of buffer
  if (!res)
  {
    if (context->numInputBuffers == 0)
    {
      PRINT("error, cannot use 0 input buffers \n");
      res = P7_H264_FAILED;
    }
    if (context->numOutputBuffers == 0)
    {
      PRINT("error, cannot use 0 output buffers \n");
      res = P7_H264_FAILED;
    }
  }

  /* Encoder initialization */
  if(OpenEncoder(context) != 0)
  {
      res= P7_H264_FAILED;
  }

  if (!res)
  {
    /* Allocate input and output buffers */
    if(AllocRes(context) != 0)
    {
      PRINT( "P7 h264 Failed to allocate the external resources!\n");
      FreeRes(context);
      CloseEncoder(context);
      return -1;
    }
  }

  /* init buffers */
  if (!res)
  {
    uint32_t i;
    // init all buffers status
    for (i=0;i<(context->numInputBuffers); i++)
    {
      context->inputBuffers[i].status = BUFFER_FREE;
    }
    for (i=0;i<(context->numOutputBuffers); i++)
    {
      context->outputBuffers[i].status = BUFFER_FREE;
      context->outputBuffers[i].size = 0;
    }
  }

  /* init counters */
  if (!res)
  {
    context->inputFrameCounter = 0;
    context->outputFrameCounter = 0;
    context->encoderFrameCounter = 0;
    context->intraCounter = 0;
  }

  /* create and launch encoder thread */
  if (!res)
  {
    /* init server state */
    context->serverStatus = SERVER_INIT;
    context->threadRun = 1;

    // init mutex&cond
    int pthreadRes;
    pthreadRes = pthread_mutex_init(&context->mutex,NULL);
    if (!pthreadRes)
    {
      pthreadRes = pthread_cond_init(&context->cond,NULL);
    }

    // attach a thread and launch it
    if (!pthreadRes)
    {
      pthreadRes = pthread_attr_init(&context->attr);
    }
    if (!pthreadRes)
    {
      pthread_attr_setdetachstate(&context->attr, PTHREAD_CREATE_JOINABLE);
    }

    if (!pthreadRes)
    {
      pthread_create(&context->thread,&context->attr,P7_H264_encoderThread, context);
    }

    if (!pthreadRes)
    {
      /* wait for server to be ready */
      pthread_mutex_lock(&context->mutex);
      {
        if (context->serverStatus == SERVER_INIT)
        {
          pthread_cond_wait(&context->cond,&context->mutex);
        }
      }
      pthread_mutex_unlock(&context->mutex);
    }

    if (pthreadRes)
    {
      res = P7_H264_FAILED;
    }
  }

  /* generate start frame */
  if (!res)
  {
    res = P7_H264_initStartBuffer(context);
  }
  return ((int) res);
}


/*
 * @brief Get input buffer to write a frame
 * @param context       an opened context
 * @param bufferIndex   return a valid bufferIndex, or -1 if an error happened
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_getInputBuffer(P7_H264_context_t* context, int32_t* bufferIndex)
{
  /* check arguments */
  if (!context || !bufferIndex)
  {
    return P7_H264_INVALID_ARGUMENTS;
  }

  /* find a free buffer */
  *bufferIndex=-1;
  P7_H264_res_t res = P7_H264_NO_INPUT_BUFFER_AVAILABLE;

  pthread_mutex_lock(&context->mutex);
  {
    uint32_t i;
    for (i=0;i<context->numInputBuffers;i++)
    {
      if (context->inputBuffers[i].status == BUFFER_FREE)
      {
        // lock this buffer
        context->inputBuffers[i].status = BUFFER_CLIENT_LOCKED;
        res = P7_H264_OK;
        *bufferIndex = i;
        VERBOSE_PRINT("user took input %d\n",i);
        break;
      }
    }
  }
  pthread_mutex_unlock(&context->mutex);

  if (res)
  {
    VERBOSE_PRINT("error, failed to get input buffer\n");
  }

  return res;
}


/*
 * Release input buffer and start to encode it
 * @param context       an opened context
 * @param bufferIndex   a valid bufferIndex obtained with P7_H264_getInputBuffer()
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_releaseInputBuffer(P7_H264_context_t* context, int32_t bufferIndex)
{
  /* check arguments */
  if (!context)
  {
    return P7_H264_INVALID_ARGUMENTS;
  }

  if (bufferIndex < 0 || bufferIndex >= (int32_t)context->numInputBuffers)
  {
    return P7_H264_INVALID_BUFFER_INDEX;
  }

  /* prepare buffer for encoding */
  P7_H264_res_t res = P7_H264_INVALID_BUFFER_INDEX;

  pthread_mutex_lock(&context->mutex);
  {
    /* check that buffers was locked by user */
    /*if (context->inputBuffers[bufferIndex].status == BUFFER_CLIENT_LOCKED)*/
    {
      context->inputBuffers[bufferIndex].status = BUFFER_READY;
      res = P7_H264_OK;
      // order frame
      context->inputFrameCounter++;
      context->inputBuffers[bufferIndex].frameIndex = context->inputFrameCounter;
      // choose frame type
      if (context->intraCounter == 0)
      {
        context->inputBuffers[bufferIndex].frameType = P7_H264_I_FRAME;
      }
      else
      {
        context->inputBuffers[bufferIndex].frameType = P7_H264_P_FRAME;
      }
      context->intraCounter++;
      if (context->intraCounter >= context->intraRate)
      {
        context->intraCounter=0;
      }
      VERBOSE_PRINT("user released %d (frame type %d) frame rank %d\n",bufferIndex,context->inputBuffers[bufferIndex].frameType,context->inputBuffers[bufferIndex].frameIndex);
      /* wake up encoder thread */
      pthread_cond_signal(&context->cond);
    }
  }
  pthread_mutex_unlock(&context->mutex);

  if (res)
  {
    VERBOSE_PRINT("error, failed to release input buffer %d (%d)\n",bufferIndex,res);
  }

  return res;
}

/*
 * Get ouput buffer (encoded frame)
 * @param context       an opened context
 * @param bufferIndex   return a valid bufferIndex, or -1 if an error happened
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_getOutputBuffer(P7_H264_context_t* context, int32_t* bufferIndex)
{
  /* check arguments */
  if (!context || !bufferIndex)
  {
    return P7_H264_INVALID_ARGUMENTS;
  }

  /* find an encoded buffer */
  *bufferIndex=-1;
  P7_H264_res_t res = P7_H264_NO_OUTPUT_BUFFER_AVAILABLE;

  pthread_mutex_lock(&context->mutex);
  {
    uint32_t i;
    // look over all buffers
    for (i=0;i<context->numOutputBuffers;i++)
    {
      if (context->outputBuffers[i].status == BUFFER_READY)
      {
        // it's an encoded frame, but is it the oldest ?
        if (context->outputBuffers[i].frameIndex == (context->outputFrameCounter+1))
        {
          // yes, lock this buffer
          context->outputBuffers[i].status = BUFFER_CLIENT_LOCKED;
          context->outputFrameCounter++;
          res = P7_H264_OK;
          *bufferIndex = i;
          VERBOSE_PRINT("user took output %d (frame type %d)\n",i,context->outputBuffers[i].frameType);
          break;
        }
      }
    }
  }
  pthread_mutex_unlock(&context->mutex);

  if (res)
  {
    VERBOSE_PRINT("error, failed to get ouput buffer\n");
  }

  return res;
}

/*
 * Release ouput buffer, so that it can be used to encoded another frame
 * @param context       an opened context
 * @param bufferIndex   a valid bufferIndex obtained with P7_H264_getOutputBuffer()
 * @return              P7_H264_OK if succeed, an error otherwise
 */
P7_H264_res_t P7_H264_releaseOutputBuffer(P7_H264_context_t* context, int32_t bufferIndex)
{
  /* check arguments */
  if (!context)
  {
    return P7_H264_INVALID_ARGUMENTS;
  }

  if (bufferIndex < 0 || bufferIndex >= (int32_t)context->numOutputBuffers)
  {
    return P7_H264_INVALID_BUFFER_INDEX;
  }

  /* prepare buffer for encoding */
  P7_H264_res_t res = P7_H264_INVALID_BUFFER_INDEX;

  pthread_mutex_lock(&context->mutex);
  {
    /* check that buffers was locked by user */
    if (context->outputBuffers[bufferIndex].status == BUFFER_CLIENT_LOCKED)
    {
      context->outputBuffers[bufferIndex].status = BUFFER_FREE;
      res = P7_H264_OK;
      VERBOSE_PRINT("user released ouput %d (frame type %d)\n",bufferIndex,context->outputBuffers[bufferIndex].frameType);
      /* wake up encoder thread */
      pthread_cond_signal(&context->cond);
    }
  }
  pthread_mutex_unlock(&context->mutex);

  if (res)
  {
    VERBOSE_PRINT("error, failed to release ouput buffer %d (%d)\n",bufferIndex,res);
  }

  return res;
}

/*
 * Finalyse stream and close encoder instance
 * @param context       context to destroy
 */
void P7_H264_close(P7_H264_context_t* context)
{
  FreeRes(context);

  CloseEncoder(context);
}


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
uint8_t* P7_H264_bufferIndex2InputPointer(P7_H264_context_t* context, int32_t bufferIndex)
{
  if (context && bufferIndex >= 0 && bufferIndex < (int32_t)context->numInputBuffers)
  {
    return (uint8_t*)context->inputBuffers[bufferIndex].vencMem.virtualAddress;
  }
  else
  {
    return NULL;
  }
}

/*
 * @brief Get encoded frame pointer from bufferIndex
 *
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              output data pointer or NULL if the index is not valid
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
uint8_t* P7_H264_bufferIndex2OutputPointer(P7_H264_context_t* context, int32_t bufferIndex)
{
  if (context && bufferIndex >= 0 && bufferIndex < (int32_t)context->numOutputBuffers)
  {
    return (uint8_t*)context->outputBuffers[bufferIndex].vencMem.virtualAddress;
  }
  else
  {
    return NULL;
  }
}

/*
 * @brief Get buffer index from encoded frame pointer
 *
 * @param context       encoder context
 * @param outputPointer pointer to convert
 * @return              a valid buffer index or -1 if an error occured
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
int32_t P7_H264_OutputPointer2bufferIndex(P7_H264_context_t* context, uint8_t* outputPointer)
{
  int res=-1;
  if (context)
  {
    uint32_t i;
    for (i=0; i<context->numOutputBuffers; i++)
    {
      if ((uint8_t*)context->outputBuffers[i].vencMem.virtualAddress == outputPointer)
      {
        res = i;
        break;
      }
    }
  }
  return res;
}

/*
 * @brief Get encoded frame size from bufferIndex
 *
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              output data pointer or 0 if the index is not valid
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
uint32_t P7_H264_bufferIndex2OutputSize(P7_H264_context_t* context, int32_t bufferIndex)
{
  if (context && bufferIndex >= 0 && bufferIndex < (int32_t)context->numOutputBuffers)
  {
    return context->outputBuffers[bufferIndex].size;
  }
  else
  {
    return 0;
  }
}

/*
 * @brief Get encoded frame type from bufferIndex
 * @param context       encoder context
 * @param bufferIndex   bufferIndex to convert
 * @return              frame type or P7_H264_INVALID_FRAME if an error occured
 *
 * This function doesn't check that the buffer has been acquired with P7_H264_getOutputBuffer();
 */
P7_H264_frameType_t P7_H264_bufferIndex2OutputType(P7_H264_context_t* context, int32_t bufferIndex)
{
  if (context && bufferIndex >= 0 && bufferIndex < (int32_t)context->numOutputBuffers)
  {
    return context->outputBuffers[bufferIndex].frameType;
  }
  else
  {
    return P7_H264_INVALID_FRAME;
  }
}

int P7_H264_find_FreeBuffer(P7_H264_buffer_t *inputBuffers, int state, uint32_t nbuf)
{
  int itr;
  for(itr = 0; itr < nbuf; itr++)
    if (inputBuffers[itr].status == state)
	return itr;
  return -1;
}
