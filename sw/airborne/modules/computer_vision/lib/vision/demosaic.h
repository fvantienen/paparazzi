#ifndef DEMOSAIC_H
#define DEMOSAIC_H

/**
 * @brief Bilinear demosaicing
 * @param Output pointer to memory to store the demosaiced image
 * @param Input the input image as a flattened 2D array
 * @param Width, Height the image dimensions
 * @param RedX, RedY the coordinates of the upper-rightmost red pixel
 *
 * Bilinear demosaicing is considered to be the simplest demosaicing method and
 * is used as a baseline for comparing more sophisticated methods.
 *
 * The Input image is a 2D float array of the input RGB values of size
 * Width*Height in row-major order.  RedX, RedY are the coordinates of the
 * upper-rightmost red pixel to specify the CFA pattern.
 */
void BilinearDemosaicGray(uint8_t *Output, const uint16_t *Input, int Width, int Height,
                          int RedX, int RedY)
{
  const int Green = 1 - ((RedX + RedY) & 1);
  uint16_t AverageH, AverageV, AverageC, AverageX;
  int i, x, y;


  for (y = 0, i = 0; y < Height; y++)
    {
      for (x = 0; x < Width; x++, i++)
        {
          /* The following computes four quantities:
           *   AverageH: average of the horizontal neighbors
           *   AverageV: average of the vertical neighbors
           *   AverageC: average of the axial neighbors
           *   AverageX: average of the diagonal neighbors
           *
           * Near a boundary, the average is computed using only those
           * samples which are defined in the image.  For example on the left
           * boundary of the image, the left neighbors are omitted and
           * AverageH is simply set to the value of the right neighbor.
           */
          if (y == 0)
            {
              AverageV = Input[i + Width];

              if (x == 0)
                {
                  AverageH = Input[i + 1];
                  AverageC = (Input[i + 1] + Input[i + Width]) / 2;
                  AverageX = Input[i + 1 + Width];
                }
              else if (x < Width - 1)
                {
                  AverageH = (Input[i - 1] + Input[i + 1]) / 2;
                  AverageC = (Input[i - 1] + Input[i + 1]
                              + Input[i + Width]) / 3;
                  AverageX = (Input[i - 1 + Width]
                              + Input[i + 1 + Width]) / 2;
                }
              else
                {
                  AverageH = Input[i - 1];
                  AverageC = (Input[i - 1] + Input[i + Width]) / 2;
                  AverageX = Input[i - 1 + Width];
                }
            }
          else if (y < Height - 1)
            {
              AverageV = (Input[i - Width] + Input[i + Width]) / 2;

              if (x == 0)
                {
                  AverageH = Input[i + 1];
                  AverageC = (Input[i + 1] +
                              Input[i - Width] + Input[i + Width]) / 3;
                  AverageX = (Input[i + 1 - Width]
                              + Input[i + 1 + Width]) / 2;
                }
              else if (x < Width - 1)
                {
                  AverageH = (Input[i - 1] + Input[i + 1]) / 2;
                  AverageC = (AverageH + AverageV) / 2;
                  AverageX = (Input[i - 1 - Width] + Input[i + 1 - Width]
                              + Input[i - 1 + Width] + Input[i + 1 + Width]) / 4;
                }
              else
                {
                  AverageH = Input[i - 1];
                  AverageC = (Input[i - 1] +
                              Input[i - Width] + Input[i + Width]) / 3;
                  AverageX = (Input[i - 1 - Width]
                              + Input[i - 1 + Width]) / 2;
                }
            }
          else
            {
              AverageV = Input[i - Width];

              if (x == 0)
                {
                  AverageH = Input[i + 1];
                  AverageC = (Input[i + 1] + Input[i - Width]) / 2;
                  AverageX = Input[i + 1 - Width];
                }
              else if (x < Width - 1)
                {
                  AverageH = (Input[i - 1] + Input[i + 1]) / 2;
                  AverageC = (Input[i - 1]
                              + Input[i + 1] + Input[i - Width]) / 3;
                  AverageX = (Input[i - 1 - Width]
                              + Input[i + 1 - Width]) / 2;
                }
              else
                {
                  AverageH = Input[i - 1];
                  AverageC = (Input[i - 1] + Input[i - Width]) / 2;
                  AverageX = Input[i - 1 - Width];
                }
            }

          /* Generate grayscale output*/
          float GrayPx = 0;
          if (((x + y) & 1) == Green)
            {
              /* Center pixel is green */
              GrayPx += 0.7152 * (float)(Input[i]);

              if ((y & 1) == RedY)
                {
                  /* Left and right neighbors are red */
                  GrayPx += 0.2126 * (float)(AverageH);
                  GrayPx += 0.0722 * (float)(AverageV);
                }
              else
                {
                  /* Left and right neighbors are blue */
                  GrayPx += 0.2126 * (float)(AverageV);
                  GrayPx += 0.0722 * (float)(AverageH);
                }
            }
          else
            {
              GrayPx += 0.7152 * (float)(AverageC);

              if ((y & 1) == RedY)
                {
                  /* Center pixel is red */
                  GrayPx += 0.2126 * (float)(Input[i]);
                  GrayPx += 0.0722 * (float)(AverageX);
                }
              else
                {
                  /* Center pixel is blue */
                  GrayPx += 0.2126 * (float)(AverageX);
                  GrayPx += 0.0722 * (float)(Input[i]);
                }
            }

            Output[i] = (uint8_t)(GrayPx/256);

        }
    }
}

#endif /* DEMOSAIC_H */
