#ifndef RANDOM_H
#define RANDOM_H

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>

// Random number from uniform[0,1] distribution
double rand_uniform(void);

// Random number from gaussian(0, 1) distribution
double rand_gaussian(void);

#endif // RANDOM_H
