#include <sys/times.h>
#include "nrr_utility.h"

long getTickCount()
{
   struct tms tm;
   return times(&tm);
}
