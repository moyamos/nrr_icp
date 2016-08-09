#include "nrr_time.h"

nrr_time::nrr_time()
{
   update();
}

nrr_time::nrr_time(const nrr_time& time)
{
   second = time.second;
   microsecond = time.microsecond;
}

nrr_time::~nrr_time()
{
}

void nrr_time::update()
{
   gettimeofday (&systemTime, &no_good);
   second = systemTime.tv_sec;
   microsecond = systemTime.tv_usec;
}

long int nrr_time::usecdifference(const nrr_time& time)
{
   return 1000000 * (second - time.second ) + (microsecond - time.microsecond);
}

long int nrr_time::msecdifference(const nrr_time& time)
{
   return 1000 * (second - time.second ) + (microsecond - time.microsecond)/1000;
}

long int nrr_time::secdifference(const nrr_time& time)
{
   return second - time.second;
}

void nrr_time::set(const timeval& time)
{
   second = time.tv_sec;
   microsecond = time.tv_usec;
}

void nrr_time::get(timeval& time)
{
   time.tv_sec = second;
   time.tv_usec = microsecond;
}
