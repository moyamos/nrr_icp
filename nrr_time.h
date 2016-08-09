#include <sys/time.h>

class nrr_time
{
   public:
      nrr_time();
      nrr_time(const nrr_time& time);

      ~nrr_time();
      void update();

      long int usecdifference(const nrr_time& time);
      long int msecdifference(const nrr_time& time);
      long int secdifference(const nrr_time& time);

      void set(const timeval& time);
      void get(timeval& time);

   private:
      long int microsecond;
      long int second;      
      timeval systemTime;
      struct timezone no_good;
};
