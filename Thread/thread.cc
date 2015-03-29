#ifndef THREAD_CC
#define THREAD_CC

#include <errno.h>
#include <unistd.h>
#include <cstring>
#include "./thread.hh"
#include <stdio.h>
namespace util{
  Thread::Thread() :
    handle(0)
    //agent(a)
  { }

  Thread::~Thread()
  {
    if (handle != 0)
    {
      stop();
    }
  }

  void Thread::start()
  {
    if (pthread_create(&handle, NULL, threadFunc, this) != 0)
    {
    //_debugLevel4("Could not create thread: " << strerror(errno));
    }
  }

  void Thread::sleep(long ms)
  {
    usleep(ms * 1000);
  }

  void Thread::stop()
  {
   // assert(handle);

    // BUG changing this from pthread_exit to pthread_cancel as sleep in vThread caused SIGABORT
    pthread_cancel(handle);

    handle = 0;
  }
 /* void Thread::setPriority(int tp)
  {

    assert(handle); // Thread object is null. Priority cannot be set
    int ret = SetThreadPriority(handle, tp);
    assert(ret); // Failed to set priority
  }

  void Thread::suspend()
  {
    assert(handle); // Thread object is null
    int ret = SuspendThread(handle);
    assert(ret >= 0); // Failed to suspend thread
  }

  void Thread::resume()
  {
    assert(handle); // Thread object is null
    int ret = ResumeThread(handle);
    assert(ret >= 0); // Failed to resume thread
  }*/
  void* Thread::threadFunc(void* arg)
  {

    printf("workinggggg\n");
    reinterpret_cast<Thread * >(arg)->run();
    return 0;
  }
}
  #endif