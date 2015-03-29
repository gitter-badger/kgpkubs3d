// This filec contains the Thread class

#ifndef THREAD_HH
#define THREAD_HH

# include <pthread.h>
//#include "../Core/CoreAgent/Coreagent.hh"
/* class Thread
   * Represents a thread of execution
   * in the process. To create a new Thread
   * write an inherited class of Thread and
   * override the run() function
   */
   namespace util {
  class Thread
  {
  private:
    pthread_t    handle;
    //CoreAgent agent;
    static void* threadFunc(void* arg);

  public:
    Thread();

    ~Thread();

    /* start(int flags)
     * creates a low-level thread object and calls the run() function
     */
    void start();

    virtual void run()=0;

    /* sleep(long ms)
     * holds back the thread's execution for
     * "ms" milliseconds
     */
    void sleep(long ms);

    /* stop()
     * stops the running thread and frees the thread handle
     */
    void stop();
  }; // class Thread
}
#endif // THREAD_HH