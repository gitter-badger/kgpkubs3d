#include "CoreAgent/Coreagent.hh"
#include <pthread.h>
int main()
{
  CoreAgent agent[5];
  pthread_t threads[5];
  int rc;
  for(int i=0;i<5;i++)
  {
      rc = pthread_create(&threads[i],NULL,&CoreAgent::threadrun,(void*)&agent[i]);
                          
      if (rc){
         //std::cout << "Error:unable to create thread, " << rc << endl;
         exit(-1);
      }
   }
   pthread_exit(NULL);
}

