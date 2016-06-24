#STM32-FreeRTOS
==============

An example project configuring FreeRTOS on the STM32VL Discovery board

## Notes

-Threads are indepently running tasks which are scheduled by priority by the task scheduler 

- Synchronization
  - Threads cannot safely share data structures or HW without some synchronization mechanisms. 
  - A semaphore is a standard synchronization primitive, which provides a thread safe interface to manage shared resources.
  - A thread requests a resource and is either granted or blocked by the scheudler until the resource become free. 
  - A thread releasing a resource, may as a side effect, un-block a waiting thread. 
  - Three types of semaphores exists in FreeRTOS
    - Binary
    - Mutexes
    - Counting
  - Mutexes and Binary semaphores are similar, but behave differently with respect to thread priority. 
  - The two important operations which can be performed on a mutex are GIVE and TAKE. 
  - TAKE - removes the mutex's token if available, or blocks the calling thread if not. 
  - GIVE - restores the token. 
    - Counting semaphores can have multiple tokens. 
