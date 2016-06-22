int i = 0;
int off = 5;

void inc(void){
  i += off;
}
/** @brief: Gets an idle event from the scheduler
 * */
void vApplicationIdleHook(void)
{

}

/** @brief:
 */
void vApplicationStackOverflowHook(void)
{

}

/** @brief:
 */
void vApplicationMallocFailedHook(void)
{

}

int main(void){
  while (1) {
    inc();
  }
}


