
#include "Ultrasonic_Task.h"
#include "cmsis_os.h"


extern transmission_en  gl_transmission_en;
extern SemaphoreHandle_t Semaphore_Ultrasonic;
ULTRASONIC_PASS Pass_Signal=Green_Flag;
void Ultrasonic_Task (void*pvParameter )
{
 transmission_en  transmission_Ultra;
 uint16_t  DISTANCE;    


   vTaskSuspend(NULL);
  for(;;)
  {
	   transmission_Ultra=gl_transmission_en;
	   if(Ultrasonic_getdistance(transmission_Ultra,& DISTANCE)==SUCCESS_){
	
		 	  if(DISTANCE<CRACH_DISTANCE){

		 		  Pass_Signal= Red_Flag;
		 	  }
		 	 else{
		 		  Pass_Signal= Green_Flag;
		 	  }
	   }
	   else{
		   //do_nothing 
	   }

	  osDelay(20);
  }
}


void Ultrasonictest_Task(void*pvParameter){
	for(;;){
if(gl_transmission_en==Drive||gl_transmission_en==Reverse)	{
xSemaphoreGive(Semaphore_Ultrasonic);
}
osDelay(100);

	}
}
/* USER CODE END Header_StartDefaultTask */

/* Test_Ultrasonic_code

 * void Ultrasonictest_Task(void*pvParameter){
	for(;;){
if(gl_transmission_en==Drive||gl_transmission_en==Reverse)	{
xSemaphoreGive(Semaphore_Ultrasonic);
}
osDelay(100);
}

}
*/
