
#include "Ultrasonic_Task.h"
#include "cmsis_os.h"


extern transmission_en  gl_transmission_en;
ULTRASONIC_PASS Pass_Signal;
static ULTRASONIC_Status Status=Time_Out_;
extern TaskHandle_t Ultrasonic_Timeout_Handel;
extern TaskHandle_t Ultra_Handel;
extern uint8_t gl_u8_throttle ;
extern SemaphoreHandle_t semaphore_transmissionHandle;
void Ultrasonic_Task (void*pvParameter )
    {

	  vTaskSuspend(NULL);

        for(;;)
        {
        	 vTaskResume( Ultrasonic_Timeout_Handel);

            if(gl_transmission_en==Drive)
            {
                set_Ultrasonic_num(ULTRASONIC1);
            }
            else
            {
            	set_Ultrasonic_num(ULTRASONIC2);
            }
            Ultrasonic_Updatedistance();
            if(Status==Success_)
            {
                if(Ultrasonc_getdistace()<CRACH_DISTANCE )
                {
                    Pass_Signal= Red_Flag;
                     gl_u8_throttle = 0;
                     xSemaphoreGive(semaphore_transmissionHandle);

                }
                else
                {
                    Pass_Signal= Green_Flag;
                }
            }
            else if( Status== Time_Out_)
            {
                Pass_Signal= Green_Flag;
            }

            osDelay(250);
        }
    }



 void Ultrasonic_Timeout_Task(void*pvParameter)
    {
        vTaskSuspend(NULL);
        static uint8_t enetrfunc_count=0;
        ULTRASONIC_STAGE statge;
        for(;;)
        {
        	if(enetrfunc_count==0){
        	enetrfunc_count++;
               }
        	else if(enetrfunc_count==1)
            {

                statge=Ultrasonc_getstatge();
                if(statge==Half_way_operation)
                {
                    Status= Time_Out_; //time_out cause the ultrasonic took more than 20ms that means the car doesn't detect any obstacle within 4 meter range
                    Ultrasonic_Int_Timeout( gl_transmission_en);

                }
                else if (statge==Complete_operation)
                {

                    Status= Success_;

                }
                enetrfunc_count=0;
                vTaskSuspend(NULL);
            }
            osDelay(20);
        }

    }


