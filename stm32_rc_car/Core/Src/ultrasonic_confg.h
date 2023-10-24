#ifndef ULTRASONIC_CONFG_H_
#define ULTRASONIC_CONFG_H_

/* the 2 ultrasonic use Timer 1 through channel 3 and 2
*forward use channel 3 and back uses channel 2
*/

#define    TRIGER_FORWARD_PORT   GPIOA
#define    TRIGER_BACKWARD_PORT  GPIOA
#define    TRIGER_FORWARD_PIN   GPIO_PIN_8
#define    TRIGER_BACKWARD_PIN  GPIO_PIN_11
#define    CRACH_DISTANCE       25
#endif //ULTRASONIC_CONFG_H_
