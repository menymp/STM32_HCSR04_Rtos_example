# STM32_HCSR04_Rtos_example
simple hcsr04 example with freertos using timer as asynchronus trigger timing output control

The idea is to demostrate the use of a timer as an acurated tool for uS response pulse measure of
HCSR04

The application outputs the reading over serial.

Two task compose the program one for measuring and trigger the timer, and other for output serial. in order to keep the output task
from sending a wrong measure number during echo reception, a mutex blocks the critical section.


ToDo list:
Add electrical diagram
