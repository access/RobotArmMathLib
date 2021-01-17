#include <RobotArm.h>

void InitRobotArm(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	osDelay(5000);
}



class CLed
{
  GPIO_TypeDef* _port;                                      //GPIOA,GPIOB....
  uint16_t _pin;                                            //GPIO_PIN_0,GPIO_PIN_1....
  uint16_t _toggleTime;                                     //Toggle time in ms
  uint16_t counter;                                         //Toggle time counter
  void on(){HAL_GPIO_WritePin(_port,_pin,GPIO_PIN_SET);}    //turn on LED
  void off(){HAL_GPIO_WritePin(_port,_pin,GPIO_PIN_RESET);} //turn off LED
  void toggle(){HAL_GPIO_TogglePin(_port,_pin);}            //toggle LED
public:
  CLed(GPIO_TypeDef* port,uint16_t pin,uint16_t toggleTim); //constructor
  void runToggle();                                         //run toggling LED from system tick every 1ms
};

CLed::CLed(GPIO_TypeDef* port,uint16_t pin,uint16_t toggleTime)
{
  _port=port;
  _pin=pin;
  _toggleTime=toggleTime;
  counter=0;
  off();
}

void CLed::runToggle()
{
  if(++counter>=_toggleTime)
  {
    counter=0;
    toggle();
  }
}

