extern "C"
{
#include "main.h"
#include "splitmind_f401_hal_lib.h"
}

#include "Eigen/Core"

void setup()
{
  initPeriph();

  UART_printStrLn("Initialization...");
  UART_printStrLn("Initialization done!");
}

int main()
{
	setup();

	while(1)
	{

	}

  return 0;
}
