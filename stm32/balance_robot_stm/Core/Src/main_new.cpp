extern "C"
{
#include "main.h"
}

#include "stm32f401re/motor_driver/motor_driver.h"

volatile unsigned long last_time = 0;
volatile float velocity = 0;
bool flag_tick = 0;
bool is_button_pressed = 0;
volatile uint32_t var = 0;

MotorDriver motor;

void setup()
{
  initPeriph();

  UART_printStrLn("Initialization...");

  motor.setupKF();

  UART_printStrLn("Initialization done!");

  turnLed(1);
  HAL_Delay(500);
  turnLed(0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM11) // check if the interrupt comes from TIM11
  {
    st_ms++;
  }
}

void testButton(void)
{
  static uint16_t t = 0;

  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0)
  {
    t++;
    // UART_printLn(t);
    turnLed(1);
    // is_button_pressed = !is_button_pressed;
    is_button_pressed = true;
    // HAL_Delay(100);
  }
  else
  {
    turnLed(0);
  }
}

void motorSinTest()
{
  if (is_button_pressed)
  {
    static unsigned long ms = 0;
    static float w = 4;
    static float A = 6.0 / 12.0 * 100.0;
    static float B = 6.0 / 12.0 * 100.0;
    static float u = 0;
    static float fi = 3.1415;
    static float t = 0;

    static bool start = 0;
    static unsigned long t_start = 0;

    if (start == 0)
    {
      motor.standBy(0);
      // motor.shortBreak();
      t_start = HAL_GetTick();
      start = 1;
      motor.time_start = getStMcs() + HAL_GetTick();
    }

    ms = HAL_GetTick();
    u = generateCos(t_start, A, B, w, 3.1415);
    // u = generateRamp(100, 2);

    motor.setVoltage(u);
    // motor.setMext(0);
    motor.evalSensorData();
    // motor.runKalmanFilter();

    Eigen::Vector3f x;
    x = motor.getKalmanState();

    UART_printStr("t: ");
    UART_print(getStMcs());
    // UART_printStr(" ticks: ");
    // UART_print(ticks);
    UART_printStr(" q: ");
    UART_printDiv(motor.data.q);
    UART_printStr(" u: ");
    UART_print(u);
    UART_printStr(" I: ");
    UART_printDivLn(0);
    // UART_printStr(" w: ");
    // UART_printDiv(motor.data.w);
    // UART_printStr(" dw: ");
    // UART_printDivLn(motor.data.dw);
    // UART_printStr(" wk: ");
    // UART_printDiv(x(0));
    // UART_printStr(" dwk: ");
    // UART_printDivLn(x(1));
  }
}

int main()
{
  setup();

  Eigen::Vector4f vec4(0, 0, 0, 0);
  Eigen::Vector2f vec2(1, 2);
  vec4.block<2, 1>(0, 0) = vec2;
  UART_printDivLn(vec4(0));
  UART_printDivLn(vec4(1));
  UART_printDivLn(vec4(2));
  UART_printDivLn(vec4(3));

  UART_printDivLn(4.9999);
  UART_printDivLn(4.99);
  UART_printDivLn(4.896);
  UART_printDivLn(4.4);
  motor.standBy(1);
  // motor.setVoltage(50);

  while (1)
  {
    testButton();

    motorSinTest();

    var++;
  }

  return 0;
}
