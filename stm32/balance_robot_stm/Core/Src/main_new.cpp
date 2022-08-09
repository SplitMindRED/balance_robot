extern "C"
{
#include "main.h"
}

#include "stm32f401re/ina219/ina219.h"
#include "stm32f401re/motor_driver/motor_driver.h"

volatile unsigned long last_time = 0;
volatile float velocity = 0;
bool flag_tick = 0;
bool is_button_pressed = 0;
volatile uint32_t var = 0;

MotorDriver motor;

MotorParameters maxon_params;
MotorParameters chi_params;

void setup()
{
  initPeriph();

  UART_printStrLn("Initialization...");

  motor.setupKF();

  maxon_params.L = 1.0551;
  maxon_params.R = 5.9043;
  maxon_params.Kw = 1.9437;
  maxon_params.Km = 1.9437;
  maxon_params.J = 0.0085;
  maxon_params.Lam = 0.0020;
  maxon_params.ticks_per_round = 62000.0;

  chi_params.L = 0.0314;
  chi_params.J = 0.0011;
  chi_params.R = 11.8227;
  chi_params.Kw = 0.2923;
  chi_params.Km = 0.2923;
  chi_params.Lam = 0.0019;
  chi_params.ticks_per_round = 1180.0;

  // motor.setParams(maxon_params);
  motor.setParams(chi_params);

  getData();
  HAL_Delay(100);
  printData();

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
void motorStepTest()
{
  if (is_button_pressed)
  {
    static float u = 0;
    static bool start = 0;
    static unsigned long t_start = 0;
    static unsigned long t_prev = 0;
    static uint16_t delta_t = 2500; // ms
    static uint8_t num_of_steps = 10;
    static uint8_t step = 0;

    if (start == 0)
    {
      motor.standBy(0);
      t_start = getStMcs();
      start = 1;
      motor.time_start = getStMcs();
      t_prev = HAL_GetTick();
    }

    if (HAL_GetTick() > t_prev + delta_t)
    {
      t_prev = HAL_GetTick();

      if (step < num_of_steps)
      {
        u += 10;
        step++;
      }
    }

    motor.setVoltage(u);
    motor.setMext(0);
    motor.evalSensorData();
    // motor.runKalmanFilter();

    Eigen::Vector3f x;
    // x = motor.getKalmanState();

    readCurrent();

    UART_printStr("t: ");
    UART_print(getStMcs());
    // UART_printStr(" ticks: ");
    // UART_print(ticks);
    UART_printStr(" q: ");
    UART_printDiv(motor.getData().q);
    UART_printStr(" w: ");
    UART_printDiv(motor.getData().w);
    // UART_printDivLn(motor.getData().w);
    UART_printStr(" dw: ");
    UART_printDiv(motor.getData().dw);
    // UART_printStr(" wk: ");
    // UART_printDiv(x(0));
    // UART_printStr(" dwk: ");
    // UART_printDivLn(x(1));
    UART_printStr(" u: ");
    UART_print(u);
    UART_printStr(" I: ");
    UART_printDivLn(current / 10.0 / 1000.0);
  }
}

void motorSinTest()
{
  if (is_button_pressed)
  {
    static unsigned long ms = 0;
    static float w = 1;
    // static float A = 6.0 / 12.0 * 100.0;
    // static float B = 6.0 / 12.0 * 100.0;
    static float A = 3.0 / 6.0 * 100.0;
    static float B = 3.0 / 6.0 * 100.0;
    static float u = 0;
    static float fi = 3.1415;
    static float t = 0;

    static bool start = 0;
    static unsigned long t_start = 0;

    if (start == 0)
    {
      motor.standBy(0);
      // motor.shortBreak();
      // t_start = HAL_GetTick();
      t_start = getStMcs();
      start = 1;
      // motor.time_start = getStMcs() + HAL_GetTick();
      motor.time_start = getStMcs();
    }

    ms = HAL_GetTick();
    u = motor.generateCos(t_start, A, B, w, 3.1415);
    // u = generateRamp(100, 2);
    // u = 100;

    motor.setVoltage(u);
    // motor.setMext(-12.3e-3);
    motor.evalSensorData();
    motor.runKalmanFilter();

    Eigen::Vector3f x;
    x = motor.getKalmanState();

    readCurrent();

    UART_printStr("t: ");
    UART_print(getStMcs());
    // UART_printStr(" ticks: ");
    // UART_print(ticks);
    UART_printStr(" q: ");
    UART_printDiv(motor.getData().q);
    UART_printStr(" w: ");
    UART_printDiv(motor.getData().w);
    // UART_printDiv(motor.getData().ew);
    UART_printStr(" dw: ");
    UART_printDiv(motor.getData().dw);
    // UART_printDiv(motor.getData().edw);
    // UART_printStr(" wk: ");
    // UART_printDiv(x(0));
    // UART_printStr(" dwk: ");
    // UART_printDivLn(x(1));
    UART_printStr(" u: ");
    UART_print(u);
    UART_printStr(" I: ");
    UART_printDivLn(current / 10.0 / 1000.0);
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

  UART_printStr("q: ");
  UART_printDiv(4.4);
  UART_printStr(" dq: ");
  UART_printDiv(123.9986);
  UART_printStr(" ddq: ");
  UART_printDivLn(0.001);

  motor.standBy(1);
  // motor.standBy(0);
  // motor.setVoltage(100);

  volatile static unsigned long ms = 0;
  volatile static unsigned long my_ms = 0;

  while (1)
  {
    ms = HAL_GetTick();
    my_ms = getStMcs();

    // readCurrent();
    // UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);

    // UART_printLn(ticks);

    testButton();

    motorSinTest();
    // motorStepTest();

    HAL_Delay(6);

    var++;
  }

  return 0;
}
