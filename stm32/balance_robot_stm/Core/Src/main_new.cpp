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

MotorDriver motor_left;
MotorDriver motor_right;

MotorData motor_data;
volatile float w_est = 0;

MotorParameters maxon_params;
MotorParameters chi_params;

void setup()
{
  initPeriph();

  UART_printStrLn("Initialization...");

  motor_left.in1.port = GPIOA;
  motor_left.in1.pin = GPIO_PIN_7;
  motor_left.in2.port = GPIOC;
  motor_left.in2.pin = GPIO_PIN_7;
  motor_left.stndby.port = GPIOA;
  motor_left.stndby.pin = GPIO_PIN_6;
  motor_left.pwm.tim = &htim4;
  motor_left.pwm.tim_channel = TIM_CHANNEL_1;
  motor_left.side_sign = -1;

  motor_right.in1.port = GPIOA;
  motor_right.in1.pin = GPIO_PIN_9;
  motor_right.in2.port = GPIOA;
  motor_right.in2.pin = GPIO_PIN_8;
  motor_right.stndby.port = GPIOA;
  motor_right.stndby.pin = GPIO_PIN_6;
  motor_right.pwm.tim = &htim2;
  motor_right.pwm.tim_channel = TIM_CHANNEL_3;

  motor_left.setupKF();
  motor_right.setupKF();

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

  // motor_left.setParams(maxon_params);
  motor_left.setParams(chi_params);
  motor_right.setParams(chi_params);

  motor_left.motor_num = 0;
  motor_right.motor_num = 1;

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
      motor_left.standBy(0);
      t_start = getStMcs();
      start = 1;
      motor_left.time_start = getStMcs();
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

    motor_left.setVoltage(u);
    motor_left.setMext(0);
    motor_left.evalSensorData();
    // motor_left.runKalmanFilter();

    Eigen::Vector3f x;
    // x = motor_left.getKalmanState();

    readCurrent();

    UART_printStr("t: ");
    UART_print(getStMcs());
    // UART_printStr(" ticks: ");
    // UART_print(ticks);
    UART_printStr(" q: ");
    UART_printDiv(motor_left.getData().q);
    UART_printStr(" w: ");
    UART_printDiv(motor_left.getData().w);
    // UART_printDivLn(motor_left.getData().w);
    UART_printStr(" dw: ");
    UART_printDiv(motor_left.getData().dw);
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
      motor_left.standBy(0);
      // motor_left.shortBreak();
      // t_start = HAL_GetTick();
      t_start = getStMcs();
      start = 1;
      // motor_left.time_start = getStMcs() + HAL_GetTick();
      motor_left.time_start = getStMcs();
    }

    ms = HAL_GetTick();
    u = motor_left.generateCos(t_start, A, B, w, 3.1415);
    // u = generateRamp(100, 2);
    // u = 100;

    motor_left.setVoltage(u);
    // motor_left.setMext(-12.3e-3);
    motor_left.evalSensorData();
    motor_left.runKalmanFilter();

    Eigen::Vector3f x;
    x = motor_left.getKalmanState();

    // readCurrent();

    // UART_printStr("t: ");
    // UART_print(getStMcs());
    // UART_printStr(" q: ");
    // UART_printDiv(motor_left.getData().q);
    // UART_printStr(" w: ");
    // UART_printDiv(motor_left.getData().w);
    // // UART_printDiv(motor_left.getData().ew);
    // UART_printStr(" dw: ");
    // UART_printDiv(motor_left.getData().dw);
    // // UART_printDiv(motor_left.getData().edw);
    // // UART_printStr(" wk: ");
    // // UART_printDiv(x(0));
    // // UART_printStr(" dwk: ");
    // // UART_printDivLn(x(1));
    // UART_printStr(" u: ");
    // UART_print(u);
    // UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);
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

  // motor_left.standBy(1);
  motor_left.standBy(0);
  motor_left.setVoltage(100);

  motor_right.standBy(0);
  motor_right.setVoltage(100);

  volatile static unsigned long ms = 0;
  volatile static unsigned long my_ms = 0;

  while (1)
  {
    ms = HAL_GetTick();
    my_ms = getStMcs();

    // readCurrent();
    // UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);

    motor_left.evalSensorData();
    motor_right.evalSensorData();
    UART_printStr("1: ");
    UART_print(motor_left.getData().ticks);
    UART_printStr(" 2: ");
    UART_printLn(motor_right.getData().ticks);

    testButton();
    // motor_data = motor_left.getData();
    w_est = motor_left.getData().ew;
    motorSinTest();
    // motorStepTest();

    HAL_Delay(6);

    var++;
  }

  return 0;
}
