extern "C"
{
#include "main.h"
}

#include "stm32f401re/ina219/ina219.h"
#include "stm32f401re/motor_driver/motor_driver.h"
// #include "stm32f401re/mpu6250/mpu6250.h"

volatile unsigned long last_time = 0;
volatile float velocity = 0;
bool flag_tick = 0;
bool is_button_pressed = 0;
volatile uint32_t var = 0;

struct RobotParams
{
  float L;
  float M;
  float r;
  float w;
  float h;
  float cm;
  float a;
  float m;
  float tau_max;
};

RobotParams robot_params;

MotorDriver motor_left("left");
MotorDriver motor_right("right");

MotorData motor_data;
volatile float w_est = 0;

MotorParameters maxon_params;
MotorParameters chi_params;

// MPU6250 mpu(MPU6250_ADDRESS);

void setup()
{
  initPeriph();

  UART_printStrLn("Initialization...");

  HAL_Delay(500);

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
  chi_params.R = 3.3201;
  chi_params.Kw = 0.2326;
  // chi_params.Km = 0.2923;
  // chi_params.Km = 0.4032;
  chi_params.Km = 0.18395;
  chi_params.Lam = 0.0011;
  chi_params.ticks_per_round = 1180.0;

  motor_left.setParams(chi_params);
  motor_right.setParams(chi_params);

  motor_left.motor_num = 0;
  motor_right.motor_num = 1;

  changeConfig();
  HAL_Delay(100);
  getData();
  HAL_Delay(100);
  printData();

  // mpu.init();

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

void motorStepTest(MotorDriver& driver)
{
  if (is_button_pressed)
  {
    static float u = 0;
    static bool start = 0;
    static unsigned long t_start = 0;
    static unsigned long t_prev = 0;
    static uint16_t delta_t = 5000; // ms
    static uint16_t num_of_steps = 10;
    static uint16_t step = 0;
    static long ticks_prev = 0;
    static float w = 0;

    if (start == 0)
    {
      driver.standBy(0);
      t_start = getStMcs();
      start = 1;
      driver.time_start = getStMcs();
      t_prev = HAL_GetTick();
    }

    readCurrent();
    HAL_Delay(1);
    float I = (float)current / 10.0 / 1e3;
    readVoltage();
    HAL_Delay(1);
    float V = (float)(voltage >> 3) * 4.0 / 1e3;

    if (HAL_GetTick() > t_prev + delta_t)
    {
      t_prev = HAL_GetTick();

      long tick = driver.getData().ticks;

      w = (float)(tick - ticks_prev) / 1180.0 * 360.0;
      ticks_prev = tick;

      if (step < num_of_steps)
      {
        u += 10;
        // u += 1;
        step++;
      }

      // readCurrent();
      // HAL_Delay(1);
      // float I = (float)current / 10.0 / 1e3;
      // readVoltage();
      // HAL_Delay(1);
      // float V = (float)(voltage >> 3) * 4.0 / 1e3;

      driver.setVoltage(u);
      driver.evalSensorData();
      // motor_left.runKalmanFilter();
    }

    driver.evalSensorData();

    UART_printStr("t: ");
    UART_print(getStMcs());
    // UART_printStr(" ticks: ");
    // UART_print(driver.getData().ticks);
    UART_printStr(" q: ");
    UART_printDiv(driver.getData().q);
    UART_printStr(" w: ");
    UART_printDiv(driver.getData().w);
    // UART_printDiv(w);
    UART_printStr(" dw: ");
    UART_printDiv(driver.getData().dw);
    // UART_printStr(" wk: ");
    // UART_printDiv(x(0));
    // UART_printStr(" dwk: ");
    // UART_printDivLn(x(1));
    UART_printStr(" u: ");
    UART_print(u);
    UART_printStr(" V: ");
    UART_printDiv(V);
    UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);
    UART_printDivLn(I);
    // UART_printDivLn(0);
  }
}

void motorSinTest(MotorDriver& driver)
{
  if (is_button_pressed)
  {
    static unsigned long ms = 0;
    static float w = 1;
    // static float w = 0.5e-1;
    // static float A = 6.0 / 12.0 * 100.0;
    // static float B = 6.0 / 12.0 * 100.0;
    static float A = 3.0 / 6.0 * 100.0;
    static float B = 3.0 / 6.0 * 100.0;
    static float u = 0.0;
    static float fi = 3.1415;
    static float t = 0;

    static bool start = 0;
    static unsigned long t_start = 0;

    if (start == 0)
    {
      driver.standBy(0);
      t_start = getStMcs();
      start = 1;
      driver.time_start = getStMcs();
    }

    ms = HAL_GetTick();
    u = driver.generateCos(t_start, A, B, w, 3.1415);
    // u = generateRamp(100, 2);
    // u = 100;

    driver.setVoltage(u);
    // motor_left.setMext(-12.3e-3);
    driver.evalSensorData();
    driver.runKalmanFilter();

    Eigen::Vector3f x;
    x = driver.getKalmanState();

    readCurrent();
    HAL_Delay(1);
    float I = (float)current / 10.0 / 1e3;
    readVoltage();
    HAL_Delay(1);
    // float V = (float)voltage / 8000.0 * 32.0;
    float V = (float)(voltage >> 3) * 4.0 / 1e3;

    // UART_printStr(driver.name);
    // UART_printStr(" ");
    UART_printStr("t: ");
    UART_print(getStMcs());
    UART_printStr(" q: ");
    UART_printDiv(driver.getData().q);
    // UART_printStr(" ticks: ");
    // UART_printDiv(driver.getData().ticks);
    UART_printStr(" w: ");
    UART_printDiv(driver.getData().w);
    UART_printStr(" ew: ");
    UART_printDiv(motor_left.getData().ew);
    UART_printStr(" dw: ");
    UART_printDiv(driver.getData().dw);
    // UART_printDiv(motor_left.getData().edw);
    // UART_printStr(" wk: ");
    // UART_printDiv(x(0));
    // UART_printStr(" dwk: ");
    // UART_printDivLn(x(1));
    UART_printStr(" eMext: ");
    UART_printDiv(driver.getData().eM_ext);
    UART_printStr(" u: ");
    UART_print(u);
    UART_printStr(" V: ");
    UART_printDiv(V);
    UART_printStr(" eI: ");
    UART_printDiv(driver.getData().eI);
    UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);
    UART_printDivLn(I);
    // UART_printDivLn(0);
  }
}

void testTorqueControl(MotorDriver& driver, float tau_des)
{
  if (is_button_pressed)
  {
    static float M_des = 0;
    static float M_act = 0;
    static float M_prev = 0;
    static unsigned long ms = 0;
    static float u = 0.0;
    static float t = 0;
    static unsigned long t_prev = 0;

    static bool start = 0;
    static unsigned long t_start = 0;

    // static float Kp = 1e2;
    // static float Ki = 2.5e2;
    // static float Kd = 6e-1;

    static float Kp = 1e2;
    static float Ki = 1e2;
    static float Kd = 1;

    // static float Kp = 8e2;
    // static float Ki = 8e2;
    // static float Kd = 1;

    static float sum = 0;
    static float dt = 0;
    // static float sum_max = 0.07;
    static float sum_max = 0.5;

    if (start == 0)
    {
      driver.standBy(0);
      t_start = getStMcs();
      start = 1;
      driver.time_start = getStMcs();
      t_prev = t_start;
      // u = 50;
      u = 0.0;
      driver.setVoltage(u);

      return;
    }

    driver.evalSensorData();
    driver.runKalmanFilter();

    M_act = driver.getData().M;

    // M_act = 0.0022 * driver.getData().dw + 0.0029 * driver.getData().w;
    // M_act = (u / 100 * 6.0 + 2.3041e-07 * driver.getData().w) / 85.7144;

    t = getStMcs();

    dt = (float)(t - t_prev) / 1e6; //sec

    //Mdes 0.095 max
    // M_des = 0.0005; //80g
    // M_des = -0.073575; //50g
    // M_des = -0.161865; //50g
    u = 100;
    M_des = tau_des;
    sum += M_des - M_act;

    if (sum > sum_max)
    {
      sum = sum_max;
    }

    if (sum < -sum_max)
    {
      sum = -sum_max;
    }

    // u = Kp * (M_des - M_act) + Ki * sum + Kd * (0.0 - (M_act - M_prev) / dt) + 9;
    // if (abs(M_des) < 0.0001)
    // {
    //   u = 0;
    // }
    // else
    // {
    //   // u = Kp * (M_des - M_act) + Ki * sum + Kd * (0.0 - (M_act - M_prev) / dt) + abs(M_des) / M_des * 5;
    //   u = Kp * (M_des - M_act) + Ki * sum + Kd * (0.0 - (M_act - M_prev) / dt);

    //   u = abs(u) * abs(M_des) / M_des;
    //   u += getSign(u) * 9;
    // }

    if (u > 100)
    {
      u = 100;
    }

    if (u < -100)
    {
      u = -100;
    }
    driver.setVoltage(u);

    M_prev = M_act;
    t_prev = t;

    // readCurrent();

    UART_printStr(driver.name);
    UART_printStr(" ");
    UART_printStr("U: ");
    UART_printDiv(u);
    // UART_printStr(" I: ");
    // UART_printDiv(current / 10.0 / 1000.0 - 0.048);
    // UART_printDiv(0);
    UART_printStr(" eI: ");
    UART_printDiv(driver.getData().eI);
    UART_printStr(" Mdes: ");
    UART_printDiv(M_des);
    UART_printStr(" Mact: ");
    UART_printDivLn(M_act);
  }
}

void testBalance()
{
  // static float ddq = 0;
  // static float q = 0;
  // static float q_prev = 0;
  // static float dq = 0;
  // static float x_act = 0;
  // static float v_act = 0;
  // static float tau_des = 0.0;

  // static float is_start = true;
  // static float q_calib_sum = 0;
  // static float q_offset = 269.69 * 3.1415 / 180.0;

  // static unsigned long t = 0;
  // static unsigned long t_prev = 0;
  // static float dt = 0;
  // static float u = 0;

  // static float Kp = 1;
  // static float Kd = 0;
  // static float P = 0;
  // static float D = 0;
  // static float tau_max = 0.22;
  // static float u_max = 100;

  // //calibration
  // if (is_start)
  // {
  //   is_start = false;
  //   t = getStMcs();
  //   t_prev = t;

  //   motor_left.standBy(0);

  //   return;

  //   // for (size_t i = 0; i < 100; i++)
  //   // {
  //   //   mpu.getData();
  //   //   q = atan2(mpu.data.AcZ, mpu.data.AcX) + 3.1415 / 2.0;
  //   //   q_calib_sum += q;
  //   //   HAL_Delay(100);
  //   // }
  //   // q_offset = q_calib_sum / 99.0;
  // }

  // motor_right.evalSensorData();
  // motor_right.runKalmanFilter();

  // // motor_left.evalSensorData();
  // // motor_left.runKalmanFilter();

  // // mpu.getData();
  // // mpu.lowPassFilter();
  // // mpu.complimentaryFilter();

  // q = mpu.data.pitch + 1.3 * 3.1415 / 180.0;
  // dq = mpu.data.dOri(1);

  // t = getStMcs();
  // dt = (float)(t - t_prev) / 1e6;

  // float k_dq = 0.8;
  // // dq = dq * (1.0 - k_dq) + k_dq * (q - q_prev) / dt;
  // // dq = dq * (1.0 - k_dq) + k_dq * (mpu.data.GyX) * 3.1415 / 180.0 / 50.0;
  // // dq = 0;

  // ddq = Kp * (0.0 - q) + Kd * (0.0 - dq);

  // // tau_des = -((0.186 * (0.0198 * ddq + 0.613 * sin(q))) / cos(q) - 0.00212 * ddq * cos(q) + 0.00212 * sin(q) * dq * dq) / (0.371 / cos(q) + 2.0);
  // tau_des = -((0.377 * (0.0109 * ddq + 0.694 * sin(q))) / cos(q) - 0.00241 * ddq * cos(q) + 0.00241 * sin(q) * dq * dq) / (0.754 / cos(q) + 2.0);

  // // static float u_prev = 0;
  // // float f_u = 0.5;
  // // u = u_prev * (1.0 - f_u) + f_u * u;
  // // u_prev = u;

  // if (tau_des > tau_max)
  // {
  //   tau_des = tau_max;
  // }

  // if (tau_des < -tau_max)
  // {
  //   tau_des = -tau_max;
  // }

  // if (u > u_max)
  // {
  //   u = u_max;
  // }

  // if (u < -u_max)
  // {
  //   u = -u_max;
  // }

  // if (abs(q) > 3.1415 / 6)
  // {
  //   u = 0;
  //   tau_des = 0;
  // }

  // t_prev = t;
  // q_prev = q;

  // if (!is_button_pressed)
  // {
  //   return;
  // }

  // UART_printStr("q: ");
  // UART_printDiv(q * 180.0 / 3.1415);
  // UART_printStr(" dq: ");
  // UART_printDiv(dq * 180.0 / 3.1415);
  // // UART_printStr(" GyX: ");
  // // UART_printDiv(mpu.data.GyX + 23);
  // UART_printStr(" tau_des: ");
  // UART_printDiv(tau_des);
  // // UART_printStr(" u_des: ");
  // // UART_printDiv(u);
  // // UART_printStr(" q_offset: ");
  // // UART_printDivLn(q_offset * 180.0 / 3.1415);
  // // UART_printStr(" dt: ");
  // // UART_printDivLn(dt);
  // UART_printStrLn("");

  // testTorqueControl(motor_right, tau_des);
  // testTorqueControl(motor_left, tau_des);
}

void testPIDBalance()
{
  // static float q = 0;
  // static float dq = 0;
  // static float x_act = 0;
  // static float v_act = 0;
  // static float integral_sum = 0;
  // static float max_integral = 0.2;

  // static float is_start = true;

  // static unsigned long t = 0;
  // static unsigned long t_prev = 0;
  // static float dt = 0;
  // static float u = 0;
  // static float u_max = 100;
  // static float q_window = 0.5 * 3.1415 / 180.0;

  // // float Kp = 2.8e2;
  // // float Ki = 5e-1;
  // // float Kd = 3.5e1;

  // // float Kp = 0.92e3;
  // float Kp = 1e3;
  // float Ki = 0;
  // float Kd = 1e1;

  // // float Kp = 2e2;
  // // float Ki = 0;
  // // float Kd = 3;

  // float P = 2;
  // float I = 0;
  // float D = 2e1;

  // float f_q = 1.0;
  // float f_dq = 1.0;

  // if (is_start)
  // {
  //   is_start = false;
  //   t = getStMcs();
  //   t_prev = t;

  //   motor_left.standBy(0);

  //   return;
  // }

  // mpu.getData();
  // mpu.lowPassFilter();
  // mpu.complimentaryFilter();

  // q = mpu.data.pitch + 1.3 * 3.1415 / 180.0;
  // dq = mpu.data.dOri(1);

  // motor_right.evalSensorData();
  // motor_right.runKalmanFilter();
  // x_act = motor_right.getData().q;
  // v_act = motor_right.getData().w;
  // // if (abs(u) > 1e-3)
  // // {
  // //   f_q = 1.0 / abs(u) * 5.0;
  // //   f_dq = 1.0 / abs(u) * 5.0;
  // // }
  // // q = q * (1.0 - f_q) + f_q * (mpu.data.pitch + 1.8 * 3.1415 / 180.0);
  // // dq = dq * (1.0 - f_dq) + f_dq * mpu.data.dOri(1);

  // integral_sum += 0.0 - q;

  // if (integral_sum > max_integral)
  // {
  //   integral_sum = max_integral;
  // }
  // if (integral_sum < -max_integral)
  // {
  //   integral_sum = -max_integral;
  // }

  // u = Kp * (0.0 - q) + Ki * integral_sum + Kd * (0.0 - dq) + P * (0.0 - x_act) + D * (0.0 - v_act);
  // u *= -1.0;
  // u += getSign(u) * 9;

  // if (u > u_max)
  // {
  //   u = u_max;
  // }

  // if (u < -u_max)
  // {
  //   u = -u_max;
  // }

  // if (abs(q) > 3.1415 / 6)
  // {
  //   u = 0;
  // }

  // if (abs(q) < q_window)
  // {
  //   u = 0;
  // }

  // t_prev = t;

  // // UART_printStr("q: ");
  // // UART_printDiv(q * 180.0 / 3.1415);
  // // UART_printStr(" dq: ");
  // // UART_printDiv(dq * 180.0 / 3.1415);
  // // // UART_printStr(" GyX: ");
  // // // UART_printDiv(mpu.data.GyX + 23);
  // // // UART_printStr(" tau_des: ");
  // // // UART_printDiv(tau_des);
  // // UART_printStr(" u_des: ");
  // // UART_printDivLn(u);
  // // // UART_printStr(" q_offset: ");
  // // // UART_printDivLn(q_offset * 180.0 / 3.1415);
  // // // UART_printStr(" dt: ");
  // // // UART_printDivLn(dt);

  // UART_printStr("q: ");
  // UART_printDiv(q * 180.0 / 3.1415);
  // // UART_printStr(", dq: ");
  // // UART_printDiv(dq * 180.0 / 3.1415);
  // // UART_printStr(", u: ");
  // // UART_printDiv(u);
  // UART_printStrLn("\r");

  // // mpu.printRawData();

  // if (!is_button_pressed)
  // {
  //   return;
  // }

  // motor_right.setVoltage(u);
  // motor_left.setVoltage(u);
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

  // UART_printStr("q: ");
  // UART_printDiv(4.4);
  // UART_printStr(" dq: ");
  // UART_printDiv(123.9986);
  // UART_printStr(" ddq: ");
  // UART_printDivLn(0.001);

  motor_left.standBy(1);
  // motor_left.standBy(0);
  // motor_left.setVoltage(60);
  // motor_left.setVoltage(9);

  // motor_right.standBy(0);
  // motor_right.setVoltage(60);

  volatile static unsigned long ms = 0;
  volatile static unsigned long my_ms = 0;

  static unsigned long t = 0;
  static unsigned long t_prev = 0;

  float dt = 0;
  t = getStMcs();
  t_prev = t;

  while (1)
  {
    t = getStMcs();
    dt = (float)(t - t_prev); //mcs
    t_prev = t;
    ms = HAL_GetTick();
    my_ms = getStMcs();

    // mpu.getData();
    // mpu.lowPassFilter();
    // mpu.complimentaryFilter();
    // mpu.plotData();

    // readCurrent();
    // UART_printStr(" I: ");
    // UART_printDivLn(current / 10.0 / 1000.0);

    // motor_left.evalSensorData();
    // motor_right.evalSensorData();
    // UART_printStr("1: ");
    // UART_print(motor_left.getData().ticks);
    // UART_printStr(" 2: ");
    // UART_printLn(motor_right.getData().ticks);

    testButton();
    // motor_data = motor_left.getData();
    w_est = motor_left.getData().ew;
    // motorSinTest(motor_right);
    motorSinTest(motor_left);
    // testBalance();
    // testPIDBalance();
    // testTorqueControl(motor_right, 0.01);
    // testTorqueControl(motor_left);
    // motorStepTest(motor_left);

    // HAL_Delay(8);
    HAL_Delay(6);
    // HAL_Delay(100);

    // UART_printStr("dt: ");
    // UART_printDivLn(dt);

    var++;
  }

  return 0;
}
