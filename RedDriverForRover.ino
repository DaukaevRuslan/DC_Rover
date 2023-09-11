#include "BringSmart.h"
#include "drv_pwm.h"

BringSmartMotor motor1;
BringSmartMotor motor2;
BringSmartMotor motor3;
BringSmartMotor motor4;
void interruptListener1() {
  motor1.interruptListener();
}
void interruptListener2() {
  motor2.interruptListener();
}
void interruptListener3() {
  motor3.interruptListener();
}
void interruptListener4() {
  motor4.interruptListener();
}

volatile long tim = 0;
int ch1 = 0;
int ch2 = 0;
int ch1Prev = 0;
int ch2Prev = 0;
int speed = 0;
unsigned long long timer = 0;
bool flagStop = false;


TIM_HandleTypeDef *pTIM;
TIM_OC_InitTypeDef *pOC;
extern TIM_OC_InitTypeDef hOC1;
extern TIM_OC_InitTypeDef hOC2;
extern TIM_OC_InitTypeDef hOC3;
extern TIM_OC_InitTypeDef hOC4;
uint32_t tim_ch = TIM_CHANNEL_2;
uint32_t uwPeriodValue;
uint32_t uwPrescalerValue = 1;
uint32_t tim_clk;
bool is_timer_16bit = true;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);


  motor1.init(2, 0, 3, 5);
  motor2.init(4, 1, 6, 9);
  motor3.init(7, 14, 10, 11);
  motor4.init(8, 15, 12, A0);



  attachInterrupt(digitalPinToInterrupt(2), interruptListener1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), interruptListener2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), interruptListener3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), interruptListener4, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(3);
  tim = millis();

  pinMode(5, OUTPUT);
  analogWrite(5, 0);
  pinMode(11, OUTPUT);
  analogWrite(11, 0);


  analogWrite(12, 0);

  drv_pwm_set_freq(A0, 300);
  drv_pwm_setup(A0);


  {
    pTIM = g_Pin2PortMapArray[A0].TIMx;
    tim_ch = g_Pin2PortMapArray[A0].timerChannel;
    if (tim_ch == TIM_CHANNEL_1) {
      pOC = &hOC1;
    }
    if (tim_ch == TIM_CHANNEL_2) {
      pOC = &hOC2;
    }
    if (tim_ch == TIM_CHANNEL_3) {
      pOC = &hOC3;
    }

    GPIO_InitTypeDef GPIO_InitStruct;



    uwPeriodValue = (uint32_t)(SystemCoreClock / 600);

    if (is_timer_16bit == true) {
      uwPrescalerValue = (uwPeriodValue / 0xFFFF) + 1;
      uwPeriodValue /= uwPrescalerValue;
    }
    pTIM->Init.Prescaler = uwPrescalerValue - 1;
    pTIM->Init.Period = uwPeriodValue - 1;
    pTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
    pTIM->Init.RepetitionCounter = 0;

    __HAL_RCC_TIM2_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_TIM_PWM_Init(pTIM);

    memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));

    pOC->OCMode = TIM_OCMODE_PWM1;
    pOC->OCPolarity = TIM_OCPOLARITY_HIGH;
    pOC->OCFastMode = TIM_OCFAST_DISABLE;
    pOC->OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
    pOC->OCIdleState = TIM_OCIDLESTATE_RESET;

    pOC->Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
    HAL_TIM_PWM_Start(pTIM, tim_ch);

    //drv_pwm_set_duty(13, 8, 120);
  }

  drv_pwm_set_freq(12, 300);
  drv_pwm_setup(12);

  {
    pTIM = g_Pin2PortMapArray[12].TIMx;
    tim_ch = g_Pin2PortMapArray[12].timerChannel;
    if (tim_ch == TIM_CHANNEL_1) {
      Serial.println("AAAAAA");
      pOC = &hOC1;
    }
    if (tim_ch == TIM_CHANNEL_2) {
      pOC = &hOC2;
    }
    if (tim_ch == TIM_CHANNEL_3) {
      pOC = &hOC3;
    }

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_TIM12_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;


    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uwPeriodValue = (uint32_t)(SystemCoreClock / 600);

    if (is_timer_16bit == true) {
      uwPrescalerValue = (uwPeriodValue / 0xFFFF) + 1;
      uwPeriodValue /= uwPrescalerValue;
    }
    pTIM->Init.Prescaler = uwPrescalerValue - 1;
    pTIM->Init.Period = uwPeriodValue - 1;
    pTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
    pTIM->Init.RepetitionCounter = 0;


    HAL_TIM_PWM_Init(pTIM);
    memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));


    pOC->OCMode = TIM_OCMODE_PWM1;
    pOC->OCPolarity = TIM_OCPOLARITY_HIGH;
    pOC->OCFastMode = TIM_OCFAST_DISABLE;
    pOC->OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
    pOC->OCIdleState = TIM_OCIDLESTATE_RESET;

    pOC->Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
    HAL_TIM_PWM_Start(pTIM, tim_ch);



    //drv_pwm_set_duty(12, 8, 120);
  }
  while (ch2 < 950) {
    ch2 = pulseIn(A4, HIGH, 30000);
  }



  Serial.print("tempVel, count");
}

float diff1 = 0;
int count1 = 0;
float tempVel1 = 0;
float speed1 = 0;

float diff2 = 0;
int count2 = 0;
float tempVel2 = 0;
float speed2 = 0;


void loop() {
  motor1.tick();
  motor2.tick();
  motor3.tick();
  motor4.tick();
  ch1 = pulseIn(A3, HIGH, 30000);
  ch2 = pulseIn(A4, HIGH, 30000);
  ch2 = 1500 - ch2;
  ch2 += 1500;

  if (ch2 < 2000) {
    if (millis() - tim > 50) {
      tim = millis();
      flagStop = false;

      if (ch1 < 1505 && ch1 > 1495) ch1 = 1500;
      if (ch2 < 1505 && ch2 > 1495) ch2 = 1500;

      if (abs(ch1Prev - ch1) > 5) {
        count1 = 0;
        speed1 = mapfloat(ch1, 1050, 1950, -35, 35);
        diff1 = (speed1 - motor1.GoalRadianVelocity) / 10;

      }
      if (count1 < 10) {
        count1++;
        tempVel1 += diff1;
      }
      if (count1 == 10) {
        tempVel1 = speed1;
      }

      motor1.setGoalVelocity(tempVel1);
      motor2.setGoalVelocity(tempVel1);

      if (abs(ch2Prev - ch2) > 5) {
        count2 = 0;
        speed2 = mapfloat(ch2, 1050, 1950, -35, 35);
        diff2 = (speed2 - motor3.GoalRadianVelocity) / 10;
      }
      if (count2 < 10) {
        count2++;
        tempVel2 += diff2;
      }
      if (count2 == 10) {
        tempVel2 = speed2;
      }

      motor3.setGoalVelocity(tempVel2);
      motor4.setGoalVelocity(tempVel2);


      ch1Prev = ch1;
      ch2Prev = ch2;

      Serial.print(motor1.GoalRadianVelocity);
      Serial.print(" , ");
      Serial.print(motor1.HardPosition);
      Serial.print(" , ");
      Serial.print(motor2.GoalRadianVelocity);
      Serial.print(" , ");
      Serial.print(motor2.HardPosition);
      Serial.print(" , ");
      Serial.println(ch2);

    }
  } else {
    if (!flagStop) {
      flagStop = true;

      tempVel1 = motor1.GoalRadianVelocity;
      tempVel2 = motor3.GoalRadianVelocity;
      diff1 = tempVel1 / 10;
      diff2 = tempVel2 / 10;


      for (int i = 0; i < 10; i++) {
        Serial.println(tempVel1);
        motor1.tick();
        motor2.tick();
        motor3.tick();
        motor4.tick();
        motor1.setGoalVelocity(tempVel1);
        motor2.setGoalVelocity(tempVel1);
        motor3.setGoalVelocity(tempVel2);
        motor4.setGoalVelocity(tempVel2);

        tempVel1 -= diff1;
        tempVel2 -= diff2;

        delay(50);
      }
      speed1 = 0;
      speed2 = 0;
      tempVel1 = 0;
      tempVel2 = 0;
      diff1 = 0;
      diff2 = 0;
      count1 = 0;
      count2 = 0;

      motor1.setGoalVelocity(0);
      motor2.setGoalVelocity(0);
      motor3.setGoalVelocity(0);
      motor4.setGoalVelocity(0);
    }
  }
}
