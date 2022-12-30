#include <Arduino.h>
#include <String.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"



#define SERIAL_BAUDRATE 115200
#define TIME_SEARCH_BLE 1
#define LED_BLUE GPIO_NUM_2
#define V_OUT GPIO_NUM_13

void Task_Undershoot(void *params);

ledc_channel_config_t channel_config;
ledc_timer_config_t timer_config ;

void setup() 
{

  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("");
  Serial.println("Supply Undershoot Simulator");
  Serial.print("By: HIDROXID0");
  // -- Configuração dos pinos --
  //gpio_pad_select_gpio(GPIO_NUM_13);
  gpio_pad_select_gpio(GPIO_NUM_2);

  // -- Direção dos pinos --
  gpio_set_direction(V_OUT, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
  // -- Internals Resistors Pull-Up --
  gpio_pullup_dis(V_OUT);
  gpio_pullup_dis(V_OUT);
  // -- Internals Resistors Pull-Down ---
  gpio_pulldown_dis(V_OUT);


// -- Config Timer para PWM --
  timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_config.duty_resolution = LEDC_TIMER_8_BIT;
  timer_config.timer_num = LEDC_TIMER_0;
  timer_config.freq_hz = 1000;
  ledc_timer_config(&timer_config);

// -- Config channel do PWM --
  channel_config.gpio_num = GPIO_NUM_13;
  channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
  channel_config.channel = LEDC_CHANNEL_0;
  channel_config .timer_sel = LEDC_TIMER_0;
  channel_config.duty = 0;
  channel_config.hpoint = 0;
  ledc_channel_config(&channel_config);


  //xTaskCreate(&Task_, "Descrição/nome da Task", Tamanho da Stack Memory 1024 = 4K de memória,Parametro da Função, Prioridade de 0 até uxPriority ,Task Random );
  //xTaskCreatePinnedToCore(&Task_, "Descrição/nome da Task", Tamanho da Stack Memory 1024 = 4K de memória,Parametro da Função, Prioridade de 0 até uxPriority ,Task Random, Núcleo de processamento)
  xTaskCreatePinnedToCore(&Task_Undershoot, "Simulate Undershoot Supply", 2048, NULL, 1, NULL,1);

  ledc_fade_func_install(0);

}


void loop() 
{
  
  
  gpio_set_level(LED_BLUE, 1);
  
  
  Serial.println("loop teste");
  vTaskDelay( 1000 / portTICK_PERIOD_MS); // Função de Delay do FreeRTOS, em tempo real. 1000ms = 1s


}

void Task_Undershoot(void * params)
{
  while(true)
  {
    Serial.println("UnderShoot");
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,130,2000, LEDC_FADE_WAIT_DONE);
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0,2000, LEDC_FADE_WAIT_DONE);
    vTaskDelay( 10000 / portTICK_PERIOD_MS); // Função de Delay do FreeRTOS, em tempo real. 1000ms = 1s

  }
}

/*



 gpio_get_level() [leitura de pinos definidos como input]


  for (int i = 0; i < 255 ; i ++)
  {
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,i);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  for (int i = 255; i < 0 ; i --)
  {
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,i);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  OR

  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0,1000, LEDC_FADE_WAIT_DONE);
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,255,1000, LEDC_FADE_WAIT_DONE);
*/

