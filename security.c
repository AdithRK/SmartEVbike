#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "m95640_driver 2.h"
#include "lcd.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_SYSTEM_LOCKED,
    STATE_WAITING_FOR_USER_ID,
    STATE_WAITING_FOR_SECURITY_CODE,
    STATE_ACCESS_GRANTED,
    STATE_ACCESS_DENIED,
    STATE_MENU_DISPLAY,               // New State
    STATE_PASSWORD_CHANGE_NEW_CODE,   // New State
    STATE_SYSTEM_ENTERED              // New State
} SystemState_t;

/* --- Forward declarations --- */

/* Queue handle for IR codes */
QueueHandle_t irInputQueue;
TaskHandle_t  securityTaskHandle = NULL;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASSWORD_MAX_LEN 4
#define EEPROM_INIT_FLAG_ADDR 0x0000
#define EEPROM_INIT_FLAG_VAL  0xAA
#define USER1_PASS_ADDR (EEPROM_INIT_FLAG_ADDR + 1)
#define USER2_PASS_ADDR (USER1_PASS_ADDR + PASSWORD_MAX_LEN)
#define USER3_PASS_ADDR (USER2_PASS_ADDR + PASSWORD_MAX_LEN)
#define USER4_PASS_ADDR (USER3_PASS_ADDR + PASSWORD_MAX_LEN)
#define USER5_PASS_ADDR (USER4_PASS_ADDR + PASSWORD_MAX_LEN)

// --- IR Remote Codes (Fill these with your values) ---
#define IR_POWER  0x01FE48B7
#define IR_STAR   0x01FE58A7
#define IR_HASH   0x01FE7887
#define IR_0      0x01FEE01F
#define IR_1      0x01FE50AF
#define IR_2      0x01FED827
#define IR_3      0x01FEF807
#define IR_4      0x01FE30CF
#define IR_5      0x01FEB04F
#define IR_6      0x01FE708F
#define IR_7      0x01FE00FF
#define IR_8      0x01FEF00F
#define IR_9      0x01FE9867
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void vSecurityTask(void *argument);
void vIRTask(void *argument);
char translate_ir_code(uint32_t code);
void Initialize_EEPROM_Passwords(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  if (SpiEepromInit_HAL() != HAL_OK)
    {
        Error_Handler(); // Freeze if SPI setup fails
    }
  /* USER CODE BEGIN 2 */
  HX1838_Init(&htim2, GPIO_PIN_5);   // only pin, no port
  Initialize_EEPROM_Passwords();
  irInputQueue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(vSecurityTask, "SecurityTask", 512, NULL, 2, NULL);
    xTaskCreate(vIRTask, "IRTask", 128, NULL, 1, NULL);


  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void Initialize_EEPROM_Passwords(void)
{
    uint8_t flag = EepromReadByte_HAL(EEPROM_INIT_FLAG_ADDR);

    if (flag != EEPROM_INIT_FLAG_VAL)
    {
        LcdInit();
        lprint(0x80, "First Run...");
        lprint(0xC0, "Writing Pass...");

        char *passwords[] = {"1234", "2345", "3456", "1111", "2222"};
        uint16_t addresses[] = {USER1_PASS_ADDR, USER2_PASS_ADDR, USER3_PASS_ADDR, USER4_PASS_ADDR, USER5_PASS_ADDR};

        for (int i = 0; i < 5; i++) {
            EepromWriteBuffer_HAL(addresses[i], (uint8_t*)passwords[i], PASSWORD_MAX_LEN);
        }

        flag = EEPROM_INIT_FLAG_VAL;
        EepromWriteByte_HAL(EEPROM_INIT_FLAG_ADDR, flag);
        HAL_Delay(1000);
    }
}

char translate_ir_code(uint32_t code)
{
    switch (code) {
        case IR_POWER: return 'P'; case IR_0: return '0'; case IR_1: return '1';
        case IR_2: return '2'; case IR_3: return '3'; case IR_4: return '4';
        case IR_5: return '5'; case IR_6: return '6'; case IR_7: return '7';
        case IR_8: return '8'; case IR_9: return '9'; case IR_STAR: return '*';
        case IR_HASH: return '#'; default: return '\0';
    }
}

void vSecurityTask(void *argument)
{
    SystemState_t currentState = STATE_SYSTEM_LOCKED;
    uint32_t received_code;
    char inputChar;
    int selectedUserID = 0;
    char securityCodeActual[PASSWORD_MAX_LEN + 1] = {0};
    char storedPassword[PASSWORD_MAX_LEN + 1] = {0};
    int codeIndex = 0;

    LcdInit();

    for(;;)
    {
        switch (currentState)
        {
            case STATE_SYSTEM_LOCKED:
            	lprint(0x01, "             ");
                lprint(0x80, "Smart EV-Bike");
                lprint(0xC0, "System Locked");
                selectedUserID = 0; // Reset user ID on lock
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                if (translate_ir_code(received_code) == 'P') {
                    currentState = STATE_WAITING_FOR_USER_ID;
                }
                break;

            case STATE_WAITING_FOR_USER_ID:
            	lprint(0x01, "             ");
                lprint(0x80, "Enter User ID");
                lprint(0xC0, "(1-5): #");
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                inputChar = translate_ir_code(received_code);

                if (inputChar >= '1' && inputChar <= '5') {
                    selectedUserID = inputChar - '0';
                    lprint(0xC6, &inputChar);
                } else if (inputChar == '#') {
                    if (selectedUserID != 0) {
                        uint16_t addr = USER1_PASS_ADDR + ((selectedUserID - 1) * PASSWORD_MAX_LEN);
                        EepromReadBuffer_HAL(addr, (uint8_t*)storedPassword, PASSWORD_MAX_LEN);
                        storedPassword[PASSWORD_MAX_LEN] = '\0';
                        currentState = STATE_WAITING_FOR_SECURITY_CODE;
                        memset(securityCodeActual, 0, sizeof(securityCodeActual));
                        codeIndex = 0;
                    }
                } else if (inputChar == '*') {
                    currentState = STATE_SYSTEM_LOCKED;
                }
                break;

            case STATE_WAITING_FOR_SECURITY_CODE:
            	lprint(0x01, "             ");
                lprint(0x80, "Security Code:");
                char stars[5] = {0};
                memset(stars, '*', codeIndex);
                lprint(0xC0, stars);
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                inputChar = translate_ir_code(received_code);

                if (inputChar >= '0' && inputChar <= '9' && codeIndex < PASSWORD_MAX_LEN) {
                    securityCodeActual[codeIndex++] = inputChar;
                } else if (inputChar == '#' && codeIndex == PASSWORD_MAX_LEN) {
                    if (strcmp(securityCodeActual, storedPassword) == 0) {
                        currentState = STATE_ACCESS_GRANTED;
                    } else {
                        currentState = STATE_ACCESS_DENIED;
                    }
                } else if (inputChar == '*') {
                    currentState = STATE_WAITING_FOR_USER_ID;
                }
                break;

            case STATE_ACCESS_GRANTED:
            	lprint(0x01, "             ");
                lprint(0x80, "Access Granted");
                vTaskDelay(pdMS_TO_TICKS(1000));
                // *** CHANGED: Go to menu instead of locking ***
                currentState = STATE_MENU_DISPLAY;
                break;

            case STATE_ACCESS_DENIED:
            	lprint(0x01, "             ");
                lprint(0x80, "Access Denied");
                lprint(0xC0, "Try Again!");
                vTaskDelay(pdMS_TO_TICKS(2000));
                currentState = STATE_WAITING_FOR_USER_ID;
                break;

            // ---- NEW STATES ADDED BELOW ----

            case STATE_MENU_DISPLAY:
            	lprint(0x01, "             ");
                lprint(0x80, "1:Chg Pwd 2:Enter");
                lprint(0xC0, "* to Logout");
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                inputChar = translate_ir_code(received_code);

                if (inputChar == '1') {
                    currentState = STATE_PASSWORD_CHANGE_NEW_CODE;
                    memset(securityCodeActual, 0, sizeof(securityCodeActual));
                    codeIndex = 0;
                } else if (inputChar == '2') {
                    currentState = STATE_SYSTEM_ENTERED;
                } else if (inputChar == '*') {
                    currentState = STATE_SYSTEM_LOCKED;
                }
                break;

            case STATE_PASSWORD_CHANGE_NEW_CODE:
            	lprint(0x01, "             ");
                lprint(0x80, "New Security Code:");
                char new_stars[5] = {0};
                memset(new_stars, '*', codeIndex);
                lprint(0xC0, new_stars);
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                inputChar = translate_ir_code(received_code);

                if (inputChar >= '0' && inputChar <= '9' && codeIndex < PASSWORD_MAX_LEN) {
                    securityCodeActual[codeIndex++] = inputChar;
                } else if (inputChar == '#' && codeIndex == PASSWORD_MAX_LEN) {
                    // Save the new password to EEPROM
                    uint16_t addr = USER1_PASS_ADDR + ((selectedUserID - 1) * PASSWORD_MAX_LEN);
                    EepromWriteBuffer_HAL(addr, (uint8_t*)securityCodeActual, PASSWORD_MAX_LEN);

                    lprint(0x01, "             ");
                    lprint(0x80, "Password Changed!");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    currentState = STATE_WAITING_FOR_USER_ID; // Return to User ID selection
                } else if (inputChar == '*') { // Allow cancel
                    currentState = STATE_MENU_DISPLAY;
                }
                break;

            case STATE_SYSTEM_ENTERED:
                lprint(0x01, "             ");
                lprint(0x80, "System Entered");
                lprint(0xC0, "* to Logout");
                xQueueReceive(irInputQueue, &received_code, portMAX_DELAY);
                inputChar = translate_ir_code(received_code);
                if (inputChar == '*') {
                    currentState = STATE_SYSTEM_LOCKED;
                }
                break;
        }
    }
}

void vIRTask(void *argument)
{
    uint32_t received_code;
    for(;;)
    {
        if (HX1838_GetCode(&received_code, portMAX_DELAY) == true)
        {
            xQueueSend(irInputQueue, &received_code, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
void vTempMonitorTask(void *argument)
{
    for(;;)
    {
        // Check if the temperature sensor pin is HIGH
        if (HAL_GPIO_ReadPin(TEMP_SENSOR_D0_GPIO_Port, TEMP_SENSOR_D0_Pin) == GPIO_PIN_SET)
        {
            // Temperature has crossed the threshold.
            // Send a notification to the main security task.
            if (securityTaskHandle != NULL)
            {
                // We'll use '1' as the notification value for over-temperature.
                xTaskNotify(securityTaskHandle, 1, eSetValueWithOverwrite);
            }

            // Wait for a while before checking again to prevent spamming notifications.
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        // Check the sensor every 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */