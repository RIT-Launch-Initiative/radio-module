/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "sched/macros.h"
#include "init/init.h"
#include "common/utils/nmea.h"

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
#include "device/peripherals/LED/LED.h"

#include "device/peripherals/wiznet/wiznet.h"
#include "device/peripherals/MAXM10S/MAXM10S.h"
#include "device/peripherals/RFM9XW/RFM9XW.h"

#include "net/packet/Packet.h"
#include "net/stack/IPv4UDP/IPv4UDPStack.h"
#include "net/stack/IPv4UDP/IPv4UDPSocket.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static HALUARTDevice uart("UART", &huart4);
static Packet packet = alloc::Packet<
        IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();
static Packet rf_packet = alloc::Packet<
        IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();


static HALSPIDevice wiz_spi("Wiznet SPI", &hspi1);
static HALGPIODevice wiz_cs("Wiznet CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
static HALGPIODevice wiz_reset("Wiznet Reset", ETH_RST_GPIO_Port, ETH_RST_Pin);
static HALGPIODevice wiz_led_gpio("Wiznet LED", ETH_ACTLED_GPIO_Port, ETH_ACTLED_Pin); // No actual LED on the board atm
static Wiznet w5500(wiz_spi, wiz_cs, wiz_reset, wiz_led_gpio, packet);

static IPv4UDPStack network_stack(10, 10, 10, 3, 255, 255, 255, 0, w5500);
static IPv4UDPSocket *stack_socket = network_stack.get_socket();

static HALSPIDevice rfm_spi("RFM SPI", &hspi2);
static HALGPIODevice rfm_cs("RFM CS", RF_CS_GPIO_Port, RF_CS_Pin);
static HALGPIODevice rfm_reset("RFM Reset", RF_RST_GPIO_Port, RF_RST_Pin);
static HALGPIODevice rfm_dio_zero("RFM DIO0", RF_DIO0_GPIO_Port, RF_DIO0_Pin);
static HALGPIODevice rfm_dio_one("RFM DIO1", RF_DIO1_GPIO_Port, RF_DIO1_Pin);
static HALGPIODevice rfm_dio_two("RFM DIO2", RF_DIO2_GPIO_Port, RF_DIO2_Pin);
static HALGPIODevice rfm_dio_three("RFM DIO3", RF_DIO3_GPIO_Port, RF_DIO3_Pin);
static HALGPIODevice rfm_dio_four("RFM DIO4", RF_DIO4_GPIO_Port, RF_DIO4_Pin);
static HALGPIODevice rfm_dio_five("RFM DIO5", RF_DIO5_GPIO_Port, RF_DIO5_Pin);
static RFM9XW rfm9xw(rfm_spi, rfm_cs, rfm_reset, rfm_dio_zero, rfm_dio_one, rfm_dio_two, rfm_dio_three, rfm_dio_four,
                     rfm_dio_five);

static HALI2CDevice maxm10s_i2c("MAXM10S I2C", &hi2c1);
static HALUARTDevice maxm10s_uart("MAXM10S UART", &huart2);
static HALGPIODevice maxm10s_reset("MAXM10S RESET", GPS_RST_GPIO_Port, GPS_RST_Pin);
static HALGPIODevice maxm10s_interrupt("MAXM10S INTERRUPT", GPS_INT_GPIO_Port, GPS_INT_Pin);
static MAXM10S maxm10s(maxm10s_i2c, maxm10s_uart, maxm10s_reset, maxm10s_interrupt);

static HALGPIODevice led_one_gpio("LED1", LED1_GPIO_Port, LED1_Pin);
static LED led_one(led_one_gpio);

static HALGPIODevice led_two_gpio("LED2", LED2_GPIO_Port, LED2_Pin);
static LED led_two(led_two_gpio);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_SPI1_Init(void);

static void MX_UART4_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

RetType pollTask(void *) {
    RESUME();
    CALL(wiz_spi.poll());
    CALL(rfm_spi.poll());
    CALL(maxm10s_i2c.poll());
    CALL(uart.poll());
    RESET();
    return RET_SUCCESS;
}

RetType pollWiznet(void *) {
    RESUME();
    CALL(w5500.poll());
    RESET();
    return RET_SUCCESS;
}

typedef struct {
    uint8_t id;
    uint64_t count;
    uint32_t timestamp;
} test_struct;

RetType wizRcvTestTask(void *) {
    RESUME();
    static uint8_t buff[1000];
    static test_struct *test = (test_struct *) buff;
    static size_t len;

    static IPv4UDPSocket::addr_t addr;
    addr.ip[0] = 239;
    addr.ip[1] = 255;
    addr.ip[2] = 255;
    addr.ip[3] = 255;
    addr.port = 8000;

    RetType ret = CALL(stack_socket->recv(buff, &len, &addr));
    if (RET_SUCCESS == ret) {
        snprintf((char *) buff, 1000, "Tick %lu: %llu\r\n", test->timestamp, test->count);
        CALL(uart.write(buff, strlen((char *) buff)));
    }

    RESET();
    return RET_SUCCESS;
}

RetType wizTransTestTask(void *) {
    RESUME();

    static IPv4UDPSocket::addr_t addr;
    addr.ip[0] = 10;
    addr.ip[1] = 10;
    addr.ip[2] = 10;
    addr.ip[3] = 10;
    addr.port = 8000;

    static uint8_t buff[] = "Launch!";
    RetType ret = CALL(stack_socket->send(buff, 7, &addr));

    RESET();
    return RET_SUCCESS;
}

RetType maxm10sTask(void *) {
    RESUME();

    static uint8_t data[1000];
    static char *messages;
    static size_t bytes_read = 0;
    static GPSData gps_data;
    static uint8_t uart_buff[1000];

    CALL(led_one.set_state(LED_ON));

    RetType ret = CALL(maxm10s.read_data_rand_access(data, 1000, &bytes_read));
    if (RET_SUCCESS == ret) {
        messages = strtok(reinterpret_cast<char *>(data), "\r\n");
        for (char *message = messages; message != nullptr; message = strtok(nullptr, "\r\n")) {
            if (strstr(message, "GGA") != nullptr) {
                size_t len = strlen(message);
                nmea::parse_gga(message, &gps_data, len);
                // TODO: Any processing here

                static size_t len2 = snprintf((char *) uart_buff, 1000, "GPS Data:\r\n"
                                                                        "\tLatitude: %f\r\n"
                                                                        "\tLongitude: %f\r\n"
                                                                        "\tAltitude: %f\r\n"
                                                                        "\tSatellites: %d\r\n"
                                                                        "\tFix: %d\r\n"
                                                                        "\tSeconds since midnight: %f\r\n",
                                              gps_data.latitude, gps_data.longitude, gps_data.alt, gps_data.num_sats,
                                              gps_data.quality, gps_data.time);
                RetType ret = CALL(uart.write(uart_buff, len2));
                if (RET_SUCCESS != ret) {
                    RESET();
                    return RET_ERROR;
                }


                break;
            }
        }

    }

    RESET();
    return RET_SUCCESS;
}

RetType rfmTxTask(void *) {
    RESUME();

    static uint8_t data[] = "Launch!";
    RetType ret = CALL(rfm9xw.transmit_data(data, 7));
    if (RET_SUCCESS == ret) {
        CALL(led_two.set_state(LED_ON));
        CALL(uart.write(data, 7));
    } else {
        CALL(led_two.set_state(LED_OFF));
        CALL(uart.write((uint8_t *) "Send error!\r\n", 13));
    }

    RESET();
    return RET_SUCCESS;
}

RetType rfmRxTask(void *) {
    RESUME();

    static uint8_t data[100] = {0};
    static uint8_t len = 0;
    RetType ret = CALL(rfm9xw.continuous_rx(data, 100, &len));
    if (RET_SUCCESS == ret) {
        CALL(led_two.toggle());
        CALL(uart.write(data, len));
    }

    RESET();
    return RET_SUCCESS;
}

RetType device_init() {
    RESUME();

    uart.init();
    led_one_gpio.init();
    led_one.init();
    led_two_gpio.init();
    led_two.init();

    wiz_spi.init();
    wiz_cs.init();
    wiz_reset.init();
    wiz_led_gpio.init();

    maxm10s_i2c.init();
    maxm10s_uart.init();
    maxm10s_reset.init();
    maxm10s_interrupt.init();

    rfm_spi.init();
    rfm_cs.init();
    rfm_reset.init();

    RetType ret = CALL(maxm10s.init());
    if (RET_SUCCESS == ret) {
        CALL(uart.write((uint8_t *) "MAX-M10S Initialized\r\n", 22));
        sched_start(maxm10sTask, {});
    }

    ret = CALL(rfm9xw.init());
    if (RET_SUCCESS == ret) {
        CALL(uart.write((uint8_t *) "RFM95W Initialized\r\n", 20));
#ifdef RECEIVER
        ret = CALL(rfm9xw.enable_continuous_rx());
        if (RET_SUCCESS == ret) {
            CALL(uart.write((uint8_t *) "RFM95W Continuous RX Enabled\r\n", 30));
        }
        sched_start(rfmRxTask, {});
#else
        sched_start(rfmTxTask, {});
#endif
    }

    RESET();
    return RET_SUCCESS;
}


RetType network_init() {
    RESUME();

    static uint8_t ip_addr[4] = {10, 10, 10, 3};
    static uint8_t subnet_mask[4] = {255, 255, 255, 0};
    static uint8_t gateway_addr[4] = {192, 168, 1, 1};
    static uint8_t mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static IPv4UDPSocket::addr_t addr;

    w5500.set_upper(&network_stack.get_eth());

    addr.ip[0] = addr.ip[1] = addr.ip[2] = addr.ip[3] = 0;
    addr.port = 8000;
    stack_socket->bind(addr); // TODO: Error handling

    ipv4::IPv4Addr_t temp_addr;
    ipv4::IPv4Address(239, 255, 255, 255, &temp_addr);
    network_stack.add_multicast(temp_addr);

    CALL(uart.write((uint8_t *) "W5500: Initializing\r\n", 23));
    RetType ret = CALL(w5500.init());
    if (ret != RET_SUCCESS) {
        CALL(uart.write((uint8_t *) "W5500: Failed to initialize\r\n", 29));
        goto netStackInitDone;
    }

    if (RET_SUCCESS != network_stack.init()) {
        CALL(uart.write((uint8_t *) "Failed to initialize network stack\r\n", 35));
        goto netStackInitDone;
    }

    CALL(uart.write((uint8_t *) "Successfully initialized network interface\n\r", 44));

    sched_start(wizRcvTestTask, {});
    sched_start(pollWiznet, {});

    netStackInitDone:
RESET();
    return RET_ERROR; // Kill task
}


RetType init_task(void *) {
    RESUME();

    CALL(device_init());
    CALL(network_init());

    RESET();
    return RET_ERROR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    constexpr auto launch_name_text = "\t ________  ___  _________        ___       ________  ___  ___  ________   ________  ___  ___\r\n"
                                      "\t|\\   __  \\|\\  \\|\\___   ___\\     |\\  \\     |\\   __  \\|\\  \\|\\  \\|\\   ___  \\|\\   ____\\|\\  \\|\\  \\\r\n"
                                      "\t\\ \\  \\|\\  \\ \\  \\|___ \\  \\_|     \\ \\  \\    \\ \\  \\|\\  \\ \\  \\\\\\  \\ \\  \\\\ \\  \\ \\  \\___|\\ \\  \\\\\\  \\\r\n"
                                      "\t \\ \\   _  _\\ \\  \\   \\ \\  \\       \\ \\  \\    \\ \\   __  \\ \\  \\\\\\  \\ \\  \\\\ \\  \\ \\  \\    \\ \\   __  \\\r\n"
                                      "\t  \\ \\  \\\\  \\\\ \\  \\   \\ \\  \\       \\ \\  \\____\\ \\  \\ \\  \\ \\  \\\\\\  \\ \\  \\\\ \\  \\ \\  \\____\\ \\  \\ \\  \\\r\n"
                                      "\t   \\ \\__\\\\ _\\\\ \\__\\   \\ \\__\\       \\ \\_______\\ \\__\\ \\__\\ \\_______\\ \\__\\\\ \\__\\ \\_______\\ \\__\\ \\__\\\r\n"
                                      "\t    \\|__|\\|__|\\|__|    \\|__|        \\|_______|\\|__|\\|__|\\|_______|\\|__| \\|__|\\|_______|\\|__|\\|__|\r\n";

    constexpr int launch_name_len = []() constexpr {
        const char *ptr = launch_name_text;
        while (*ptr) ++ptr;
        return ptr - launch_name_text;
    }();

    constexpr auto radio_module_text = " ________  ________  ________  ___  ________          _____ ______   ________  ________  ___  ___  ___       _______\r\n"
                                       "|\\   __  \\|\\   __  \\|\\   ___ \\|\\  \\|\\   __  \\        |\\   _ \\  _   \\|\\   __  \\|\\   ___ \\|\\  \\|\\  \\|\\  \\     |\\  ___ \\\r\n"
                                       "\\ \\  \\|\\  \\ \\  \\|\\  \\ \\  \\_|\\ \\ \\  \\ \\  \\|\\  \\       \\ \\  \\\\\\__\\ \\  \\ \\  \\|\\  \\ \\  \\_|\\ \\ \\  \\\\\\  \\ \\  \\    \\ \\   __/|\r\n"
                                       " \\ \\   _  _\\ \\   __  \\ \\  \\ \\\\ \\ \\  \\ \\  \\\\\\  \\       \\ \\  \\\\|__| \\  \\ \\  \\\\\\  \\ \\  \\ \\\\ \\ \\  \\\\\\  \\ \\  \\    \\ \\  \\_|/__\r\n"
                                       "  \\ \\  \\\\  \\\\ \\  \\ \\  \\ \\  \\_\\\\ \\ \\  \\ \\  \\\\\\  \\       \\ \\  \\    \\ \\  \\ \\  \\\\\\  \\ \\  \\_\\\\ \\ \\  \\\\\\  \\ \\  \\____\\ \\  \\_|\\ \\\r\n"
                                       "   \\ \\__\\\\ _\\\\ \\__\\ \\__\\ \\_______\\ \\__\\ \\_______\\       \\ \\__\\    \\ \\__\\ \\_______\\ \\_______\\ \\_______\\ \\_______\\ \\_______\\\r\n"
                                       "    \\|__|\\|__|\\|__|\\|__|\\|_______|\\|__|\\|_______|        \\|__|     \\|__|\\|_______|\\|_______|\\|_______|\\|_______|\\|_______|\r\n";

    constexpr int radio_module_len = []() constexpr {
        const char *ptr = radio_module_text;
        while (*ptr) ++ptr;
        return ptr - radio_module_text;
    }();

    constexpr auto line_text = " ____________  ____________  ____________  ____________  ____________  ____________  ____________  ____________  ____________\r\n"
                               "|\\____________\\\\____________\\\\____________\\\\____________\\\\____________\\\\____________\\\\____________\\\\____________\\\\____________\\\r\n"
                               "\\|____________\\|____________\\|____________\\|____________\\|____________\\|____________\\|____________\\|____________\\|____________|\r\n";

    constexpr int line_text_len = []() constexpr {
        const char *ptr = line_text;
        while (*ptr) ++ptr;
        return ptr - line_text;
    }();


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
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_UART4_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Transmit(&huart4, (uint8_t *) launch_name_text, launch_name_len, 1000);
    HAL_UART_Transmit(&huart4, (uint8_t *) radio_module_text, radio_module_len, 1000);
    HAL_UART_Transmit(&huart4, (uint8_t *) line_text, line_text_len, 1000);

    if (!sched_init(&HAL_GetTick)) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *) "Failed to init scheduler\n\r", 30);
        return -1;
    }

    sched_start(pollTask, {});
    sched_start(init_task, {});

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        sched_dispatch();
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void) {

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void) {

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPS_RST_Pin | MEM_RST_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, ETH_SPDLED_Pin | ADDR0_Pin | ADDR1_Pin | ADDR2_Pin
                             | ETH_LINKLED_Pin | ETH_DUPLED_Pin | LED1_Pin | MEM_CS_Pin
                             | MEM_WP_Pin | RF_DIO0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, ADDR3_Pin | ETH_CS_Pin | RF_CS_Pin | RF_DIO5_Pin
                             | RF_DIO4_Pin | RF_DIO3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, ETH_ACTLED_Pin | LED2_Pin | RF_DIO2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RF_DIO1_GPIO_Port, RF_DIO1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : GPS_RST_Pin ETH_SPDLED_Pin ADDR0_Pin ADDR1_Pin
                             ADDR2_Pin ETH_LINKLED_Pin ETH_DUPLED_Pin LED1_Pin
                             MEM_RST_Pin MEM_CS_Pin MEM_WP_Pin RF_DIO0_Pin */
    GPIO_InitStruct.Pin = GPS_RST_Pin | ETH_SPDLED_Pin | ADDR0_Pin | ADDR1_Pin
                          | ADDR2_Pin | ETH_LINKLED_Pin | ETH_DUPLED_Pin | LED1_Pin
                          | MEM_RST_Pin | MEM_CS_Pin | MEM_WP_Pin | RF_DIO0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ADDR3_Pin ETH_CS_Pin RF_CS_Pin RF_RST_Pin
                             RF_DIO5_Pin RF_DIO4_Pin RF_DIO3_Pin */
    GPIO_InitStruct.Pin = ADDR3_Pin | ETH_CS_Pin | RF_CS_Pin | RF_RST_Pin
                          | RF_DIO5_Pin | RF_DIO4_Pin | RF_DIO3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : ETH_RST_Pin ETH_ACTLED_Pin LED2_Pin RF_DIO2_Pin */
    GPIO_InitStruct.Pin = ETH_RST_Pin | ETH_ACTLED_Pin | LED2_Pin | RF_DIO2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : ETH_INT_Pin GPS_INT_Pin */
    GPIO_InitStruct.Pin = ETH_INT_Pin | GPS_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : RF_DIO1_Pin */
    GPIO_InitStruct.Pin = RF_DIO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RF_DIO1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : GPS_HBEAT_Pin */
    GPIO_InitStruct.Pin = GPS_HBEAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPS_HBEAT_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
