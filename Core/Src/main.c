/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CANIDの定義
#define CAN_ID_FEEDBACK 0x103 // グローバル座標と回転角の送信

#define M_PI 3.14159265358979323846 //math.hから除外されたとき用
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAN_ID_MACRO_kaetekudasai 0x7FE // example CAN ID macro for testing
#define R 30.0//mm  //radious of wheel
#define CONV M_PI / 180.0f // 度/秒 を ラジアン/秒 に変換する時に掛ける
#define PPR 2000.0 //pulses per revolution

// 機体の機構パラメータ (実測値をmm単位等で設定)
const float L = 150.0f; // ロボットの中心線（真ん中の縦軸）から、縦向きホイールがどれだけズレているか

#define AVG_WINDOW_SIZE 25
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;

//IMU制御の変数
typedef struct {
    int16_t gx, gy, gz; //gyro (x,y,z)
    int16_t ax, ay, az; //accel(x,y,z)
} IMUData;

uint8_t whoami, whoami2, whoami3; // IMUセンサが正しく返答するかを確認するためのデバッグ用ID保持変数
volatile IMUData imu[3];
volatile float gyro_x, gyro_y, gyro_z;
volatile float accel_x, accel_y, accel_z;

// ジャイロセンサの静止時オフセット（ゼロ点バイアス誤差）を保持
float gyro_x_bias = 0.0f;
float gyro_y_bias = 0.0f;
float gyro_z_bias = 0.0f;

const float G_sensitivity = 0.070f; // 生データをdpsに変換
const float A_sensitivity = 0.488f; // 生データをミリgに変換
const float GYRO_Z_SCALE = 0.9965f; //要変更 // 実測誤差（ゲイン誤差）を微修正するための補正係数

float cos_30, sin_30;
bool isSettingBias = 1; // IMUのゼロ点バイアス（静止時ノイズ）を測定中かどうかを表すフラグ
bool isSettingWheel = 1; // オドメティ（計測輪）の位置・角度情報を初期化（リセット）中かどうかを表すフラグ
float beta = 0.1f; //Madgwickフィルタのゲイン //IMUのデータからクォータニオン（姿勢）を計算するMadgwick（マドウィック）フィルタにおける「加速度センサによる補正の強さを決めるハイパーパラメータ
float roll, pitch, yaw;
GPIO_TypeDef* const IMU_CS_PORTS[3] = {IMU1_CS_GPIO_Port, IMU2_CS_GPIO_Port, IMU3_CS_GPIO_Port}; // CSピンが属している GPIOポートを格納した配列
const uint16_t IMU_CS_PINS[3]       = {IMU1_CS_Pin, IMU2_CS_Pin, IMU3_CS_Pin}; // CSピンの ピン番号を格納

//計測輪制御の変数
volatile int value[3]; // 1ms（前回からの間隔）の間に各エンコーダがカウントしたパルス数（変位量）
volatile int sum_value[3]; // プログラム起動（またはリセット）からの全累積パルス数（総回転量）

// 各計測輪が1秒間にどれくらい回転しているかを表す角速度
volatile double deg1 = 0.0f; //[rad]
volatile double deg2 = 0.0f; //[rad]
volatile double deg3 = 0.0f; //[rad]

// ロボット自身から見た（ローカル座標系における）現在速度。
volatile double dxl; // 前後方向の移動速度
volatile double dyl; // 左右方向の移動速度
volatile double dwl; // 旋回（回転）角速度

// ノイズ除去（移動平均フィルタ）を適用した後の、ロボットの移動速度および旋回速度を保持する変数群
volatile double filtered_vx; // 移動平均フィルタ処理後の X軸方向（前後方向）の移動速度
volatile double filtered_vy; // 移動平均フィルタ処理後の Y軸方向（左右方向）の移動速度
volatile double filtered_omega; // 移動平均フィルタ処理後の 旋回（回転）角速度
volatile double theta;

//共通の変数
float dt = 0.001f; // 制御・積分計算のサンプリング周期
uint32_t last_time = 0; // 前回処理を実行した時刻（ミリ秒単位のタイムスタンプ）を記録するための変数
uint32_t print_counter = 0; // デバッグ用シリアル通信（printf）の出力間隔を間引くためのダウンカウンタ ex count=1000になったら通信
volatile uint8_t flag_1ms_update = 0; // 1msの割り込み処理が完了したことをメインループ（while(1)）へ伝える同期フラグ（フラグ変数）

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint32_t DataLength, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader);
void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len);
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len);
void float_to_u8(float *req, uint8_t *des, uint32_t float_len);
void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len);

void LSM6_Write(uint8_t reg, uint8_t data, int port);
void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int port);
uint8_t LSM6_Read(uint8_t reg, int port);
void INIT_IMU(int port);
float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void getEulerAngles();
void resetBias();
void resetWheel();

typedef struct {
    float buffer[AVG_WINDOW_SIZE]; // 静的に確保
    int index;
    int count;
    double sum;
} MovingAvgData;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static MovingAvgData avg_x, avg_y, avg_omega;
TIM_TypeDef* const TIM[4] = {TIM2, TIM8, TIM1}; // エンコーダ入力として使用している4つのタイマー（TIM）のハードウェアレジスタアドレスをまとめたポインタ配列

int16_t read_encoder_value(int port)
{
  TIM_TypeDef* tim = TIM[port-1];
  int16_t count_t = (int16_t)tim->CNT;
  tim->CNT = 0;

  return count_t;
}

int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
  return len;
}

void init_averages(MovingAvgData *ma) {
    for(int i = 0; i < AVG_WINDOW_SIZE; i++) ma->buffer[i] = 0.0f;
    ma->index = 0;
    ma->count = 0;
    ma->sum = 0.0f;
}
float update_ma_isr(MovingAvgData *ma, float next_val) {
    // 古い値を引いて新しい値を足す (O(1)の計算)
    if (ma->count == AVG_WINDOW_SIZE) {
        ma->sum -= ma->buffer[ma->index];
    } else {
        ma->count++;
    }

    ma->buffer[ma->index] = next_val;
    ma->sum += next_val;
    ma->index = (ma->index + 1) % AVG_WINDOW_SIZE;

    return ma->sum / (float)ma->count;
}

// CANのRX FIFO1にメッセージが届いたら、それを取り出してCAN IDごとに処理する 使うかわからない
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

    /* Retrieve Rx messages from RX FIFO1 */
		uint8_t RxData[64] = {};
    FDCAN_RxHeaderTypeDef RxHeader;
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}
    switch (RxHeader.Identifier)
    {
      default:
        // printf("unknown CAN ID received: 0x%03lX\r\n", RxHeader.Identifier); // printf should be commented out within Callback
        break;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //タイマー割り込み
	if (&htim6 == htim) { // 1000Hz
    if (isSettingBias == 0){ //IMUがON、バイアス算出済みのとき
        uint8_t buffer[12];
        for (int i=0; i<3; i++){
          LSM6_ReadMulti(0x22, buffer, 12, i+1); // IMU:一括読み出し
          imu[i].gx = ((int16_t)(buffer[1] << 8 | buffer[0]));
          imu[i].gy = ((int16_t)(buffer[3] << 8 | buffer[2]));
          imu[i].gz = ((int16_t)(buffer[5] << 8 | buffer[4]));
          imu[i].ax = ((int16_t)(buffer[7] << 8 | buffer[6]));
          imu[i].ay = ((int16_t)(buffer[9] << 8 | buffer[8]));
          imu[i].az = ((int16_t)(buffer[11] << 8 | buffer[10]));
        }

        // 軸まわりの角速度
        gyro_x = (float)(-imu[1].gy + imu[0].gy*sin_30 - imu[0].gx*cos_30 + imu[2].gx*cos_30 + imu[2].gy*sin_30)*G_sensitivity/3.0f - gyro_x_bias;
        gyro_y = (float)( imu[1].gx - imu[0].gy*cos_30 - imu[0].gx*sin_30 - imu[2].gx*sin_30 + imu[2].gy*cos_30)*G_sensitivity/3.0f - gyro_y_bias;
        gyro_z = (float)( imu[0].gz + imu[1].gz + imu[2].gz )*G_sensitivity/3.0f - gyro_z_bias;
        gyro_z = gyro_z * GYRO_Z_SCALE; // 微小なスケール誤差補正

        accel_x = (float)(-imu[1].ay + imu[0].ay*sin_30 - imu[0].ax*cos_30 + imu[2].ax*cos_30 + imu[2].ay*sin_30)*A_sensitivity/3.0f;
        accel_y = (float)( imu[1].ax - imu[0].ay*cos_30 - imu[0].ax*sin_30 - imu[2].ax*sin_30 + imu[2].ay*cos_30)*A_sensitivity/3.0f;
        accel_z = (float)( imu[0].az + imu[1].az + imu[2].az )*A_sensitivity/3.0f;

        // 微小なノイズを0とみなす, fabs:絶対値
        if (fabs(gyro_x) < 0.5) gyro_x = 0.0;
        if (fabs(gyro_y) < 0.5) gyro_y = 0.0;
        if (fabs(gyro_z) < 0.5) gyro_z = 0.0;
    }

    if (isSettingWheel == 0) { // 計測輪有効のとき
        value[0] = read_encoder_value(1);
        value[1] = read_encoder_value(2);
        value[2] = read_encoder_value(3);

        sum_value[0] += value[0];
        sum_value[1] += value[1];
        sum_value[2] += value[2];
    
        // // こっちが間違ってる理由がまだ飲み込めてない
        // deg1 = (((float)value[0] / PPR) * 2 * M_PI) / dt; //各計測輪の角速度
        // deg2 = (((float)value[1] / PPR) * 2 * M_PI) / dt;
        // deg3 = (((float)value[2] / PPR) * 2 * M_PI) / dt;

        // こっちが正しい
        deg1 = (((float)value[0] / (PPR * 4.0f)) * 2.0f * M_PI) / dt;
        deg2 = (((float)value[1] / (PPR * 4.0f)) * 2.0f * M_PI) / dt;
        deg3 = (((float)value[2] / (PPR * 4.0f)) * 2.0f * M_PI) / dt;

        dwl = gyro_z * CONV;
        dxl = (deg1 - deg2) * R / 2.0f;
        dyl = deg3 * R - (L * dwl); // Y輪の回転干渉をキャンセル
        
        // 移動平均を計算（計算負荷は非常に低い）
        filtered_vx = update_ma_isr(&avg_x, dxl);
        filtered_vy = update_ma_isr(&avg_y, dyl);
        filtered_omega = update_ma_isr(&avg_omega, dwl);

        // 角度の積分計算
        theta += filtered_omega * dt;
    }
    flag_1ms_update = 1; // 1msが経過し無事割り込み処理ができたことを通知
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // 変数などの宣言を書くコーナー
  // printfなどによる stdout（標準出力）の「バッファリング（一時保持）」を無効化（OFF）する設定
  setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // ハードウェアの初期化、または独自の値の初期化を書くコーナー
  cos_30 = sqrt(3)/2; sin_30 = 0.50f;//IMU用
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // ハードウェアの準備がすべて整った後の動作開始・設定を書くコーナー
  HAL_Delay(2000);

  // 初期化時にエラーが発生した時の処理
  if (HAL_OK != interboard_comms_CAN_RxTxSettings_init(&TxHeader)) Error_Handler();
  
  // 1msタイマーのスタート
  HAL_TIM_Base_Start_IT(&htim6);

  // ロリコン用のエンコーダーの起動
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  // IMUの初期化
  INIT_IMU(1);
  INIT_IMU(2);
  INIT_IMU(3);

  /*whoami = LSM6_Read(0x0F,1);
  whoami2 = LSM6_Read(0x0F,2);
  whoami3 = LSM6_Read(0x0F,3);*/
  
  resetBias();
  resetWheel();
  last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (flag_1ms_update == 1){
      flag_1ms_update = 0;

      float txdata_f[4] = {filtered_vx, filtered_vy, filtered_omega, theta};
      uint8_t txdata2_u8[16] = {0};
      float_to_u8(txdata_f, txdata2_u8, 4);
      CAN_SEND(CAN_ID_FEEDBACK, FDCAN_DLC_BYTES_16, txdata2_u8, &hfdcan1, &TxHeader);

      //以下デバッグ用
      print_counter++;
      if (print_counter >= 100) { // 100回に1回（＝100ms周期、10Hz）
        print_counter = 0; // カウンタをリセット
        printf(">"); //シリアルプロッタ表示
        //IMU
        // printf("1:%d,2:%d,3:%d\r\n",whoami,whoami2,whoami3);
        printf("gyro_x:%d,gyro_y:%d,gyro_z:%d,\r\n",(int)gyro_x,(int)gyro_y,(int)gyro_z);
        printf("accel_x:%d,accel_y:%d,accel_z:%d,",(int)accel_x,(int)accel_y,(int)accel_z);
        printf("yaw:%.2f,pitch:%.2f,roll:%.2f\r\n",yaw,pitch,roll);
    
        //計測輪
        printf("%d",isSettingWheel);
        printf("1:%d,2:%d,3:%d\r\n",value[0],value[1],value[2]);
        printf("%d,%d,%d",sum_value[0],sum_value[1],sum_value[2]);
        printf("vx:%.4f,vy:%.4f,vz:%.4f,\r\n",dxl,dyl,dwl);
        printf("vx':%.4f,vy':%.4f,vz':%.4f,\r\n",filtered_vx,filtered_vy,filtered_omega);
    
        printf("\r\n"); //シリアルプロッタ表示
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB4 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU3_CS_Pin IMU2_CS_Pin IMU1_CS_Pin */
  GPIO_InitStruct.Pin = IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// こっちで関数定義するならプロトタイプ宣言がいる

// CAN受信側のフィルタ設定を作る関数 使うか不明
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x7ff;
}

// CAN通信送信側の設定
void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x00;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_FD_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

// CANのReceive/Transmit設定をして、FDCANを起動する関数
HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  interboard_comms_CAN_filter_init(&FDCAN_Filter_settings);
  interboard_comms_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan1))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

// CANでデータを1個送るための共通関数
// CAN_SEND(0x7FE, TxData, &hfdcan1, &TxHeader); // example usage
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint32_t DataLength, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->DataLength = DataLength;
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

// LSM6系IMUにSPIでレジスタを書き込む関数
void LSM6_Write(uint8_t reg, uint8_t data, int port)
{
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t tx[2] = {reg & 0x7F, data};

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

// SPI通信で読み込みを行うドライバー関数
uint8_t LSM6_Read(uint8_t reg, int port) { //1つのIMUから単独で読み出す場合
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t tx = reg | 0x80;  // Read（MSB=1）
    uint8_t rx = 0;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY); // アドレス送信
    HAL_SPI_Receive(&hspi2, &rx, 1, HAL_MAX_DELAY); // データ受信
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET); // CS HIGH

    return rx;
}

// IMUのセンサー値を最高速・超安定で丸ごと取得するためのメイン関数
void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int port) { //1つのIMUからまとめて読み出す場合
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t tx_buf[13] = {0};
    uint8_t rx_buf[13] = {0};
    tx_buf[0] = reg | 0x80;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    // 送信と受信を同時に行う（サイズは アドレス1byte + データ分）
    if(HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, size + 1, HAL_MAX_DELAY) == HAL_OK) {
        // rx_buf[0]はアドレス送信時のゴミなので、rx_buf[1]からコピー
        for(int i = 0; i < size; i++) {
            pData[i] = rx_buf[i + 1];
        }
    }
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

// IMUの初期化関数
void INIT_IMU(int port){
  /*センサの初期化*/
  LSM6_Write(0x12, 0x44,port); // CTRL3: reboot,BDU有効化,アドレス自動インクリメント有効化
  /*ジャイロの初期化*/
  LSM6_Write(0x15, 0x04,port); // CTRL6: FS=±2000dps
  LSM6_Write(0x11, 0x10,port); // CTRL2: ODR=1.92kHz
  /*加速度の初期化*/
  LSM6_Write(0x17, 0x03,port); // CTRL8: FS=±16g
  LSM6_Write(0x10, 0x10,port); // CTRL1: ODR=1.92kHz
}

float invSqrt(float x){
  return 1.0f / sqrtf(x);
}

void resetBias(){
  // 起動時に1000回計測して平均をとる
  isSettingBias = 1;
  gyro_x_bias = 0.0f;
  gyro_y_bias = 0.0f;
  gyro_z_bias = 0.0f;
  for(int i=0; i<1000; i++) {
    uint8_t buffer6[12];
    for (int j=0; j<3; j++) {
      LSM6_ReadMulti(0x22, buffer6, 12, j+1); // IMU:一括読み出し
      imu[j].gx = ((int16_t)(buffer6[1] << 8 | buffer6[0]));
      imu[j].gy = ((int16_t)(buffer6[3] << 8 | buffer6[2]));
      imu[j].gz = ((int16_t)(buffer6[5] << 8 | buffer6[4]));
      imu[j].ax = ((int16_t)(buffer6[7] << 8 | buffer6[6]));
      imu[j].ay = ((int16_t)(buffer6[9] << 8 | buffer6[8]));
      imu[j].az = ((int16_t)(buffer6[11]<< 8 | buffer6[10]));
    }
    
    gyro_x_bias += (float)(-imu[1].gy + imu[0].gy*sin_30 - imu[0].gx*cos_30 + imu[2].gx*cos_30 + imu[2].gy*sin_30);
    gyro_y_bias += (float)( imu[1].gx - imu[0].gy*cos_30 - imu[0].gx*sin_30 - imu[2].gx*sin_30 + imu[2].gy*cos_30);
    gyro_z_bias += (float)( imu[0].gz + imu[1].gz + imu[2].gz );

    HAL_Delay(1);
  }
  gyro_x_bias = gyro_x_bias *G_sensitivity * 0.001f / 3.0f;
  gyro_y_bias = gyro_y_bias *G_sensitivity * 0.001f / 3.0f;
  gyro_z_bias = gyro_z_bias *G_sensitivity * 0.001f / 3.0f;

  isSettingBias = 0;
}

void resetWheel(){
  isSettingWheel = 1;
  init_averages(&avg_x);
  init_averages(&avg_y);
  init_averages(&avg_omega);
  theta = 0.0;
  sum_value[0] = 0;
  sum_value[1] = 0;
  sum_value[2] = 0;
  isSettingWheel = 0;
}

// uint8_t 型のバイト列（4バイト単位）を、IEEE 754 形式の float（単精度浮動小数点数）配列へ復元・変換する関数
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t f32_u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    union IntAndFloat target;
    target.ival = f32_u32;
    des[i] = target.fval;
  }
}

void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len)
{
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    des[i] = (int32_t)u32;
  }
}

void float_to_u8(float *req, uint8_t *des, uint32_t float_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for (int i = 0; i < float_len; i++)
  {
    union IntAndFloat target;
    target.fval = req[i];
    uint32_t val = target.ival;
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
}

void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len)
{
  for (int i = 0; i < int_len; i++)
  {
    uint32_t val = (uint32_t)req[i];
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
