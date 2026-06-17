# Balance Car

收錄以 STM32F103C8T6 為核心的平衡車專案，包含自平衡、TCRT5000 循跡，以及爬坡循跡整合版本。

## 專案列表

- `control`：基礎自平衡車程式，是後續專案使用的平衡控制基底。
- `go side`：在平衡基底上加入 TCRT5000，保留一份循跡效果成功的參考邏輯。
- `climb-ramp`：目前整合完成的版本，可執行平衡、循跡與爬坡。
- `climb-ramp_good`：改進 `climb-ramp` 過程中較穩定的備份版本，保留作為回復與比較用。

## 腳位配置

| 功能 | 腳位 |
| --- | --- |
| PWMA | PA0 |
| PWMB | PA1 |
| AIN1 | PA4 |
| AIN2 | PA5 |
| BIN1 | PA6 |
| BIN2 | PA7 |
| MPU6050 SCL | PB6 |
| MPU6050 SDA | PB7 |
| USART1_TX/RX | PA9 / PA10 |
| TCRT5000 D0 | PA2 / PB1 |
| USB_DM/DP | PA11 / PA12 |

## 開發環境

- MCU：STM32F103C8T6
- 開發工具：Keil uVision5、STM32CubeMX
- 主要模組：
  - MPU6050 姿態感測器
  - TB6612FNG 馬達驅動模組
  - TCRT5000 循跡感測器
  - USB CDC / UART 除錯通訊

## 專案結構

```text
balance-car/
|-- control/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
|-- go side/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
|-- climb-ramp/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
`-- climb-ramp_good/
    |-- Core/
    |-- Drivers/
    |-- Middlewares/
    |-- USB_DEVICE/
    |-- MDK-ARM/
    `-- control.ioc
```

## 目前版本 : `climb-ramp` 

目前的 `climb-ramp` 版本整合了改進後的平衡控制、成功的循跡邏輯，以及爬坡輔助與爬坡循跡能力。

### 平衡控制

- 使用 `control` 改進後的平衡邏輯。
- 使用 MPU6050 進行姿態角估測。
- 使用陀螺儀角速度作為 D 項。
- 加入積分限制、輸出限制、PWM 斜率限制與倒車保護。

### 循跡控制

- 使用從 `go side` 成功版本整理出的 TCRT5000 循跡邏輯。
- TCRT5000 判斷方式：
  - LED 亮代表白色地面。
  - LED 不亮代表黑線。
- 平地與爬坡時都能套用一黑一白修正。
- 加入短暫丟線保持，車體晃動導致感測器短暫讀不到黑線時，會保留上一個修正方向一小段時間。

### 爬坡控制

- 當車體角度進入坡道區間時，啟動爬坡模式。
- 平地循跡維持較慢且穩定的行為。
- 爬坡模式只在爬坡時提高馬達輸出。
- 爬坡中仍會套用一黑一白循跡修正。
- 爬坡修正時，較慢的輪子會維持小幅非零轉速，降低單邊打滑或空轉機率。
- 車體回到接近平地角度後，會退出爬坡模式。

## 編譯與燒錄

1. 使用 Keil uVision5 開啟要編譯的專案。
2. 開啟 `MDK-ARM` 底下的 project，例如：

```text
climb-ramp/MDK-ARM/control.uvprojx
```

3. 確認目標晶片為 STM32F103C8T6。
4. 編譯後使用 ST-Link 燒錄。

Keil 編譯產生的 `.o`、`.d`、`.axf`、`.hex`、`.map` 與 build log 會被 `.gitignore` 排除。

## 注意事項

- 測試或修改時，盡量只更動目標專案資料夾。
- `climb-ramp_good` 是較穩定的備份版本，可用來和目前 `climb-ramp` 做比較或回復。
- 若使用 STM32CubeMX 重新產生程式碼，請注意保留 `USER CODE BEGIN` / `USER CODE END` 內的自訂程式。
