# Balance Car

### 版本更迭紀錄

- `control`：平衡車控制專案，已成功實現平衡車效果。
- `go side`：在 `control` 的基礎上加入 TCRT5000 和循跡程式，達成循跡功能，能夠在老師提供的循跡線上完成實作。

## 腳位配置

| 功能 | 腳位 |
|------|------|
| PWMA | PA0 |
| PWMB | PA1 |
| AIN1 | PA4 |
| AIN2 | PA5 |
| BIN1 | PA6 |
| BIN2 | PA7 |
| MPU6050 SCL | PB6 |
| MPU6050 SDA | PB7 |
| USART1_TX/RX | PA9 / PA10|
| TCRT5000 D0| PA2 / PB1 |
| USB_DM/DP | PA11 / PA12|

## 系統架構

### 硬體平台

- MCU：STM32F103C8T6
- 開發工具：Keil uVision5、STM32CubeMX
- 主要感測與控制模組：
  - MPU6050 姿態感測器
  - TB6612 馬達驅動模組
  - TCRT5000 循跡模組
  - USB CDC / UART 通訊介面

### 專案結構

```text
balance-car/
├── control/
│   ├── Core/
│   │   ├── Inc/                 # 使用者標頭檔
│   │   └── Src/                 # 主程式與控制邏輯
│   ├── Drivers/                 # STM32 HAL / CMSIS 驅動
│   ├── Middlewares/             # USB Device Library
│   ├── USB_DEVICE/              # USB CDC 設定
│   ├── MDK-ARM/                 # Keil5 專案檔
│   └── control.ioc              # STM32CubeMX 設定與腳位配置
│
└── go side/
    ├── Core/
    │   ├── Inc/                 # 使用者標頭檔
    │   └── Src/                 # 平衡控制與循跡控制邏輯
    ├── Drivers/                 # STM32 HAL / CMSIS 驅動
    ├── Middlewares/             # USB Device Library
    ├── USB_DEVICE/              # USB CDC 設定
    ├── MDK-ARM/                 # Keil5 專案檔
    └── control.ioc              # STM32CubeMX 設定與腳位配置
```

### 控制流程

```text
MPU6050 / 感測器輸入
        ↓
姿態角度與速度資料處理
        ↓
平衡控制演算法
        ↓
馬達 PWM / 方向控制
        ↓
平衡車本體運動
```

`go side` 專案在上述平衡控制流程外，另外加入 TCRT5000 模組和循跡程式：

```text
TCRT 循跡模組輸入
        ↓
路線偏移判斷
        ↓
左右輪速度修正
        ↓
維持平衡並沿線行走
```

## 編譯方式

### 需求環境

- Keil uVision5
- STM32F1xx Device Pack
- STM32CubeMX，若需要查看或重新產生初始化程式碼
- ST-Link 驅動與燒錄工具

### 編譯 `control`

1. 開啟 Keil uVision5。
2. 開啟檔案：

```text
control/MDK-ARM/control.uvprojx
```

3. 確認目標晶片為 `STM32F103C8T6`。
4. 若需要確認腳位，開啟 `control/control.ioc` 查看 STM32CubeMX 設定。
5. 點選 `Build` 或按下 `F7` 進行編譯。
6. 編譯成功後，可使用 Keil 或 ST-Link 將程式燒錄到 STM32F103C8T6。

### 編譯 `go side`

1. 開啟 Keil uVision5。
2. 開啟檔案：

```text
go side/MDK-ARM/control.uvprojx
```

3. 確認目標晶片為 `STM32F103C8T6`。
4. 若需要確認腳位，開啟 `go side/control.ioc` 查看 STM32CubeMX 設定。
5. 點選 `Build` 或按下 `F7` 進行編譯。
6. 編譯成功後，可使用 Keil 或 ST-Link 將程式燒錄到 STM32F103C8T6。

## 使用說明

### `control` 平衡車專案

`control` 專案用於實現平衡車基本功能。燒錄到 STM32F103C8T6 後，系統會讀取 MPU6050 姿態資料，經由控制演算法計算馬達輸出，使車體維持平衡。

使用步驟：

1. 接好 STM32F103C8T6、MPU6050、馬達驅動與電源。
2. 將 `control` 專案編譯並燒錄到 STM32F103C8T6。
3. 將車體放置於可平衡角度附近。
4. 上電後觀察車體平衡狀態。
5. 若車體方向或反應異常，檢查馬達接線、MPU6050 安裝方向與電源供應。

### `go side` 循跡平衡車專案

`go side` 是 `control` 的改進版，在原本平衡車功能上加入 TCRT5000 模組。系統會同時維持車體平衡，並根據 TCRT5000 模組輸入修正左右輪輸出，使車體沿著路線移動。

使用步驟：

1. 接好 STM32F103C8T6、MPU6050、馬達驅動、TCRT 循跡模組與電源。
2. 將 `go side` 專案編譯並燒錄到 STM32F103C8T6。
3. 將車體放置在循跡線上，並使車體接近平衡角度。
4. 上電後觀察車體是否能維持平衡並沿線行走。
5. 若循跡方向異常，檢查 TCRT 循跡模組接線、安裝方向與判斷邏輯。

## 版本說明

- `control`：基礎平衡車版本，已實作成功。
- `go side`：加入 TCRT 循跡模組的改進版本，已實作成功。

## 注意事項

- 兩個專案皆燒錄於 STM32F103C8T6。
- 腳位配置以各專案的 `.ioc` 檔為準。
- 燒錄前請確認電源供應穩定，避免馬達啟動造成 MCU 重啟。
- 第一次測試時建議架高車體或扶住車身，避免馬達輸出過大造成車體摔落。
- 若重新使用 STM32CubeMX 產生程式碼，請注意保留 `USER CODE BEGIN` 與 `USER CODE END` 區塊內的自訂程式。
