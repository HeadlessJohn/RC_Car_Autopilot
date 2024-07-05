# 자율주행 RC Car Project

<img width = "30%" src="https://github.com/HeadlessJohn/rc_car_autopilot/blob/main/img/20240705_100208.jpg?raw=true"> 

## Nucleo F411RE 보드를 이용한 자율주행 RC Car

### 1. 기능

- 블루투스로 연결된 조이스틱으로부터 ADC 값 수신 [`RC_CAR_Controller`](https://github.com/HeadlessJohn/RC_CAR_CONTROLLER)
- 수신된 값을 계산하여 좌륜, 우륜 속도 분배
- LCD에 상태 표시
- 자율주행 모드

### 2. 사용한 부품

- `SSD1306` SPI OLED 모듈
- DC모터 4개서
- `L298N` 모터 드라이버
- `HC-05` 블루투스 모듈
- `VL53L0X` ToF 적외선 센서
- 18650 3.6V 배터리 x3

### 3. OS, API

- FreeRTOS CMSIS-RTOS V2
- ssd1306 API
- VL53L0X API

### 4. 주행 영상

- <https://youtu.be/baAECXsCBlI>
- <https://youtu.be/HT1B8OWjBzg>