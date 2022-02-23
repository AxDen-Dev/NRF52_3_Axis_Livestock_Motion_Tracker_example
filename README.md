# 3-Axis Bluetooth Motion Tracker

<img src="./assets/ble_3_axis_motion_tracker.jpeg">
<br>

----

3-Axis Bluetooth Motion Tracker 은 가속도, 온도와 같은 Motion Tracker 에 필요한 핵심적인 정보를 수집하고 Bluetooth 통신을 이용하여 전송합니다.
<br>
Android, iOS, Gateway 통신이 필요한 다양한 서비스 시나리오를 빠르게 테스트 할 수 있도록 제공하는 예제입니다.
<br>
<br>
3-Axis Bluetooth Motion Tracker collects key information required for Motion Tracker such as acceleration and temperature and transmits it using Bluetooth communication.
<br>
This is an example provided to quickly test various service scenarios that require Android, iOS, and Gateway communication.
<br>

----

[네이버 스마트 스토어](https://smartstore.naver.com/axden)
<br>

----

### 주요 특징 및 기능

MCU | 설명
:-------------------------:|:-------------------------:
NRF52811 | Bluetooth SoC

센서 | 설명
:-------------------------:|:-------------------------:
KXTJ3 | 3 Axis Accelerometer
Si7201 | Hall Sensor
NTCG104BH103JT1 | NTC Temperature Sensor
Battery | 1/2 AA Battery 1200mAh

<br>


BLE 통신이 가능한 Motion tracker 입니다.
<br>

NRF52811 SoC 를 이용하여 BLE 통신이 가능합니다.
<br>

SI7201 Hall 센서와 자석을 이용하여 On Off 할 수 있습니다.
<br>

KXTJ3 을 이용하여 가속도 값을 수집합니다.
<br>

NTC 를 이용하여 온도 값을 수집합니다.
<br>

배터리를 이용하여 5년동안 사용 가능합니다.
<br>

----

### Note

해당 프로그램으로 예제 프로그램으로 양산 및 상용화에 적합하지 않습니다.
<br>

양산 및 대량 구매 고객께서는 development@axden.io 으로 문의 주시기 바랍니다.
<br>

양산 및 대량 구매 고객분들께는 저전력, 안정화, 게이트웨이와 통신 등 사용하시는 목적에 맞는 최적화된 Firmware 를 무료로 개발해 드립니다.
<br>

<table>
  <tr align="center">
    <td>Top</td>
    <td>Bottom</td>
  </tr>
  <tr align="center">
    <td><img src="./assets/ble_3_axis_motion_tracker_top.jpeg"></td>
    <td><img src="./assets/ble_3_axis_motion_tracker_bottom.jpeg"></td>
  </tr>
</table>

Pinmap 은 ```board_define.h``` 파일에서 확인 가능합니다.
<br>

```

#define BATTERY_LEVEL_ADC 4 //AIN2
#define NTC_ADC 5 //AIN3
#define NTC_VCC_GPIO 18

#define LED_RED_GPIO 28
#define LED_BLUE_GPIO 25

#define I2C_SDA_GPIO 12
#define I2C_SCL_GPIO 14

#define HALL_INPUT_GPIO 30


```

<br>

Works with SoftDevice S112 v7.2.0, provided with SDK 17.1.0.
<br>

To compile it, clone the repository in the [SDK]/examples/ble_peripheral folder.
