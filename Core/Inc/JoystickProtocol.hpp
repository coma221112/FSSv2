/*
 * JoystickProtocol.hpp
 *
 *  Created on: Jan 25, 2026
 *      Author: Drac
 */

#ifndef INC_JOYSTICKPROTOCOL_HPP_
#define INC_JOYSTICKPROTOCOL_HPP_

typedef struct
{
	uint8_t reportID;
  union {
    struct {
      uint32_t button1  : 1;
      uint32_t button2  : 1;
      uint32_t button3  : 1;
      uint32_t button4  : 1;
      uint32_t button5  : 1;
      uint32_t button6  : 1;
      uint32_t button7  : 1;
      uint32_t button8  : 1;
      uint32_t button9  : 1;
      uint32_t button10 : 1;
      uint32_t button11 : 1;
      uint32_t button12 : 1;
      uint32_t button13 : 1;
      uint32_t button14 : 1;
      uint32_t button15 : 1;
      uint32_t button16 : 1;
      uint32_t button17 : 1;
      uint32_t button18 : 1;
      uint32_t button19 : 1;
      uint32_t button20 : 1;
      uint32_t button21 : 1;
      uint32_t button22 : 1;
      uint32_t button23 : 1;
      uint32_t button24 : 1;
      uint32_t padding  : 8;  // Padding to align to 32 bits
    } bits;
    uint32_t all_buttons;      // Access all buttons as a single uint32_t
  } buttons;

  int16_t x;      // Axis X (-32768 to 32767)
  int16_t y;      // Axis Y (-32768 to 32767)
  int16_t z;      // Axis Z (-32768 to 32767)
  int16_t rx;     // Axis Rx (-32768 to 32767)
  int16_t ry;     // Axis Ry (-32768 to 32767)
  int16_t rz;     // Axis Rz (-32768 to 32767)
  int16_t lt;     // Axis Rz (-32768 to 32767)

} __attribute__((packed)) USB_HID_JoystickReport_t;

typedef struct{
	uint8_t reportID;
    // --- 传感器量程与灵敏度 ---
	uint32_t maxRange;          // 对应代码中的 maxRange (默认 2400000)
    // --- 校准逻辑参数 ---
    uint32_t calibLongPressMs; // 触发校准所需的长按时间 (默认 5000)

    // --- 摇杆性能微调 ---
    float deadzone;         // 软件死区，防止静止时抖动

    // 预留槽位方便后续扩展而不破坏结构体大小
    uint8_t  reserved[51];
} __attribute__((packed)) JoystickConfig_t;

#ifdef __cplusplus
  // C++ 编译器看到这个，实现真正的 Header-only 全局变量
  inline USB_HID_JoystickReport_t joystickReport = {1};
#else
  // C 编译器（编译 .c 文件时）只看到声明
  // 实际的变量实例会由其中一个 .cpp 文件提供
  extern USB_HID_JoystickReport_t joystickReport;
#endif

#ifdef __cplusplus
volatile inline JoystickConfig_t joystickConfig={2, 800000, 2000, 0.1};
#else
extern volatile JoystickConfig_t joystickConfig;
#endif

#ifdef __cplusplus
volatile inline uint8_t configNeedsSending = 0;
#else
extern volatile uint8_t configNeedsSending;
#endif

#endif /* INC_JOYSTICKPROTOCOL_HPP_ */
