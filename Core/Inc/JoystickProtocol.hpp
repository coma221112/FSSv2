/*
 * JoystickProtocol.hpp
 *
 *  Created on: Jan 25, 2026
 *      Author: Drac
 */

#ifndef INC_JOYSTICKPROTOCOL_HPP_
#define INC_JOYSTICKPROTOCOL_HPP_

typedef struct {
    uint8_t reportID;

#ifdef __cplusplus
    struct BitProxy {
        uint32_t* data_ref;
        int index;

        BitProxy& operator=(bool value) {
            if (value) *data_ref |= (1UL << index);
            else       *data_ref &= ~(1UL << index);
            return *this;
        }

        operator bool() const {
            return (*data_ref >> index) & 1UL;
        }
    };

    struct ButtonManager {
        uint32_t data; // 实际存储 24 bits + 8 bits padding
        BitProxy operator[](int i) {
            return {&data, i};
        }
    } buttons;
#else
    uint32_t buttons;
#endif

    int16_t x, y, z, rx, ry, rz, lt;

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
