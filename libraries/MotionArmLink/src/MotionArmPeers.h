#pragma once
#include <stdint.h>

namespace motionarm {
// STA MACs verified locally through the USB bootloader on COM9 and COM7.
constexpr uint8_t wearableMac[] = {0xec, 0xe3, 0x34, 0xef, 0x81, 0x64};
constexpr uint8_t robotMac[] = {0xc0, 0xcd, 0xd6, 0xd0, 0x9b, 0x54};
}
