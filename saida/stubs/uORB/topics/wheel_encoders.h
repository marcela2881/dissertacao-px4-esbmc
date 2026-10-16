#pragma once
#include <cstdint>
/* mesmo layout de msg/WheelEncoders.msg (timestamp, wheel_speed[2], wheel_angle[2]) */
struct wheel_encoders_s { uint64_t timestamp; float wheel_speed[2]; float wheel_angle[2]; };
