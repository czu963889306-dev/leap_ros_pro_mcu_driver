#pragma once

#pragma pack(push, 1)
struct ServoMsg {
    uint8_t channel = 1;
    float angle = 90.0f;
};
#pragma pack(pop)
