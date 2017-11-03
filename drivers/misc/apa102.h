#ifndef _LEDRING_H
#define _LEDRING_H

#pragma pack(push, 1)
struct apa102_color {
    u8    b;
    u8    g;
    u8    r;
};

struct apa102_frame {
    struct list_head frame_entry;
    struct apa102_color leds[0];
};

struct apa102_command {
    u8     command;
    u16    transition_time;
};

struct apa102_cmd_animation {
    struct apa102_command command;
    u8 frame_rate;
    u8 target_rate;
    s8 rotation_per_minute;
    u8 frame_count;
    struct apa102_color frames[0];
};

struct apa102_cmd_brightness {
    struct apa102_command command;
    u8    brightness;
};

struct apa102_cmd_solid {
    struct apa102_command command;
    struct apa102_color solid;
};

struct apa102_cmd_progress {
    struct apa102_command command;
    struct apa102_color color;
    u8 progress;
};

#pragma pack(pop)


#define CMD_ANIMATION 2
#define CMD_BRIGHTNESS 3
#define CMD_SOLID 4
#define CMD_PROGRESS 5

#endif