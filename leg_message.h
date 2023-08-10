#ifndef _leg_message
#define _leg_message
 
#include <stdint.h>
 
typedef union _packet{
    int16_t data;
    uint8_t buffer[2];
}Packet;

typedef union _packet2{
    uint16_t data;
    uint8_t buffer[2];
}Packet2;

typedef union _packet3{
    uint32_t data;
    uint8_t buffer[4];
}Packet3;

// 60 bytes
// 30 16-bit words
struct spi_data_t
{
    Packet2 q_abad;
    Packet2 q_hip;
    Packet2 q_knee;
    Packet qd_abad;
    Packet qd_hip;
    Packet qd_knee;
    // float q_abad[2];
    // float q_hip[2];
    // float q_knee[2];
    // float qd_abad[2];
    // float qd_hip[2];
    // float qd_knee[2];
    int32_t flags;
    int32_t checksum;
};
 
// 132 bytes
// 66 16-bit words
struct spi_command_t
{
    float q_des_abad[2];
    float q_des_hip[2];
    float q_des_knee[2];
    float qd_des_abad[2];
    float qd_des_hip[2];
    float qd_des_knee[2];
    float kp_abad[2];
    float kp_hip[2];
    float kp_knee[2];
    float kd_abad[2];
    float kd_hip[2];
    float kd_knee[2];
    float tau_abad_ff[2];
    float tau_hip_ff[2];
    float tau_knee_ff[2];
    int32_t flags[2];
    int32_t checksum;
};
 
struct joint_control{
    Packet tau;
    };
    
struct joint_state{
    Packet v, t;
    Packet2 p;
    float angle;
    float vel;
    int cnt;
    };
    
struct leg_state{
    joint_state a, h, k;
    };
struct leg_control{
    joint_control a, h, k;
    };

struct joint_state_init{
    Packet3 angle;
    Packet2 p;
};

struct leg_state_init{
    joint_state_init a, h, k;
};
#endif