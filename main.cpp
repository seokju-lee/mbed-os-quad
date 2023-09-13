#include "CAN.h"
#include "DigitalIn.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "can_helper.h"
#include "mbed.h"
#include <cstdint>
#include <cstring>
#include "leg_message.h"
#include "mbed_wait_api.h"

#define RX_LEN   66
#define TX_LEN   66
#define CMD_LEN  66
#define DATA_LEN 30
#define PI       3.1415926535898f

spi_data_t       spi_data;
spi_command_t    spi_command;

uint16_t         rx_buff[RX_LEN];
uint16_t         tx_buff[TX_LEN];

Serial           pc(PA_2, PA_3);

CAN              can1(PB_8, PB_9, 1000000);
CAN              can2(PB_12, PB_13, 1000000);
CANMessage       rxMsg1, rxMsg2;
CANMessage       a1_can, h1_can, k1_can, a2_can, h2_can, k2_can;

int              counter = 0;
int              enabled = 0;
int              init_enabled = 0;
int              bytecount = 0;

int              spi_enabled = 0;
InterruptIn      cs(PA_4);

leg_state        l1_state, l2_state;
leg_state_init   l1_state_init, l2_state_init;
leg_control      l1_control, l2_control;

uint16_t         prev_posa = 0;
uint16_t         prev_posh = 0;
uint16_t         prev_posk = 0;
uint16_t         init_posa = 0;
uint16_t         init_posh = 0;
uint16_t         init_posk = 0;
int16_t          prev_vela = 0;
int16_t          prev_velh = 0;
int16_t          prev_velk = 0;
int              cnta = 0; 
int              cnth = 0; 
int              cntk = 0;

uint16_t         prev_posa2 = 0;
uint16_t         prev_posh2 = 0;
uint16_t         prev_posk2 = 0;
uint16_t         init_posa2 = 0;
uint16_t         init_posh2 = 0;
uint16_t         init_posk2 = 0;
int16_t          prev_vela2 = 0;
int16_t          prev_velh2 = 0;
int16_t          prev_velk2 = 0;
int              cnta2 = 0; 
int              cnth2 = 0; 
int              cntk2 = 0;

float            init_angle_abad1, init_angle_abad2;
float            init_angle_hip1, init_angle_hip2;
float            init_angle_knee1, init_angle_knee2;

float            f = 0.1;
float            Ts = 0.002;

void control();

void pack_cmd(CANMessage * msg, joint_control joint){
    msg->data[0] = 0xA1;
    for (int i = 0; i < 2; i++){
        msg->data[i+1] = 0x00;
    }
    msg->data[3] = 0x00;
    msg->data[4] = joint.tau.buffer[0];
    msg->data[5] = joint.tau.buffer[1];
}

void save_init_data(CANMessage rx, leg_state_init * leg){
    if(rx.id == 0x141){
        for(int i=0; i<2; i++){
            leg->a.p.buffer[i] = rx.data[i+6];
        }
        leg->a.p.data = leg->a.p.data >> 2;
    }
    else if(rx.id == 0x142){
        for(int i=0; i<2; i++){
            leg->h.p.buffer[i] = rx.data[i+6];
        }
        leg->h.p.data = leg->h.p.data >> 2;
    }
    else if(rx.id == 0x143){
        for(int i=0; i<2; i++){
            leg->k.p.buffer[i] = rx.data[i+6];
        }
        leg->k.p.data = leg->k.p.data >> 2;
    }
}

void countEncoder(){
	if ((l1_state.a.p.data < 1500) && (prev_posa > 14000)){
		cnta += 1;
	}
	else if ((prev_posa < 1500) && (l1_state.a.p.data > 14000)){
		cnta -= 1;
	}
	if ((l1_state.h.p.data < 1500) && (prev_posh > 14000)){
		cnth += 1;
	}
	else if ((prev_posh < 1500) && (l1_state.h.p.data > 14000)){
		cnth -= 1;
	}
	if ((l1_state.k.p.data < 1500) && (prev_posk > 14000)){
		cntk += 1;
	}
	else if ((prev_posk < 1500) && (l1_state.k.p.data > 14000)){
		cntk -= 1;
	}    
}

void countEncoder2(){
    if ((l2_state.a.p.data < 1500) && (prev_posa2 > 14000)){
		cnta2 += 1;
	}
	else if ((prev_posa2 < 1500) && (l2_state.a.p.data > 14000)){
		cnta2 -= 1;
	}
	if ((l2_state.h.p.data < 1500) && (prev_posh2 > 14000)){
		cnth2 += 1;
	}
	else if ((prev_posh2 < 1500) && (l2_state.h.p.data > 14000)){
		cnth2 -= 1;
	}
	if ((l2_state.k.p.data < 1500) && (prev_posk2 > 14000)){
		cntk2 += 1;
	}
	else if ((prev_posk2 < 1500) && (l2_state.k.p.data > 14000)){
		cntk2 -= 1;
	}   
}

float lowerbound(float angle){
    while(angle < -2*PI){
        angle += 2*PI;
    }
    return angle;
}

float upperbound(float angle){
    while(angle > 2*PI){
        angle -= 2*PI;
    }
    return angle;
}

void unpack_reply(CANMessage msg, leg_state* leg, int state){
    if (state == 1){
        if (counter == 1){
            init_angle_abad1 = ((float) l1_state_init.a.angle.data)*0.001*(PI/180);
            init_angle_hip1 = ((float) l1_state_init.h.angle.data)*0.001*(PI/180);
            init_angle_knee1 = ((float) l1_state_init.k.angle.data)*0.001*(PI/180);
        }
        else if (counter > 1){
            prev_posa = leg->a.p.data;
            prev_vela = leg->a.v.data;
            prev_posh = leg->h.p.data;
            prev_velh = leg->h.v.data;
            prev_posk = leg->k.p.data;
            prev_velk = leg->k.v.data;

            if(msg.id == 0x141){
                for(int i = 0; i < 2; i++){
                    leg->a.p.buffer[i] = msg.data[i+6];
                    leg->a.v.buffer[i] = msg.data[i+4];
                    leg->a.t.buffer[i] = msg.data[i+2];
                }
                leg->a.p.data = leg->a.p.data >> 2;     // 14 bit encoder value
                if (leg->a.p.data == 0){
                    leg->a.p.data = prev_posa;
                }
                if ((leg->a.v.data == 1) | (leg->a.v.data == 2)){
                    leg->a.v.data = prev_vela;
                }
            }
            else if(msg.id == 0x142){
                for(int i = 0; i < 2; i++){
                    leg->h.p.buffer[i] = msg.data[i+6];
                    leg->h.v.buffer[i] = msg.data[i+4];
                    leg->h.t.buffer[i] = msg.data[i+2];
                }
                leg->h.p.data = leg->h.p.data >> 2;     // 14 bit encoder value
                if (leg->h.p.data == 0){
                    leg->h.p.data = prev_posh;
                }
                if ((leg->h.v.data == 1) | (leg->h.v.data == 2)){
                    leg->h.v.data = prev_velh;
                }
            }
            else if(msg.id == 0x143){
                for(int i = 0; i < 2; i++){
                    leg->k.p.buffer[i] = msg.data[i+6];
                    leg->k.v.buffer[i] = msg.data[i+4];
                    leg->k.t.buffer[i] = msg.data[i+2];
                }
                leg->k.p.data = leg->k.p.data >> 2;     // 14 bit encoder value
                if (leg->k.p.data == 0){
                    leg->k.p.data = prev_posk;
                }
                if ((leg->k.v.data == 1) | (leg->k.v.data == 2)){
                    leg->k.v.data = prev_velk;
                }
            }
        }
        countEncoder();
        leg->a.angle = upperbound(lowerbound(init_angle_abad1 + fmod(((leg->a.p.data - l1_state_init.a.p.data) + 16383*cnta)*2*PI/163840, 4*PI)));
        leg->h.angle = upperbound(lowerbound(init_angle_hip1 + fmod(((leg->h.p.data - l1_state_init.h.p.data) + 16383*cnth)*2*PI/163840, 4*PI)));
        leg->k.angle = upperbound(lowerbound(init_angle_knee1 + fmod(((leg->k.p.data - l1_state_init.k.p.data) + 16383*cntk)*2*PI/163840, 4*PI)));
    }
    else if (state == 2){
        if (counter == 1){
            init_angle_abad2 = ((float) l2_state_init.a.angle.data)*0.001*(PI/180);
            init_angle_hip2 = ((float) l2_state_init.h.angle.data)*0.001*(PI/180);
            init_angle_knee2 = ((float) l2_state_init.k.angle.data)*0.001*(PI/180);
        }
        else if(counter > 1){
            prev_posa2 = leg->a.p.data;
            prev_vela2 = leg->a.v.data;
            prev_posh2 = leg->h.p.data;
            prev_velh2 = leg->h.v.data;
            prev_posk2 = leg->k.p.data;
            prev_velk2 = leg->k.v.data;

            if(msg.id == 0x141){
                for(int i = 0; i < 2; i++){
                    leg->a.p.buffer[i] = msg.data[i+6];
                    leg->a.v.buffer[i] = msg.data[i+4];
                    leg->a.t.buffer[i] = msg.data[i+2];
                }
                leg->a.p.data = leg->a.p.data >> 2;     // 14 bit encoder value
                if (leg->a.p.data == 0){
                    leg->a.p.data = prev_posa2;
                }
                if ((leg->a.v.data == 1) | (leg->a.v.data == 2)){
                    leg->a.v.data = prev_vela2;
                }
            }
            else if(msg.id == 0x142){
                for(int i = 0; i < 2; i++){
                    leg->h.p.buffer[i] = msg.data[i+6];
                    leg->h.v.buffer[i] = msg.data[i+4];
                    leg->h.t.buffer[i] = msg.data[i+2];
                }
                leg->h.p.data = leg->h.p.data >> 2;     // 14 bit encoder value
                if (leg->h.p.data == 0){
                    leg->h.p.data = prev_posh2;
                }
                if ((leg->h.v.data == 1) | (leg->h.v.data == 2)){
                    leg->h.v.data = prev_velh2;
                }
            }
            else if(msg.id == 0x143){
                for(int i = 0; i < 2; i++){
                    leg->k.p.buffer[i] = msg.data[i+6];
                    leg->k.v.buffer[i] = msg.data[i+4];
                    leg->k.t.buffer[i] = msg.data[i+2];
                }
                leg->k.p.data = leg->k.p.data >> 2;     // 14 bit encoder value
                if (leg->k.p.data == 0){
                    leg->k.p.data = prev_posk2;
                }
                if ((leg->k.v.data == 1) | (leg->k.v.data == 2)){
                    leg->k.v.data = prev_velk2;
                }
            }
        }
        countEncoder2();
        leg->a.angle = upperbound(lowerbound(init_angle_abad2 + fmod(((leg->a.p.data - l2_state_init.a.p.data) + 16383*cnta2)*2*PI/163840, 4*PI)));
        leg->h.angle = upperbound(lowerbound(init_angle_hip2 + fmod(((leg->h.p.data - l2_state_init.h.p.data) + 16383*cnth2)*2*PI/163840, 4*PI)));
        leg->k.angle = upperbound(lowerbound(init_angle_knee2 + fmod(((leg->k.p.data - l2_state_init.k.p.data) + 16383*cntk2)*2*PI/163840, 4*PI)));
    }

    leg->a.vel = ((float) leg->a.v.data)*(PI/1800);
    leg->h.vel = ((float) leg->h.v.data)*(PI/1800);
    leg->k.vel = ((float) leg->k.v.data)*(PI/1800);
}

void PackAll(){
    pack_cmd(&a1_can, l1_control.a);
    pack_cmd(&a2_can, l2_control.a);
    pack_cmd(&h1_can, l1_control.h);
    pack_cmd(&h2_can, l2_control.h);
    pack_cmd(&k1_can, l1_control.k);
    pack_cmd(&k2_can, l2_control.k);
}

void WriteAll(){
    can1.write(a1_can);
    wait(.00001);
    can2.write(a2_can);
    wait(.00001);
    can1.write(h1_can);
    wait(.00001);
    can2.write(h2_can);
    wait(.00001);
    can1.write(k1_can);
    wait(.00001);
    can2.write(k2_can);
    wait(.00001);
}

void Zero(CANMessage * msg){
    for(int i = 0; i < 7; i++){
        msg->data[i] = 0xFF;
    }
    msg->data[7] = 0xFE;
    WriteAll();
}

void EnterMotorMode(CANMessage * msg){
    for(int i = 0; i < 7; i++){
        msg->data[i] = 0xFF;
    }
    msg->data[7] = 0xFC;
    WriteAll();
}

void ExitMotorMode(CANMessage * msg){
    for(int i = 0; i < 7; i++){
        msg->data[i] = 0xFF;
    }
    msg->data[7] = 0xFD;
    WriteAll();
}

int16_t pdcontroller(float Kp, float Kd, float angle, float vel, float tar_q, float tar_qd, float tau_ff){
	float poerror;
	float verror;
	Packet input;

	poerror = (upperbound(lowerbound(tar_q)) - angle);
	verror = (tar_qd - vel);
    input.data = Kp*poerror + Kd*verror + tau_ff;

    if (input.data > 1500) {
		input.data = 1500;
	}
	else if (input.data < -1500){
		input.data = -1500;
	}
	else{
		input.data = input.data;
	}

    return input.data;
}


void serial_isr(){
    while(pc.readable()){
        char c = pc.getc();
        switch(c){
            case(27):
                printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&a1_can);
                ExitMotorMode(&a2_can);
                ExitMotorMode(&h1_can);
                ExitMotorMode(&h2_can);
                ExitMotorMode(&k1_can);
                ExitMotorMode(&k2_can);
                enabled = 0;
                break;
            case('m'):
                printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&a2_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&h2_can);
                EnterMotorMode(&k1_can);
                EnterMotorMode(&k2_can);
                wait(.5);
                enabled = 1;
                break;
            case('s'):
                printf("\n\r standing \n\r");
                break;
            case('z'):
                printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&a2_can);
                Zero(&h1_can);
                Zero(&h2_can);
                Zero(&k1_can);
                Zero(&k2_can);
                break;
        }
    }
}

uint32_t xor_checksum(uint32_t* data, size_t len){
    uint32_t t = 0;
    for(int i = 0; i < len; i++){
        t = t ^ data[i];
    }
    return t;
}

void spi_isr(void){
    GPIOC->ODR |= (1 << 8);
    GPIOC->ODR &= ~(1 << 8);
    int bytecount = 0;
    SPI1->DR = tx_buff[0];
    while(cs.read() == 0){
        if(SPI1->SR & 0x1){
            rx_buff[bytecount] = SPI1->DR;
            bytecount++;
            if(bytecount < TX_LEN){
                SPI1->DR = tx_buff[bytecount];
            }
        }
    }

    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff, 32);
    for(int i = 0; i < CMD_LEN; i++){
        ((uint16_t*)(&spi_command))[i] = rx_buff[i];
    }

    if(calc_checksum != spi_command.checksum){
        spi_data.flags[1] = 0xdead;
    }

    control();
    PackAll();
    WriteAll();
}

// void convertcommand(){
//     spi_command.q_des_abad[0] += 2*PI;
//     spi_command.q_des_hip[0] += 2*PI;
//     spi_command.q_des_knee[0] += 2*PI;

//     spi_command.q_des_abad[1] += 2*PI;
//     spi_command.q_des_hip[1] += 2*PI;
//     spi_command.q_des_knee[1] += 2*PI;
// }

// void convertdata(){
//     spi_data.q_abad[0] -= 2*PI;
//     spi_data.q_hip[0] -= 2*PI;
//     spi_data.q_knee[0] -= 2*PI;

//     spi_data.q_abad[1] -= 2*PI;
//     spi_data.q_hip[1] -= 2*PI;
//     spi_data.q_knee[1] -= 2*PI;
// }

void control(){
    if(((spi_command.flags[0] & 0x1) == 1) && (enabled == 0)){
        enabled = 1;
        EnterMotorMode(&a1_can);
        can1.write(a1_can);
        EnterMotorMode(&a2_can);
        can2.write(a2_can);
        EnterMotorMode(&h1_can);
        can1.write(h1_can);
        EnterMotorMode(&h2_can);
        can2.write(h2_can);
        EnterMotorMode(&k1_can);
        can1.write(k1_can);
        EnterMotorMode(&k2_can);
        can2.write(k2_can);
        printf("e\n\r");
        return;
    }
    else if(((spi_command.flags[0] & 0x1) == 0) && (enabled == 1)){
        enabled = 0;
        ExitMotorMode(&a1_can);
        can1.write(a1_can);
        ExitMotorMode(&a2_can);
        can2.write(a2_can);
        ExitMotorMode(&h1_can);
        can1.write(h1_can);
        ExitMotorMode(&h2_can);
        can2.write(h2_can);
        ExitMotorMode(&k1_can);
        can1.write(k1_can);
        ExitMotorMode(&k2_can);
        can2.write(k2_can);
        printf("x\n\r");
        return;
    }

    spi_data.q_abad[0] = l1_state.a.angle;
    spi_data.q_hip[0] = l1_state.h.angle;
    spi_data.q_knee[0] = l1_state.k.angle;
    spi_data.qd_abad[0] = l1_state.a.vel;
    spi_data.qd_hip[0] = l1_state.h.vel;
    spi_data.qd_knee[0] = l1_state.k.vel;

    spi_data.q_abad[1] = l2_state.a.angle;
    spi_data.q_hip[1] = l2_state.h.angle;
    spi_data.q_knee[1] = l2_state.k.angle;
    spi_data.qd_abad[1] = l2_state.a.vel;
    spi_data.qd_hip[1] = l2_state.h.vel;
    spi_data.qd_knee[1] = l2_state.k.vel;

    memset(&l1_control, 0, sizeof(l1_control));
    memset(&l2_control, 0, sizeof(l2_control));

    l1_control.a.tau.data = pdcontroller(spi_command.kp_abad[0], spi_command.kd_abad[0], l1_state.a.angle, l1_state.a.vel, spi_command.q_des_abad[0], spi_command.qd_des_abad[0], spi_command.tau_abad_ff[0]);
    l1_control.h.tau.data = pdcontroller(spi_command.kp_hip[0], spi_command.kd_hip[0], l1_state.h.angle, l1_state.h.vel, spi_command.q_des_hip[0], spi_command.qd_des_hip[0], spi_command.tau_hip_ff[0]);
    l1_control.k.tau.data = pdcontroller(spi_command.kp_knee[0], spi_command.kd_knee[0], l1_state.k.angle, l1_state.k.vel, spi_command.q_des_knee[0], spi_command.qd_des_knee[0], spi_command.tau_knee_ff[0]);

    l2_control.a.tau.data = pdcontroller(spi_command.kp_abad[1], spi_command.kd_abad[1], l2_state.a.angle, l2_state.a.vel, spi_command.q_des_abad[1], spi_command.qd_des_abad[1], spi_command.tau_abad_ff[1]);
    l2_control.h.tau.data = pdcontroller(spi_command.kp_hip[1], spi_command.kd_hip[1], l2_state.h.angle, l2_state.h.vel, spi_command.q_des_hip[1], spi_command.qd_des_hip[1], spi_command.tau_hip_ff[1]);
    l2_control.k.tau.data = pdcontroller(spi_command.kp_knee[1], spi_command.kd_knee[1], l2_state.k.angle, l2_state.k.vel, spi_command.q_des_knee[1], spi_command.qd_des_knee[1], spi_command.tau_knee_ff[1]);

    spi_data.flags[0] = 0;
    spi_data.flags[1] = 0;
    spi_data.checksum = xor_checksum((uint32_t*)&spi_data, TX_LEN);
    if(init_enabled == 0 | init_enabled == 2){
        for(int i = 0; i < DATA_LEN; i++){
            tx_buff[i] = ((uint16_t*)(&spi_data))[i];
        }
    }
    else{
        init_enabled = 2;
    }
}

void getInitAngle(CANMessage * msg){
    msg->data[0] = 0x94;
    for(int i = 1; i < 8; i++){
        msg->data[i] = 0x00;
    }
}

void unpack_InitAngle(CANMessage msg, leg_state_init * leg){
    if(msg.id == 0x141){
        for(int i = 0; i < 4; i++){
            leg->a.angle.buffer[i] = msg.data[i+4];
        }
    }
    else if(msg.id == 0x142){
        for(int i = 0; i < 4; i++){
            leg->h.angle.buffer[i] = msg.data[i+4];
        }
    }
    else if(msg.id == 0x143){
        for(int i = 0; i < 4; i++){
            leg->k.angle.buffer[i] = msg.data[i+4];
        }
    }
}

void init_spi(void){
    SPISlave *spi = new SPISlave(PA_7, PA_6, PA_5, PA_4);
    spi->format(16, 0);
    spi->frequency(12000000);
    spi->reply(0x0);
    cs.fall(&spi_isr);
    cnta = 0;
    cnth = 0;
    cntk = 0;
    cnta2 = 0;
    cnth2 = 0;
    cntk2 = 0;
    printf("done\n\r");
}

void angleinit(CANMessage tx, CANMessage rx, leg_state_init *leg){
    getInitAngle(&tx);
    can1.write(tx);
    wait(.0005);
    can1.read(rx);
    unpack_InitAngle(rx, leg);
    wait(.0005);
    init_enabled = 1;
}

void angleinit2(CANMessage tx, CANMessage rx, leg_state_init *leg){
    getInitAngle(&tx);
    can2.write(tx);
    wait(.0005);
    can2.read(rx);
    unpack_InitAngle(rx, leg);
    wait(.0005);
}

void posinit(CANMessage tx, joint_control joint){
    pack_cmd(&tx, joint);
    can1.write(tx);
    wait(.0005);
    can1.read(rxMsg1);
    save_init_data(rxMsg1, &l1_state_init);
}

void posinit2(CANMessage tx, joint_control joint){
    pack_cmd(&tx, joint);
    can2.write(tx);
    wait(.0005);
    can2.read(rxMsg2);
    save_init_data(rxMsg2, &l2_state_init);
}

int main(){
    pc.baud(921600);
    pc.attach(&serial_isr);

    can1.filter(0x0, 0x00, CANStandard, 0);
    can2.filter(0x0, 0x00, CANStandard, 0);

    memset(&tx_buff, 0, TX_LEN*sizeof(uint16_t));
    memset(&spi_data, 0, sizeof(spi_data_t));
    memset(&spi_command, 0, sizeof(spi_command_t));

    NVIC_SetPriority(TIM5_IRQn, 1);

    printf("\n\r SPIne \n\r");

    a1_can.len = 8;
    a1_can.id = 0x141;
    
    h1_can.len = 8;
    h1_can.id = 0x142;
    
    k1_can.len = 8;
    k1_can.id = 0x143;

    a2_can.len = 8;
    a2_can.id = 0x141;
    
    h2_can.len = 8;
    h2_can.id = 0x142;
    
    k2_can.len = 8;
    k2_can.id = 0x143;

    rxMsg1.len = 8;
    rxMsg2.len = 8;
    
    angleinit(a1_can, rxMsg1, &l1_state_init);
    angleinit2(a2_can, rxMsg2, &l2_state_init);
    angleinit(h1_can, rxMsg1, &l1_state_init);
    angleinit2(h2_can, rxMsg2, &l2_state_init);
    angleinit(k1_can, rxMsg1, &l1_state_init);
    angleinit2(k2_can, rxMsg2, &l2_state_init);
    
    l1_control.a.tau.data = 0;
    l1_control.h.tau.data = 0;
    l1_control.k.tau.data = 0;

    l2_control.a.tau.data = 0;
    l2_control.h.tau.data = 0;
    l2_control.k.tau.data = 0;
    
    posinit(a1_can, l1_control.a);
    posinit2(a2_can, l2_control.a);
    posinit(h1_can, l1_control.h);    
    posinit2(h2_can, l2_control.h);    
    posinit(k1_can, l1_control.k);
    posinit2(k2_can, l2_control.k);
    
    if(!spi_enabled){
        while((spi_enabled == 0) && (cs.read() == 0)){wait_us(10);}
        init_spi();
        spi_enabled = 1;
    }

    while(1){
        counter++;
        can2.read(rxMsg2);
        unpack_reply(rxMsg2, &l2_state, 2);
        can1.read(rxMsg1);
        unpack_reply(rxMsg1, &l1_state, 1);
        wait_us(20);
    }
}
