#include "CAN.h"
#include "DigitalIn.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "can_helper.h"
#include "mbed.h"
#include <cstdint>
#include <cstring>
#include "leg_message.h"

#define RX_LEN  66
#define TX_LEN  66
#define CMD_LEN 66
#define PI      3.1415926535898f

spi_data_t      spi_data;
spi_command_t   spi_command;

uint16_t         rx_buff[RX_LEN];
uint16_t         tx_buff[TX_LEN];

Serial          pc(PA_2, PA_3);

CAN             can1(PB_12, PB_13, 1000000);
CANMessage      rxMsg1;
CANMessage      a1_can, h1_can, k1_can;

int             counter = 0;
int             enabled = 0;
int             init_enabled = 0;
int             bytecount = 0;

int             spi_enabled = 0;
InterruptIn     cs(PA_4);

leg_state       l1_state;
leg_state_init  l1_state_init;
leg_control     l1_control;

uint16_t        prev_posa = 0;
uint16_t        prev_posh = 0;
uint16_t        prev_posk = 0;
uint16_t        init_posa = 0;
uint16_t        init_posh = 0;
uint16_t        init_posk = 0;
int16_t         prev_vela = 0;
int16_t         prev_velh = 0;
int16_t         prev_velk = 0;
Packet3         init_pos1, init_pos2, init_pos3;
int             cnta = 0; 
int             cnth = 0; 
int             cntk = 0;

float           init_angle_abad;
float           init_angle_hip;
float           init_angle_knee;

float           f = 0.1;
float           Ts = 0.002;

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
    }
    else if(rx.id == 0x142){
        for(int i=0; i<2; i++){
            leg->h.p.buffer[i] = rx.data[i+6];
        }
    }
    else if(rx.id == 0x143){
        for(int i=0; i<2; i++){
            leg->k.p.buffer[i] = rx.data[i+6];
        }
    }
    leg->a.p.data = leg->a.p.data >> 2;
    leg->h.p.data = leg->h.p.data >> 2;
    leg->k.p.data = leg->k.p.data >> 2;
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

float lowerbound(float angle){
    while(angle < 0){
        angle += 2*PI;
    }
    return angle;
}

float upperbound(float angle){
    while(angle > 4*PI){
        angle -= 2*PI;
    }
    return angle;
}

void unpack_reply(CANMessage msg, leg_state* leg){
    if(counter == 1){
        init_angle_abad = ((float) l1_state_init.a.angle.data)*0.001*(PI/180);
        init_angle_hip = ((float) l1_state_init.h.angle.data)*0.001*(PI/180);
        init_angle_knee = ((float) l1_state_init.k.angle.data)*0.001*(PI/180);
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

        if(init_enabled == 1){
            l1_state_init.a.p.data = leg->a.p.data;
            l1_state_init.h.p.data = leg->h.p.data;
            l1_state_init.k.p.data = leg->k.p.data;
        }
    }
    countEncoder();
    leg->a.angle = upperbound(lowerbound(init_angle_abad + fmod(((leg->a.p.data - l1_state_init.a.p.data) + 16383*cnta)*2*PI/163840, 4*PI)));
    leg->h.angle = upperbound(lowerbound(init_angle_hip + fmod(((leg->h.p.data - l1_state_init.h.p.data) + 16383*cnth)*2*PI/163840, 4*PI)));
    leg->k.angle = upperbound(lowerbound(init_angle_knee + fmod(((leg->k.p.data - l1_state_init.k.p.data) + 16383*cntk)*2*PI/163840, 4*PI)));

    leg->a.vel = ((float) leg->a.v.data)*(PI/1800);
    leg->h.vel = ((float) leg->h.v.data)*(PI/1800);
    leg->k.vel = ((float) leg->k.v.data)*(PI/1800);
}

void PackAll(){
    pack_cmd(&a1_can, l1_control.a);
    pack_cmd(&h1_can, l1_control.h);
    pack_cmd(&k1_can, l1_control.k);
}

void WriteAll(){
    can1.write(a1_can);
    wait(.00002);
    can1.write(h1_can);
    wait(.00002);
    can1.write(k1_can);
    wait(.00002);
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

int16_t pdcontroller(float Kp, float Kd, float angle, float vel, float tar_q, float tar_qd, float tau_ext){
	float poerror;
	float verror;
	float tau_ff;
	Packet input;

	poerror = (upperbound(lowerbound(tar_q)) - angle);
	verror = (tar_qd - vel);
	// tau_ff = -0.0056680*pow(2*PI*f,2)*tar_q*625/14;
	input.data = Kp*poerror + Kd*verror;// + tau_ff + tau_ext;
    // input.data = 0;

    if (input.data > 2000) {
		input.data = 1999;
	}
	else if (input.data < -2000){
		input.data = -1999;
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
                ExitMotorMode(&h1_can);
                ExitMotorMode(&k1_can);
                enabled = 0;
                break;
            case('m'):
                printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&k1_can);
                wait(.5);
                enabled = 1;
                break;
            case('s'):
                printf("\n\r standing \n\r");
                break;
            case('z'):
                printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&h1_can);
                Zero(&k1_can);
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
    
    // for (int i = 0; i < TX_LEN; i++){
    //     printf("Send Message[%d]: %d \r\n", i, tx_buff[i]);
    // }

    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff, 32);
    for(int i = 0; i < CMD_LEN; i++){
        if (i+1 < CMD_LEN){
            ((uint16_t*)(&spi_command))[i] = rx_buff[i+1];
        }
    }

    if(calc_checksum != spi_command.checksum){
        spi_data.flags = 0xdead;
    }

    control();
    PackAll();
    WriteAll();
}

void convertcommand(){
    spi_command.q_des_abad[0] += 2*PI;
    spi_command.q_des_hip[0] += 2*PI;
    spi_command.q_des_knee[0] += 2*PI;
}

// void convertdata(){
//     spi_data.q_abad[0] -= 2*PI;
//     spi_data.q_hip[0] -= 2*PI;
//     spi_data.q_knee[0] -= 2*PI;
// }

void control(){
    if(((spi_command.flags[0] & 0x1) == 1) && (enabled == 0)){
        enabled = 1;
        EnterMotorMode(&a1_can);
        can1.write(a1_can);
        EnterMotorMode(&h1_can);
        can1.write(h1_can);
        EnterMotorMode(&k1_can);
        can1.write(k1_can);
        printf("e\n\r");
        return;
    }
    else if(((spi_command.flags[0] & 0x1) == 0) && (enabled == 1)){
        enabled = 0;
        ExitMotorMode(&a1_can);
        can1.write(a1_can);
        ExitMotorMode(&h1_can);
        can1.write(h1_can);
        ExitMotorMode(&k1_can);
        can1.write(k1_can);
        printf("x\n\r");
        return;
    }

    spi_data.q_abad.data = l1_state.a.p.data;
    spi_data.q_hip.data = l1_state.h.p.data;
    spi_data.q_knee.data = l1_state.k.p.data;
    spi_data.qd_abad.data = l1_state.a.v.data;
    spi_data.qd_hip.data = l1_state.h.v.data;
    spi_data.qd_knee.data = l1_state.k.v.data;

    // spi_data.q_abad[0] = l1_state.a.angle;
    // spi_data.q_hip[0] = l1_state.h.angle;
    // spi_data.q_knee[0] = l1_state.k.angle;

    memset(&l1_control, 0, sizeof(l1_control));

    for (int i = 0; i < 2; i++){
        printf("Ab/Ad Target: %.2f \n\r", spi_command.q_des_abad[i]);
        printf("Hip Target: %.2f \n\r", spi_command.q_des_hip[i]);
        printf("Knee Target: %.2f \n\r", spi_command.q_des_knee[i]);
        printf("Kp of Ab/AD: %.2f \n\r", spi_command.kp_abad[i]);
        printf("Kd of Ab/AD: %.2f \n\r", spi_command.kd_abad[i]);
        printf("Kp of Hip: %.2f \n\r", spi_command.kp_hip[i]);
        printf("Kd of Hip: %.2f \n\r", spi_command.kd_hip[i]);
        printf("Kp of Knee: %.2f \n\r", spi_command.kp_knee[i]);
        printf("Kd of Knee: %.2f \n\r", spi_command.kd_knee[i]);
    }
    // for (int i = 0; i < 132; i++){
    //     printf("Read[%d]: %u \r\n", i, ((uint16_t*)(&spi_command))[i]);
    // }
    // printf("Kp of Ab/Ad: %.2f \n\r", spi_command.kp_abad[0]);
    
    // printf("Ab/Ad Angle: %f \n\r", l1_state.a.angle);
    // // printf("CNT: %d \r\n", cnth);
    // printf("Hip Angle: %f \n\r", l1_state.h.angle);
    // printf("Knee Angle: %f \n\r", l1_state.k.angle);

    printf("--------Finish--------\n\r");

    convertcommand();

    l1_control.a.tau.data = pdcontroller(spi_command.kp_abad[0], spi_command.kd_abad[0], l1_state.a.angle, l1_state.a.vel, spi_command.q_des_abad[0], spi_command.qd_des_abad[0], spi_command.tau_abad_ff[0]);
    l1_control.h.tau.data = pdcontroller(spi_command.kp_hip[0], spi_command.kd_hip[0], l1_state.h.angle, l1_state.h.vel, spi_command.q_des_hip[0], spi_command.qd_des_hip[0], spi_command.tau_hip_ff[0]);
    l1_control.k.tau.data = pdcontroller(spi_command.kp_knee[0], spi_command.kd_knee[0], l1_state.k.angle, l1_state.k.vel, spi_command.q_des_knee[0], spi_command.qd_des_knee[0], spi_command.tau_knee_ff[0]);

    spi_data.flags = 0;
    spi_data.checksum = xor_checksum((uint32_t*)&spi_data, TX_LEN);
    if(init_enabled == 0 | init_enabled == 2){
        for(int i = 0; i < 2; i++){
            tx_buff[i] = spi_data.q_abad.buffer[i];
            tx_buff[i+2] = spi_data.qd_abad.buffer[i];
            tx_buff[i+4] = spi_data.q_hip.buffer[i];
            tx_buff[i+6] = spi_data.qd_hip.buffer[i];
            tx_buff[i+8] = spi_data.q_knee.buffer[i];
            tx_buff[i+10] = spi_data.qd_knee.buffer[i];
        }
        if (init_enabled == 2){
            tx_buff[12] = init_enabled;
            init_enabled = 0;
        }
        else{
            tx_buff[12] = init_enabled;
        }
    }
    else{
        for (int i = 0; i < 4; i++){
            tx_buff[i] = l1_state_init.a.angle.buffer[i];
            tx_buff[i+4] = l1_state_init.h.angle.buffer[i];
            tx_buff[i+8] = l1_state_init.k.angle.buffer[i];
        }
        tx_buff[12] = init_enabled;
        init_enabled = 2;
    }
    // for(int i = 0; i < TX_LEN; i++){
    //     // tx_buff[i] = ((uint16_t*)(&spi_data))[i];
    //     tx_buff[i] = 0x10+i;
    // }
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
    spi->frequency(90000000);
    spi->reply(0x0);
    cnta = 0;
    cnth = 0;
    cntk = 0;
    cs.fall(&spi_isr);
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

void posinit(CANMessage tx, joint_control joint){
    pack_cmd(&tx, joint);
    can1.write(tx);
    wait(.0002);
    can1.read(rxMsg1);
    save_init_data(rxMsg1, &l1_state_init);
}

int main(){
    pc.baud(921600);
    pc.attach(&serial_isr);

    can1.filter(0x0, 0x00, CANStandard, 0);

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

    rxMsg1.len = 8;
    
    angleinit(a1_can, rxMsg1, &l1_state_init);
    angleinit(h1_can, rxMsg1, &l1_state_init);
    angleinit(k1_can, rxMsg1, &l1_state_init);
    
    l1_control.a.tau.data = 0;
    l1_control.h.tau.data = 0;
    l1_control.k.tau.data = 0;
    
    posinit(a1_can, l1_control.a);
    posinit(h1_can, l1_control.h);    
    posinit(k1_can, l1_control.k);
    
    if(!spi_enabled){
        while((spi_enabled == 0) && (cs.read() == 0)){wait_us(10);}
        init_spi();
        spi_enabled = 1;
    }

    while(1){
        counter++;
        can1.read(rxMsg1);
        unpack_reply(rxMsg1, &l1_state);
        wait_us(10);
    }
}
