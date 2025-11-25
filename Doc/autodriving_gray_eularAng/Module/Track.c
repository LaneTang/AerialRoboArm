
#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "Serial.h"

/**编号车头往外看从右到左
    6 5 4 3 2 1 0
    |————————————|
    |    car`s   |
    |    head    |
    |            |
*/  

uint8_t tubes[7];

void get_Info(void) {   // get level for each tube, if 1 - meet black 
    tubes[0] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_1_PORT, GPIO_SensorTracking_PIN_1_PIN) || 0);
    tubes[1] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_2_PORT, GPIO_SensorTracking_PIN_2_PIN) || 0);
    tubes[2] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_3_PORT, GPIO_SensorTracking_PIN_3_PIN) || 0);  
    tubes[3] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_4_PORT, GPIO_SensorTracking_PIN_4_PIN) || 0);  
    tubes[4] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_5_PORT, GPIO_SensorTracking_PIN_5_PIN) || 0);  
    tubes[5] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_6_PORT, GPIO_SensorTracking_PIN_6_PIN) || 0);  
    tubes[6] = (DL_GPIO_readPins(GPIO_SensorTracking_PIN_7_PORT, GPIO_SensorTracking_PIN_7_PIN) || 0);
}

/** 检测到black色块的通道数
*/
uint8_t meet_black(void) {  
    uint8_t channel_num = 0;
    get_Info();
    // UART_transmitArray(tubes, 7);
    
    for (uint8_t i = 0; i < 7; i++) {
        if (tubes[i])   
            channel_num += 1;
    }
    return channel_num;
}

// 跟着线走
const uint16_t tinySpeed = 100;
const uint16_t lowSpeed = 150;
const uint16_t midSpeed = 200;
const uint16_t highSpeed = 250;

void follow_blackline_singleChannel(uint8_t meetTubes) {
    if (meetTubes == 1){
        if (tubes[3]) {               // 中间 检测到black
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[4]) {          // 左1 检测到black - 小偏右
            L_Wheel_SetSpeed(midSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[5]) {          // 左2 检测到black - 中偏右
            L_Wheel_SetSpeed(lowSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[6]) {          // 左3 检测到black - 大偏右
            L_Wheel_SetSpeed(tinySpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[2]) {          // 右1 检测到black - 小偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(midSpeed);
        }
        else if (tubes[1]) {          // 右2 检测到black - 中偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(lowSpeed);
        }
        else if (tubes[0]) {          // 右3 检测到black - 大偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(tinySpeed);
        }
    }
    

}


void follow_blackline_multiChannel(uint8_t meetTubes) {
    if (meetTubes == 1){
        if (tubes[3]) {                // 中间 检测到black
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[4]) {           // 左1 检测到black - 小偏右
            L_Wheel_SetSpeed(midSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[5]) {           // 左2 检测到black - 中偏右
            L_Wheel_SetSpeed(lowSpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[6]) {           // 左3 检测到black - 大偏右
            L_Wheel_SetSpeed(tinySpeed);
            R_Wheel_SetSpeed(highSpeed);
        }
        else if (tubes[2]) {           // 右1 检测到black - 小偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(midSpeed);
        }
        else if (tubes[1]) {           // 右2 检测到black - 中偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(lowSpeed);
        }
        else if (tubes[0]) {           // 右3 检测到black - 大偏左
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(tinySpeed);
        }
    } else if (meetTubes > 1) {
        if (tubes[3] && (tubes[2] || tubes[4])) {  // 中间和两边有黑线
            L_Wheel_SetSpeed(midSpeed);
            R_Wheel_SetSpeed(midSpeed);
        } else if (tubes[4] && (tubes[5] || tubes[6])) {  // 左1和左2或左3有黑线
            L_Wheel_SetSpeed(lowSpeed);
            R_Wheel_SetSpeed(highSpeed);
        } else if (tubes[5] && tubes[6]) {  // 左2和左3有黑线
            L_Wheel_SetSpeed(tinySpeed);
            R_Wheel_SetSpeed(highSpeed);
        } else if (tubes[2] && (tubes[1] || tubes[0])) {  // 右1和右2或右3有黑线
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(lowSpeed);
        } else if (tubes[1] && tubes[0]) {  // 右2和右3有黑线
            L_Wheel_SetSpeed(highSpeed);
            R_Wheel_SetSpeed(tinySpeed);
        }
    } else {
        // 没有检测到黑线的情况，可根据需要停止或继续直行
        L_Wheel_SetSpeed(tinySpeed);
        R_Wheel_SetSpeed(tinySpeed); 
    } 
}

