#ifndef __GM6020_H__
#define __GM6020_H__
#include "main.h"
#include "can_bsp.h"
//各项参数的意义
//motor_id: 电机ID，取值范围0x205~0x20B
//motor_index_from1: 电机编号，取值范围1~7

#define GM6020_VOLTAGE_MAX  30000
#define GM6020_VOLTAGE_MIN  -30000




typedef struct _GM6020_PID_Speed
{
    double target;
    double error_curr; 
    double error_last; 
    double dt;
    double Kp, Ki, Kd; 
    double derivative;
    double integral;
    double integral_max, integral_min;
    double integral_start_error;
    double integral_dead_error;
    double output; 
    double output_max, output_min; 

    double last_measure;
} GM6020_PID_Speed;

typedef struct _GM6020_PID_Position
{
    double target;
    double error_curr;
    double error_last;
    double dt;
    double Kp, Ki, Kd;
    double derivative;
    double integral;
    double integral_max, integral_min;
    double integral_start_error;
    double integral_dead_error;
    double output;
    double output_max, output_min;

    double last_measure;
} GM6020_PID_Position;

typedef struct _motor_state
{
    int16_t encoder;            // 电机角度
    int16_t speed;              // 电机转速
    int16_t torque;             // 电机转矩
    int8_t  temperature;        // 电机温度

    int64_t encoder_total;     // 从启动到现在的编码器总值
    int64_t encoder_offset;    // 编码器零点偏移值，用于将当前电机的位置设置为零点

} motor_state;

typedef struct _GM6020_InitTypeDef
{
    uint8_t motor_index_from1;
    GM6020_PID_Speed pid_speed;
    GM6020_PID_Position pid_position;
    CAN_HandleTypeDef* hcan;
} GM6020_InitTypeDef;

typedef struct _GM6020_Config
{
    motor_state state;//存储当前电机的状态
    GM6020_PID_Speed pid_speed;//速度环PID
    GM6020_PID_Position pid_position;//位置环PID
} GM6020_Config;



 

typedef int64_t count_t;            //编码器计数值类型
typedef double  angle_t;            //角度值类型
typedef double  speed_t;            //速度值类型

HAL_StatusTypeDef GM6020_CAN_Transmit(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID);
void GM6020_LoadCAN(CAN_HandleTypeDef* hcan);

void GM6020_Init(struct _GM6020_PID_Speed* pid_speed, struct _GM6020_PID_Position* pid_position, uint8_t motor_index_from1);

void GM6020_RecvCanFeedback(uint32_t motor_id, uint8_t* pdata);
HAL_StatusTypeDef GM6020_Set_Voltage(int target_v, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Speed(int target_speed, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position(int target_position, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position_Offset(int offset, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position_ZeroPoint(uint8_t motor_index_from1);//设置当前位置为零点


double GM6020_PID_Speed_Calculate(GM6020_PID_Speed* pid, double current);
double GM6020_PID_Position_Calculate(GM6020_PID_Position* pid, double current);









#endif // !__GM6020_H__
