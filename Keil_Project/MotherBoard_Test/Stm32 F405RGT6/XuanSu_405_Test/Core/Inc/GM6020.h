#ifndef __GM6020_H__
#define __GM6020_H__
#include "main.h"
#include "can_bsp.h"
//�������������
//motor_id: ���ID��ȡֵ��Χ0x205~0x20B
//motor_index_from1: �����ţ�ȡֵ��Χ1~7

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
    int16_t encoder;            // ����Ƕ�
    int16_t speed;              // ���ת��
    int16_t torque;             // ���ת��
    int8_t  temperature;        // ����¶�

    int64_t encoder_total;     // �����������ڵı�������ֵ
    int64_t encoder_offset;    // ���������ƫ��ֵ�����ڽ���ǰ�����λ������Ϊ���

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
    motor_state state;//�洢��ǰ�����״̬
    GM6020_PID_Speed pid_speed;//�ٶȻ�PID
    GM6020_PID_Position pid_position;//λ�û�PID
} GM6020_Config;



 

typedef int64_t count_t;            //����������ֵ����
typedef double  angle_t;            //�Ƕ�ֵ����
typedef double  speed_t;            //�ٶ�ֵ����

HAL_StatusTypeDef GM6020_CAN_Transmit(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID);
void GM6020_LoadCAN(CAN_HandleTypeDef* hcan);

void GM6020_Init(struct _GM6020_PID_Speed* pid_speed, struct _GM6020_PID_Position* pid_position, uint8_t motor_index_from1);

void GM6020_RecvCanFeedback(uint32_t motor_id, uint8_t* pdata);
HAL_StatusTypeDef GM6020_Set_Voltage(int target_v, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Speed(int target_speed, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position(int target_position, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position_Offset(int offset, uint8_t motor_index_from1);
HAL_StatusTypeDef GM6020_Set_Position_ZeroPoint(uint8_t motor_index_from1);//���õ�ǰλ��Ϊ���


double GM6020_PID_Speed_Calculate(GM6020_PID_Speed* pid, double current);
double GM6020_PID_Position_Calculate(GM6020_PID_Position* pid, double current);









#endif // !__GM6020_H__
