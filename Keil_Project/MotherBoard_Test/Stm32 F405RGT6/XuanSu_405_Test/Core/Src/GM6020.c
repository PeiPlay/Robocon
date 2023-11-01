#include "GM6020.h"

static CAN_HandleTypeDef* can_handler = NULL;
static GM6020_Config GM6020_Configs[7] = {0};//�洢�����������Ϣ

uint8_t can_send_buffer[8] = {0};

#define _ABS(x) ((x) > 0 ? (x) : -(x))
#define _LIMIT(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))


HAL_StatusTypeDef GM6020_CAN_Transmit(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID)
{
    return CAN_Transmit_Default(hcan, ID, pData);
}
uint32_t _MotorID2Index(uint32_t motor_id)//�����IDת��Ϊ������
{
    assert_param(motor_id >= 0x205 && motor_id <= 0x20B);
    return motor_id - 0x205;
}
void GM6020_Init(struct _GM6020_PID_Speed* pid_speed, struct _GM6020_PID_Position* pid_position, uint8_t motor_index_from1)
{
    GM6020_Configs[motor_index_from1 - 1].pid_speed = *pid_speed;
    GM6020_Configs[motor_index_from1 - 1].pid_position = *pid_position;
}
void GM6020_LoadCAN(CAN_HandleTypeDef* hcan)
{
    can_handler = hcan;
}
void GM6020_RecvCanFeedback(uint32_t motor_id, uint8_t* pdata)
{
    //������ش��ı������ݽ��������״̬�ṹ����
    //���ID��0x205��ʼ����Ӧmotors������±��0��ʼ

    int index = _MotorID2Index(motor_id);
    GM6020_Configs[index].state.encoder = ((uint16_t)(pdata[0]) << 8) | pdata[1];
    GM6020_Configs[index].state.speed = ((uint16_t)(pdata[2]) << 8) | pdata[3];
    GM6020_Configs[index].state.torque = ((uint16_t)(pdata[4]) << 8) | pdata[5];
    GM6020_Configs[index].state.temperature = pdata[6];



    static int64_t position_base[8] = {0};
    static int32_t last_encoder[8] = {0};

    if (GM6020_Configs[index].state.encoder - last_encoder[index] > 4095) {                 //��ǰ��������ֵ��0ͻ�䵽8191���պ÷�תһȦ
        position_base[index] -= 8191;                                                       //base��һȦ
    } else if (GM6020_Configs[index].state.encoder - last_encoder[index] < -4095) {         //��ǰ��������ֵ��8191ͻ�䵽0���պ���תһȦ
        position_base[index] += 8191;                                                       //base��һȦ
    }

    last_encoder[index] = GM6020_Configs[index].state.encoder;                                                  //������һ�εĽǶ�ֵ
    GM6020_Configs[index].state.encoder_total = position_base[index] + GM6020_Configs[index].state.encoder;     //�ܱ�����ֵ=base+��ǰ������ֵ
}
HAL_StatusTypeDef GM6020_Set_Voltage(int target_v, uint8_t motor_index_from1)
{
    if(can_handler == NULL)
        return HAL_ERROR;

    //���Ƶ�ѹֵ�������Сֵ֮��
    if(target_v > GM6020_VOLTAGE_MAX)        {target_v = GM6020_VOLTAGE_MAX;}
    else if(target_v < GM6020_VOLTAGE_MIN)   {target_v = GM6020_VOLTAGE_MIN;}
    
    
    uint32_t can_send_id = 0;
    if(motor_index_from1 <= 4)
		can_send_id = 0x1ff;
	else
	{
		can_send_id = 0x2ff;
		motor_index_from1 -= 4;
	}
    can_send_buffer[2 * motor_index_from1 - 2] = target_v >> 8;		  //��ѹֵ��8λ
	can_send_buffer[2 * motor_index_from1 - 1] = target_v & 0x00ff;	  //��ѹֵ��8λ

    return GM6020_CAN_Transmit(can_handler, can_send_buffer, can_send_id);
}
HAL_StatusTypeDef GM6020_Set_Speed(int target_speed, uint8_t motor_index_from1)
{
    if(motor_index_from1 > 7 || motor_index_from1 < 1)
        return HAL_ERROR;

    GM6020_Configs[motor_index_from1 - 1].pid_speed.target = target_speed;
    return GM6020_Set_Voltage(
        (int)GM6020_PID_Speed_Calculate(//PID���㺯����ͨ��PIDģ�ͣ�pid_speed���͵�ǰ�ٶȣ�state.speed����������ֵ
            &GM6020_Configs[motor_index_from1 - 1].pid_speed, 
            GM6020_Configs[motor_index_from1 - 1].state.speed), 
        motor_index_from1);
}
HAL_StatusTypeDef GM6020_Set_Position(int target_position, uint8_t motor_index_from1)
{
    if(motor_index_from1 > 7 || motor_index_from1 < 1)
        return HAL_ERROR;

    GM6020_Configs[motor_index_from1 - 1].pid_position.target = target_position;
    return GM6020_Set_Voltage(
        (int)GM6020_PID_Position_Calculate(//PID���㺯����ͨ��PIDģ�ͣ�pid_position���͵�ǰλ�ã�state.encoder_total����������ֵ
            &GM6020_Configs[motor_index_from1 - 1].pid_position, 
            (double)GM6020_Configs[motor_index_from1 - 1].state.encoder_total - (double)GM6020_Configs[motor_index_from1 - 1].state.encoder_offset), 
        motor_index_from1);     
}
HAL_StatusTypeDef GM6020_Set_Position_Offset(int offset, uint8_t motor_index_from1)
{
    if(motor_index_from1 > 7 || motor_index_from1 < 1)
        return HAL_ERROR;

    GM6020_Configs[motor_index_from1 - 1].state.encoder_offset = offset;

    return HAL_OK;
}
HAL_StatusTypeDef GM6020_Set_Position_ZeroPoint(uint8_t motor_index_from1)
{
    return GM6020_Set_Position_Offset(GM6020_Configs[motor_index_from1 - 1].state.encoder_total, motor_index_from1);
}

double GM6020_PID_Speed_Calculate(GM6020_PID_Speed* pid, double measure)
{
    //dt�����Ӽ�ʱ����ɱ�
    pid->error_curr = pid->target - measure;
    
    pid->derivative = (pid->error_curr - pid->error_last) / pid->dt;

    //�������
	//�����������趨ֵ��ƫ���Сʱ�������п��ƣ��Ա�������΢С��ƫ������Ŀ���������
	//���ƫ���Сʱ�����������㣬�������𶶶�
	//���ٽ��п��ƣ�������һ�εĿ��������������𶶶�
    if(_ABS(pid->error_curr) <= pid->integral_dead_error)
    {
        return pid->output;
    }

    //���ַ��룬�������������������������趨ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
	//��ʹ����������ִ�л�������������������Χ��Ӧ�ļ��޿�����������ϵͳ�ϴ�ĳ���
	//����˼·��
	//�������������趨ֵƫ��ϴ�ʱ��ȡ���������ã��Ա������ڻ�������ʹϵͳ�ȶ��Խ��ͣ�����������
	//���������ӽ�����ֵʱ��������ֿ��ƣ��Ա����������߿��ƾ��ȡ�
    if(_ABS(pid->error_curr) <= pid->integral_start_error)
        pid->integral += pid->error_curr * pid->dt;
    else
        pid->integral = 0;


    //�����޷�
    pid->integral = _LIMIT(pid->integral * pid->Ki, pid->integral_min * pid->Ki, pid->integral_max * pid->Ki) / pid->Ki;
    
    //�������
    pid->output = pid->Kp * pid->error_curr + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    pid->output = _LIMIT(pid->output, pid->output_min, pid->output_max);
    //������һ�β���ֵ
    pid->last_measure = measure;
    pid->error_last = pid->error_curr;

    return pid->output;
}
double GM6020_PID_Position_Calculate(GM6020_PID_Position* pid, double measure)
{
    //dt�����Ӽ�ʱ����ɱ�
    pid->error_curr = pid->target - measure;
    
    pid->derivative = (pid->error_curr - pid->error_last) / pid->dt;


    //���ַ��룬�������������������������趨ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
	//��ʹ����������ִ�л�������������������Χ��Ӧ�ļ��޿�����������ϵͳ�ϴ�ĳ���
	//����˼·��
	//�������������趨ֵƫ��ϴ�ʱ��ȡ���������ã��Ա������ڻ�������ʹϵͳ�ȶ��Խ��ͣ�����������
	//���������ӽ�����ֵʱ��������ֿ��ƣ��Ա����������߿��ƾ��ȡ�
    if(_ABS(pid->error_curr) <= pid->integral_start_error)
        pid->integral += pid->error_curr * pid->dt;
    else
        pid->integral /= 1.1;

    if(pid->error_curr == 0 || pid->error_curr * pid->error_last < 0)
    {
        pid->derivative *= 10;
    }
    //�����޷�
    pid->integral = _LIMIT(pid->integral * pid->Ki, pid->integral_min * pid->Ki, pid->integral_max * pid->Ki) / pid->Ki;
    
    //�������
    pid->output = pid->Kp * pid->error_curr + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    pid->output = _LIMIT(pid->output, pid->output_min, pid->output_max);
    //������һ�β���ֵ
    pid->last_measure = measure;
    pid->error_last = pid->error_curr;

    return pid->output;


}


