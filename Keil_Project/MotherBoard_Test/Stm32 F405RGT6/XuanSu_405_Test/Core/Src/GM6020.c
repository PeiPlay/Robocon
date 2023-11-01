#include "GM6020.h"

static CAN_HandleTypeDef* can_handler = NULL;
static GM6020_Config GM6020_Configs[7] = {0};//存储电机的配置信息

uint8_t can_send_buffer[8] = {0};

#define _ABS(x) ((x) > 0 ? (x) : -(x))
#define _LIMIT(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))


HAL_StatusTypeDef GM6020_CAN_Transmit(CAN_HandleTypeDef* hcan, uint8_t *pData, uint16_t ID)
{
    return CAN_Transmit_Default(hcan, ID, pData);
}
uint32_t _MotorID2Index(uint32_t motor_id)//将电机ID转换为电机编号
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
    //将电机回传的报文数据解析到电机状态结构体中
    //电机ID从0x205开始，对应motors数组的下标从0开始

    int index = _MotorID2Index(motor_id);
    GM6020_Configs[index].state.encoder = ((uint16_t)(pdata[0]) << 8) | pdata[1];
    GM6020_Configs[index].state.speed = ((uint16_t)(pdata[2]) << 8) | pdata[3];
    GM6020_Configs[index].state.torque = ((uint16_t)(pdata[4]) << 8) | pdata[5];
    GM6020_Configs[index].state.temperature = pdata[6];



    static int64_t position_base[8] = {0};
    static int32_t last_encoder[8] = {0};

    if (GM6020_Configs[index].state.encoder - last_encoder[index] > 4095) {                 //当前编码器数值从0突变到8191，刚好反转一圈
        position_base[index] -= 8191;                                                       //base减一圈
    } else if (GM6020_Configs[index].state.encoder - last_encoder[index] < -4095) {         //当前编码器数值从8191突变到0，刚好正转一圈
        position_base[index] += 8191;                                                       //base加一圈
    }

    last_encoder[index] = GM6020_Configs[index].state.encoder;                                                  //更新上一次的角度值
    GM6020_Configs[index].state.encoder_total = position_base[index] + GM6020_Configs[index].state.encoder;     //总编码器值=base+当前编码器值
}
HAL_StatusTypeDef GM6020_Set_Voltage(int target_v, uint8_t motor_index_from1)
{
    if(can_handler == NULL)
        return HAL_ERROR;

    //限制电压值在最大最小值之间
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
    can_send_buffer[2 * motor_index_from1 - 2] = target_v >> 8;		  //电压值高8位
	can_send_buffer[2 * motor_index_from1 - 1] = target_v & 0x00ff;	  //电压值低8位

    return GM6020_CAN_Transmit(can_handler, can_send_buffer, can_send_id);
}
HAL_StatusTypeDef GM6020_Set_Speed(int target_speed, uint8_t motor_index_from1)
{
    if(motor_index_from1 > 7 || motor_index_from1 < 1)
        return HAL_ERROR;

    GM6020_Configs[motor_index_from1 - 1].pid_speed.target = target_speed;
    return GM6020_Set_Voltage(
        (int)GM6020_PID_Speed_Calculate(//PID计算函数将通过PID模型（pid_speed）和当前速度（state.speed）计算出输出值
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
        (int)GM6020_PID_Position_Calculate(//PID计算函数将通过PID模型（pid_position）和当前位置（state.encoder_total）计算出输出值
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
    //dt在增加计时器后可变
    pid->error_curr = pid->target - measure;
    
    pid->derivative = (pid->error_curr - pid->error_last) / pid->dt;

    //误差死区
	//当被控量与设定值的偏差较小时，不进行控制，以避免由于微小的偏差引起的控制量抖动
	//如果偏差较小时，将积分清零，避免引起抖动
	//不再进行控制，保留上一次的控制量，避免引起抖动
    if(_ABS(pid->error_curr) <= pid->integral_dead_error)
    {
        return pid->output;
    }

    //积分分离，过程启动、结束或大幅度增减设定时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
	//致使控制量超过执行机构可能允许的最大动作范围对应的极限控制量，引起系统较大的超调
	//基本思路：
	//当被控制量与设定值偏差较大时，取消积分作用，以避免由于积分作用使系统稳定性降低，超调量增大；
	//当被控量接近给定值时，引入积分控制，以便消除静差，提高控制精度。
    if(_ABS(pid->error_curr) <= pid->integral_start_error)
        pid->integral += pid->error_curr * pid->dt;
    else
        pid->integral = 0;


    //积分限幅
    pid->integral = _LIMIT(pid->integral * pid->Ki, pid->integral_min * pid->Ki, pid->integral_max * pid->Ki) / pid->Ki;
    
    //输出计算
    pid->output = pid->Kp * pid->error_curr + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    pid->output = _LIMIT(pid->output, pid->output_min, pid->output_max);
    //更新上一次测量值
    pid->last_measure = measure;
    pid->error_last = pid->error_curr;

    return pid->output;
}
double GM6020_PID_Position_Calculate(GM6020_PID_Position* pid, double measure)
{
    //dt在增加计时器后可变
    pid->error_curr = pid->target - measure;
    
    pid->derivative = (pid->error_curr - pid->error_last) / pid->dt;


    //积分分离，过程启动、结束或大幅度增减设定时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
	//致使控制量超过执行机构可能允许的最大动作范围对应的极限控制量，引起系统较大的超调
	//基本思路：
	//当被控制量与设定值偏差较大时，取消积分作用，以避免由于积分作用使系统稳定性降低，超调量增大；
	//当被控量接近给定值时，引入积分控制，以便消除静差，提高控制精度。
    if(_ABS(pid->error_curr) <= pid->integral_start_error)
        pid->integral += pid->error_curr * pid->dt;
    else
        pid->integral /= 1.1;

    if(pid->error_curr == 0 || pid->error_curr * pid->error_last < 0)
    {
        pid->derivative *= 10;
    }
    //积分限幅
    pid->integral = _LIMIT(pid->integral * pid->Ki, pid->integral_min * pid->Ki, pid->integral_max * pid->Ki) / pid->Ki;
    
    //输出计算
    pid->output = pid->Kp * pid->error_curr + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    pid->output = _LIMIT(pid->output, pid->output_min, pid->output_max);
    //更新上一次测量值
    pid->last_measure = measure;
    pid->error_last = pid->error_curr;

    return pid->output;


}


