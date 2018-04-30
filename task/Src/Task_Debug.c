
#include "Handle.h"

#include "BSP_UART.h"

#include "Driver_Debug.h"
#include "Driver_Gimbal.h"
#include "Driver_Chassis.h"
#include "Driver_Friction.h"
#include "Driver_SynchroBelt.h"

#include "Task_Debug.h"

#include "Algorithm_Estimator.h"

/**
  * @brief  上位机调试任务
  * @param  unused
  * @retval void
  */
void Task_Debug(void *Parameters)
{
	_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Debug++;
		
		static uint32_t cnt = 0;

		cnt ++;

		if((cnt % 5) == 0)  //电机
			_send_flag.send_motopwm = true;

		if(_send_flag.send_check) {
			_send_flag.send_check = !Send_Check();
		}

		else if(_send_flag.send_motopwm) {
		//        _send_flag.send_motopwm = !Send_int16_data();
			_send_flag.send_motopwm = !Send_float_data();

		}
		//若有PID写入，则改变PID参数
		if(debug_write)
		{
			//接收第一组pid为pitch内环
			PitchParam.kp[0] = _pid_debug[0].p;
			PitchParam.ki[0] = _pid_debug[0].i;
			PitchParam.kd[0] = _pid_debug[0].d;

			//接收第二组pid为pitch外环
			PitchParam.kp[1] = _pid_debug[1].p;
			PitchParam.ki[1] = _pid_debug[1].i;
			PitchParam.kd[1] = _pid_debug[1].d;

			//接收第三组pid为yaw内环
			YawParam.kp[0] = _pid_debug[2].p;
			YawParam.ki[0] = _pid_debug[2].i;
			YawParam.kd[0] = _pid_debug[2].d;

			//接收第四组pid为yaw外环
			YawParam.kp[1] = _pid_debug[3].p;
			YawParam.ki[1] = _pid_debug[3].i;
			YawParam.kd[1] = _pid_debug[3].d;

			//接收第五组pid为CM1速度环
			_ChassisParam.kp[0] = _pid_debug[4].p;
			_ChassisParam.ki[0] = _pid_debug[4].i;
			_ChassisParam.kd[0] = _pid_debug[4].d;

			//接收第六组pid为CM2速度环
			_ChassisParam.kp[1] = _pid_debug[5].p;
			_ChassisParam.ki[1] = _pid_debug[5].i;
			_ChassisParam.kd[1] = _pid_debug[5].d;

			//接收第七组pid为CM3速度环
			_ChassisParam.kp[2] = _pid_debug[6].p;
			_ChassisParam.ki[2] = _pid_debug[6].i;
			_ChassisParam.kd[2] = _pid_debug[6].d;

			//接收第八组pid为CM4速度环
			_ChassisParam.kp[3] = _pid_debug[7].p;
			_ChassisParam.ki[3] = _pid_debug[7].i;
			_ChassisParam.kd[3] = _pid_debug[7].d;

			//接收第九组pid为大拨弹电机
//			_params.stir_att_p = _pid_debug[8].p;
//			_params.stir_p = _pid_debug[8].i;
//			_params.stir_i = _pid_debug[8].d;

			//要发送的数据
			_send.f_data[0] = _attitude.euler[0];
			_send.f_data[1] = _attitude.euler[1];
			_send.f_data[2] = _attitude.euler[2];
		}
		//获取剩余栈大小
		_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(2);
    }
}
