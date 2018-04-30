
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
  * @brief  ��λ����������
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

		if((cnt % 5) == 0)  //���
			_send_flag.send_motopwm = true;

		if(_send_flag.send_check) {
			_send_flag.send_check = !Send_Check();
		}

		else if(_send_flag.send_motopwm) {
		//        _send_flag.send_motopwm = !Send_int16_data();
			_send_flag.send_motopwm = !Send_float_data();

		}
		//����PIDд�룬��ı�PID����
		if(debug_write)
		{
			//���յ�һ��pidΪpitch�ڻ�
			PitchParam.kp[0] = _pid_debug[0].p;
			PitchParam.ki[0] = _pid_debug[0].i;
			PitchParam.kd[0] = _pid_debug[0].d;

			//���յڶ���pidΪpitch�⻷
			PitchParam.kp[1] = _pid_debug[1].p;
			PitchParam.ki[1] = _pid_debug[1].i;
			PitchParam.kd[1] = _pid_debug[1].d;

			//���յ�����pidΪyaw�ڻ�
			YawParam.kp[0] = _pid_debug[2].p;
			YawParam.ki[0] = _pid_debug[2].i;
			YawParam.kd[0] = _pid_debug[2].d;

			//���յ�����pidΪyaw�⻷
			YawParam.kp[1] = _pid_debug[3].p;
			YawParam.ki[1] = _pid_debug[3].i;
			YawParam.kd[1] = _pid_debug[3].d;

			//���յ�����pidΪCM1�ٶȻ�
			_ChassisParam.kp[0] = _pid_debug[4].p;
			_ChassisParam.ki[0] = _pid_debug[4].i;
			_ChassisParam.kd[0] = _pid_debug[4].d;

			//���յ�����pidΪCM2�ٶȻ�
			_ChassisParam.kp[1] = _pid_debug[5].p;
			_ChassisParam.ki[1] = _pid_debug[5].i;
			_ChassisParam.kd[1] = _pid_debug[5].d;

			//���յ�����pidΪCM3�ٶȻ�
			_ChassisParam.kp[2] = _pid_debug[6].p;
			_ChassisParam.ki[2] = _pid_debug[6].i;
			_ChassisParam.kd[2] = _pid_debug[6].d;

			//���յڰ���pidΪCM4�ٶȻ�
			_ChassisParam.kp[3] = _pid_debug[7].p;
			_ChassisParam.ki[3] = _pid_debug[7].i;
			_ChassisParam.kd[3] = _pid_debug[7].d;

			//���յھ���pidΪ�󲦵����
//			_params.stir_att_p = _pid_debug[8].p;
//			_params.stir_p = _pid_debug[8].i;
//			_params.stir_i = _pid_debug[8].d;

			//Ҫ���͵�����
			_send.f_data[0] = _attitude.euler[0];
			_send.f_data[1] = _attitude.euler[1];
			_send.f_data[2] = _attitude.euler[2];
		}
		//��ȡʣ��ջ��С
		_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(2);
    }
}
