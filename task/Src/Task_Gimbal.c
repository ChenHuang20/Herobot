
#include "Handle.h"

#include "BSP_TIM.h"

#include "Driver_Gimbal.h"
#include "Driver_DBUS.h"
#include "Driver_ICM20600.h"

#include "Task_Gimbal.h"

#include "Task_ModeSwitch.h"
#include "Driver_ModeSwitch.h"
#include "Algorithm_Estimator.h"
#include "Algorithm_Pid.h"
#include <math.h>

const float     ENCODERANGLE_TO_RAD      =   0.7669903939208984f;  //2π*1000/8192
const uint16_t  MAX_YAW_TO_MIDDLE        =   1365;                 //8192/360*60=1365.3   
const uint8_t   MAX_DELTA_ECODERANGLE    =   82;                   //6623电机最大速度 600rpm = 81.91 encoderangle/ms = 10r/s = 20π rad/s = 3.6°/s 此值在6623不超过最大速度时正确
const uint16_t  MAX_CURRENT              =   5000;                 //6623电调所发送的最大电流值
const uint16_t  YAW_MIDDLE_ENCODEANGLE   =   3650;                 //Yaw  中间位置的机械角度
const uint16_t  PIT_MIDDLE_ENCODEANGLE   =   2847;                 //Pitch中间位置的机械角度
const uint16_t  YAW_MAX_ENCODEANGLE      =   4900;                 //Yaw  两侧最大的机械角度
const uint16_t  YAW_MIN_ENCODEANGLE      =   2300;
const uint16_t  PIT_MAX_ENCODEANGLE      =   3311;                 //Yaw  两侧最大的机械角度
const uint16_t  PIT_MIN_ENCODEANGLE      =   2542;
/*真实值未测*/const uint16_t  YAW_WATCH_TV             =   0;      //Yaw  指向显示屏的机械角度
/*真实值未测*/const uint16_t  PIT_WATCH_TV             =   0;      //Pitch指向显示屏的机械角度
const uint8_t   IMU                      =   0;                    //标志位
const uint8_t   BACK                     =   1;
const uint8_t   DIFFER                   =   2;
const uint8_t   WATCH_TV                 =   3;

pid_t _pid = { 0 };

/**
  * @brief  云台任务
  * @param  unused
  * @retval void
  */

uint16_t usDeltaWakeTime;          //两次任务时间差
int16_t sDeltaEncoderAngle[2];     //两次任务机械角度变化量
float RealSpeed[2];                //实际编码器反馈速度
float RealRates[2];                //实际IMU反馈角速度
float RealEuler[2];                //实际IMU反馈角度
float TargetRates[2];              //期望IMU反馈角速度
float TargetEuler[2] = {0};        //期望IMU反馈角度
float TargetSpeed[2];              //期望编码器反馈速度
uint16_t TargetEncodeAngle[2];     //期望编码器反馈机械角度
int16_t sToMiddle[2] = { 0 };      //当前位置与中间位置的机械角度差
int16_t sToTarget[2] = { 0 };      //当前位置与目标位置的机械角度差
int16_t sToTV[2] = { 0 };          //当前位置与显示器位置的机械角度差
double fPitchCompensation;
static uint8_t flag = IMU;                //标志位
int16_t yyy;
uint8_t limit_flag = 0;
uint8_t lock_yaw = 0;
uint8_t lock_pit = 0;
float pre_yaw = 0;
void Task_Gimbal(void *Parameters)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    _StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		_Tick.Gimbal++;
        
        /*计算两次任务时间差*/
        static uint64_t ullLastWakeTime = 0;
        uint64_t ullPreWakeTime = ullLastWakeTime;
        ullLastWakeTime = Tick ;
        usDeltaWakeTime = (uint16_t)(Tick - ullPreWakeTime);

        /*IMU反馈角度和角速度*/
        RealEuler[YAW] = _attitude.euler[2];
        RealEuler[PIT] = _attitude.euler[1];
        RealRates[YAW] = _attitude.rates[2];
        RealRates[PIT] = _attitude.rates[1];

/****************************模式控制*******************************/

        if(GlobalMode == PC_Ctrl_Mode) {
            if(mode.WatchBack.status == -1) {
                /*计算与显示器位置机械角度差*/
                sToTV[YAW] = (int16_t)(YAW_WATCH_TV - _GimbalParam[YAW].RealEncodeAngle);
                sToTV[PIT] = (int16_t)(PIT_WATCH_TV - _GimbalParam[PIT].RealEncodeAngle);
                sToTV[YAW] = sToTV[YAW] >   4096 ? (sToTV[YAW] - 8192):
                            (sToTV[YAW] < - 4096 ? (sToTV[YAW] + 8192) : sToTV[PIT]);
                sToTV[PIT] = sToTV[PIT] >   4096 ? (sToTV[PIT] - 8192):
                            (sToTV[PIT] < - 4096 ? (sToTV[PIT] + 8192) : sToTV[PIT]);
                _GimbalParam[YAW].SendCurrent = -sToTV[YAW] * 1.22f;
                _GimbalParam[YAW].SendCurrent = -sToTV[PIT] * 1.22f;
                flag = WATCH_TV;
            }
            else if(_radio.mouse.yaw || _radio.mouse.pitch || flag == IMU) {
//                /*Pitch补偿 实验拟合函数*/
//                fPitchCompensation = - 0.2038f*RealEuler[PIT]*RealEuler[PIT]
//                                     - 0.0169f*RealEuler[PIT]
//                                     + 0.4796f;

                if(_radio.mouse.pitch > 0)
                {
                    _radio.mouse.pitch *= 1.06f;
                }

                if (_radio.mouse.pitch) {
                    TargetEuler[PIT] = RealEuler[PIT];
                    TargetRates[PIT] = _radio.mouse.pitch * 0.04f;//因为遥控器通道量化为1 而鼠标位移不确定 要测了改这个值
                    flag=IMU;
                } else {
                    TargetRates[PIT] = (TargetEuler[PIT] - RealEuler[PIT]) * _GimbalParam[PIT].kp[EULER];
                }

                PID_Calc(&Pitch_speed_pid, RealRates[PIT], TargetRates[PIT], POSITION_PID);
                _GimbalParam[PIT].SendCurrent = (int16_t)(Pitch_speed_pid.output + fPitchCompensation);

                /*输出限幅*/
                _GimbalParam[YAW].SendCurrent = _GimbalParam[YAW].SendCurrent >   MAX_CURRENT ? MAX_CURRENT :
                (_GimbalParam[YAW].SendCurrent < - MAX_CURRENT ? - MAX_CURRENT : _GimbalParam[YAW].SendCurrent);
                _GimbalParam[PIT].SendCurrent = _GimbalParam[PIT].SendCurrent >   MAX_CURRENT ? MAX_CURRENT :
                (_GimbalParam[PIT].SendCurrent < - MAX_CURRENT ? - MAX_CURRENT : _GimbalParam[PIT].SendCurrent);

            }
            else if(flag == WATCH_TV) {//按昊宇写的 在任意位置双击B可回正
            /*计算与中间位置机械角度差*/
                sToMiddle[YAW] =  (int16_t)(YAW_MIDDLE_ENCODEANGLE - _GimbalParam[YAW].RealEncodeAngle);
                sToMiddle[PIT] =  (int16_t)(PIT_MIDDLE_ENCODEANGLE - _GimbalParam[PIT].RealEncodeAngle); 
                sToMiddle[YAW] = sToMiddle[YAW] >   4096 ? (sToMiddle[YAW] - 8192):
                                (sToMiddle[YAW] < - 4096 ? (sToMiddle[YAW] + 8192) : sToMiddle[PIT]);
                sToMiddle[PIT] = sToMiddle[PIT] >   4096 ? (sToMiddle[PIT] - 8192):
                                (sToMiddle[PIT] < - 4096 ? (sToMiddle[PIT] + 8192) : sToMiddle[PIT]);
                _GimbalParam[YAW].SendCurrent = -sToMiddle[YAW] * 1.22f;
                _GimbalParam[YAW].SendCurrent = -sToMiddle[PIT] * 1.22f;
            }

        }
        else if(GlobalMode == ShutDown) {
            _GimbalParam[YAW].SendCurrent=0;
            _GimbalParam[YAW].SendCurrent=0;
        }
        else if(GlobalMode == RC_Ctrl_Mode) {

			//外环输出角度和角速度
			float euler[3] = { _attitude.euler[0], _attitude.euler[1], _attitude.euler[2] };
			float rates[3] = { _attitude.rates[0], _attitude.rates[1], _attitude.rates[2] };
			//若遥控器推动
			if (fabsf(_radio.rc.pitch) > 0.05f) {

				if(_radio.rc.pitch > 0 && _GimbalParam[PIT].RealEncodeAngle < PIT_MIN_ENCODEANGLE + 130)
				{
					lock_pit = 1;
				}
				else if(_radio.rc.pitch < 0 && _GimbalParam[PIT].RealEncodeAngle > PIT_MAX_ENCODEANGLE - 130)
				{
					lock_pit = 1;
				}
				else {
					lock_pit = 0;
				}
			//pitch期望角度
				_pid._euler_sp[1] = euler[1];
			//pitch期望角速度
				_pid._rates_sp[1] = _radio.rc.pitch * 2.0f;
				if(lock_pit == 1) {
					_pid._rates_sp[1] = 0;
				}

			//若遥控器不推动
			} else {
				
//				if(_pid._euler_sp[1] > PIT_MAX_ENCODEANGLE - 200) {
//					_pid._euler_sp[1] = PIT_MAX_ENCODEANGLE - 200;
//				}
//				else if(_pid._euler_sp[1] < PIT_MIN_ENCODEANGLE + 200) {
//					_pid._euler_sp[1] = PIT_MIN_ENCODEANGLE + 200;
//				}
				_pid._rates_sp[1] = (_pid._euler_sp[1] - euler[1]) * _params.att_p[1];
			}
			//若遥控器推动
			if (fabsf(_radio.rc.yaw) > 0.05f) {
			//yaw期望角度
				if(_radio.rc.yaw > 0 && _GimbalParam[YAW].RealEncodeAngle < YAW_MIN_ENCODEANGLE + 400)
				{
					lock_yaw = 1;
				}
				else if(_radio.rc.yaw < 0 && _GimbalParam[YAW].RealEncodeAngle > YAW_MAX_ENCODEANGLE - 400)
				{
					lock_yaw = 1;
				}
				else {
					lock_yaw = 0;
				}
				_pid._euler_sp[2] = euler[2];
			//yaw期望角速度
				_pid._rates_sp[2] = _radio.rc.yaw * 2.0f;
				if(lock_yaw == 1) {
					_pid._rates_sp[2] = 0;
				}
			//若遥控器不推动
			} else {

				_pid._rates_sp[2] = (_pid._euler_sp[2] - euler[2]) * _params.att_p[2];
			}

			for (int i = 0; i < 3; i++) {
				//当前角速度误差=期望角速度-当前角速度(内环)
				_pid.err_gimbal_now[i] = _pid._rates_sp[i] - rates[i];
				//内环输出=P项+I项+D项
				_pid.output_gimbal[i] = _pid.err_gimbal_now[i] * _params.rate_p[i]
									  + _pid._rates_int[i]
									  + (_pid._rates_prev[i] - rates[i]) * _params.rate_d[i];
				_pid.output_gimbal[i] = fmaxf(-1.0f, fminf(1.0f, _pid.output_gimbal[i]));

				_pid._rates_prev[i] = rates[i];

				_pid._rates_int[i] += _pid.err_gimbal_now[i] * _params.rate_i[i];
			}
        }
		pre_yaw = _radio.rc.yaw;

        if( GlobalMode == ShutDown)
        {
            _pid.output_gimbal[1] = 0;
            _pid.output_gimbal[0] = 0;

		}
        if((BinSemaphoreGimbal != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreGimbal);
		}

		//获取剩余栈大小
		_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );
    }
    
}
