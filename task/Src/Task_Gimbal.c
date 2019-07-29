
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

const float     ENCODERANGLE_TO_RAD      =   0.7669903939208984f;  //2��*1000/8192
const uint16_t  MAX_YAW_TO_MIDDLE        =   1365;                 //8192/360*60=1365.3   
const uint8_t   MAX_DELTA_ECODERANGLE    =   82;                   //6623�������ٶ� 600rpm = 81.91 encoderangle/ms = 10r/s = 20�� rad/s = 3.6��/s ��ֵ��6623����������ٶ�ʱ��ȷ
const uint16_t  MAX_CURRENT              =   5000;                 //6623��������͵�������ֵ
const uint16_t  YAW_MIDDLE_ENCODEANGLE   =   3650;                 //Yaw  �м�λ�õĻ�е�Ƕ�
const uint16_t  PIT_MIDDLE_ENCODEANGLE   =   2847;                 //Pitch�м�λ�õĻ�е�Ƕ�
const uint16_t  YAW_MAX_ENCODEANGLE      =   4900;                 //Yaw  �������Ļ�е�Ƕ�
const uint16_t  YAW_MIN_ENCODEANGLE      =   2300;
const uint16_t  PIT_MAX_ENCODEANGLE      =   3311;                 //Yaw  �������Ļ�е�Ƕ�
const uint16_t  PIT_MIN_ENCODEANGLE      =   2542;
/*��ʵֵδ��*/const uint16_t  YAW_WATCH_TV             =   0;      //Yaw  ָ����ʾ���Ļ�е�Ƕ�
/*��ʵֵδ��*/const uint16_t  PIT_WATCH_TV             =   0;      //Pitchָ����ʾ���Ļ�е�Ƕ�
const uint8_t   IMU                      =   0;                    //��־λ
const uint8_t   BACK                     =   1;
const uint8_t   DIFFER                   =   2;
const uint8_t   WATCH_TV                 =   3;

pid_t _pid = { 0 };

/**
  * @brief  ��̨����
  * @param  unused
  * @retval void
  */

uint16_t usDeltaWakeTime;          //��������ʱ���
int16_t sDeltaEncoderAngle[2];     //���������е�Ƕȱ仯��
float RealSpeed[2];                //ʵ�ʱ����������ٶ�
float RealRates[2];                //ʵ��IMU�������ٶ�
float RealEuler[2];                //ʵ��IMU�����Ƕ�
float TargetRates[2];              //����IMU�������ٶ�
float TargetEuler[2] = {0};        //����IMU�����Ƕ�
float TargetSpeed[2];              //���������������ٶ�
uint16_t TargetEncodeAngle[2];     //����������������е�Ƕ�
int16_t sToMiddle[2] = { 0 };      //��ǰλ�����м�λ�õĻ�е�ǶȲ�
int16_t sToTarget[2] = { 0 };      //��ǰλ����Ŀ��λ�õĻ�е�ǶȲ�
int16_t sToTV[2] = { 0 };          //��ǰλ������ʾ��λ�õĻ�е�ǶȲ�
double fPitchCompensation;
static uint8_t flag = IMU;                //��־λ
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
        
        /*������������ʱ���*/
        static uint64_t ullLastWakeTime = 0;
        uint64_t ullPreWakeTime = ullLastWakeTime;
        ullLastWakeTime = Tick ;
        usDeltaWakeTime = (uint16_t)(Tick - ullPreWakeTime);

        /*IMU�����ǶȺͽ��ٶ�*/
        RealEuler[YAW] = _attitude.euler[2];
        RealEuler[PIT] = _attitude.euler[1];
        RealRates[YAW] = _attitude.rates[2];
        RealRates[PIT] = _attitude.rates[1];

/****************************ģʽ����*******************************/

        if(GlobalMode == PC_Ctrl_Mode) {
            if(mode.WatchBack.status == -1) {
                /*��������ʾ��λ�û�е�ǶȲ�*/
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
//                /*Pitch���� ʵ����Ϻ���*/
//                fPitchCompensation = - 0.2038f*RealEuler[PIT]*RealEuler[PIT]
//                                     - 0.0169f*RealEuler[PIT]
//                                     + 0.4796f;

                if(_radio.mouse.pitch > 0)
                {
                    _radio.mouse.pitch *= 1.06f;
                }

                if (_radio.mouse.pitch) {
                    TargetEuler[PIT] = RealEuler[PIT];
                    TargetRates[PIT] = _radio.mouse.pitch * 0.04f;//��Ϊң����ͨ������Ϊ1 �����λ�Ʋ�ȷ�� Ҫ���˸����ֵ
                    flag=IMU;
                } else {
                    TargetRates[PIT] = (TargetEuler[PIT] - RealEuler[PIT]) * _GimbalParam[PIT].kp[EULER];
                }

                PID_Calc(&Pitch_speed_pid, RealRates[PIT], TargetRates[PIT], POSITION_PID);
                _GimbalParam[PIT].SendCurrent = (int16_t)(Pitch_speed_pid.output + fPitchCompensation);

                /*����޷�*/
                _GimbalParam[YAW].SendCurrent = _GimbalParam[YAW].SendCurrent >   MAX_CURRENT ? MAX_CURRENT :
                (_GimbalParam[YAW].SendCurrent < - MAX_CURRENT ? - MAX_CURRENT : _GimbalParam[YAW].SendCurrent);
                _GimbalParam[PIT].SendCurrent = _GimbalParam[PIT].SendCurrent >   MAX_CURRENT ? MAX_CURRENT :
                (_GimbalParam[PIT].SendCurrent < - MAX_CURRENT ? - MAX_CURRENT : _GimbalParam[PIT].SendCurrent);

            }
            else if(flag == WATCH_TV) {//�����д�� ������λ��˫��B�ɻ���
            /*�������м�λ�û�е�ǶȲ�*/
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

			//�⻷����ǶȺͽ��ٶ�
			float euler[3] = { _attitude.euler[0], _attitude.euler[1], _attitude.euler[2] };
			float rates[3] = { _attitude.rates[0], _attitude.rates[1], _attitude.rates[2] };
			//��ң�����ƶ�
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
			//pitch�����Ƕ�
				_pid._euler_sp[1] = euler[1];
			//pitch�������ٶ�
				_pid._rates_sp[1] = _radio.rc.pitch * 2.0f;
				if(lock_pit == 1) {
					_pid._rates_sp[1] = 0;
				}

			//��ң�������ƶ�
			} else {
				
//				if(_pid._euler_sp[1] > PIT_MAX_ENCODEANGLE - 200) {
//					_pid._euler_sp[1] = PIT_MAX_ENCODEANGLE - 200;
//				}
//				else if(_pid._euler_sp[1] < PIT_MIN_ENCODEANGLE + 200) {
//					_pid._euler_sp[1] = PIT_MIN_ENCODEANGLE + 200;
//				}
				_pid._rates_sp[1] = (_pid._euler_sp[1] - euler[1]) * _params.att_p[1];
			}
			//��ң�����ƶ�
			if (fabsf(_radio.rc.yaw) > 0.05f) {
			//yaw�����Ƕ�
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
			//yaw�������ٶ�
				_pid._rates_sp[2] = _radio.rc.yaw * 2.0f;
				if(lock_yaw == 1) {
					_pid._rates_sp[2] = 0;
				}
			//��ң�������ƶ�
			} else {

				_pid._rates_sp[2] = (_pid._euler_sp[2] - euler[2]) * _params.att_p[2];
			}

			for (int i = 0; i < 3; i++) {
				//��ǰ���ٶ����=�������ٶ�-��ǰ���ٶ�(�ڻ�)
				_pid.err_gimbal_now[i] = _pid._rates_sp[i] - rates[i];
				//�ڻ����=P��+I��+D��
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
        if((BinSemaphoreGimbal != NULL))//���յ����ݣ����Ҷ�ֵ�ź�����Ч
		{
			xSemaphoreGive(BinSemaphoreGimbal);
		}

		//��ȡʣ��ջ��С
		_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );
    }
    
}
