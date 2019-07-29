
#include "Handle.h"
#include "BSP_TIM.h"

#include "Driver_DBUS.h"
#include "Driver_ModeSwitch.h"
#include "Driver_Friction.h"
#include "Driver_Steering.h"

#include "Task_Shoot.h"
#include "Task_ModeSwitch.h"

mode_t mode = { 0 };
uint16_t press_tick = 0;

/**
  * @brief  ģʽѡ������
  * @param  unused
  * @retval void
  */
void Task_ModeSwitch(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.ModeSwitch++;
		//ѡ�����ģʽ
		if(_radio.rc.mode == RC_UP)			 	//��
		{
			GlobalMode = PC_Ctrl_Mode;			//����ģʽ
		}
		else if(_radio.rc.mode == RC_MI)		//��
		{
			GlobalMode = RC_Ctrl_Mode;			//ң����ģʽ
		}
		else if(_radio.rc.mode == RC_DN)		//��
		{
			GlobalMode = ShutDown;				//�����ģʽ
		}
		switch(GlobalMode)
		{
			//ң����ģʽ
			case RC_Ctrl_Mode:

		switch(_radio.rc.shoot)
		{
			case 1:
//				if(_radio.rc.shoot_pre != 1 && _shoot.friction17 == 0 && _radio.rc.mode == RC_MI)
//					Fric17_SET(700,700);
//				else if(_radio.rc.shoot_pre != 1 && _shoot.friction17 == 1 && _radio.rc.mode == RC_MI)
//					Fric17_OFF();
				if(_radio.rc.shoot_pre != 1 && _shoot.steering == 0 && _shoot.friction42 == 1 && _radio.rc.mode == RC_MI)
                {SteeringShoot_ON();
                    _shoot.steering = 1;}
                else if(_radio.rc.shoot_pre != 1 && _shoot.steering == 1 && _shoot.friction42 == 1 && _radio.rc.mode == RC_MI)
                {SteeringShoot_OFF();
                    _shoot.steering = 0;}
				_radio.rc.shoot_pre = 1;
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				break;
			case 2:
				if(_radio.rc.shoot_pre != 2)
				{
					_shoot.shooting_off17 = 0;
					_shoot.reverse_count17 = 0;
					_shoot.arti_shoot17 = 1;

                    _shoot.shooting_off42 = 0;
					_shoot.reverse_count42 = 0;
					_shoot.arti_shoot42 = 1;
				}
                if(_radio.rc.shoot_pre != 2 && _shoot.friction42 == 0)
					Fric42_SET(1.6f);
				else if(_radio.rc.shoot_pre != 2 && _shoot.friction42 == 1)
					Fric42_OFF();
            
				_radio.rc.shoot_pre = 2;
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
				break;
			case 3:
				if(_radio.rc.shoot_pre == 2)
				{
					_shoot.arti_shoot42 = 0;
                    _shoot.arti_shoot17 = 0;
				}
				_radio.rc.shoot_pre = 3;
				break;
			default:
				break;
		}

				break;

			//����ģʽ
			case PC_Ctrl_Mode:

/************************************************���̿���******************************************************/

				//����
                if(_radio.key.Shift)
                {
                    if(1 == mode.Speed.Rising_flag)
                    {
                        mode.Speed.Gear++;
                        mode.Speed.Gear = mode.Speed.Gear >= 3 ? 3 : mode.Speed.Gear;
                        mode.Speed.Rising_flag = 0 ;
                    }
				}
				else
				{
					mode.Speed.Rising_flag = 1;
				}

                //����
                if(_radio.key.Ctrl)
                {
                    if(1 == mode.Speed.Falling_flag)
                    {
                        mode.Speed.Gear--;
                        mode.Speed.Gear = mode.Speed.Gear <= 0 ? 0 : mode.Speed.Gear;
                        mode.Speed.Falling_flag = 0 ;
                    }
                }
                else 
                {
                    mode.Speed.Falling_flag = 1;
                }                    
                
                //���
                if(_radio.key.B)
                {
                    if(1 == mode.WatchBack.flag)
                    {
                        mode.WatchBack.status *= -1;
                        mode.WatchBack.flag = 0;
                    }
                }
                else 
                {
                    mode.WatchBack.flag = 1;
                }
                
                if(_radio.key.C)
                {
                    
                }
                
                //Ť��      ��Ť��ʱ����Ť�� Ť��ʱ����ֹͣ ������ʼ������Ť��
//                static uint32_t F_tickms = 0;
//                if(_radio.key.F)
//                {
////                    mode.TwistWaist.status = 1 == mode.TwistWaist.flag ? 1 : 0;
//                    mode.TwistWaist.status = mode.TwistWaist.flag;
//                    if(F_tickms >= 500 )
//                    {
//                        mode.TwistWaist.status = 2 ;
//                    }
//                    F_tickms += 4;
//                }
//                else
//                {
//                    F_tickms = 0;
////                    mode.TwistWaist.flag = 1 == mode.TwistWaist.status || 2 == mode.TwistWaist.status ? 0 : 1;
//                    mode.TwistWaist.flag = !(1 == mode.TwistWaist.status || 2 == mode.TwistWaist.status);
//                }
                
                if(single_press(_radio.key.G))
                {
                    
                }
                
                if(single_press(_radio.key.R))
                {
                    
                }
//                //�����ӽ�
                if(_radio.key.V)
                {
                    if(1 == mode.Freeview.flag)
                    {
                        mode.Freeview.status *= -1;
                        mode.Freeview.flag = 0;
                    }
                }
                else
                {
                    mode.Freeview.flag = 1;
                }
                
                if( 1 == mode.Freeview.status )
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
                }

                switch(_radio.key.R)
                {
                    case 0:
						_radio.key.R_pre = 0;
						break;
					case 1:
						if(_radio.key.R_pre == 0 && _shoot.friction42 == 0)
							Fric42_SET(1.6f);
						else if(_radio.key.R_pre == 0 && _shoot.friction42 == 1)
							Fric42_OFF();
						_radio.key.R_pre = 1;
                        if(_radio.key.R_pre == 0 && _shoot.friction17 == 0)
							Fric17_SET(700,700);
						else if(_radio.key.R_pre == 0 && _shoot.friction17 == 1)
							Fric17_OFF();
						_radio.key.R_pre = 1;
						break;
                }
//				switch(_radio.key.Z)
//				{
//					case 0:
//						_radio.key.Z_pre = 0;
//						break;
//					case 1:
//						if(_radio.key.Z_pre == 0 && _shoot.friction17 == 0)
//							Fric17_SET(700,700);
//						else if(_radio.key.Z_pre == 0 && _shoot.friction17 == 1)
//							Fric17_OFF();
//						_radio.key.Z_pre = 1;
//						break;
//				}
/************************************************������******************************************************/

				if(_radio.mouse.press_r == 1 ) _shoot.steering =0; 
                    if(_shoot.steering == 0)	   //�Ҽ�����
				{
					SteeringShoot_ON();
//                    _shoot.steering = 1;
                    press_tick+=4;
                    if(press_tick >= 80 && press_tick<200)
                    {
                        SteeringShoot_OFF();
                    }
                    if(press_tick >= 200)
                    {
                    press_tick = 0;_shoot.steering = 1;
                    }
				}
//                else {
//                    SteeringShoot_OFF();
//                    _shoot.steering = 0;
//                }
				if(_radio.mouse.press_l == 1)	   //�������
				{
					_shoot.shooting_off17 = 0;
					_shoot.reverse_count17 = 0;
					_shoot.arti_shoot17 = 1;
				}
                else
                    _shoot.arti_shoot17 = 0;
				break;

			//�����ģʽ
			case ShutDown:
				//output == 0;
				break;
			default:
				break;
		}
		//��ȡʣ��ջ��С
		_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&xLastWakeTime, (4 / portTICK_RATE_MS) );
    }
}
