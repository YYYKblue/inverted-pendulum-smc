/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��1.0
�޸�ʱ�䣺2021-12-09

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:1.0
Update��2021-12-09

All rights reserved
***********************************************/
#include "control.h"
int Balance_Pwm, Position_Pwm; // Ŀ��Ƕ�PWM��Ŀ��λ��PWM
u8 Position_Target;			   // ���ڱ��λ�ÿ��Ƶ�ʱ��
u8 Swing_up = 1;			   // ���ڱ���ֶ����ʱ���Ƿ��ǵ�һ�ν����ֶ���ں���

// ���PD�������õ��Ĳ���
float Bias;				 // ���ƫ��
float Last_Bias, D_Bias; // PID��ر���
int balance;			 // PWM����ֵ

// λ��PD�������õ��Ĳ���
float Position_PWM, Last_Position, Position_Bias, Position_Differential;
float Position_Least;

// UAS-SMC�������õ��Ĳ���
float k1 = 200, k2 = 2000; // �������е�ϵ��
int pos_d;
float theta_d;

float pos_last = 0;
float theta_last = 0;

float N;	// ������
int u_d;	// ����ĵ�ѹ
int u_real; // �����PWMֵ

// ������ģ��ϵ��
float k11[4] = {0.4, -0.2, -0.4, 0.2};
float k12[4] = {0.8, 0.4, -0.8, -0.4};
float k21[4] = {0.2, -0.1, -0.2, 0.1};
float k22[4] = {0.4, 0.2, -0.4, -0.2};

// ״̬����
float A[4] = {-135.3752, -1.9188, 380.1344, 32.9066}; // A22,A23,A42,A43
// ���ƾ���
float B[2] = {10.5032, -29.4932}; // B21,B41

u8 auto_run = 0;		  // �ֶ���ڻ��Զ���ڱ�־λ��Ĭ�����ֶ����
u8 autorun_step0 = 0;	  // �Զ���ڵ�0�����ҵ��е㣬�ȴ����
u8 autorun_step1 = 1;	  // �Զ���ڵ�1��
u8 autorun_step2 = 0;	  // �Զ���ڵ�2��
long Target_Position;	  // Ŀ��λ��
float D_Angle_Balance;	  // �ڸ˽Ƕȱ仯��
long success_count = 0;	  // �ڸ���ƽ��λ�õĳɹ�������¼
u8 success_flag = 0;	  // �Զ����ʱ����ƽ�����������ڵı�־λ
long wait_count = 0;	  // �ȴ���������ʱʱ�䵽�󣬻�ȡ��ڳɹ���λ��
long D_Count;			  // ���ڸ�����ȡ�ڸ˽Ƕȱ仯�ʵ��м����
float Last_Angle_Balance; // ���ڻ�ȡ�ڸ˽Ƕȱ仯�ʺ����У�������һ�νǶ�

u8 left, right;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
		  TIM1���Ƶ�5ms��ʱ�ж�
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1)) // ���ָ����TIM1�жϷ������ //5ms��ʱ�ж�,Ҳ���ǿ���Ƶ��Ϊ200Hz //ʵ���ϵ��PWM�ķ���Ƶ����10kHz
	{
		if (delay_flag == 1)
		{
			if (++delay_50 == 10)
				delay_50 = 0, delay_flag = 0; //===���������ṩ50ms�ľ�׼��ʱ  10��*5ms = 50ms��Ŀ����Ϊ�˸���λ��������
		}
		Encoder = Read_Encoder(4);				//===���±�����λ����Ϣ����ǰλ�ã�
		Angle_Balance = Get_Adc_Average(3, 10); //===���°ڸ����
		Get_D_Angle_Balance();					//===��ðڸ˽��ٶ�

		//Turn_Off(Voltage);
		
		if (auto_run == 1) // UAS_SMC����ģʽ(����Ԥ����������)
		{
			if (Flag_Stop == 0)
			{
				Turn_Off(Voltage); // ��ǡ���ѹ����
				if (Swing_up == 0) // Ĭ��Ϊ1������ʹ���������Ǳ����Ȱ�һ���û��������ܽ�����ƣ����Դ˴��ض������
					Position_Zero = Encoder, pos_last = 0, theta_last = 0, Swing_up = 1;

				if (Flag_Stop == 0)
				{
					Moto = UAS_SMC_Control(Angle_Balance, Encoder);
					Xianfu_Pwm();
					Set_Pwm(Moto);
				}
			}
		}
		if (auto_run == 0)
		{
			Turn_Off(Voltage); // ��ǡ���ѹ����
			if (Swing_up == 0)
				Position_Zero = Encoder, Last_Position = 0, Last_Bias = 0, Position_Target = 0, Swing_up = 1;

			if (Flag_Stop == 0)
			{
				Balance_Pwm = Balance(Angle_Balance); //===�Ƕ�PD����
				if (++Position_Target > 4)
					Position_Pwm = Position(Encoder), Position_Target = 0; //===λ��PD���� 25ms����һ��λ�ÿ���
				Moto = Balance_Pwm - Position_Pwm;						   //===����������PWM
				Xianfu_Pwm();											   //===PWM�޷� ��ֹռ�ձ�100%������ϵͳ���ȶ�����
				Set_Pwm(Moto);											   //===��ֵ��PWM�Ĵ���
			}
		}

		if (Flag_Stop == 1)
			Set_Pwm(0);
	}
	if (Flag_Stop == 0)
		Led_Flash(100);			  //===LED��˸ָʾϵͳ��������
	Voltage = Get_battery_volt(); //===��ȡ��ص�ѹ
	Key();						  //===ɨ�谴���仯
}

/**************************************************************************
�������ܣ�������ֵת��Ϊλ��
��ڲ�����������ֵ
����  ֵ��λ��
**************************************************************************/
float pos_real(int pos_encoder)
{
	return (pos_encoder * K);
}
/**************************************************************************
�������ܣ�ADCֵת��Ϊ����
��ڲ�����ADC��ѹֵ��δ������
����  ֵ������ֵ
**************************************************************************/
float angle_real(float angel_ADC)
{
	return ((2 * PI) * (angel_ADC / 4096.0f));
}

/**************************************************************************
�������ܣ�UAS-SMC����
��ڲ������Ƕ�
����  ֵ����ǿ���PWM
**************************************************************************/
int UAS_SMC_Control(float theta, int pos)
{
	float theta_e, theta_dot, theta_e_dot; // ���
	float pos_e, pos_dot, pos_e_dot;	   // λ��

	theta_d = ZHONGZHI;
	pos_d = Position_Zero;

	float p = pos_real(pos - pos_d), p_d = 0;
	float a = angle_real(theta - theta_d), a_d = 0;

	float fx1, fx2;
	float gx1 = B[0], gx2 = B[1];

	float s1[2] = {0};
	float s2[2] = {0};

	float m1[2] = {0}, m2[2] = {0}; // ��ǰ��ģ��ϵ��

	pos_dot = (p - pos_last) / TS;
	theta_dot = (a - theta_last) / TS;

	fx1 = A[0] * pos_dot + A[1] * a; // ����ط�Ҫ�ȰѶ������ı�����ֵ�͵�ѹֵ�����λ�úͽǶȣ���������ǲ�׼ȷ��
	fx2 = A[2] * pos_dot + A[3] * a;

	pos_e = p - p_d;
	pos_e_dot = pos_dot; // ֱ�ӰѲο�ֵ��Ϊ����������

	theta_e = a - a_d;
	theta_e_dot = theta_dot;

	s1[0] = pos_e + 1.5f * pos_e_dot;
	s1[1] = pos_e + (1.0f / 6.0f) * pos_e_dot;

	s2[0] = theta_e + 1.5f * theta_e_dot;
	s2[1] = theta_e + (1.0f / 6.0f) * theta_e_dot;

	N = k1 * fabsf(pos_e) + k2 * fabsf(theta_e);

	if (s1[0] < 0 && s1[1] < 0)
		m1[0] = k11[0], m1[1] = k12[0];
	else if (s1[0] < 0 && s1[1] >= 0)
		m1[0] = k11[1], m1[1] = k12[1];
	else if (s1[0] >= 0 && s1[1] >= 0)
		m1[0] = k11[2], m1[1] = k12[2];
	else if (s1[0] >= 0 && s1[1] < 0)
		m1[0] = k11[3], m1[1] = k12[3];

	if (s2[0] < 0 && s2[1] < 0)
		m2[0] = k21[0], m2[1] = k22[0];
	else if (s2[0] < 0 && s2[1] >= 0)
		m2[0] = k21[1], m2[1] = k22[1];
	else if (s2[0] >= 0 && s2[1] >= 0)
		m2[0] = k21[2], m2[1] = k22[2];
	else if (s2[0] >= 0 && s2[1] < 0)
		m2[0] = k21[3], m2[1] = k22[3];

	pos_last = p;
	theta_last = a;

	float den = (m1[1] * gx1 + m2[1] * gx2); // ��ĸΪ�㱣�������ﲻ�ᵽ0
	if (fabsf(den) < 1e-6f)
		den = (den >= 0 ? 1e-6f : -1e-6f);
	u_d = (1.0f / den) * (-(m1[0] * pos_e_dot + m2[0] * theta_e_dot) - (m1[1] * fx1 + m2[1] * fx2) + N);

	u_real = (int)(U2PWM * u_d);
	return -u_real; // ע������ĸ��ţ���������������Ϊ��ǰ�����Ƕȷ�����ǰ��Ϊ���������������PWM�ķ���Ӧ���������һ�£�
}

/**************************************************************************
�������ܣ����PD����
��ڲ������Ƕ�
����  ֵ����ǿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Balance(float Angle)
{
	Bias = Angle - ZHONGZHI;							// �����ƽ��ĽǶ���ֵ�Ĳ�ֵ��ZHONGZHI�ͻ�е��أ�
	D_Bias = Bias - Last_Bias;							// ���ƫ���΢�� ����΢�ֿ���
	balance = -Balance_KP * Bias - D_Bias * Balance_KD; //===������ǿ��Ƶĵ��PWM  PD����
	Last_Bias = Bias;									// ������һ�ε�ƫ��
	return balance;
}

/**************************************************************************
�������ܣ�λ��PD����
��ڲ�����������
����  ֵ��λ�ÿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Position(int Encoder)
{
	Position_Least = Encoder - Position_Zero; //===���ƽ��λ��
	Position_Bias *= 0.8;
	Position_Bias += Position_Least * 0.2; //===һ�׵�ͨ�˲���
	Position_Differential = Position_Bias - Last_Position;
	Last_Position = Position_Bias;
	Position_PWM = Position_Bias * Position_KP + Position_Differential * Position_KD; //===�ٶȿ���
																					  //    Position_PWM=Position_Bias*(Position_KP+Basics_Position_KP)/2+Position_Differential*(Position_KD+Basics_Position_KD)/2; //===λ�ÿ���
	return Position_PWM;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto)
{
	if (moto < 0)
		BIN2 = 1, BIN1 = 0; // ת���ж�
	else
		BIN2 = 0, BIN1 = 1;
	PWMB = myabs(moto);
}

/**************************************************************************
�������ܣ�����PWM��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{
	int Amplitude = 6900; //===PWM������7200 ������6900
	if (Moto < -Amplitude)
		Moto = -Amplitude;
	if (Moto > Amplitude)
		Moto = Amplitude;
}
/**************************************************************************
�������ܣ������޸Ŀ��ưڸ˵�λ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
	// ��Ʒʹ�õ���13�߻��������������20���ٱȣ��������õ���4��Ƶ�����תһȦ�������ϵ���ֵ��13*4*20 = 1040
	int position = 1040; // Ŀ��λ�� ���ԭʼλ����10000  תһȦ��1040 �ͱ����������йأ�Ĭ���ǰڸ�Z��תһȦ�����1040��������
	static int tmp, flag, count;
	tmp = click_N_Double(100);

	if (tmp == 1)
		flag = 1; //++
	if (tmp == 2)
		flag = 2; //--

	if (flag == 1) // �ڸ����������˶�
	{
		Position_Zero += 4;
		count += 4;
		if (count == position)
			flag = 0, count = 0;
	}
	if (flag == 2) // �ڸ��򷴷����˶�
	{
		Position_Zero -= 4;
		count += 4;
		if (count == position)
			flag = 0, count = 0;
	}

	if (Long_Press_KEY2() == 1)
	{
		Position_Zero = 10000;
		auto_run = !auto_run;
	}

	if (Flag_Stop == 1) // �ڵ����ֹ��ʱ�򳤰�Ԥ�����������л�ģʽ����ɫLED�ƾ���ָʾ�ƣ�����ʱ���Զ����
	{
		if (auto_run == 1)
		{
			// ��ʾ��Ϣ
			// �Զ����ģʽ
			LED = 0;
		}

		else if (auto_run == 0)
			LED = 1;
	}
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(int voltage)
{
	u8 temp;
	if (Flag_Stop == 1) // ��ص�ѹ���ͣ��رյ��
	{
		Flag_Stop = 1;
		temp = 1;
		BIN1 = 0;
		BIN2 = 0;
	}
	else
		temp = 0;

	if (!(Angle_Balance > (ZHONGZHI - 500) && Angle_Balance < (ZHONGZHI + 500)) || (voltage < 700))
	{
		Flag_Stop = 1;
		temp = 1;
	}

	return temp;
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{
	int temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/**************************************************************************
�������ܣ���λ�궨
��ڲ�������
����  ֵ����
**************************************************************************/
void Find_Zero(void)
{
	static float count;

	// ��ʼ��ʱ autorun_step0=0����Ȼ��ִ��
	if (autorun_step0 == 0) // �ص���λ
	{
		Position_Zero = POSITION_MIDDLE; // ����Ŀ���м�λ��

		// ˳��pid���ƣ��ðڸ˾����ȶ�������Ŀ��Ƕ�����ʼ�ǣ�Ŀ��λ������ʼλ�ã�
		Balance_Pwm = Balance(Angle_Balance + 2070) / 8; // ���PD����
		Position_Pwm = Pre_Position(Encoder);

		// ���ƫ�����е�̫�࣬��ʼPID���ƻ�������
		if (Encoder < 7950)
			Moto = Incremental_PI(Encoder, POSITION_MIDDLE); // λ�ñջ�����;//�뿪��Ե�ط��ٿ�ʼPID����
		else
			Moto = -Balance_Pwm + Position_Pwm; // ��ȡPD���������PWM

		// �жϽǶȺ�λ���Ƿ���ԭʼλ�ã���� ��⵽200�� ��ԭʼλ�ã������Եȴ����
		// �ȴ���ڣ��Ѹո��ù��ı�־λ�����pwmֵȫ�����㣬����Target_Position=POSITION_MIDDL-668�����������ʱ�ĵ�һ��Ŀ��㣬ʹautorun_step0=1��Ҫ�ٽ����������
		if (Angle_Balance < (ANGLE_ORIGIN + 300) && Angle_Balance > (ANGLE_ORIGIN - 300) && (Encoder > (POSITION_MIDDLE - 50) && Encoder < (POSITION_MIDDLE + 50)))
			count++;
		if (Angle_Balance < (ANGLE_ORIGIN + 300) && Angle_Balance > (ANGLE_ORIGIN - 300))
			count += 0.1;
		if (count > 200)
			autorun_step0 = 1, autorun_step1 = 0, Moto = 0, Target_Position = POSITION_MIDDLE - 668; // �ڸ��˶����м�λ�ã�ֹͣ //����Ŀ��λ�ã�׼��˦��
	}

	// �Ե���ٶ��޷�����ֹ����ջ�λ�ÿ��ƹ���
	if (Moto > 2500)
		Moto = 2500; // ����λ�ñջ����ƹ��̵��ٶ�
	if (Moto < -2500)
		Moto = -2500;
	Set_Pwm(Moto); // ���Ƶ��
}
/**************************************************************************
�������ܣ��Զ����
��ڲ�������
����  ֵ����
**************************************************************************/
void Auto_run(void)
{
	static u8 speed = 0;
	static u8 help_count = 0;
	static u16 pid_adjust = 0;

	if (autorun_step1 == 0) // �Զ���ڵ�һ��   ����һ��ִ�У�һ���ǽ������
	{
		// �ж�Ӧ������һ�߻���
		if ((Angle_Balance > (ANGLE_ORIGIN - 120) && Angle_Balance < (ANGLE_ORIGIN + 120)))
		{
			if (D_Angle_Balance <= 0)
				right = 1;
			else if (D_Angle_Balance > 0)
				left = 1;
		}

		// �жϵ��ڸ˻ص���ʼλ��ʱӦ�ø����ٶȺ�λ��
		if (left == 1)
		{
			if ((Angle_Balance > (ANGLE_ORIGIN - 50) && Angle_Balance < (ANGLE_ORIGIN + 50)))
			{
				left = 0;
				Target_Position = POSITION_MIDDLE + 800;
				if (speed > 1)
					Target_Position = POSITION_MIDDLE + 160; // �ðڸ˻�������ֱ�������Ȱ�
			}
		}

		else if (right == 1)
		{
			if ((Angle_Balance > (ANGLE_ORIGIN - 50) && Angle_Balance < (ANGLE_ORIGIN + 50)))
			{
				right = 0;
				Target_Position = POSITION_MIDDLE - 482;
				if (speed > 1)
					Target_Position = POSITION_MIDDLE - 160; // �ðڸ˻�������ֱ�������Ȱ�
			}
		}

		// λ�ñջ�����
		Moto = Position_PID(Encoder, Target_Position);

		// �ڸ��Ѿ������ƽ��㸽������ʼ�������ڽ׶Ρ�
		if (Angle_Balance < (ANGLE_MIDDLE + 385) && Angle_Balance > (ANGLE_MIDDLE - 385))
		{
			speed++;
		}

		// �жϵ�ǰ����Ƿ������ڣ����Ҫ�أ�λ�ò��ڱ�Ե���Ƕ���ƽ��㸽�������ٶȽӽ���0
		if (Angle_Balance < (ANGLE_MIDDLE + 120) && Angle_Balance > (ANGLE_MIDDLE - 120) && (Encoder > 6300 && Encoder < 9300) && (D_Angle_Balance > -30 && D_Angle_Balance < 30))
		{
			speed++;
			success_count++;
		}

		else
			success_count = 0; // ���û��������Χ�ڣ�����Ҫ��������ȴ��´��ж�

		// ����3������������ǿ��Գɹ���ڣ����������˲���pid
		if (success_count > 3)
		{
			autorun_step1 = 1, success_flag = 1;
			Balance_KP = 210, Balance_KD = 150, Position_KP = 8, Position_KD = 130; // �Զ����˲���pid����
		}

		// �޷�
		if (Moto > 4100)
			Moto = 4100; // ����λ�ñջ����ƹ��̵��ٶ�
		if (Moto < -5100)
			Moto = -5100;
	}

	// ��������������������
	else if (success_flag == 1) // ����ߵ㣬���
	{

		if (wait_count == 0)
			Position_Zero = Encoder;		  // ���ǰ���Ȼ�ȡ��ǰλ����Ϊƽ���Ŀ���
		Balance_Pwm = Balance(Angle_Balance); //===�Ƕ�PD����
		if (++Position_Target > 4)
			Position_Pwm = Position(Encoder), Position_Target = 0; //===λ��PD���� 25ms����һ��λ�ÿ���

		// ���һ��ʱ����ðڸ˻����ָ����м��
		wait_count++;
		if (wait_count > 100 && wait_count < 2000)
		{
			if (Position_Zero > 8100)
				Position_Zero--;
			else if (Position_Zero < 8100)
				Position_Zero++;
		}
		if (wait_count > 2000)
			wait_count = 2001; // ��ס������ֵ��ֹ���ѭ��

		// ��ں����pid�׶�	 ���˲�����Ȱ���Ҫ��pid������һ�� ���Ե���ڳɹ������Ȱں󣬼����԰�pid���������Ȱ�ʱ����ֵ
		if (help_count == 0)
		{
			pid_adjust++;
			if (pid_adjust % 100 == 0) // �������� ���������ʧ��
			{
				Balance_KP += 10;
				Balance_KD += 10;
				Position_KP += 0.5;
				Position_KD += 10;
			}
			if (Balance_KP > 400)
				Balance_KP = 400;
			if (Balance_KD > 400)
				Balance_KD = 400;
			if (Position_KP > 20)
				Position_KP = 20;
			if (Position_KD > 300)
				Position_KD = 300;
			if (Balance_KP == 400 && Balance_KD == 400 && Position_KP == 20 && Position_KD == 300)
				help_count = 1; // ����������Ϻ��ͷ�pid��ֵ�޷�����ʱ�û�����ͨ�������ı�pid����
		}

		// �ں����������Ч��
		Moto = Balance_Pwm - Position_Pwm; // ��ȡPD���������PWM
	}
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ��
e(k-1)������һ�ε�ƫ��  �Դ�����
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI(int Encoder, int Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Encoder - Target;							  // ����ƫ��
	Pwm += 10 * (Bias - Last_bias) / 20 + 10 * Bias / 20; // ����ʽPI������
	Last_bias = Bias;									  // ������һ��ƫ��
	return Pwm;											  // �������
}

/**************************************************************************
�������ܣ���ȡ�ڸ˽Ƕȱ仯��
��ڲ�������
����  ֵ����
��    ע��д�ˣ�����û��
**************************************************************************/
void Get_D_Angle_Balance(void)
{
	if (++D_Count > 5) // ��ȡ�Ƕȱ仯�ʣ���� ʱ�䳣��25ms
	{
		D_Angle_Balance = Mean_Filter(Angle_Balance - Last_Angle_Balance); // ƽ���˲��õ�������С�İڸ˽��ٶ���Ϣ
		//			D_Angle_Balance=Angle_Balance-Last_Angle_Balance;	//�õ��ڸ˽��ٶ���Ϣ
		Last_Angle_Balance = Angle_Balance; // ������ʷ����
		D_Count = 0;						// ����������
	}
}

/**************************************************************************
�������ܣ�ƽ�� �˲�
��ڲ������ٶ�
����  ֵ���˲��������
**************************************************************************/
int Mean_Filter(int sensor)
{
	u8 i;
	s32 Sum_Speed = 0;
	s16 Filter_Speed;
	static s16 Speed_Buf[FILTERING_TIMES] = {0};
	for (i = 1; i < FILTERING_TIMES; i++)
	{
		Speed_Buf[i - 1] = Speed_Buf[i];
	}
	Speed_Buf[FILTERING_TIMES - 1] = sensor;

	for (i = 0; i < FILTERING_TIMES; i++)
	{
		Sum_Speed += Speed_Buf[i];
	}
	Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES); //
	return Filter_Speed;
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)��������ƫ��
e(k-1)������һ�ε�ƫ��
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID(int Encoder, int Target)
{

	Position_Least = Encoder - Target; //===
	Position_Bias *= 0.8;
	Position_Bias += Position_Least * 0.2; //===һ�׵�ͨ�˲���
	Position_Differential = Position_Bias - Last_Position;
	Last_Position = Position_Bias;
	Position_PWM = Position_Bias * Position_KP + Position_Differential * Position_KD; //===�ٶȿ���
	return Position_PWM;
}

/**************************************************************************
�������ܣ�˳��λ��PD����
��ڲ�����������
����  ֵ��λ�ÿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Pre_Position(int Encoder)
{
	static float Position_PWM, Last_Position, Position_Bias, Position_Differential;
	Position_Bias = Encoder - Position_Zero;						 //===�õ�ƫ��
	Position_Differential = Position_Bias - Last_Position;			 // ƫ�����
	Last_Position = Position_Bias;									 // ������һ��ƫ��
	Position_PWM = Position_Bias * 25 + Position_Differential * 600; //===λ�ÿ���
	return Position_PWM;											 // ����ֵ
}
