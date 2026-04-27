/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：1.0
修改时间：2021-12-09

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:1.0
Update：2021-12-09

All rights reserved
***********************************************/
#include "control.h"
int Balance_Pwm, Position_Pwm; // 目标角度PWM和目标位置PWM
u8 Position_Target;			   // 用于位置控制的时间
u8 Swing_up = 1;			   // 用于起摆、手动起摆时判断是否是第一次进入手动起摆函数

// 角度PD控制用到的参数
float Bias;				 // 角度偏差
float Last_Bias, D_Bias; // PID相关变量
int balance;			 // PWM返回值

// 位置PD控制用到的参数
float Position_PWM, Last_Position, Position_Bias, Position_Differential;
float Position_Least;

// UAS-SMC控制用到的参数
float k1 = 200, k2 = 2000; // 滑模运行中的系数
int pos_d;
float theta_d;

float pos_last = 0;
float theta_last = 0;

float N;	// 鲁棒项
int u_d;	// 期望的电压
int u_real; // 输出的PWM值

// 分段模糊系数
float k11[4] = {0.4, -0.2, -0.4, 0.2};
float k12[4] = {0.8, 0.4, -0.8, -0.4};
float k21[4] = {0.2, -0.1, -0.2, 0.1};
float k22[4] = {0.4, 0.2, -0.4, -0.2};

// 状态矩阵
float A[4] = {-135.3752, -1.9188, 380.1344, 32.9066}; // A22,A23,A42,A43
// 控制矩阵
float B[2] = {10.5032, -29.4932}; // B21,B41

u8 auto_run = 0;		  // 手动起摆或自动起摆标志位，默认是手动起摆
u8 autorun_step0 = 0;	  // 自动起摆第0步，找中心点，等待起摆
u8 autorun_step1 = 1;	  // 自动起摆第1步
u8 autorun_step2 = 0;	  // 自动起摆第2步
long Target_Position;	  // 目标位置
float D_Angle_Balance;	  // 摆杆角度变化率
long success_count = 0;	  // 摆杆在平衡位置的成功次数记录
u8 success_flag = 0;	  // 自动起摆时，平衡成功，进入起摆后标志位
long wait_count = 0;	  // 等待起摆成功后延时时间达到，获取起摆成功后位置
long D_Count;			  // 用于获取摆杆角度变化率的中间计数变量
float Last_Angle_Balance; // 用于获取摆杆角度变化率后的中间存放上一次角度

u8 left, right;
/**************************************************************************
函数功能：所有的控制代码都在这里面
		  TIM1控制的5ms定时中断
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1)) // 确定指向的是TIM1中断发生的 //5ms定时中断,也就是控制频率为200Hz //实际上的发PWM的发送频率是10kHz
	{
		if (delay_flag == 1)
		{
			if (++delay_50 == 10)
				delay_50 = 0, delay_flag = 0; //===为主函数提供50ms的精准延时  10次*5ms = 50ms。目的仅为让上位机波形传输稳定
		}
		Encoder = Read_Encoder(4);				//===更新编码器位置信息（当前位置）
		Angle_Balance = Get_Adc_Average(3, 10); //===更新摆杆角度
		Get_D_Angle_Balance();					//===获取摆杆角速度

		//Turn_Off(Voltage);
		
		if (auto_run == 1) // UAS_SMC控制模式(长按预留按键二即可)
		{
			if (Flag_Stop == 0)
			{
				Turn_Off(Voltage); // 倾角、电压保护
				if (Swing_up == 0) // 默认为1，这里使能其实没有被用到（一开始是不想按一下复位键就能进入滑膜，想用此来做特定的进入条件）
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
			Turn_Off(Voltage); // 倾角、电压保护
			if (Swing_up == 0)
				Position_Zero = Encoder, Last_Position = 0, Last_Bias = 0, Position_Target = 0, Swing_up = 1;

			if (Flag_Stop == 0)
			{
				Balance_Pwm = Balance(Angle_Balance); //===角度PD控制
				if (++Position_Target > 4)
					Position_Pwm = Position(Encoder), Position_Target = 0; //===位置PD控制 25ms进行一次位置控制
				Moto = Balance_Pwm - Position_Pwm;						   //===计算出来的PWM
				Xianfu_Pwm();											   //===PWM限幅 防止占空比超100%，保证系统稳定性
				Set_Pwm(Moto);											   //===赋值给PWM的寄存器
			}
		}

		if (Flag_Stop == 1)
			Set_Pwm(0);
	}
	if (Flag_Stop == 0)
		Led_Flash(100);			  //===LED闪烁指示系统正常运行
	Voltage = Get_battery_volt(); //===读取电池电压
	Key();						  //===扫描按键变化
}

/**************************************************************************
函数功能：将编码器值转化为位置
入口参数：编码器值
返回  值：位置
**************************************************************************/
float pos_real(int pos_encoder)
{
	return (pos_encoder * K);
}
/**************************************************************************
函数功能：ADC值转化为角度
入口参数：ADC电压值、未转化角度
返回  值：角度值
**************************************************************************/
float angle_real(float angel_ADC)
{
	return ((2 * PI) * (angel_ADC / 4096.0f));
}

/**************************************************************************
函数功能：UAS-SMC控制
入口参数：角度
返回  值：平衡控制PWM
**************************************************************************/
int UAS_SMC_Control(float theta, int pos)
{
	float theta_e, theta_dot, theta_e_dot; // 误差
	float pos_e, pos_dot, pos_e_dot;	   // 位置

	// [修复 Bug 2]：静态变量用于微分项的低通滤波
	static float last_pos_dot = 0;
	static float last_theta_dot = 0;
	const float alpha = 0.3f; // 滤波系数，可以微调，越小平滑度越高

	theta_d = ZHONGZHI;
	pos_d = Position_Zero;

	float p = pos_real(pos - pos_d), p_d = 0;
	float a = angle_real(theta - theta_d), a_d = 0;

	float fx1, fx2;
	float gx1 = B[0], gx2 = B[1];

	float s1[2] = {0};
	float s2[2] = {0};

	float m1[2] = {0}, m2[2] = {0}; // 当前分段模型系数

	// 1. 计算原始微分
	float raw_pos_dot = (p - pos_last) / TS;
	float raw_theta_dot = (a - theta_last) / TS;

	// 2. 应用一阶低通滤波
	pos_dot = (1.0f - alpha) * last_pos_dot + alpha * raw_pos_dot;
	theta_dot = (1.0f - alpha) * last_theta_dot + alpha * raw_theta_dot;

	last_pos_dot = pos_dot;
	last_theta_dot = theta_dot;

	fx1 = A[0] * pos_dot + A[1] * a; // 根据状态方程求解出对应的系统的误差模型和电压值的映射
	fx2 = A[2] * pos_dot + A[3] * a;

	pos_e = p - p_d;
	pos_e_dot = pos_dot; // 直接把参考值作为零来求导得到

	theta_e = a - a_d;
	theta_e_dot = theta_dot;

	s1[0] = pos_e + 1.5f * pos_e_dot;
	s1[1] = pos_e + (1.0f / 6.0f) * pos_e_dot;

	s2[0] = theta_e + 1.5f * theta_e_dot;
	s2[1] = theta_e + (1.0f / 6.0f) * theta_e_dot;

	// [修复 Bug 1 - 方案A] 去掉绝对值 fabsf()，保留方向性，避免单侧累积误差推倒小车
	N = k1 * pos_e + k2 * theta_e;

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

	float den = (m1[1] * gx1 + m2[1] * gx2); // 分母为零保护（理论上不会到0）
	if (fabsf(den) < 1e-6f)
		den = (den >= 0 ? 1e-6f : -1e-6f);
	u_d = (1.0f / den) * (-(m1[0] * pos_e_dot + m2[0] * theta_e_dot) - (m1[1] * fx1 + m2[1] * fx2) + N);

	u_real = (int)(U2PWM * u_d);
	return -u_real; // 注意有个负号，这个负号是因为当前设定的角度方向（向前为正）和电机输出PWM的方向（应向前却为负）不一致
}

/**************************************************************************
函数功能：角度PD控制
入口参数：角度
返回  值：平衡控制PWM
作    者：平衡小车之家
**************************************************************************/
int Balance(float Angle)
{
	Bias = Angle - ZHONGZHI;							// 求出平衡的角度中值和机械的重合，ZHONGZHI是机械原点，
	D_Bias = Bias - Last_Bias;							// 求出偏差的微分 进行微分控制
	balance = -Balance_KP * Bias - D_Bias * Balance_KD; //===计算平衡控制的电机PWM  PD控制
	Last_Bias = Bias;									// 保存上一次的偏差
	return balance;
}

/**************************************************************************
函数功能：位置PD控制
入口参数：编码器
返回  值：位置控制PWM
作    者：平衡小车之家
**************************************************************************/
int Position(int Encoder)
{
	Position_Least = Encoder - Position_Zero; //===求出平衡位置
	Position_Bias *= 0.8;
	Position_Bias += Position_Least * 0.2; //===一阶低通滤波器
	Position_Differential = Position_Bias - Last_Position;
	Last_Position = Position_Bias;
	Position_PWM = Position_Bias * Position_KP + Position_Differential * Position_KD; //===速度控制
																					  //    Position_PWM=Position_Bias*(Position_KP+Basics_Position_KP)/2+Position_Differential*(Position_KD+Basics_Position_KD)/2; //===位置控制
	return Position_PWM;
}

/**************************************************************************
函数功能：赋值给PWM的寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto)
{
	if (moto < 0)
		BIN2 = 1, BIN1 = 0; // 转向判断
	else
		BIN2 = 0, BIN1 = 1;
	PWMB = myabs(moto);
}

/**************************************************************************
函数功能：限制PWM赋值
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
	int Amplitude = 6900; //===PWM满幅是7200 限制在6900
	if (Moto < -Amplitude)
		Moto = -Amplitude;
	if (Moto > Amplitude)
		Moto = Amplitude;
}
/**************************************************************************
函数功能：按键修改控制摆杆的位置
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	// 本产品使用的电机13线霍尔编码器，减速比20，使用定时器4倍频，轮子转一圈，编码器读数是 13*4*20 = 1040
	int position = 1040; // 目标位置 小车原始位置是10000  转一圈是1040 也就是如果小车往左转，原本是摆杆Z向转一圈，这里减1040，就是右边
	static int tmp, flag, count;
	tmp = click_N_Double(100);

	if (tmp == 1)
		flag = 1; //++
	if (tmp == 2)
		flag = 2; //--

	if (flag == 1) // 摆杆往左边方向运动
	{
		Position_Zero += 4;
		count += 4;
		if (count == position)
			flag = 0, count = 0;
	}
	if (flag == 2) // 摆杆往反方向运动
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

	if (Flag_Stop == 1) // 在电机停止的时候长按预留按键二切换模式并把蓝色LED灯作为指示灯，亮起时是自动起摆
	{
		if (auto_run == 1)
		{
			// 显示信息
			// 自动起摆模式
			LED = 0;
		}

		else if (auto_run == 0)
			LED = 1;
	}
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(int voltage)
{
	u8 temp;
	if (Flag_Stop == 1) // 电池电压过低，关闭电机
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
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
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
函数功能：找位标定
入口参数：无
返回  值：无
**************************************************************************/
void Find_Zero(void)
{
	static float count;

	// 初始化时 autorun_step0=0，必然被执行
	if (autorun_step0 == 0) // 回到零位
	{
		Position_Zero = POSITION_MIDDLE; // 设置目标为中间位置

		// 顺序pid控制，让摆杆绝对稳定向下，目标角度是初始角，目标位置也是初始位置，
		Balance_Pwm = Balance(Angle_Balance + 2070) / 8; // 角度PD控制
		Position_Pwm = Pre_Position(Encoder);

		// 防止偏离零位太多，起摆PID控制会撞向边缘
		if (Encoder < 7950)
			Moto = Incremental_PI(Encoder, POSITION_MIDDLE); // 位置闭环控制;//离开边缘地方才开始PID起摆
		else
			Moto = -Balance_Pwm + Position_Pwm; // 获取PD控制出来的PWM

		// 判断角度和位置是否在原始位置，是在 检测到200次 在原始位置，认为稳定等起摆
		// 起摆期间，把刚才的标志位置一，把pwm值全置零，并在Target_Position=POSITION_MIDDL-668设置起摆时的第一个目标点，使autorun_step0=1不再进入此函数
		if (Angle_Balance < (ANGLE_ORIGIN + 300) && Angle_Balance > (ANGLE_ORIGIN - 300) && (Encoder > (POSITION_MIDDLE - 50) && Encoder < (POSITION_MIDDLE + 50)))
			count++;
		if (Angle_Balance < (ANGLE_ORIGIN + 300) && Angle_Balance > (ANGLE_ORIGIN - 300))
			count += 0.1;
		if (count > 200)
			autorun_step0 = 1, autorun_step1 = 0, Moto = 0, Target_Position = POSITION_MIDDLE - 668; // 摆杆运动到中间位置，停止 //设置目标位置，准备晃起
	}

	// 适当速度限幅，防止撞向边缘或者闭环位置控制过快
	if (Moto > 2500)
		Moto = 2500; // 限制位置闭环控制过程的速度
	if (Moto < -2500)
		Moto = -2500;
	Set_Pwm(Moto); // 控制电机
}
/**************************************************************************
函数功能：自动起摆
入口参数：无
返回  值：无
**************************************************************************/
void Auto_run(void)
{
	static u8 speed = 0;
	static u8 help_count = 0;
	static u16 pid_adjust = 0;

	if (autorun_step1 == 0) // 自动起摆第一步   （起摆一直执行，一旦起摆进入起摆）
	{
		// 判断应该往哪边晃动
		if ((Angle_Balance > (ANGLE_ORIGIN - 120) && Angle_Balance < (ANGLE_ORIGIN + 120)))
		{
			if (D_Angle_Balance <= 0)
				right = 1;
			else if (D_Angle_Balance > 0)
				left = 1;
		}

		// 判断到摆杆回到初始位置时应该给速度和位置
		if (left == 1)
		{
			if ((Angle_Balance > (ANGLE_ORIGIN - 50) && Angle_Balance < (ANGLE_ORIGIN + 50)))
			{
				left = 0;
				Target_Position = POSITION_MIDDLE + 800;
				if (speed > 1)
					Target_Position = POSITION_MIDDLE + 160; // 让摆杆回去时一直直给到刚好起摆
			}
		}

		else if (right == 1)
		{
			if ((Angle_Balance > (ANGLE_ORIGIN - 50) && Angle_Balance < (ANGLE_ORIGIN + 50)))
			{
				right = 0;
				Target_Position = POSITION_MIDDLE - 482;
				if (speed > 1)
					Target_Position = POSITION_MIDDLE - 160; // 让摆杆回去时一直直给到刚好起摆
			}
		}

		// 位置闭环控制
		Moto = Position_PID(Encoder, Target_Position);

		// 摆杆已经跨越过平衡点附近，下次进入判断角度阶段。
		if (Angle_Balance < (ANGLE_MIDDLE + 385) && Angle_Balance > (ANGLE_MIDDLE - 385))
		{
			speed++;
		}

		// 判断当前角度是否在起摆期，位置要好，位置不能在边缘并且角度在平衡点附近且角速度接近于0
		if (Angle_Balance < (ANGLE_MIDDLE + 120) && Angle_Balance > (ANGLE_MIDDLE - 120) && (Encoder > 6300 && Encoder < 9300) && (D_Angle_Balance > -30 && D_Angle_Balance < 30))
		{
			speed++;
			success_count++;
		}

		else
			success_count = 0; // 只要没有在这个范围内，说明起摆要失败，等待下次判断

		// 经过3次的检测如果还是在起摆期内，就进入起摆后起步pid
		if (success_count > 3)
		{
			autorun_step1 = 1, success_flag = 1;
			Balance_KP = 210, Balance_KD = 150, Position_KP = 8, Position_KD = 130; // 自动起摆起步pid参数
		}

		// 限幅
		if (Moto > 4100)
			Moto = 4100; // 限制位置闭环控制过程的速度
		if (Moto < -5100)
			Moto = -5100;
	}

	// 开始执行起步起摆起步进入起步起摆
	else if (success_flag == 1) // 到最高点，接管
	{

		if (wait_count == 0)
			Position_Zero = Encoder;		  // 刚起摆时先获取当前位置作为平衡的目标
		Balance_Pwm = Balance(Angle_Balance); //===角度PD控制
		if (++Position_Target > 4)
			Position_Pwm = Position(Encoder), Position_Target = 0; //===位置PD控制 25ms进行一次位置控制

		// 经过一段时间后再让摆杆回回到中间位
		wait_count++;
		if (wait_count > 100 && wait_count < 2000)
		{
			if (Position_Zero > 8100)
				Position_Zero--;
			else if (Position_Zero < 8100)
				Position_Zero++;
		}
		if (wait_count > 2000)
			wait_count = 2001; // 卡住计数变量防止其溢出

		// 起摆后的辅助pid阶段	 起步参数起摆需要让pid稍微大一点 小车到起摆成功后能起摆后，开始用起步pid慢慢渐变到起摆起摆起摆的参数值
		if (help_count == 0)
		{
			pid_adjust++;
			if (pid_adjust % 100 == 0) // 参数变大 防止起摆后失控
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
				help_count = 1; // 辅助起步达到最佳释放pid阀值限幅进入平时稳定通过按键改变pid状态
		}

		// 起步起步后的接管起效
		Moto = Balance_Pwm - Position_Pwm; // 获取PD控制出来的PWM
	}
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI(int Encoder, int Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Encoder - Target;							  // 计算偏差
	Pwm += 10 * (Bias - Last_bias) / 20 + 10 * Bias / 20; // 增量式PI控制器
	Last_bias = Bias;									  // 保存上一次偏差
	return Pwm;											  // 增量输出
}

/**************************************************************************
函数功能：获取摆杆角度变化率
入口参数：无
返回  值：无
作    者：大轮明王，大隐于市
**************************************************************************/
void Get_D_Angle_Balance(void)
{
	if (++D_Count > 5) // 获取角度变化率，积分 时间常数25ms
	{
		D_Angle_Balance = Mean_Filter(Angle_Balance - Last_Angle_Balance); // 平滑滤波得到噪声更小的摆杆角速度信息
		//			D_Angle_Balance=Angle_Balance-Last_Angle_Balance;	//得到摆杆角速度信息
		Last_Angle_Balance = Angle_Balance; // 保存历史数据
		D_Count = 0;						// 清零计数变量
	}
}

/**************************************************************************
函数功能：平滑 滤波
入口参数：速度
返回  值：滤波后的速度
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
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID(int Encoder, int Target)
{

	Position_Least = Encoder - Target; //===
	Position_Bias *= 0.8;
	Position_Bias += Position_Least * 0.2; //===一阶低通滤波器
	Position_Differential = Position_Bias - Last_Position;
	Last_Position = Position_Bias;
	Position_PWM = Position_Bias * Position_KP + Position_Differential * Position_KD; //===速度控制
	return Position_PWM;
}

/**************************************************************************
函数功能：顺序位置PD控制
入口参数：编码器
返回  值：位置控制PWM
作    者：平衡小车之家
**************************************************************************/
int Pre_Position(int Encoder)
{
	static float Position_PWM, Last_Position, Position_Bias, Position_Differential;
	Position_Bias = Encoder - Position_Zero;						 //===得到偏差
	Position_Differential = Position_Bias - Last_Position;			 // 偏差微分
	Last_Position = Position_Bias;									 // 保存上一次偏差
	Position_PWM = Position_Bias * 25 + Position_Differential * 600; //===位置控制
	return Position_PWM;											 // 返回值
}
