void PostionInner_control()
{
	//时间计算
	static unsigned int PosInnertPre=0;
	unsigned int PosInnert;
	static float PosInnerPID_dt;
	PosInnert=micros();
	PosInnerPID_dt = (PosInnertPre>0)?((PosInnert-PosInnertPre)/1000000.0f):1;
	PosInnertPre=PosInnert;

	/************************ 高度速度内环 ************************/
	//如果要测试纯油门内环实验，把期望（pidHeight.value）改为—>Target_Info.AccHeight
	float verro_us100 = pidHeight.value - RT_Info.US100_Alt_V;
	float vdelta_us100 = (verro_us100 - heightVErr_History)/PosInnerPID_dt;
	/*20Hz低通滤波器*/
	vdelta_us100 = lastheightVDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_us100 - lastheightVDelta);
	
	lastheightVDelta = vdelta_us100;
	heightVErr_History = verro_us100;
	
	pidAccHeight.pOut = verro_us100  * Para_Info.accHeight.Kp;
	pidAccHeight.dOut = vdelta_us100 * Para_Info.accHeight.Kd;
	pidAccHeight.iOut += verro_us100 * Para_Info.accHeight.Ki;
	
	pidAccHeight.iOut = Limits_data(pidAccHeight.iOut,120,-120);	

	pidAccHeight.value = pidAccHeight.pOut
													+pidAccHeight.dOut
															+pidAccHeight.iOut
																	+ pidHeight.feedforwardOut;//加入前馈校准

	pidAccHeight.value = Limits_data(pidAccHeight.value,300,-100);	

	/************************ 位置速度内环 ************************/
	if(BlackspotsFlag == 1 && OpticalflowFlag == 0 && RT_Info.US100_Alt>0.1f)
	{
		/************************ 位置速度内环 ************************/
		float pointvx_Kp = -4.0;
		float pointvx_Ki = -0.001;
		float pointvx_Kd = -0.30;
		// X轴位置速度
		float verro_pointx = pidPointX.value - RT_Info.PointX_V;
		float vdelta_pointx = (verro_pointx - pointxVErr_History)/PosInnerPID_dt;
		/*20Hz低通滤波器*/
		vdelta_pointx = lastpointVxDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_pointx - lastpointVxDelta);
		
		lastpointVxDelta = vdelta_pointx;
		pointxVErr_History = verro_pointx;

		pidXSpeed.pOut = verro_pointx  * pointvx_Kp;
		pidXSpeed.dOut = vdelta_pointx * pointvx_Kd;
		pidXSpeed.iOut += verro_pointx * pointvx_Ki;

		pidXSpeed.iOut = Limits_data(pidXSpeed.iOut,4,-4);	

		pidXSpeed.value = pidXSpeed.pOut
														+pidXSpeed.dOut
																	+pidXSpeed.iOut;
																			 //+pidPointX.feedforwardOut;//加入前馈校准

		pidXSpeed.value = Limits_data(pidXSpeed.value,10,-10);	

		Target_Info.Roll = pidXSpeed.value;

		float pointvy_Kp = -4.0;
		float pointvy_Ki = -0.001;
		float pointvy_Kd = -0.30;
		// Y轴位置速度
		float verro_pointy = pidPointY.value - RT_Info.PointY_V;
		float vdelta_pointy = (verro_pointy - pointyVErr_History)/PosInnerPID_dt;
		/*20Hz低通滤波器*/
		vdelta_pointy = lastpointVyDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_pointy - lastpointVyDelta);
		
		lastpointVyDelta = vdelta_pointy;
		pointyVErr_History = verro_pointy;

		pidYSpeed.pOut = verro_pointy  * pointvy_Kp;
		pidYSpeed.dOut = vdelta_pointy * pointvy_Kd;
		pidYSpeed.iOut += verro_pointy * pointvy_Ki;

		pidYSpeed.iOut = Limits_data(pidYSpeed.iOut,4,-4);	

		pidYSpeed.value = pidYSpeed.pOut
														+pidYSpeed.dOut
																	+pidYSpeed.iOut;
																			//+ pidPointY.feedforwardOut;//加入前馈校准

		pidYSpeed.value = Limits_data(pidYSpeed.value,10,-10);	

		Target_Info.Pitch = pidYSpeed.value;
	}
}
void Calculate_output()
{
	if(FlightControl.droneMode==Drone_Mode_4Axis)
	{										
			Throttle_Info.M1 =  - pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ throttleBasic;
		
			Throttle_Info.M2 =  + pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ throttleBasic;
		
			Throttle_Info.M3 =  + pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ throttleBasic;
		
			Throttle_Info.M4 =  - pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ throttleBasic;
	}
	else if(FlightControl.droneMode==Drone_Mode_Pitch || FlightControl.droneMode==Drone_Mode_RatePitch)
	{
			Throttle_Info.M1 = - pidRatePitch.value + throttleBasic;
			Throttle_Info.M2 = + pidRatePitch.value + throttleBasic;
			Throttle_Info.M3 = + pidRatePitch.value + throttleBasic;
			Throttle_Info.M4 = - pidRatePitch.value + throttleBasic;
	}
	else if(FlightControl.droneMode==Drone_Mode_Roll || FlightControl.droneMode==Drone_Mode_RateRoll)
	{
			Throttle_Info.M1 = - pidRateRoll.value + throttleBasic;
			Throttle_Info.M2 = - pidRateRoll.value + throttleBasic;
			Throttle_Info.M3 = + pidRateRoll.value + throttleBasic;
			Throttle_Info.M4 = + pidRateRoll.value + throttleBasic;
	}
	
	if(Throttle_Info.M1 > 900)  Throttle_Info.M1=900;
	if(Throttle_Info.M2 > 900)  Throttle_Info.M2=900;
	if(Throttle_Info.M3 > 900)  Throttle_Info.M3=900;
	if(Throttle_Info.M4 > 900)  Throttle_Info.M4=900;
	
	if(Throttle_Info.M1 < 50)  Throttle_Info.M1=50;
	if(Throttle_Info.M2 < 50)  Throttle_Info.M2=50;
	if(Throttle_Info.M3 < 50)  Throttle_Info.M3=50;
	if(Throttle_Info.M4 < 50)  Throttle_Info.M4=50;
	
	PID_OUT(Throttle_Info.M1,Throttle_Info.M2,Throttle_Info.M3,Throttle_Info.M4);
}
/*根据不同实验选择不同输出通道
* Motor1-4：电机1-4的PWM输出占空比，范围为0-1000
* FlightMode：飞行模式，也就是实验选择
*/
void PID_OUT(unsigned int Motor1,
						 unsigned int Motor2,
						 unsigned int Motor3,
						 unsigned int Motor4)
{
		Motor1+=1000;
		Motor2+=1000;
		Motor3+=1000;
		Motor4+=1000;
	
	  if(RT_Info.lowPowerFlag==1)
		{
			TIM2->CCR1=1000;
			TIM2->CCR2=1000;
			TIM2->CCR3=1000;
			TIM2->CCR4=1000;
		}
		else
		{
			TIM2->CCR1=Motor1;
			TIM2->CCR2=Motor2;
			TIM2->CCR3=Motor3;
			TIM2->CCR4=Motor4;
		}
}

