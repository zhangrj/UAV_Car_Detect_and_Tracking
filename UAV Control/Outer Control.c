void PostionOuter_control()
{
	/************************ 高度外环调节************************/
	static float Takeoff_weight = 0.015;
	static float Landing_weight = 0.005;
	if(FlightControl.landFlag==1)
	{
		tgtHeight=tgtHeight - Landing_weight;		
		if(RT_Info.US100_Alt<0.10f)
		{
			FlightControl.OnOff = Drone_Land;
		}
	}
	/*********************起飞设置********************/	
	else
	{
		if(tgtHeight < Target_Info.Height)
		{
			tgtHeight = tgtHeight + Takeoff_weight;
		}
		else
		{
			tgtHeight = Target_Info.Height;
		}
	}	
	float Feedforward_height = 35;
	float heightErro = tgtHeight - RT_Info.US100_Alt;
	//前馈校准
	pidHeight.feedforwardOut = Feedforward_height * heightErro;
	
	pidHeight.value = Neurons_PID_Hight(heightErro);

	pidHeight.value = Limits_data(pidHeight.value,3,-3);

	if(RT_Info.US100_Alt>0.1f && BlackspotsFlag == 1 && OpticalflowFlag == 0)
	{	
		/***************X轴PID调节***************/
		//与图像中心点做差获得误差
		float Feedforward_Pointx = 1.0f;
		float PointxErro = RT_Info.PointX;
		//前馈校准
		pidPointX.feedforwardOut = Feedforward_Pointx * Pix_Xinfo;
		
		pidPointX.value = Neurons_PID_Postionx(PointxErro);
	
		pidPointX.value = Limits_data(pidPointX.value,3,-3);	
		/***************Y轴PID调节***************/	
		//与图像中心点做差获得误差	
		float Feedforward_Pointy = 1.0f;
		float PointyErro = RT_Info.PointY;
		//前馈校准
		pidPointY.feedforwardOut = Feedforward_Pointy * Pix_Yinfo;
		
		pidPointY.value = Neurons_PID_Postiony(PointyErro);
		
		pidPointY.value = Limits_data(pidPointY.value,3,-3);	
	}
}



