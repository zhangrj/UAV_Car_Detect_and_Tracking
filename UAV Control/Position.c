void Position_Estimation(float Ultrasonic,float Xvision,float Yvision,float *Accel)
{
	unsigned int i,j;
	static unsigned int PostiontPre=0;
	unsigned int Postiont;
	static float Position_dt;
	Postiont=micros();
  Position_dt = (PostiontPre>0)?((Postiont-PostiontPre)/1000000.0f):1;
  PostiontPre=Postiont;	
	//预测参数
	static float x_vision=8.0f;
	static float y_vision=8.0f;
	static float z_ultra=10.0f;
	static float acc_bias=0.8f;

	static float X_est[2] = {0.0f,0.0f};//X轴的位移和速度
	static float Y_est[2] = {0.0f,0.0f};//X轴的位移和速度
	static float Z_est[2] = {0.0f,0.0f};//Z轴的高度和速度

	static float Accel_ned[3] = {0.0f,0.0f,0.0f};//地理坐标系下的加速度数据
	static float Accel_bias[3] = {0.0f,0.0f,0.0f};//机体坐标系下的加速度??量

	static float Corr_Ultra = 0.0f;
	static float Corr_Xvision = 0.0f;
	static float Corr_Yvision = 0.0f;
	float Accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
	float Accel_now[3] = {0.0f,0.0f,0.0f};
									
	Accel_now[0] = Accel[0];
	Accel_now[1] = Accel[1];
	Accel_now[2] = Accel[2];
	//位置参数校准
	Corr_Xvision = 0 - Xvision - X_est[0];
	Corr_Yvision = 0 - Yvision - Y_est[0];
  Corr_Ultra = 0 - Ultrasonic - Z_est[0];
	//加速度除?テ?量
  Accel_now[0] -= Accel_bias[0];
  Accel_now[1] -= Accel_bias[1];
  Accel_now[2] -= Accel_bias[2];
	//转化为NED坐标系
  for(i=0; i<3; i++)
  {
		Accel_ned[i]=0.0f;
		for(j=0; j<3; j++)
		{
			Accel_ned[i] += RDrone_R[i][j]* Accel_now[j];
		}
  }
	Accel_ned[2] += CONSTANTS_ONE_G;
	//正?返募铀俣???向量
	Accel_bias_corr[0] -= Corr_Yvision * y_vision * y_vision;
	Accel_bias_corr[1] -= Corr_Xvision * x_vision * x_vision;
  Accel_bias_corr[2] -= Corr_Ultra * z_ultra * z_ultra;
	//转化为机体坐标系
  for (i = 0; i < 3; i++)
  {
		float c = 0.0f;
		for (j = 0; j < 3; j++)
		{
			c += RDrone_R[j][i] * Accel_bias_corr[j];
		}
		Accel_bias[i] += c * acc_bias * Position_dt;
  }
	//X轴预测
	inertial_filter_predict(Position_dt, X_est, Accel_ned[1]);
  inertial_filter_correct(Corr_Xvision, Position_dt, X_est, 0, x_vision);
	//Y轴预测
	inertial_filter_predict(Position_dt, Y_est, Accel_ned[0]);
  inertial_filter_correct(Corr_Yvision, Position_dt, Y_est, 0, y_vision);
	//Z轴预测
	inertial_filter_predict(Position_dt, Z_est, Accel_ned[2]);
  inertial_filter_correct(Corr_Ultra, Position_dt, Z_est, 0, z_ultra);
	
	RT_Info.PointX = X_est[0];
	RT_Info.PointX_V = -X_est[1];

	RT_Info.PointY = Y_est[0];
	RT_Info.PointY_V = -Y_est[1];
	
	RT_Info.US100_Alt = -Z_est[0];
	RT_Info.US100_Alt_V = -Z_est[1];

}