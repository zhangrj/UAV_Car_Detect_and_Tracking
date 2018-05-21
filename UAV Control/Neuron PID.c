/*
	单神经元PID
	利用单神经元公式
	u(k) = u(k-1) + K*(w11(k)*x1(k) + w22(k)*x2(k) + w33(k)*x3(k))
	权重归一
	w11(k) = w1(k)/(w1(k) + w2(k) + w3(k))
	w22(k) = w2(k)/(w1(k) + w2(k) + w3(k))
	w33(k) = w3(k)/(w1(k) + w2(k) + w3(k))
	学习规则
	w1(k) = w1(k-1) + ni*z(k)*u(k)*x1(k)
	w2(k) = w2(k-1) + np*z(k)*u(k)*x2(k)
	w3(k) = w3(k-1) + nd*z(k)*u(k)*x3(k)
	pid增量参数
	x1(k) = e(k)
	x2(k) = e(k) - e(k-1)
	x3(k) = e(k) - 2*e(k-1) + e(k-2)
	z(k) = e(k)
	
	其中ni，np，nd分别为积分，比例，微分的学习速率，K为神经元的比例参数，K>0。
*/
float Neurons_PID_Hight(float Errdata)
{
	static float x1,x2,x3;
	height_err[2] = Errdata;
	//Hebb learning algorithm
	heightw1 = heightlastw1 + np_height * height_err[2] * output_height * x1;
	heightw2 = heightlastw2 + ni_height * height_err[2] * output_height * x2;
	heightw3 = heightlastw3 + nd_height * height_err[2] * output_height * x3;	
	
	x1 = height_err[2] - height_err[1];
	x2 = height_err[2];
	x3 = height_err[2] - 2*height_err[1] + height_err[0];	

	height_Average = (Abs_Funcation(heightw1) + Abs_Funcation(heightw2) + Abs_Funcation(heightw3));	
	
	if(height_Average == 0)
	{
		height_Average = 0.0001;
	}
	
	heightAverage_w1 = heightw1/height_Average;
	heightAverage_w2 = heightw2/height_Average;
	heightAverage_w3 = heightw3/height_Average;
	
	output_height = lastoutput_height + Neurons_k_hight * (heightAverage_w1*x1 + heightAverage_w2*x2 + heightAverage_w3*x3);
	
	height_err[0] = height_err[1];
	height_err[1] = height_err[2];
	
	lastoutput_height = output_height;

	heightlastw1 = heightw1;
	heightlastw2 = heightw2;
	heightlastw3 = heightw3;

	return output_height;
}

float Neurons_PID_Postionx(float Errdatax)
{
	static float x1_x,x2_x,x3_x;
	posx_err[2] = Errdatax;
	//Hebb learning algorithm
	posxw1 = posxlastw1 + np_posx * posx_err[2] * output_posx * x1_x;
	posxw2 = posxlastw2 + ni_posx * posx_err[2] * output_posx * x2_x;
	posxw3 = posxlastw3 + nd_posx * posx_err[2] * output_posx * x3_x;
	
	x1_x = posx_err[2] - posx_err[1];
	x2_x = posx_err[2];
	x3_x = posx_err[2] - 2*posx_err[1] + posx_err[0];
	
	posx_Average = (Abs_Funcation(posxw1) + Abs_Funcation(posxw2) + Abs_Funcation(posxw3));
	
	if(posx_Average == 0)
	{
		posx_Average = 0.0001;
	}
	
	posxAverage_w1 = posxw1/posx_Average;
	posxAverage_w2 = posxw2/posx_Average;
	posxAverage_w3 = posxw3/posx_Average;
	
	output_posx = lastoutput_posx + Neurons_k_posx * (posxAverage_w1*x1_x + posxAverage_w2*x2_x + posxAverage_w3*x3_x);
	
	posx_err[0] = posx_err[1];
	posx_err[1] = posx_err[2];
	
	lastoutput_posx = output_posx;
	
	posxlastw1 = posxw1;
	posxlastw2 = posxw2;
	posxlastw3 = posxw3;
	
	return output_posx;
}

float Neurons_PID_Postiony(float Errdatay)
{
	static float x1_y,x2_y,x3_y;
	posy_err[2] = Errdatay;
	//Hebb learning algorithm
	posyw1 = posylastw1 + np_posy * posy_err[2] * output_posy * x1_y;
	posyw2 = posylastw2 + ni_posy * posy_err[2] * output_posy * x2_y;
	posyw3 = posylastw3 + nd_posy * posy_err[2] * output_posy * x3_y;
	
	x1_y = posy_err[2] - posy_err[1];
	x2_y = posy_err[2];
	x3_y = posy_err[2] - 2*posy_err[1] + posy_err[0];
	
	posy_Average = (Abs_Funcation(posyw1) + Abs_Funcation(posyw2) + Abs_Funcation(posyw3));
	
	if(posy_Average == 0)
	{
		posy_Average = 0.0001;
	}
	
	posyAverage_w1 = posyw1/posy_Average;
	posyAverage_w2 = posyw2/posy_Average;
	posyAverage_w3 = posyw3/posy_Average;
	
	output_posy = lastoutput_posy + Neurons_k_posy * (posyAverage_w1*x1_y + posyAverage_w2*x2_y + posyAverage_w3*x3_y);
	
	posy_err[0] = posy_err[1];
	posy_err[1] = posy_err[2];
	
	lastoutput_posy = output_posy;
	
	posylastw1 = posyw1;
	posylastw2 = posyw2;
	posylastw3 = posyw3;
	
	return output_posy;
}


float Abs_Funcation(float pSrc)
{
	float pDst;
	arm_abs_f32(&pSrc,&pDst,1);
	return pDst;
}



