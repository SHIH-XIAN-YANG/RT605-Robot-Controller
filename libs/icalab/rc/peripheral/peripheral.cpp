#include"peripheral.h"

#pragma region ATI_FtSensor
//
sensor::Axia80::Axia80(int ecat_slave_id, int ecat_io_id) {
	setEtherCATid(ecat_slave_id, ecat_io_id);
}
sensor::Axia80::Axia80(void) {}
sensor::Axia80::~Axia80(void) {}
//
void sensor::Axia80::setEtherCATid(int ecat_slave_id, int ecat_io_id) {
	m_ecat_io_id = ecat_io_id;
	m_ecat_slave_id = ecat_slave_id;
}
//
void sensor::Axia80::setupDAQ(sensor::SamplingRate fs, sensor::LPF fc) { // ATI F/T Sensor Axia-08
	unsigned char Commit_change{ 123 };
	WriteIOSdoObject(m_ecat_slave_id, 0x2020, 0x09, FALSE, &Commit_change, 1);
	//WriteSdoObject(m_ecat_slave_id, 0x2020, 0x09, &Commit_change, 1); // Write "123" to set the configuration.

	DWORD object_0x7010_0x01 = static_cast<DWORD>(fs) | static_cast<DWORD>(fc);

	WriteOutputDWord(m_ecat_io_id, 0, object_0x7010_0x01);
	//
	//DWORD tmp;
	//ReadOutputDWord(m_ecat_io_id, 0, &tmp);
	//printf("\n0x%08x\n", tmp);
	WriteIOSdoObject(m_ecat_io_id, 0x2020, 0x09, FALSE, &Commit_change, 1);
	//WriteSdoObject(m_ecat_io_id, 0x2020, 0x09, &Commit_change, 1); // Write "123" to set the configuration.
}
//
void sensor::Axia80::setTransformSensorToTCP(double* transform_sensor_tcp) { // 載入座標轉換
	memcpy_s(m_transform_sensor_to_tcp, sizeof(double) * 6, transform_sensor_tcp, sizeof(double) * 6);
}
//
void sensor::Axia80::setRobotJointPointer(double* q) {
	m_q = q;
}
//
void sensor::Axia80::setSensorZero(void) {
	// 先記錄下當下校正歸零的關節姿態 m_bias_q:
	this->setBias();
	// 感測器歸零：
	WriteOutputBit(m_ecat_io_id, 0, true);// set bias
	RtSleep(500);
	WriteOutputBit(m_ecat_io_id, 0, false);
}
//
void sensor::Axia80::setBias(void) {
	//DWORD fts_raw[6];
	//for (int i = 0; i < 6; ++i) {
	//	ReadInputDWordB(m_ecat_io_id, i * 4, fts_raw + i);
	//	m_fts_bias.at(i) = std::move(static_cast<double>((long)fts_raw[i]) / 1000000);
	//}
	// 紀錄這個當下要做為感測器校正零的 關節姿態及 對應的重力值：章節3.2.2-C 的步驟(1~(2)
	memcpy(m_bias_q.data(), m_q, sizeof(double) * 6);
	computeGravity(m_bias_q.data(), m_bias_Fg.data());
}
//
void sensor::Axia80::computeGravity(double* q, double* ret_fg) {

	double q2{ q[1] }, q3{ q[2] }, q4{ q[3] }, q5{ q[4] }, q6{ q[5] };
	ret_fg[0] = m_tool.mass * 9.81 * (cos(q2 + q3) * cos(q6) * sin(q5) - sin(q2 + q3) * sin(q4) * sin(q6) + sin(q2 + q3) * cos(q4) * cos(q5) * cos(q6));
	ret_fg[1] = m_tool.mass * -9.81 * (cos(q2 + q3) * sin(q5) * sin(q6) + sin(q2 + q3) * (cos(q4) * cos(q5) * sin(q6) + cos(q6) * sin(q4)));
	ret_fg[2] = m_tool.mass * 9.81 * (cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5));

	ret_fg[3] = (ret_fg[2] * m_tool.rcy - ret_fg[1] * m_tool.rcz + m_tool.mx0);
	ret_fg[4] = (-ret_fg[2] * m_tool.rcx + ret_fg[0] * m_tool.rcz + m_tool.my0);
	ret_fg[5] = (ret_fg[1] * m_tool.rcx - ret_fg[0] * m_tool.rcy + m_tool.mz0);
}
//
void sensor::Axia80::setToolDynamics(sensor::ToolDynamics& tool) {
	memcpy_s(&m_tool, sizeof(sensor::ToolDynamics), &tool, sizeof(sensor::ToolDynamics));
}
void sensor::Axia80::setToolDynamics(double* tool) {
	m_tool.mass = tool[0];
	m_tool.fx0 = tool[1];
	m_tool.fy0 = tool[2];
	m_tool.fz0 = tool[3];
	m_tool.rcx = tool[4];
	m_tool.rcy = tool[5];
	m_tool.rcz = tool[6];
	m_tool.mx0 = tool[7];
	m_tool.my0 = tool[8];
	m_tool.mz0 = tool[9];
}

//
void sensor::Axia80::ReadData(FtData& data_buf) {
	for (int i = 0; i < 6; ++i) {
		ReadInputDWord(m_ecat_io_id, i * 4, m_raw_measurementI32 + i);
		//ReadInputDWordB(m_ecat_io_id, i * 4, m_raw_measurementI32 + i);
		data_buf.raw.at(i) = static_cast<double>((long)m_raw_measurementI32[i]) / 1000000.0;
	}
	// Gravity Compensation:
	m_GravityCompensation(data_buf);
	// Static Force Transformation (i.e. from sensor-frame to TCP-frame)
	m_StaticForceTransform_FtsToTcp(data_buf);
}
//
void sensor::Axia80::GetRawMeasurement(DWORD* raw_data) {
	memcpy_s(raw_data, sizeof(DWORD) * 6, m_raw_measurementI32, sizeof(DWORD) * 6);
}
//
void sensor::Axia80::IdentifyToolDynamics(char* id_data_file) {
	// print results:
	sensor::ToolDynamics tool_dyn = m_tool;
	RtPrintf("Tool before id:\n");
	RtPrintf("\t\tmass:%i (g)\n", (int)(tool_dyn.mass * 1000.0));
	RtPrintf("\t\tFx0:%i (m-N)\n", (int)(tool_dyn.fx0 * 1000.0));
	RtPrintf("\t\tFy0:%i (m-N)\n", (int)(tool_dyn.fy0 * 1000.0));
	RtPrintf("\t\tFz0:%i (m-N)\n", (int)(tool_dyn.fx0 * 1000.0));
	RtPrintf("\t\tMx0:%i (m-Nm)\n", (int)(tool_dyn.mx0 * 1000.0));
	RtPrintf("\t\tMy0:%i (m-Nm)\n", (int)(tool_dyn.my0 * 1000.0));
	RtPrintf("\t\tMz0:%i (m-Nm)\n", (int)(tool_dyn.mz0 * 1000.0));
	RtPrintf("\t\trcx:%i (e-02 mm)\n", (int)(tool_dyn.rcx * 100000.0));
	RtPrintf("\t\trcy:%i (e-02 mm)\n", (int)(tool_dyn.rcy * 100000.0));
	RtPrintf("\t\trcz:%i (e-02 mm)\n", (int)(tool_dyn.rcz * 100000.0));

	unsigned int len{ 0 };
	FILE* fs = nullptr;
	fs = fopen(id_data_file, "r");

	for (int c = getc(fs); !feof(fs); c = getc(fs)) {
		if (c == '\n')
			++len;
	}
	fclose(fs);

	Eigen::MatrixXd Fs;
	Fs.resize(len * 3, 1);
	Eigen::MatrixXd Ms;
	Ms.resize(len * 3, 1);

	Eigen::MatrixXd A;
	A.resize(len * 3, 4);

	double data[18];
	double time;
	double x[6];
	double q1, q2, q3, q4, q5, q6;
	double qc[6];
	double tor[6];
	double fc[6];
	double fts[6];
	fs = nullptr;
	fopen_s(&fs, id_data_file, "r");
	if (fs != nullptr) {
		// exclude the title line:
		for (int c = getc(fs); c != '\n'; c = getc(fs)) {
			Sleep(0);
		}
		//
		for (unsigned int k = 0; k < len; ++k) {
			fscanf_s(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
				&time, x, x + 1, x + 2, x + 3, x + 4, x + 5, qc, qc + 1, qc + 2, qc + 3, qc + 4, qc + 5, &q1, &q2, &q3, &q4, &q5, &q6, tor, tor + 1, tor + 2, tor + 3, tor + 4, tor + 5, \
				fc, fc + 1, fc + 2, fc + 3, fc + 4, fc + 5, fts, fts + 1, fts + 2, fts + 3, fts + 4, fts + 5, \
				data, data + 1, data + 2, data + 3, data + 4, data + 5, data + 6, data + 7, data + 8, data + 9, data + 10, data + 11, data + 12, data + 13, data + 14, data + 15, data + 16, data + 17);

			// 章節3.2.2.B. 的步驟(a)：
			Fs(0 + k * 3, 0) = fts[0];
			Fs(1 + k * 3, 0) = fts[1];
			Fs(2 + k * 3, 0) = fts[2];

			Ms(0 + k * 3, 0) = fts[3];
			Ms(1 + k * 3, 0) = fts[4];
			Ms(2 + k * 3, 0) = fts[5];
			// 計算報告中，每個 k 下的 R_b^s (q) g：(章節3.2.2.B. 的步驟(b) )
			A(0 + k * 3, 0) = 9.81 * (cos(q2 + q3) * cos(q6) * sin(q5) - sin(q2 + q3) * sin(q4) * sin(q6) + sin(q2 + q3) * cos(q4) * cos(q5) * cos(q6));
			A(1 + k * 3, 0) = -9.81 * (cos(q2 + q3) * sin(q5) * sin(q6) + sin(q2 + q3) * (cos(q4) * cos(q5) * sin(q6) + cos(q6) * sin(q4)));
			A(2 + k * 3, 0) = 9.81 * (cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5));
			// 計算 A(q) 矩陣：(章節3.2.2.B. 的步驟(c) )
			A(0 + k * 3, 1) = 1.0; A(0 + k * 3, 2) = 0.0; A(0 + k * 3, 3) = 0.0;
			A(1 + k * 3, 1) = 0.0; A(1 + k * 3, 2) = 1.0; A(1 + k * 3, 3) = 0.0;
			A(2 + k * 3, 1) = 0.0; A(2 + k * 3, 2) = 0.0; A(2 + k * 3, 3) = 1.0;
		}


		fclose(fs);
	}
	else {
		printf("Cannot find the file: %s.\n", id_data_file);
	}
	/* Using the QR decomposition to compute the least square solution: (Eigen Docs: https://eigen.tuxfamily.org/dox/group__LeastSquares.html)
	*		     _     _
	*		    |  mass |
	* Fs =   A *|  Fx0  |
	*	        |  Fy0  |
	*	        |_ Fz0 _|
	*  _     _
	* |  mass |
	* |  Fx0  |  = pinv(A) * Fs
	* |  Fy0  |
	* |_ Fz0 _|
	*
	* where pinv() represent the pseudo-inverse operator.
	*
	*/
	//// 章節3.2.2.B.的步驟(d)：
	Eigen::MatrixXd theta1_hat = A.fullPivHouseholderQr().solve(Fs); // QR decompositio, theta1_hat is the least-square solution (i.e. theta1_hat = [mass, Fx0, Fy0, Fz0]' )
	////theta1_hat = (A.transpose() * A).inverse() * A.transpose() * Fs;

	m_tool.mass = theta1_hat(0, 0);
	m_tool.fx0 = theta1_hat(1, 0);
	m_tool.fy0 = theta1_hat(2, 0);
	m_tool.fz0 = theta1_hat(3, 0);
	////// 章節3.2.2.B.的步驟(e)，計算 B 矩陣：
	Eigen::MatrixXd B;
	B.resize(len * 3, 6);

	double Fgx, Fgy, Fgz;
	for (unsigned int k = 0; k < len; ++k) {
		Fgx = Fs(k * 3 + 0, 0) - m_tool.fx0; // Fg_x
		Fgy = Fs(k * 3 + 1, 0) - m_tool.fy0; // Fg_y
		Fgz = Fs(k * 3 + 1, 0) - m_tool.fz0; // Fg_z
		// B[k] matrix row 1:
		B(k * 3 + 0, 0) = 0.0;  B(k * 3 + 0, 1) = Fgz;  B(k * 3 + 0, 2) = -Fgy; B(k * 3 + 0, 3) = 1.0; B(k * 3 + 0, 4) = 0.0; B(k * 3 + 0, 5) = 0.0;
		// B[k] matrix row 2:
		B(k * 3 + 1, 0) = -Fgz; B(k * 3 + 1, 1) = 0.0;  B(k * 3 + 1, 2) = Fgx;  B(k * 3 + 1, 3) = 0.0; B(k * 3 + 1, 4) = 1.0; B(k * 3 + 1, 5) = 0.0;
		// B[k] matrix row 3:
		B(k * 3 + 2, 0) = Fgy;  B(k * 3 + 2, 1) = -Fgx; B(k * 3 + 2, 2) = 0.0;  B(k * 3 + 2, 3) = 0.0; B(k * 3 + 2, 4) = 1.0; B(k * 3 + 2, 5) = 1.0;
	}
	// // 章節3.2.2.B.的步驟(f)：
	Eigen::MatrixXd theta2_hat = B.fullPivHouseholderQr().solve(Ms);
	//Eigen::MatrixXd theta2_hat = (B.transpose() * B).ldlt().solve(B.transpose() * Ms);
	m_tool.rcx = theta2_hat(0, 0);
	m_tool.rcy = theta2_hat(1, 0);
	m_tool.rcz = theta2_hat(2, 0);
	m_tool.mx0 = theta2_hat(3, 0);
	m_tool.my0 = theta2_hat(4, 0);
	m_tool.mz0 = theta2_hat(5, 0);

	// print results:
	tool_dyn = m_tool;
	RtPrintf("Tool id result:\n");
	RtPrintf("\t\tmass:%i (g)\n", (int)(tool_dyn.mass * 1000.0));
	RtPrintf("\t\tFx0:%i (m-N)\n", (int)(tool_dyn.fx0 * 1000.0));
	RtPrintf("\t\tFy0:%i (m-N)\n", (int)(tool_dyn.fy0 * 1000.0));
	RtPrintf("\t\tFz0:%i (m-N)\n", (int)(tool_dyn.fx0 * 1000.0));
	RtPrintf("\t\tMx0:%i (m-Nm)\n", (int)(tool_dyn.mx0 * 1000.0));
	RtPrintf("\t\tMy0:%i (m-Nm)\n", (int)(tool_dyn.my0 * 1000.0));
	RtPrintf("\t\tMz0:%i (m-Nm)\n", (int)(tool_dyn.mz0 * 1000.0));
	RtPrintf("\t\trcx:%i (e-02 mm)\n", (int)(tool_dyn.rcx * 100000.0));
	RtPrintf("\t\trcy:%i (e-02 mm)\n", (int)(tool_dyn.rcy * 100000.0));
	RtPrintf("\t\trcz:%i (e-02 mm)\n", (int)(tool_dyn.rcz * 100000.0));
}
//
void sensor::Axia80::m_GravityCompensation(FtData& data_buf) {
	double q2{ m_q[1] }, q3{m_q[2] }, q4{ m_q[3] }, q5{ m_q[4] }, q6{ m_q[5] };
	double&& fgx = m_tool.mass * 9.81 * (cos(q2 + q3) * cos(q6) * sin(q5) - sin(q2 + q3) * sin(q4) * sin(q6) + sin(q2 + q3) * cos(q4) * cos(q5) * cos(q6));
	double&& fgy = m_tool.mass * -9.81 * (cos(q2 + q3) * sin(q5) * sin(q6) + sin(q2 + q3) * (cos(q4) * cos(q5) * sin(q6) + cos(q6) * sin(q4)));
	double&& fgz = m_tool.mass * 9.81 * (cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5));
	/*
	* m_bias_Fg[0] 到 m_bias_Fg[5] 為上一次軟體歸零時，記錄當下關節姿態的重力值，對照報告中 章節3.2.2-C 中的方程式 (3.38)，
	* 
	* data_buf.cal[0] 到 data_buf.cal[0][5] 為最後重力補償的量測訊號，對照報告中 章節3.2.2-C 中的方程式 (3.38) ~ (3.39)，但是還沒有逕行座標轉換到 TCP 座標系。
	*/
	data_buf.cal[0] = data_buf.raw[0] + m_bias_Fg[0]  -fgx;
	data_buf.cal[1] = data_buf.raw[1] + m_bias_Fg[1] - fgy;
	data_buf.cal[2] = data_buf.raw[2] + m_bias_Fg[2] - fgz;

	data_buf.cal[3] = data_buf.raw[3] + m_bias_Fg[3] - (fgz * m_tool.rcy - fgy * m_tool.rcz);
	data_buf.cal[4] = data_buf.raw[4] + m_bias_Fg[4] - (-fgz * m_tool.rcx + fgx * m_tool.rcz);
	data_buf.cal[5] = data_buf.raw[5] + m_bias_Fg[5] - (fgy * m_tool.rcx - fgx * m_tool.rcy);
}
//
void sensor::Axia80::m_StaticForceTransform_FtsToTcp(FtData& data_buf)
{
	double b_t69_tmp;
	double c_t69_tmp;
	double t10;
	double t11;
	double t12;
	double t13;
	double t14;
	double t15;
	double t16;
	double t17;
	double t2;
	double t20;
	double t21;
	double t22;
	double t23;
	double t3;
	double t32;
	double t33;
	double t34;
	double t35;
	double t4;
	double t5;
	double t6;
	double t61;
	double t62;
	double t64;
	double t65;
	double t66;
	double t66_tmp;
	double t68;
	double t69_tmp;
	double t7;
	double t70;
	double t8;
	double t9;

	t2 = cos(m_transform_sensor_to_tcp[3]);
	t3 = cos(m_transform_sensor_to_tcp[4]);
	t4 = cos(m_transform_sensor_to_tcp[5]);
	t5 = sin(m_transform_sensor_to_tcp[3]);
	t6 = sin(m_transform_sensor_to_tcp[4]);
	t7 = sin(m_transform_sensor_to_tcp[5]);
	t8 = t2 * t2;
	t9 = t3 * t3;
	t10 = t4 * t4;
	t11 = t5 * t5;
	t12 = t6 * t6;
	t13 = t7 * t7;
	t14 = t2 * t4;
	t15 = t2 * t7;
	t16 = t4 * t5;
	t17 = t5 * t7;
	t20 = t6 * t15;
	t21 = t6 * t16;
	t22 = t6 * t17;
	t23 = t6 * t14;
	t32 = t9 * t10;
	t33 = t9 * t13;
	t34 = t10 * t12;
	t35 = t12 * t13;
	t61 = t14 + t22;
	t62 = t17 + t23;
	t64 = t15 + -t21;
	t65 = t16 + -t20;
	t66_tmp = m_transform_sensor_to_tcp[2] * t6;
	t66 = ((m_transform_sensor_to_tcp[0] * t3 * t4 + m_transform_sensor_to_tcp[1] * t3 * t7) + -(t66_tmp * t10)) + -(t66_tmp * t13);
	t68 = 1.0 / (((t32 + t33) + t34) + t35);
	t69_tmp = m_transform_sensor_to_tcp[2] * t2 * t3;
	b_t69_tmp = m_transform_sensor_to_tcp[1] * t9;
	c_t69_tmp = m_transform_sensor_to_tcp[1] * t12;
	t66_tmp = m_transform_sensor_to_tcp[0] * t9;
	t70 = m_transform_sensor_to_tcp[0] * t12;
	t17 = ((((((m_transform_sensor_to_tcp[0] * t23 + m_transform_sensor_to_tcp[1] * t20) + t69_tmp * t10) + t69_tmp * t13) +
		t66_tmp * t17) +
		t70 * t17) +
		-(b_t69_tmp * t16)) +
		-(c_t69_tmp * t16);
	t16 = m_transform_sensor_to_tcp[2] * t3 * t5;
	t70 = ((((((m_transform_sensor_to_tcp[0] * t21 + m_transform_sensor_to_tcp[1] * t22) + b_t69_tmp * t14) + c_t69_tmp * t14) +
		t16 * t10) +
		t16 * t13) +
		-(t66_tmp * t15)) +
		-(t70 * t15);
	t9 = 1.0 / (((((((t8 * t33 + t8 * t34) + t11 * t32) + t8 * t35) + t11 * t33) +
		t11 * t34) +
		t11 * t35) +
		t8 * t32);
	data_buf.tcp[0] = (-data_buf.cal[2] * t6 + data_buf.cal[0] * t3 * t4) + data_buf.cal[1] * t3 * t7;
	data_buf.tcp[1] = (-data_buf.cal[0] * t64 + data_buf.cal[1] * t61) + data_buf.cal[2] * t3 * t5;
	data_buf.tcp[2] = (data_buf.cal[0] * t62 - data_buf.cal[1] * t65) + data_buf.cal[2] * t2 * t3;
	t12 = t2 * t3;
	t69_tmp = t3 * t5;
	data_buf.tcp[3] = ((((-data_buf.cal[5] * t6 - data_buf.cal[2] * (t12 * t70 * t9 - t69_tmp * t17 * t9)) -
		data_buf.cal[0] * (t62 * t70 * t9 + t64 * t17 * t9)) +
		data_buf.cal[1] * (t61 * t17 * t9 + t65 * t70 * t9)) +
		data_buf.cal[3] * t3 * t4) +
		data_buf.cal[4] * t3 * t7;
	t16 = t3 * t4;
	t66_tmp = t3 * t7;
	data_buf.tcp[4] =
		((((-data_buf.cal[3] * t64 + data_buf.cal[4] * t61) + data_buf.cal[0] * (t62 * t66 * t68 - t16 * t17 * t9)) -
			data_buf.cal[1] * (t65 * t66 * t68 + t66_tmp * t17 * t9)) +
			data_buf.cal[2] * (t6 * t17 * t9 + t12 * t66 * t68)) +
		data_buf.cal[5] * t3 * t5;
	data_buf.tcp[5] =
		((((data_buf.cal[3] * t62 - data_buf.cal[4] * t65) + data_buf.cal[0] * (t64 * t66 * t68 + t16 * t70 * t9)) -
			data_buf.cal[1] * (t61 * t66 * t68 - t66_tmp * t70 * t9)) -
			data_buf.cal[2] * (t6 * t70 * t9 + t69_tmp * t66 * t68)) +
		data_buf.cal[5] * t2 * t3;
}
//
void sensor::Axia80::getToolDynamics(sensor::ToolDynamics* tool_val) {
	memcpy(tool_val, &(this->m_tool), sizeof(sensor::ToolDynamics));
}
#pragma endregion ATI_FtSensor

#pragma region BLDC_Spindle
tool::BLDC_Spindle::BLDC_Spindle(void) {
	m_vel_cmd = 0;
	m_vel = 0;
}
tool::BLDC_Spindle::~BLDC_Spindle(void) {}

void tool::BLDC_Spindle::setEtherCATid(int* m_dac_ecat_id, int* m_dir_ecat_id) {
	m_dac_ecat_slave_id = m_dac_ecat_id[0];
	m_dac_ecat_io_id = m_dac_ecat_id[1];

	m_dir_ecat_slave_id = m_dir_ecat_id[0];
	m_dir_ecat_io_id = m_dir_ecat_id[1];

}
void tool::BLDC_Spindle::TurnOff(void) {

}
WORD tool::BLDC_Spindle::GetSpindleVel_rpm(void) {
	return 0;
}
void tool::BLDC_Spindle::operator()(float vel_cmd) {
	if (vel_cmd >= 100.0)
		vel_cmd = 100.0;
	else if (vel_cmd <= -100.0)
		vel_cmd = -100.0;
	WriteOutputWord(m_dac_ecat_io_id, 0, static_cast<WORD>(vel_cmd));

}

#pragma endregion BLDC_Spindle