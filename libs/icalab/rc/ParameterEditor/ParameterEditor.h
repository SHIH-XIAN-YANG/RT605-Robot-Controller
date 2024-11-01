/*
* -	Build: B. R. Tseng, 2022/06/06
* - Edit: B. R. Tseng, 2022/08/16
*/
#ifndef __PARAMETER_EDITOR_H__
#define __PARAMETER_EDITOR_H__
//#include<utility>
#pragma warning(suppress : 4996)
#include<array>
#include<string.h>
#include<Windows.h>
#include<stdlib.h>
#include<string.h>
#include<stdio.h>
#include"../../utility/utility_math.h"

//cJSON
#include"cJSON/cJSON.h"

// ==================================================
/*
* 在整個專案中，程式內部送給馬達驅動器的單位都是轉成編碼器的 pulse 之單位，而其餘在 GUI 中顯示的關節角度則是顯示 " 度 " 。
*/
#pragma region Parameter Editor

class KinematicParameter {
public:
	FILE* m_fs;
public:
	struct DH_table {
		char version[10];
		double a[6];
		double alpha[6];
		double d[6];
		double theta[6];
	} DH;
	// Servo motor:
	char PPU_unit;
	char joint_unit;
	char joint_vel_unit;
	char joint_home_unit;
	double PPU;
	std::array<int, 6> encoder_zero_pulse;	
	std::array<short, 6> reduction_ratio;
	std::array<double, 6> motor_rated_torque;
	// Robot joint:
	std::array<double, 6> joint_vel_limit;
	std::array<std::array<double, 6>, 2> joint_limit;
	// Robot configuration: 
	struct SingularThreshold {
		double elbow;
		double wrist;
		double shoulder;
	} singular_threshold;
	std::array<double, 6> joint_home_position;
	std::array<double, 6> endFrame_to_tcp;
	std::array<double, 6> refFrame_to_workspace;
	//
	KinematicParameter(void) {


	}
	bool ImportAllParameterJSON(char* root_dir) {
		//RtPrintf("Original Path: %s\n", root_dir);
		char* pch = strrchr(root_dir, '\\');
		*pch = '\0'; // leave the file directory alone
		pch = strrchr(root_dir, '\\');
		*pch = '\0'; // leave the file directory alone
		pch = strrchr(root_dir, '\\');
		*pch = '\0'; // leave the file directory alone
		strcat(root_dir, "\\");
		char open_file_path[MAX_PATH] = { 0 };
		errno_t err;
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "Parameters.json");
		RtPrintf("JSON file path:%s\n", open_file_path);
		err = fopen_s(&m_fs, open_file_path, "rb");
		if (m_fs == NULL) {
			RtPrintf("Unable to open file %s\n", open_file_path);
			return false;
		}
		// Seek to the end of the file to get its size
		fseek(m_fs, 0, SEEK_END);
		long file_size = ftell(m_fs);
		fseek(m_fs, 0, SEEK_SET);

		// Allocate memory for the file content
		char* file_content = (char*)malloc(file_size + 1);
		if (file_content == NULL) {
			RtPrintf("Memory allocation for JSON file failed");
			fclose(m_fs);
			return false;
		}

		// Read the file into the buffer and add a null terminator
		fread(file_content, 1, file_size, m_fs);
		file_content[file_size] = '\0';

		if (file_content == NULL) {
			return false;
		}

		//Parse the JSON content
		cJSON* json = cJSON_Parse(file_content);
		if (json == NULL) {
			RtPrintf("Error parsing JSON");
			free(file_content);
			return false;
		}

		//Access "robotINF" data
		cJSON * robotINF = cJSON_GetObjectItem(json, "robotINF");
		if (robotINF == NULL) {
			RtPrintf("Error: robotINF not found");
			cJSON_Delete(json);
			free(file_content);
			return 1;
		}

		// Extract Motor Rated Torque
		cJSON* motor_rated_torque_json = cJSON_GetObjectItem(robotINF, "motor_rated_torque");
		if (cJSON_IsArray(motor_rated_torque_json)) {
			cJSON* val;
			int i = 0;
			cJSON_ArrayForEach(val, motor_rated_torque_json) {
				motor_rated_torque[i++] = val->valuedouble;
				//printf("%.2f ", val->valuedouble);
			}
			//printf("\n");
		}

		// Extracting "reduction_ratio"
		cJSON* reduction_ratio_json = cJSON_GetObjectItem(robotINF, "reduction_ratio");
		if (cJSON_IsArray(reduction_ratio_json)) {
			for (int i = 0; i < cJSON_GetArraySize(reduction_ratio_json); i++) {
				reduction_ratio.at(i) = cJSON_GetArrayItem(reduction_ratio_json, i)->valueint;
				RtPrintf("reduction_ratio[%d]: %d\n", i, reduction_ratio.at(i));
			}
		}

		// Extracting "joint_velocity_limit"
		cJSON* joint_velocity_limit_json = cJSON_GetObjectItem(robotINF, "joint_velocity_limit");
		if (cJSON_IsArray(joint_velocity_limit_json)) {
			for (int i = 0; i < cJSON_GetArraySize(joint_velocity_limit_json); i++) {
				joint_vel_limit.at(i) = cJSON_GetArrayItem(joint_velocity_limit_json, i)->valueint;
				RtPrintf("joint_velocity_limit[%d]: %d\n", i, joint_vel_limit.at(i));
			}
		}

		//Extract Motor_zero_pulse
		cJSON* motor_zero_pulse_cjson = cJSON_GetObjectItem(robotINF, "motor_zero_pulse");
		if (cJSON_IsArray(motor_zero_pulse_cjson)) {
			int i = 0;
			cJSON* val;
			cJSON_ArrayForEach(val, motor_zero_pulse_cjson) {
				encoder_zero_pulse[i] = val->valuedouble;
			}
		}

		// Extracting "joint_lim"
		// Parsing joint_lim
		cJSON* joint_lim = cJSON_GetObjectItem(robotINF, "joint_lim");
		if (joint_lim) {
			cJSON* pos = cJSON_GetObjectItem(joint_lim, "pos");
			if (cJSON_IsArray(pos)) {
				int i=0;
				cJSON* val;
				cJSON_ArrayForEach(val, pos) {
					joint_limit[0][i++] = val->valueint;
				}
			}

			cJSON* neg = cJSON_GetObjectItem(joint_lim, "neg");
			if (cJSON_IsArray(neg)) {
				cJSON* val;
				int i = 0;
				cJSON_ArrayForEach(val, neg) {
					joint_limit[1][i++] = val->valueint;
				}
			}
			joint_limit.at(0).at(0) += 90.0; // J1_NOT
			joint_limit.at(1).at(0) += 90.0; // J1_POT
			joint_limit.at(0).at(1) += 90.0; // J2_NOT
			joint_limit.at(1).at(1) += 90.0; // J2_POT
		}


		// Extracting "ppu"
		cJSON* ppu_json = cJSON_GetObjectItem(robotINF, "ppu");
		if (cJSON_IsNumber(ppu_json)) {
			PPU = ppu_json->valuedouble;
			char str[64] = { 0 };
			sprintf(str, "%f", PPU);
			RtPrintf("ppu: %s\n", str);
		}

		// Extracting "joint_singularity"
		cJSON* joint_singularity_json = cJSON_GetObjectItem(robotINF, "joint_singularity");
		if (joint_singularity_json != NULL) {
			cJSON* wrist_threshold_deg = cJSON_GetObjectItem(joint_singularity_json, "wrist_threshold_deg");
			cJSON* elbow_threshold_mm = cJSON_GetObjectItem(joint_singularity_json, "elbow_threshold_mm");
			cJSON* shoulder_threshold_mm = cJSON_GetObjectItem(joint_singularity_json, "shoulder_threshold_mm");

			int wrist_threshold_value = wrist_threshold_deg ? wrist_threshold_deg->valueint : 0;
			int elbow_threshold_value = elbow_threshold_mm ? elbow_threshold_mm->valueint : 0;
			int shoulder_threshold_value = shoulder_threshold_mm ? shoulder_threshold_mm->valueint : 0;
			singular_threshold.wrist = wrist_threshold_value;
			singular_threshold.elbow = elbow_threshold_value;
			singular_threshold.shoulder = shoulder_threshold_value;
			RtPrintf("joint_singularity - wrist_threshold_deg: %d\n", wrist_threshold_value);
			RtPrintf("joint_singularity - elbow_threshold_mm: %d\n", elbow_threshold_value);
			RtPrintf("joint_singularity - shoulder_threshold_mm: %d\n", shoulder_threshold_value);
		}
		singular_threshold.wrist = deg2rad(singular_threshold.wrist); // deg. -> rad.
		singular_threshold.elbow *= 0.001; // mm -> m
		singular_threshold.shoulder *= 0.001; // mm -> m

		// Extracting "joint_home_position"
		cJSON* joint_home_position_json = cJSON_GetObjectItem(robotINF, "joint_home_position");
		if (cJSON_IsArray(joint_home_position_json)) {
			for (int i = 0; i < cJSON_GetArraySize(joint_home_position_json); i++) {
				joint_home_position.at(i) = cJSON_GetArrayItem(joint_home_position_json, i)->valueint;
				RtPrintf("joint_home_position[%d]: %d\n", i, joint_home_position.at(i));
			}
			// Home 之第一軸~第二軸加上 offset：
			joint_home_position[0] += 90.0;
			joint_home_position[1] += 90.0;
		}
		for (unsigned char j = 0; j < 6; ++j) {
			joint_limit[0][j] = deg2rad(joint_limit[0][j]); // NOT
			joint_limit[1][j] = deg2rad(joint_limit[1][j]); // POT
			joint_vel_limit[j] = deg2rad(joint_vel_limit[j]);
			joint_home_position[j] = deg2rad(joint_home_position[j]);
		}

		// Step 4: Clean up
		cJSON_Delete(json);
		free(file_content);
		return true;
	}

	bool ImportAllParameter(char* root_dir) {
		char open_file_path[MAX_PATH] = {0};
		errno_t err;
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\motor_zero_pulse.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import encoder-zero-pulse parameters.\n");
			fscanf(m_fs, "%i,%i,%i,%i,%i,%i", &encoder_zero_pulse.at(0), &encoder_zero_pulse.at(1), &encoder_zero_pulse.at(2), \
				&encoder_zero_pulse.at(3), &encoder_zero_pulse.at(4), &encoder_zero_pulse.at(5));
			fclose(m_fs);
		}
		printf("MotorZero: ");
		for (int i = 0; i < 6; ++i)
			printf("%i, ", encoder_zero_pulse.at(i));
		printf("\n");

		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\PPU.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import PPU parameters.\n");
			fscanf(m_fs, "%lf", &PPU);
			fclose(m_fs);
		}
		printf("PPU: %i x10^-10", (int)(10000000000.0*PPU));
		printf("\n");
		

		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\reduction_ratio.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import reduction-ratio parameters.\n");
			fscanf(m_fs, "%hi,%hi,%hi,%hi,%hi,%hi", &reduction_ratio.at(0), &reduction_ratio.at(1), &reduction_ratio.at(2), \
											  & reduction_ratio.at(3), &reduction_ratio.at(4), &reduction_ratio.at(5));
			fclose(m_fs);
		}
		printf("Ratio: ");
		for(int i = 0; i<6; ++i)
			printf("%i, ", (int)(reduction_ratio.at(i)));
		printf("\n");

		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\motor_rated_torque.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import motor_rated_torque parameters.\n");
			fscanf(m_fs, "%lf,%lf,%lf,%lf,%lf,%lf", &motor_rated_torque.at(0), &motor_rated_torque.at(1), &motor_rated_torque.at(2), \
											  & motor_rated_torque.at(3), &motor_rated_torque.at(4), &motor_rated_torque.at(5));
			fclose(m_fs);
		}
		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\joint_lim.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import joint-limit parameters.\n");
			fscanf(m_fs, "%lf,%lf,%lf,%lf,%lf,%lf,\n\
				          %lf,%lf,%lf,%lf,%lf,%lf", 
				&joint_limit.at(0).at(0), &joint_limit.at(0).at(1), &joint_limit.at(0).at(2), &joint_limit.at(0).at(3), &joint_limit.at(0).at(4), &joint_limit.at(0).at(5),\
				&joint_limit.at(1).at(0), &joint_limit.at(1).at(1), &joint_limit.at(1).at(2), &joint_limit.at(1).at(3), &joint_limit.at(1).at(4), &joint_limit.at(1).at(5));
			fclose(m_fs);
			// 第一軸~第二軸加上 offset：
			joint_limit.at(0).at(0) += 90.0; // J1_NOT
			joint_limit.at(1).at(0) += 90.0; // J1_POT
			joint_limit.at(0).at(1) += 90.0; // J2_NOT
			joint_limit.at(1).at(1) += 90.0; // J2_POT
		}
		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\joint_vel_lim.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import joint-velocity-limit parameters.\n");
			fscanf(m_fs, "%lf,%lf,%lf,%lf,%lf,%lf",
				&joint_vel_limit.at(0), &joint_vel_limit.at(1), &joint_vel_limit.at(2), &joint_vel_limit.at(3), &joint_vel_limit.at(4), &joint_vel_limit.at(5));
			fclose(m_fs);
		}
		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\joint_home_pos.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		if (m_fs != nullptr) {
			printf("Import joint-home-position parameters.\n");
			fscanf(m_fs, "%lf,%lf,%lf,%lf,%lf,%lf",
				&joint_home_position.at(0), &joint_home_position.at(1), &joint_home_position.at(2), &joint_home_position.at(3), &joint_home_position.at(4), &joint_home_position.at(5));
			fclose(m_fs);
		}
		// Home 之第一軸~第二軸加上 offset：
		joint_home_position[0] += 90.0;
		joint_home_position[1] += 90.0;
		// 單位轉換，運動極限與Home點有關之參數，從 degree 轉換成 rad：
		for (unsigned char j = 0; j < 6; ++j) {
			joint_limit[0][j] = deg2rad(joint_limit[0][j]); // NOT
			joint_limit[1][j] = deg2rad(joint_limit[1][j]); // POT
			joint_vel_limit[j] = deg2rad(joint_vel_limit[j]);
			joint_home_position[j] = deg2rad(joint_home_position[j]);
		}

		memset(open_file_path, NULL, MAX_PATH);
		strncpy(open_file_path, root_dir, MAX_PATH);
		strcat(open_file_path, "..\\..\\Parameters\\robotINF\\joint_singularity.txt");
		err = fopen_s(&m_fs, open_file_path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file %s.\n", open_file_path);
			return false;
		}
		char str_tmp[50];
		if (m_fs != nullptr) {
			printf("Import joint_singularity parameters.\n");
			fscanf(m_fs, "%s,%lf,\n%s,%lf,\n%s,%lf,", str_tmp, &singular_threshold.wrist, str_tmp, &singular_threshold.elbow, str_tmp, &singular_threshold.shoulder);
			fclose(m_fs);
		}
		else
			RtPrintf("Cannot open 'joint_singularity.txt' file.");
		// 單位轉換：
		singular_threshold.wrist = deg2rad(singular_threshold.wrist); // deg. -> rad.
		singular_threshold.elbow *= 0.001; // mm -> m
		singular_threshold.shoulder *= 0.001; // mm -> m

		return true;
	}
};

#pragma endregion Parameter Editor
// ==================================================
#endif
