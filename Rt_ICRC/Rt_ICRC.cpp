/*
*  - Create: 2022/07/07, b.r.tseng
*  - Edit: 2022/12/03. b.r.tseng
*/
#include "Rt_ICRC.h"

HANDLE ui_handle;
MailBox* ui;
HANDLE h_ui_accessEvent;
HANDLE h_ui_ackEvent;
HANDLE h_ui_PID;
PID* rt605_PID;

HANDLE h_ui_RTSS_Task;
RTSSTask_Status* rtss_task_status;

#pragma region GLOBAL_VARIABLE 
// Slave indice 可透過 RobotController::.ConfigEtherCATslaveID() 的執行以進行設定
int SERVO_DRIVER_ECAT_SLAVE_ID[6];
int ATI_FT_SENSOR_ECAT_SLAVE_ID;
int ATI_FT_SENSOR_ECAT_IO_ID;
int  BLDC_SPINDLE_DAC_ECAT_SLAVE_ID;
int BLDC_SPINDLE_DAC_IO_ID;
#pragma endregion GLOBAL_VARIABLE
//
void ReturnAckToWinUI(void);
void ReturnAckToWinUI(char*);

#include <stdio.h>
#include <windows.h>
#include <tchar.h>
#include <rtapi.h> 

#include"rt_robot_controller.h"
int _tmain(int argc, _TCHAR* argv[])
{
    // ======================= Declare the robot controller object: =======================
    RT_Robot_Controller rc;
    // 取得當前程式的絕對路徑：
    rc.GetRtssAbsolutePath(argv[0]);
    rc.ConfigEtherCATslaveID();
    
    // ======================= create the GUI mailbox: ======================
    h_ui_accessEvent = RtCreateEvent(NULL, TRUE, TRUE, UI_EVENT_NAME);
    h_ui_ackEvent    = RtCreateEvent(NULL, FALSE, FALSE, UI_ACK_EVENT_NAME);
    ui_handle        = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(MailBox), UI_MAILBOX_NAME, (void**)&ui);
    h_ui_PID         = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(PID), SHM_PID_NAME, (void**)&rt605_PID);
    h_ui_RTSS_Task   = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(RTSSTask_Status), SHM_RTSS_STATE_NAME, (void**)&rtss_task_status);
    // =============================================
    RtPrintf("======================= Program Start ======================\n");
    RtPrintf(" Main thread ID: %d\n", GetCurrentThreadId());
    RtPrintf("Wait for system connection .......\n");

    int nTry = 18000;
    while (!(ui->isConnected(h_ui_accessEvent))) {
        RtSleep(100);
        if (ui->Action == UI_Action::TerminateRtProcess) {
            goto TERMINATE;
        }
        if (--nTry == 0) {
            goto TERMINATE;
        }
        // 最多嘗試 30 分鐘，共嘗試 18000 次，若未成功連上線程式將直接結束。
    }
    ui->Ack = false;
    RtPrintf("- Connect with ICRC-WinUI\n");
    RtPrintf("======================= ICRC-RTSS: Initialize ======================\n");
    // 初始化，以進行讀取機械手臂的重要參數與相關設定：
    int ret = rc.Initialize();
    if (ret != 0) {
        RtPrintf("Initialization failed... ret: %d\n", ret);
        goto TERMINATE;
    }
    // 初始化其他子模組中會需要的功能：
    rc.ImportRobotParameter(rc.app_dir);
    rc.system_state->ksm_started = true;
    RtPrintf("======================= Start Operation ======================\n");
    // setup cyclic routines:
    rc.setFeedbackRoutine(     MOTION_FEEDBACK_CYCLE, MotionFeedbackRoutine);
    rc.setSafetyDetectRoutine( SAFETY_DETECT_CYCLE,   SafetyDetectRoutine);
    rc.setPositionMotionRoutine(    MOTION_CYCLE,     RtMotionRoutine_Position);
    //rc.setPositionForceMotionRoutine(MOTION_CYCLE, RtMotionRoutine_HybridForcePosition);
    // 啟動各個 Cyclic Routine：
    rc.LogOff(); // 關閉運動狀態紀錄器
    rc.StartFeedbackRoutine();
    rc.StartSafetyDetectRoutine();
    rc.getIOState();

    // 
    ReturnAckToWinUI();

    while (rc.system_state->ksm_started) { // event-loop:
        MailBox uiMessage_tmp;
        RtPrintf("wait cmd\n");
        RtWaitForSingleObject(h_ui_ackEvent, INFINITE);
        RtResetEvent(h_ui_ackEvent);
        RtPrintf("- UI Message: ");
        ui->ReadMessage(h_ui_accessEvent, uiMessage_tmp);
        ui->Busy = false;
        if (ui->isConnected(h_ui_accessEvent) && uiMessage_tmp.Receiver == Node::RTSS_main) {
            RtPrintf("Current Action: %d\n", uiMessage_tmp.Action);
            switch (uiMessage_tmp.Action) {
            case UI_Action::TerminateRtProcess: {
                    RtPrintf("TerminateRtProcess\n");
                    rc.system_state->ksm_started = false;
                    ui->Connected = false;
                    break;
                }
                case UI_Action::Disconnect: {
                    RtPrintf("Disconnect\n");

                    rc.stopEtherCAT();
                    break;
                }
                case UI_Action::ImportRobotPrt: {
                    RtPrintf("ImportRobotPrt\n");
                    rc.ImportRobotParameter(rc.app_dir);
                    break;
                }
                                              //----- Group-0: ------
                case UI_Action::ServoON: {

                    int ret = rc.ServoON();
                    if (ret != 0) {
                        RtPrintf("Error: Servo On failed...\n");
                    }
                    else {
                        RtPrintf("Servo ON ......\n");
                    }
                    break;
                }
                case UI_Action::ServoOFF: {
                    int ret = rc.ServoOFF();
                    if (ret != 0) {
                        RtPrintf("Error: Servo Off failed...\n");
                    }
                    else {
                        RtPrintf("Servo Off ......\n");
                    }
                    break;
                }
                case UI_Action::HaltAll: {
                    int ret = rc.HaltAll();
                    if (ret != 0) {
                        RtPrintf("Error HaltAll\n");
                    }
                    else {
                        RtPrintf("HaltAll the axis ......\n");
                    }
                    break;
                }
                case UI_Action::Stop: {

                    int ret = rc.StopMove();
                    if (ret != 0) {
                        RtPrintf("Error Stop Move\n");
                    }
                    else {
                        RtPrintf("Stop the robot ......\n");
                    }
                    break;
                }
                case UI_Action::QuickStop: {
                    int ret = rc.QuickStopRobot();
                    if (ret != 0) {
                        RtPrintf("Error Quick Stop\n");
                    }
                    else {
                        RtPrintf("Robot quick stop moving and disabling the servos ......\n");
                    }
                    break;
                }
                case UI_Action::GoHome: {
                    RtPrintf("Homing ......\n");
                    double&& joint_goHome_speed = deg2rad(30.0) * uiMessage_tmp.dataF64 / 100.0; // unit: rad./sec.

                    if (rc.GoHome(joint_goHome_speed) != 0) {
                        RtPrintf("GoHome Error\n");
                    }
                    else {
                        RtPrintf("Return Home Success\n");
                    }
                    break;
                }
                case UI_Action::LogON: {
                    RtPrintf("Log ON ......\n");
                    rc.LogOn();
                    break;
                }
                case UI_Action::LogOFF: {
                    RtPrintf("Log OFF ......\n");
                    rc.LogOff();
                    break;
                }
                //----- Group-1: ------
                case UI_Action::JogStop: {
                    int index = (int)uiMessage_tmp.dataF32;
                    int ret = rc.HaltMove(index);
                    if (ret != 0) {
                        RtPrintf("Robot jog stop failed\n");
                    }
                    else {
                        RtPrintf("Robot stop jogging ......\n");
                    }
                    break;
                }
                case UI_Action::Jog_q: {
                    RtPrintf("Robot start jogging ......\n");
                    int index = (int)uiMessage_tmp.F32array66[0];
                    double speed = deg2rad((double)uiMessage_tmp.F32array66[1]);
                    int ret = rc.MoveJointJog(index, speed);
                    if (ret != 0) {
                        RtPrintf("Robot Jog failed ......\n");
                        break;
                    }
                    RtPrintf("Jog-%i=> speed: %i (m-deg./sec.), Acc. Time: %i (ms)\n", 1 + index, (int)(1000.0f * uiMessage_tmp.F32array66[1]), (int)(rc.servo_profile[index].Acceleration * 1000.0));
                    break;
                }
                case UI_Action::MoveJointAbs: {
                    RtPrintf("Move joint Abs. ......\n");
                    double q[6];
                    for (int i = 0; i < 6; ++i)
                        q[i] = static_cast<double>(uiMessage_tmp.F32array66[i]);
                    int ret = rc.MoveJointAbs(q);
                    if (ret != 0) {
                        RtPrintf("Robot MoveJointAbs failed ......\n");
                        break;
                    }
                    break;
                }
                case UI_Action::MoveJointRel: {
                    RtPrintf("Move joint Rel. ......\n");
                    uiMessage_tmp.F32array66[1] = static_cast<float>(deg2rad(uiMessage_tmp.F32array66[1]));
                    int ret = rc.MoveJointRel((int)(uiMessage_tmp.F32array66[0]), (double)(uiMessage_tmp.F32array66[1]));
                    if (ret != 0) {
                        RtPrintf("Robot MoveJointRel failed ......\n");
                        break;
                    }
                    break;
                }
                case UI_Action::MovePtpToolOrientation: {
                    RtPrintf("Not implemented ......\n");
                    break;
                }
                case UI_Action::ConfigJogProfile: {
                    
                    JogProfileSetting jog_profile_tmp;
                    jog_profile_tmp.JointSpeed = deg2rad((double)uiMessage_tmp.F32array66[0]);
                    jog_profile_tmp.MaxJointSpeed = deg2rad((double)uiMessage_tmp.F32array66[1]);
                    jog_profile_tmp.JointAcceleration_time = (double)uiMessage_tmp.F32array66[2];
                    jog_profile_tmp.CartesianSpeed = (double)uiMessage_tmp.F32array66[3];
                    jog_profile_tmp.MaxCartesianSpeed = (double)uiMessage_tmp.F32array66[4];
                    jog_profile_tmp.CartesianAcceleration_time = (double)uiMessage_tmp.F32array66[5];
                    char num[32] = {0};
                    sprintf(num, "%f", jog_profile_tmp.JointSpeed);
                    RtPrintf("JointSpeed: %s\n", num);
                    sprintf(num, "%f", jog_profile_tmp.MaxJointSpeed);
                    RtPrintf("MaxJointSpeed: %s\n", num);
                    sprintf(num, "%f", jog_profile_tmp.JointAcceleration_time);
                    RtPrintf("JointAcceleration_time: %s\n", num);
                    sprintf(num, "%f", jog_profile_tmp.CartesianSpeed);
                    RtPrintf("CartesianSpeed: %s\n", num);
                    sprintf(num, "%f", jog_profile_tmp.MaxCartesianSpeed);
                    RtPrintf("MaxCartesianSpeed: %s\n", num);
                    sprintf(num, "%f", jog_profile_tmp.CartesianAcceleration_time);
                    RtPrintf("CartesianAcceleration_time: %s\n", num);
                    int ret = rc.ConfigJogProfile(jog_profile_tmp);
                    if (ret != 0) {
                        RtPrintf("ConfigJogProfile Error\n");
                    }else{
                        RtPrintf("Apply jog profiles Success\n");
                    }
                    break;
                }
                case UI_Action::RobotAxisReset: {
                    RtPrintf("Robot Axis Rest\n");
                    int ret = rc.ResetRobotAxis();
                    if (ret != 0) {
                        RtPrintf("Robot Axis Reset Failed\n");
                    }
                    break;
                }
                case UI_Action::AlarmReset: {
                    RtPrintf("Alarm reset. ......\n");
                    int ret = rc.ResetRobotAlarm();
                    if (ret != 0) {
                        RtPrintf("Alarm Reset Failed\n");
                    }
                    break;
                }
                case UI_Action::StartINTPtask: {
                    RtPrintf("Start INTP task .....\n");
                    rc.StartINTPtask();
                    break;
                }
                                             // ----- Group-2: ------
                case UI_Action::LoadTask: {
                    RtPrintf("Load task program (reference trajectory). ......\n");
                    int ret = rc.LoadTaskProgram(uiMessage_tmp.pathStr);
                    if (ret != 0) {
                        RtPrintf("Load Task failed\n");

                    }
                    ReturnAckToWinUI();
                    break;
                }
                // ----- Group-3: ------
                case UI_Action::CalibrateTcpFrame: {
                    RtPrintf("Calibrate Tcp coordinate frame: ");
                    double calibrate_vec_tmp[6];
                    for (int i = 0; i < 6; ++i)
                        calibrate_vec_tmp[i] = static_cast<double>(uiMessage_tmp.F32array66[i]);
                    rc.rt605.set_TcpFrame(calibrate_vec_tmp);
                    break;
                }
                case UI_Action::CalibrateWorkspaceFrame: {
                    RtPrintf("Calibrate the Workspace coordinate frame: ");
                    double calibrate_vec_tmp[6];
                    for (int i = 0; i < 6; ++i)
                        calibrate_vec_tmp[i] = static_cast<double>(uiMessage_tmp.F32array66[i]);
                    rc.rt605.set_WorkspaceFrame(calibrate_vec_tmp);
                    break;
                }
                //----- Group-5: ------
                case UI_Action::SetFrequencySweep: {
                    RtPrintf("SetFrequencySweep .....\n");
                    FreqSweep sweep_prt;
                    sweep_prt.f0 = uiMessage_tmp.F64array36[2];
                    sweep_prt.f1 = uiMessage_tmp.F64array36[3];
                    sweep_prt.ts = uiMessage_tmp.F64array36[4]; // unit: ms
                    sweep_prt.tf = uiMessage_tmp.F64array36[5]; // unit: ms
                    sweep_prt.arg = uiMessage_tmp.F64array36[6];
                    sweep_prt.sweep_slope = uiMessage_tmp.F64array36[7];
                    sweep_prt.sweep_bias = uiMessage_tmp.F64array36[8];
                    memcpy(sweep_prt.q0, uiMessage_tmp.F64array36 + 9, sizeof(double) * 6);
                    sweep_prt.joint_index = (int)(uiMessage_tmp.F64array36[15]);
                    RtPrintf("J%d, tf: %d, ts: %d\n", sweep_prt.joint_index + 1, (int)sweep_prt.tf, (int)(sweep_prt.ts * 10.0));
                    rc.mechanism_analyzer.Setup(sweep_prt);
                    rc.mechanism_analyzer.generate_INTP_Signal();
                    break;
                }
                case UI_Action::StartFrequencySweep: {
                    RtPrintf("StartFrequencySweep ......\n");
                    rc.mechanism_analyzer.StartFrequencySweep();
                    rc.mechanism_analyzer.rt_thread_running = true;
                    
                    char task[16]="";
                    sprintf(task, "sweep_%d_", rc.mechanism_analyzer.frequency_sweep_params.joint_index);
                    

                    rc.LogOn(task, LogMODE::sweep);
                    while (rc.mechanism_analyzer.rt_thread_running) {
                        RtSleep(1);
                    }
                    rc.LogOff();

                    strcpy(rtss_task_status->file_name, rc.save_file_path);
                    rtss_task_status->busy = false;

                    break;
                }
                case UI_Action::StartFrequencySweepALL: {
                    RtPrintf("StartFrequencySweep ALL JOINTS......\n");
                    FreqSweep sweep_prt;
                    sweep_prt.f0 = uiMessage_tmp.F64array36[2];
                    sweep_prt.f1 = uiMessage_tmp.F64array36[3];
                    sweep_prt.ts = uiMessage_tmp.F64array36[4]; // unit: ms
                    sweep_prt.tf = uiMessage_tmp.F64array36[5]; // unit: ms
                    sweep_prt.arg = uiMessage_tmp.F64array36[6];
                    sweep_prt.sweep_slope = uiMessage_tmp.F64array36[7];
                    sweep_prt.sweep_bias = uiMessage_tmp.F64array36[8];
                    memcpy(sweep_prt.q0, uiMessage_tmp.F64array36 + 9, sizeof(double) * 6);
                    
                    char folder_dir[256] = "";
                    char date[MAX_PATH] = {};
                    SYSTEMTIME LocalTime; GetLocalTime(&LocalTime);
                    sprintf_s(date, sizeof(date), "sweepALL_exp_%d_%d_%d_%d_%d_%d\\", LocalTime.wYear,
                        LocalTime.wMonth,
                        LocalTime.wDay,
                        LocalTime.wHour,
                        LocalTime.wMinute,
                        LocalTime.wSecond);

                    //create  folder and pass the folder to logger
                    strcpy(folder_dir, rc.mechanism_analyzer.app_dir);
                    strcat(folder_dir, "intpLog\\");
                    strcat(folder_dir, date);
                    LPCWSTR lpcstrDirectoryName = (LPCWSTR)folder_dir;
                    

                    CreateDirectory(lpcstrDirectoryName, NULL);

                    // create file name
                    char task[16] = {};
                    char filename[MAX_PATH] = {};

                    for (int i = 0; i < 6; i++) {
                        RtPrintf("Joint %i start sweeping...\n", i);
                        sweep_prt.joint_index = i;
                        rc.mechanism_analyzer.Setup(sweep_prt);
                        rc.mechanism_analyzer.generate_INTP_Signal();
                        rc.mechanism_analyzer.StartFrequencySweep();
                        rc.mechanism_analyzer.rt_thread_running = true;

                        memset(filename, 0, sizeof(char)* MAX_PATH);
                        strcpy(filename, folder_dir);
                        sprintf(task, "sweep_%d.csv", i);
                        strcat_s(filename, task);

                        rc.LogOn(filename, LogMODE::sweepALL);
                        while (rc.mechanism_analyzer.rt_thread_running) {
                            RtSleep(1);
                        }
                        rc.LogOff();
                    }

                    strcpy(rtss_task_status->file_name, folder_dir);
                    rtss_task_status->busy = false;

                    break;
                }
                case UI_Action::SetupSinusoidalTest: {
                    
                    //Go Home
                    double&& joint_goHome_speed = deg2rad(10.0) * uiMessage_tmp.dataF64 / 100.0; // unit: rad./sec.

                    if (rc.GoHome(joint_goHome_speed) != 0) {
                        RtPrintf("GoHome Error\n");
                    }
                    else {
                        RtPrintf("Return Home Success\n");
                    }
                    
                    // get parameters from UI
                    Sinusoidal sine_params;
                    sine_params.amplitude = uiMessage_tmp.F64array36[0];
                    sine_params.ts = uiMessage_tmp.F64array36[1];
                    sine_params.tf = uiMessage_tmp.F64array36[2];
                    sine_params.tukey_window_alpha = uiMessage_tmp.F64array36[3];
                    rc.sinusoidal_test.generate_Sinusoidal_signal(sine_params);
                    RtPrintf("Setup Sinusoidal Test\n");
                    
                    break;
                }
                case UI_Action::StartSinusoidalTest: {
                    RtPrintf("Run Sinusoidal Test\n");
                    rc.sinusoidal_test.StartSinusoidalTest();
                    rc.sinusoidal_test.rt_thread_running = true;
                    char task[16] = "sinusoidal";
                    rc.LogOn(task, LogMODE::sinusoidal);
                    while (rc.sinusoidal_test.rt_thread_running) {
                        RtSleep(1);
                    }
                    rc.LogOff();

                    strcpy(rtss_task_status->file_name, rc.save_file_path);
                    rtss_task_status->busy = false;
                    RtPrintf("file name copy: %s\n", rtss_task_status->file_name);
                    //ReturnAckToWinUI(rc.save_file_path);
                    break;
                }
                case UI_Action::RequestForReadServoPID: {
                    RtPrintf("Request For Read Servo PID\n");
                    PID temp;
                    int ret = rc.readPID(&temp);
                    
                    
                    if (ret != 0) {
                        RtPrintf("Request For Read Servo PID Error\n");
                    }
                    else {
                        for (int i = 0; i < 6; i++) {
                            rt605_PID->kpp[i] = temp.kpp[i];
                            rt605_PID->kpi[i] = temp.kpi[i];
                            rt605_PID->kvp[i] = temp.kvp[i];
                            rt605_PID->kvi[i] = temp.kvi[i];
                            RtPrintf("Axis %i kpp: %i\n", i, rt605_PID->kpp[i]);
                            RtPrintf("Axis %i kpi: %i\n", i, rt605_PID->kpi[i]);
                            RtPrintf("Axis %i kvp: %i\n", i, rt605_PID->kvp[i]);
                            RtPrintf("Axis %i kvi: %i\n", i, rt605_PID->kvi[i]);
                        }
                        //memcpy(rt605_PID, &temp, sizeof(PID));
                    }
                    rt605_PID->busy = false;
                    break;
                }
                case UI_Action::WriteServoPID: {
                    RtPrintf("Write Servo PID\n");
                    PID temp;
                    for (int i = 0; i < 6; i++) {
                        temp.kpp[i] = rt605_PID->kpp[i];
                        temp.kpi[i] = rt605_PID->kpi[i];
                        temp.kvp[i] = rt605_PID->kvp[i];
                        temp.kvi[i] = rt605_PID->kvi[i];
                    }
                    
                    //memcpy(&temp, rt605_PID, sizeof(PID));

                    int ret = rc.writePID(temp);
                    if (ret != 0) {
                        RtPrintf("WriteServoPID Error\n");
                    }
                    break;
                }
                //---------------------------
                default: {
                    RtPrintf("Nope.\n");
                    break;
                }
            }
        }
        ui->Busy = false;
        RtSleep(1);
     }
     // =======================================================
    TERMINATE:
    
    
    RtPrintf("==============================================\n");
    RtPrintf("- Exit ICRC-RTSS.\n");
    // ------------ Close the timers: ------------
    rc.CancelRoutineTimer();
    // ------------ Close the Handles: ------------
    RtCloseHandle(h_ui_accessEvent);
    RtCloseHandle(h_ui_ackEvent);
    RtCloseHandle(ui_handle);
    RtCloseHandle(h_ui_PID);

    // Stop the EtherCAT network before destroying KINGSTAR Subsystem or before re-configuration.
    rc.stopEtherCAT();

    // Terminates the KINGSTAR Subsystem if there is no other application connected to it.
    rc.destroyLink();

    return 0;
}
//
void ReturnAckToWinUI(void) {
    bool cmd{ true };
    ui->WriteMessage(h_ui_accessEvent, Node::RTSS_main, Node::Win_HMI, UI_Action::Rtss_Ack, 1, DataType::Bit, WRITE_CMD(cmd));
    SetEvent(h_ui_ackEvent);
    RtPrintf("Return Ack to WinUI ....\n");
}

void ReturnAckToWinUI(char *task) {
    RtPrintf("Return file path to WinUI: %s\n", task);
    ui->WriteMessage(h_ui_accessEvent, Node::RTSS_main, Node::Win_HMI, UI_Action::Rtss_Ack, 1, DataType::PathString, WRITE_CMD(task));
    SetEvent(h_ui_ackEvent);
    RtPrintf("Return Ack to WinUI ....\n");
}