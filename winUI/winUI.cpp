#include "winUI.h"

winUI::winUI(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    h_ui_accessEvent = RtCreateEvent(NULL, TRUE, TRUE, UI_EVENT_NAME);
    h_ui_ackEvent = RtCreateEvent(NULL, FALSE, FALSE, UI_ACK_EVENT_NAME);
    h_ui_mailbox = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(MailBox), UI_MAILBOX_NAME, (void**)&ui_mail);
    hShared_feedback = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(MotionFeedback), SHM_MOTIONFEEDBACK_NAME, (void**)&pShared_feedback);
    hSystem_state = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(RcSystemState), SHM_SYSTEM_STATE, (void**)&system_state);

    h_ui_PID = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(PID), SHM_PID_NAME, (void**)&rt605_pid);
    h_ui_RTSS_Task = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(RTSSTask_Status), SHM_RTSS_STATE_NAME, (void**)&rtss_task_status);

    ui.setupUi(this);
    setupSystemTab();
    setupControlPanel();
    setupJogTab();
    setupRcStateMessage();
    setupPlots();
    setupIntpScope();

    // 其他狀態的初始化：
    confirm_frequency_sweep_set = false;
    setupWaitRtssDialog();
#pragma region signal_slot_connection
    //
    QObject::connect(ui.pb_show_intp_3d, &QPushButton::clicked, &figure3d, &Plot3D::signal_show);
    //
    QObject::connect(this, &winUI::signal_select_intp_scope, this, &winUI::slot_select_intp_scope);
    //
    for (int i = 0; i < 28; ++i)
        QObject::connect(select_intp_scope_channel[i], &QRadioButton::toggled, [&]() {
        emit signal_select_intp_scope(getIntpScopeSelectChannel());
            });
    //
    QObject::connect(this, &winUI::signal_stop_csp_task, this, &winUI::slot_stop_csp_task);
    //
    QObject::connect(ui.pb_stop_intp_task, &QPushButton::clicked, [&]() {
        emit signal_stop_csp_task();
        });
    //
    QObject::connect(this, &winUI::signal_show_wait_rtss_ack_dialog, this, &winUI::slot_show_wait_rtss_ack_dialog);
    //
    QObject::connect(wait_rtss_dialog, &WaitRtssDialog::signal_gui_ack, this, &winUI::slot_EnableGUI);
    //
    QObject::connect(ui.select_joints, &QComboBox::currentIndexChanged, this, [&]() {
        int index = ui.select_joints->currentIndex();
        ui.kpp->setValue(static_cast<double>(rt605_pid->kpp[index]));
        ui.kpi->setValue(static_cast<double>(rt605_pid->kpi[index]));
        ui.kpi->setValue(static_cast<double>(rt605_pid->kpi[index]));
        ui.kvp->setValue(static_cast<double>(rt605_pid->kvp[index]));
        ui.kvi->setValue(static_cast<double>(rt605_pid->kvi[index]));
        });
    //
    uiButtonState_for_CartesianJog = false;
    // ------------ QTimers: ------------
    QObject::connect(&timer_system_status, &QTimer::timeout, this, &winUI::SystemStateMonitorRoutine);
    // ------------ Monitor Feedback Groups: ------------
    // Encoder monitor:
    m_encoder_monitor = nullptr;
    QObject::connect(ui.pb_show_encoder, &QPushButton::clicked, [&]() {
        if (m_encoder_monitor == nullptr && m_subWindow_started.encoder_monitor != true) {
            m_encoder_monitor = new EncoderMonitor(&m_subWindow_started.encoder_monitor);
            m_encoder_monitor->ConnectMotorDataBuffer(pShared_feedback->motor.data());
            m_encoder_monitor->show();
        }
        else if (m_encoder_monitor != nullptr && m_subWindow_started.encoder_monitor == false) {
            m_encoder_monitor = new EncoderMonitor(&m_subWindow_started.encoder_monitor);
            m_encoder_monitor->show();
        }
        });
    // Connect Link:
    QObject::connect(ui.pb_sys_connect, &QPushButton::clicked, [&]() {
        ui_mail->setConnected(h_ui_accessEvent, true);
        emit signal_show_wait_rtss_ack_dialog();
        
        });
    // Disconnect Link;
    QObject::connect(ui.pb_sys_disconnect, &QPushButton::clicked, [&]() {
        
        bool cmd = false;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::Disconnect, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        //ui_mail->Connected = false;
        ui_mail->Action = UI_Action::TerminateRtProcess;
        });
#pragma region Group-0
    // Servo ON:
    QObject::connect(ui.pb_servo_on, &QPushButton::clicked, [&]() {
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ServoON, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    QObject::connect(ui.pb_servo_on2, &QPushButton::clicked, [&]() {
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ServoON, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    // Servo OFF:
    QObject::connect(ui.pb_servo_off, &QPushButton::clicked, [&]() {
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ServoOFF, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    QObject::connect(ui.pb_servo_off2, &QPushButton::clicked, [&]() {
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ServoOFF, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    QObject::connect(ui.pb_open_motion_log, &QPushButton::toggled, [&]() {
        bool cmd = true;
        if (ui.pb_open_motion_log->isChecked()) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::LogON, 1, DataType::Bit, WRITE_CMD(cmd));
        }
        else {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::LogOFF, 1, DataType::Bit, WRITE_CMD(cmd));
        }
        RtSetEvent(h_ui_ackEvent);
        });
    // AlarmReset:
    QObject::connect(ui.pb_alarm_reset, &QPushButton::clicked, [&]() {
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::AlarmReset, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    // Import Robot Parmaeters:
    QObject::connect(this, &winUI::signal_update_robot_parameters, [&]() {
        // Call the RC:
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ImportRobotPrt, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    // Update Robot Parameters: (apply new parameters)
    QObject::connect(ui.pb_apply_robot_prt, &QPushButton::clicked, this, &winUI::ExportRobotParameters);
    // Show Robot Parameters:
    QObject::connect(this, &winUI::signal_show_robot_parameters, this, &winUI::ShowRobotParameters);

#pragma endregion Group-0

#pragma region Group-1

#pragma region Move_Joint
    // Jog Positive q1:
    QObject::connect(ui.pb_jogP_q1, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 0;
        UI_Action mode;
        if (ui.edit_RelDistance_q1->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q1->value();
            ui.edit_RelDistance_q1->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Positive q2:
    QObject::connect(ui.pb_jogP_q2, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 1;
        UI_Action mode;
        if (ui.edit_RelDistance_q2->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q2->value();
            ui.edit_RelDistance_q2->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Positive q3:
    QObject::connect(ui.pb_jogP_q3, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 2;
        UI_Action mode;
        if (ui.edit_RelDistance_q3->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q3->value();
            ui.edit_RelDistance_q3->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Positive q4:
    QObject::connect(ui.pb_jogP_q4, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 3;
        UI_Action mode;
        if (ui.edit_RelDistance_q4->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q4->value();
            ui.edit_RelDistance_q4->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Positive q5:
    QObject::connect(ui.pb_jogP_q5, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 4;
        UI_Action mode;
        if (ui.edit_RelDistance_q5->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q5->value();
            ui.edit_RelDistance_q5->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Positive q6:
    QObject::connect(ui.pb_jogP_q6, &QPushButton::pressed, [&]() {
        ui.jog_log->append("jog+6 pressed");
        float cmd[2];
        cmd[0] = 5;
        UI_Action mode;
        if (ui.edit_RelDistance_q6->value() == 0.0) { // Normal Jog
            cmd[1] = ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = ui.edit_RelDistance_q6->value();
            ui.edit_RelDistance_q6->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q1:
    QObject::connect(ui.pb_jogN_q1, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 0;
        UI_Action mode;
        if (ui.edit_RelDistance_q1->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q1->value();
            ui.edit_RelDistance_q1->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q2:
    QObject::connect(ui.pb_jogN_q2, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 1;
        UI_Action mode;
        if (ui.edit_RelDistance_q2->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q2->value();
            ui.edit_RelDistance_q2->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q3:
    QObject::connect(ui.pb_jogN_q3, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 2;
        UI_Action mode;
        if (ui.edit_RelDistance_q3->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q3->value();
            ui.edit_RelDistance_q3->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q4:
    QObject::connect(ui.pb_jogN_q4, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 3;
        UI_Action mode;
        if (ui.edit_RelDistance_q4->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q4->value();
            ui.edit_RelDistance_q4->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q5:
    QObject::connect(ui.pb_jogN_q5, &QPushButton::pressed, [&]() {
        float cmd[2];
        cmd[0] = 4;
        UI_Action mode;
        if (ui.edit_RelDistance_q5->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q5->value();
            ui.edit_RelDistance_q5->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Jog Negative q6: 
    QObject::connect(ui.pb_jogN_q6, &QPushButton::pressed, [&]() {
        ui.jog_log->append("jog-6 pressed");
        float cmd[2];
        cmd[0] = 5;
        UI_Action mode;
        if (ui.edit_RelDistance_q6->value() == 0.0) { // Normal Jog
            cmd[1] = -ui.edit_jog_joint_speed->value();
            mode = UI_Action::Jog_q;
        }
        else { // Relative Jog
            mode = UI_Action::MoveJointRel;
            if (uiButtonState_for_jointJog[0] == false)
                cmd[1] = -ui.edit_RelDistance_q6->value();
            ui.edit_RelDistance_q6->setValue(0.0);
        }
        if (uiButtonState_for_jointJog[0] == false)
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, mode, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        uiButtonState_for_jointJog[0] = true;
        });
    // Stop:
    // Jog Positive q1: (stop)
    QObject::connect(ui.pb_jogP_q1, &QPushButton::released, [&]() {
        ui.jog_log->append("jog+1 released");
        float cmd = 0;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q1->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Positive q2: (stop)
    QObject::connect(ui.pb_jogP_q2, &QPushButton::released, [&]() {
        ui.jog_log->append("jog+2 released");
        float cmd = 1;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q2->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Positive q3: (stop)
    QObject::connect(ui.pb_jogP_q3, &QPushButton::released, [&]() {
        ui.jog_log->append("jog+3 released");
        float cmd = 2;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q3->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Positive q4: (stop)
    QObject::connect(ui.pb_jogP_q4, &QPushButton::released, [&]() {
        float cmd = 3;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q4->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Positive q5: (stop)
    QObject::connect(ui.pb_jogP_q5, &QPushButton::released, [&]() {
        float cmd = 4;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q5->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Positive q6: (stop)
    QObject::connect(ui.pb_jogP_q6, &QPushButton::released, [&]() {
        ui.jog_log->append("jog+6 released");
        float cmd = 5;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q6->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q1: (stop)
    QObject::connect(ui.pb_jogN_q1, &QPushButton::released, [&]() {
        float cmd = 0;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q1->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q2: (stop)
    QObject::connect(ui.pb_jogN_q2, &QPushButton::released, [&]() {
        float cmd = 1;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q2->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q3: (stop)
    QObject::connect(ui.pb_jogN_q3, &QPushButton::released, [&]() {
        float cmd = 2;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q3->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q4: (stop)
    QObject::connect(ui.pb_jogN_q4, &QPushButton::released, [&]() {
        float cmd = 3;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q4->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q5: (stop)
    QObject::connect(ui.pb_jogN_q5, &QPushButton::released, [&]() {
        float cmd = 4;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q5->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
    // Jog Negative q6: (stop)
    QObject::connect(ui.pb_jogN_q6, &QPushButton::released, [&]() {
        ui.jog_log->append("jog-6 released");
        float cmd = 5;
        uiButtonState_for_jointJog[0] = false;
        if (ui.edit_RelDistance_q6->value() == 0) {
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::F32, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        });
#pragma endregion Move_Joint
    // Move Joint Abs.:
    //QObject::connect(ui.pb_jogN_q6, &QPushButton::pressed, [&]() {
    //    float cmd[6] = {ui.edit_AbsTarget_q1->value()}
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::Jog_q, 1, DataType::F32prt2, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    // Config Jog Profile:
#pragma region MovePtpToolOrientation
    // Jog Positive Tool Rx
    QObject::connect(ui.pb_jogP_Rx, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 3;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = ui.edit_RelDistance_Rx->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // Jog Positive Tool Ry
    QObject::connect(ui.pb_jogP_Ry, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 4;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = ui.edit_RelDistance_Ry->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // Jog Positive Tool Rz
    QObject::connect(ui.pb_jogP_Rz, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 5;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = ui.edit_RelDistance_Rz->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // Jog Negative Tool Rx
    QObject::connect(ui.pb_jogN_Rx, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 3;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = -ui.edit_RelDistance_Rx->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // Jog Negative Tool Ry
    QObject::connect(ui.pb_jogN_Ry, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 4;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = -ui.edit_RelDistance_Ry->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // Jog Negative Tool Rz
    QObject::connect(ui.pb_jogN_Rz, &QPushButton::clicked, [&]() {
        float cmd[2];
        cmd[0] = 5;
        //if (uiButtonState_for_CartesianJog == false) {
        cmd[1] = -ui.edit_RelDistance_Rz->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::MovePtpToolOrientation, 2, DataType::F32array66, WRITE_CMD(cmd[0]));
        //}
        RtSetEvent(h_ui_ackEvent);
        //uiButtonState_for_CartesianJog = true;
        });
    // --------------- Jog Stop: -------------------------------
        // Jog Positive Tool Rx (stop)
    //QObject::connect(ui.pb_jogP_Rx, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    //// Jog Positive Tool Ry (stop)
    //QObject::connect(ui.pb_jogP_Ry, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    //// Jog Positive Tool Rz (stop)
    //QObject::connect(ui.pb_jogP_Rz, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    //// Jog Negative Tool Rx (stop)
    //QObject::connect(ui.pb_jogN_Rx, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    //// Jog Negative Tool Ry (stop)
    //QObject::connect(ui.pb_jogN_Ry, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
    //// Jog Negative Tool Rz (stop)
    //QObject::connect(ui.pb_jogN_Rz, &QPushButton::released, [&]() {
    //    bool cmd = true;
    //    uiButtonState_for_CartesianJog == false;
    //    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::JogStop, 1, DataType::Bit, WRITE_CMD(cmd));
    //    RtSetEvent(h_ui_ackEvent);
    //    });
#pragma endregion MovePtpToolOrientation
    QObject::connect(ui.pb_apply_jogProfile, &QPushButton::clicked, [&]() {
        float cmd[6] = {    (float)ui.edit_jog_joint_speed->value(),
                            (float)ui.edit_jog_joint_maxSpeed->value(),
                            (float)ui.edit_jog_joint_accTime->value(),
                            (float)ui.edit_jog_Cartesian_speed->value(),
                            (float)ui.edit_jog_Cartesian_maxSpeed->value(),
                            (float)ui.edit_jog_Cartesian_accTime->value()};
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::ConfigJogProfile, 6, DataType::F32array66, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        });
    // Start INTP task:
    QObject::connect(ui.pb_start_intp_task, &QPushButton::clicked, [&]() {
        bool cmd{ true };
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::StartINTPtask, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
#pragma endregion Group-1

#pragma region Group-2
    // Load Task:
    QObject::connect(ui.pb_read_HRSSintp, &QPushButton::clicked, this, &winUI::slot_read_HRSS_intp);
#pragma endregion Group-2

    ui.pb_start_intp_task->setEnabled(false);

    //
#pragma region Group-5
    QObject::connect(ui.pb_confirm_freq_sweep, &QPushButton::clicked, [&]() {
        double cmd[16] = { 0.0,
                            0.0,
                            ui.edit_sweep_f0->value(),
                            ui.edit_sweep_f1->value(),
                            ui.edit_sweep_ts->value(),
                            ui.edit_sweep_signal_time->value(),
                            ui.edit_sweep_signal_amplitude->value(),
                            ui.edit_sweep_slope->value(),
                            ui.edit_sweep_signal_bias->value(),
                            ui.edit_sweep_initial_J1->value(),
                            ui.edit_sweep_initial_J2->value(),
                            ui.edit_sweep_initial_J3->value(),
                            ui.edit_sweep_initial_J4->value(),
                            ui.edit_sweep_initial_J5->value(),
                            ui.edit_sweep_initial_J6->value(),
                            (double)(ui.select_sweep_joint->currentIndex())
        };
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::SetFrequencySweep, 16, DataType::F64array36, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        this->confirm_frequency_sweep_set = true;
        });
    // Start Freq. Sweep:
    QObject::connect(ui.pb_start_sweep, &QPushButton::clicked, [&]() {
        if (this->confirm_frequency_sweep_set) {
            bool cmd = true;
            rtss_task_status->busy = true;
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::StartFrequencySweep, 1, DataType::Bit, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
            
            emit signal_RTSSwaiting();

            delete alarm_dialog_did_not_confirm_frequency_sweep, alarm_dialog_did_not_confirm_frequency_sweep;
            this->confirm_frequency_sweep_set = false;
        }
        else {
            alarm_dialog_did_not_confirm_frequency_sweep = new QDialog();
            alarm_dialog_did_not_confirm_frequency_sweep->setWindowTitle("Warning!!");
            alarm_dialog_did_not_confirm_frequency_sweep->setGeometry(100, 300, 500, 50);

            alarm_message_did_not_confirm_frequency_sweep = new QLabel("Did not confirm frequency-sweep setting.");
            alarm_message_did_not_confirm_frequency_sweep->setFont(QFont("Times", 20));
            alarm_message_did_not_confirm_frequency_sweep->setParent(alarm_dialog_did_not_confirm_frequency_sweep);

            alarm_dialog_did_not_confirm_frequency_sweep->show();
        }

        });
    // start Freq. sweep all joints
    QObject::connect(ui.pb_sweep_all, &QPushButton::clicked, [&]() {
        double cmd[16] = { 0.0,
                            0.0,
                            ui.edit_sweep_f0->value(),
                            ui.edit_sweep_f1->value(),
                            ui.edit_sweep_ts->value(),
                            ui.edit_sweep_signal_time->value(),
                            ui.edit_sweep_signal_amplitude->value(),
                            ui.edit_sweep_slope->value(),
                            ui.edit_sweep_signal_bias->value(),
                            ui.edit_sweep_initial_J1->value(),
                            ui.edit_sweep_initial_J2->value(),
                            ui.edit_sweep_initial_J3->value(),
                            ui.edit_sweep_initial_J4->value(),
                            ui.edit_sweep_initial_J5->value(),
                            ui.edit_sweep_initial_J6->value(),
                            0.0
        };
        
        
        rtss_task_status->busy = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::StartFrequencySweepALL, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);

        emit signal_RTSSwaiting();

        });
    // Import FFT result:
    QObject::connect(ui.pb_import_fft_result, &QPushButton::clicked, [&]() {
        QString file = QFileDialog::getOpenFileName(this, tr("Select the BodePlot data"), "././../../IntpLog/", tr("log files (*.csv)"));
        QByteArray byte_array = file.toLocal8Bit();
        char file_path[260] = "";
        memcpy(file_path, byte_array.data(), byte_array.size() + 1);
        plot_fft_result(file_path);
        ui.fft_save_file_label->setText(QString(file_path));
        });
    // export graph of FFT
    QObject::connect(ui.pb_export_fft_graph, &QPushButton::clicked, [&]() {
        QPixmap pixmap1(ui.bode_plot_magnitude->size());
        ui.bode_plot_magnitude->render(&pixmap1);

        QPixmap pixmap2(ui.bode_plot_phase->size());
        ui.bode_plot_phase->render(&pixmap2);

        // Create a final QPixmap to hold both plots vertically
        int combinedHeight = ui.bode_plot_magnitude->height() + ui.bode_plot_phase->height();
        QPixmap finalPixmap(ui.bode_plot_magnitude->width(), combinedHeight);
        finalPixmap.fill(Qt::white); // Optional: Fill with white background

        // Use QPainter to draw both pixmaps onto the final QPixmap
        QPainter painter(&finalPixmap);
        painter.drawPixmap(0, 0, pixmap1);
        painter.drawPixmap(0, ui.bode_plot_magnitude->height(), pixmap2);
        painter.end();
        char *filepath = ui.fft_save_file_label->text().toLocal8Bit().data();

        char* filename;

        // Find the last occurrence of the backslash
        filename = strchr(filepath, '\\');
        
        // Move pointer to the start of the filename
        if (filename != NULL) {
            filename++;  // Skip the backslash
        }
        else {
            filename = filepath;  // In case there is no backslash
        }
        

        // Remove the file extension
        char* dot = strrchr(filename, '.');
        if (dot != NULL) {
            *dot = '\0';
        }

        ui.rtss_message->append(QString(filename));

        // Save the combined pixmap to a PNG file
        finalPixmap.save(QString(filename), "PNG");
        });

#pragma endregion Group-5

#pragma region LagrangeInterpolation
    QObject::connect(ui.pb_SetupSinusoidalTest, &QPushButton::clicked, [&]() {
        double cmd[4] = { ui.sine_amp->value(), ui.sine_ts->value(), ui.sine_tf->value(), ui.tukey_alpha->value()};
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::SetupSinusoidalTest, 4, DataType::F64array36, WRITE_CMD(cmd[0]));
        RtSetEvent(h_ui_ackEvent);
        this->comfirm_sinusoidalTest_set = true;
        ui.pb_RunSinusoidalTest->setEnabled(true);
        });
    // Start Sinusoidal Test:
    QObject::connect(ui.pb_RunSinusoidalTest, &QPushButton::clicked, [&]() {
        if (this->comfirm_sinusoidalTest_set) {
            bool cmd = true;
            rtss_task_status->busy = true;
            
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::StartSinusoidalTest, 1, DataType::Bit, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);

            emit signal_RTSSwaiting();

            delete alarm_dialog_did_not_confirm_frequency_sweep, alarm_dialog_did_not_confirm_frequency_sweep;
            this->comfirm_sinusoidalTest_set = false;
            ui.pb_RunSinusoidalTest->setEnabled(false);
            //emit signal_show_wait_rtss_ack_dialog();
            
        }
        else {
            alarm_dialog_did_not_confirm_frequency_sweep = new QDialog();
            alarm_dialog_did_not_confirm_frequency_sweep->setWindowTitle("Warning!!");
            alarm_dialog_did_not_confirm_frequency_sweep->setGeometry(100, 300, 500, 50);

            alarm_message_did_not_confirm_frequency_sweep = new QLabel("Did not confirm Sinusoidal setting.");
            alarm_message_did_not_confirm_frequency_sweep->setFont(QFont("Times", 20));
            alarm_message_did_not_confirm_frequency_sweep->setParent(alarm_dialog_did_not_confirm_frequency_sweep);

            alarm_dialog_did_not_confirm_frequency_sweep->show();
        }
        });

    // Import Sinusoidal result:
    QObject::connect(ui.pb_import_sinusoidal_result, &QPushButton::clicked, [&]() {
        QString file = QFileDialog::getOpenFileName(this, tr("Select the Sinusoidal data"), "././../../IntpLog/", tr("log files (*.csv)"));
        QByteArray byte_array = file.toLocal8Bit();
        char file_path[260] = "";
        memcpy(file_path, byte_array.data(), byte_array.size() + 1);
        plot_sinusoidalTest_result(file_path);
        
        });
    // export graph of sinusoidal
    QObject::connect(ui.pb_export_sinusoidal_graph, &QPushButton::clicked, [&]() {
        
        // Calculate the total height and width of all widgets in the layout
        int width = 0;
        int height = 0;
        for (int i = 0; i < ui.sinusoidal_layout->count(); ++i) {
            QWidget* widget = ui.sinusoidal_layout->itemAt(i)->widget();
            if (widget) {
                width = std::max(width, widget->width());
                height += widget->height();
            }
        }

        // Create a pixmap of appropriate size
        QPixmap pixmap(width, height);
        pixmap.fill(Qt::white); // Fill with a background color (optional)

        QPainter painter(&pixmap);

        // Render each widget into the pixmap
        int yOffset = 0;
        for (int i = 0; i < ui.sinusoidal_layout->count(); ++i) {
            QWidget* widget = ui.sinusoidal_layout->itemAt(i)->widget();
            if (widget) {
                widget->render(&painter, QPoint(0, yOffset));
                yOffset += widget->height();
            }
        }

        char* filepath = ui.fft_save_file_label->text().toLocal8Bit().data();
        char* filename;

        // Find the last occurrence of the backslash
        filename = strrchr(filepath, '\\');

        // Move pointer to the start of the filename
        if (filename != NULL) {
            filename++;  // Skip the backslash
        }
        else {
            filename = filepath;  // In case there is no backslash
        }

        // Remove the file extension
        char* dot = strrchr(filename, '.');
        if (dot != NULL) {
            *dot = '\0';
        }
        ui.rtss_message->append(QString(filename));
        // Save the pixmap to a PNG file
        pixmap.save(QString(filename), "PNG");
        });

    QObject::connect(ui.pb_setPID, &QPushButton::clicked, [&]() {
        int idx = ui.select_joints->currentIndex();
        rt605_pid->kpp[idx] = static_cast<unsigned int>(ui.kpp->value());
        rt605_pid->kpi[idx] = static_cast<unsigned int>(ui.kpi->value());
        rt605_pid->kvp[idx] = static_cast<unsigned int>(ui.kvp->value());
        rt605_pid->kvi[idx] = static_cast<unsigned int>(ui.kvi->value());

        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::WriteServoPID, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    QObject::connect(ui.pb_readPID, &QPushButton::clicked, [&]() {
        int idx = ui.select_joints->currentIndex();
        bool cmd = true;
        rt605_pid->busy = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::RequestForReadServoPID, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        emit signal_waiting();
        });
    //
    QObject::connect(this, &winUI::signal_waiting, this, &winUI::waitstart);
    //
    QObject::connect(ui.pb_writePID2JSON, &QPushButton::clicked, this, &winUI::writePID_JSON);
    //
    QObject::connect(ui.pb_loadPIDfromJSON, &QPushButton::clicked, this, &winUI::loadPID_JSON);
    //
    QObject::connect(this, &winUI::signal_RTSSwaiting, this, &winUI::waitRTSSTaskStart);

#pragma endregion LagrangeInterpolation

    //
    QObject::connect(&timer_motion_feedback, &QTimer::timeout, this, &winUI::slot_motion_feedback_display_routine);
    //
    QObject::connect(this, &winUI::signal_DisableGUI, this, &winUI::slot_DisableGUI);
    //
    QObject::connect(this, &winUI::signal_EnableGUI, this, &winUI::slot_EnableGUI);
    //

    


#pragma endregion signal_slot_connection

    // Get the application (.rtss) absolute path: 
    TCHAR app_dir_tmp[MAX_PATH] = { 0 };
    GetModuleFileName(NULL, app_dir_tmp, MAX_PATH);
    wcstombs_s(nullptr, this->app_dir, MAX_PATH, app_dir_tmp, _TRUNCATE);
    char* pch = strrchr(this->app_dir, '\\');
    *pch = '\0';
    strcat(this->app_dir, "\\");
    setupRobotParameterTabel();
    ImportRobotParameters();

    timer_system_status.start(500);
    timer_motion_feedback.start(400);
}

winUI::~winUI()
{
    ui_mail->Action = UI_Action::TerminateRtProcess;
    wait_rtss_thread->quit();
    wait_rtss_thread->wait();
}


void winUI::closeEvent(QCloseEvent* event) {
    bool cmd = false;
    //ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::Disconnect, 1, DataType::Bit, WRITE_CMD(cmd));
    //RtSetEvent(h_ui_ackEvent);
    emit ui.pb_sys_disconnect->clicked(true);
    ui_mail->Action = UI_Action::TerminateRtProcess;

    if (m_encoder_monitor != nullptr)
        if (m_encoder_monitor->isVisible())
            m_encoder_monitor->close();
    //if (fc_imp1_ui.isEnabled() && fc_imp1_ui.isVisible())
    //    fc_imp1_ui.close();
    //if (fc_monitor.isEnabled() && fc_monitor.isVisible()) {
    //    fc_monitor.close();
    //}

    event->accept();
}

//
#pragma region SystemTab
void winUI::setupSystemTab(void) {
    // EtherCAT Slaves' Information Browser:
    int rowNum = ui.tb_ecat_slave->rowCount();
    int colNum = ui.tb_ecat_slave->columnCount();
    QTableWidgetItem* _qItem;
    for (int row = 0; row < rowNum; ++row)
        for (int col = 0; col < colNum; ++col) {
            _qItem = ui.tb_ecat_slave->item(row, col);
            if (_qItem == nullptr) {
                _qItem = new QTableWidgetItem;
                ui.tb_ecat_slave->setItem(row, col, _qItem);
            }
            auto currentFlags = ui.tb_ecat_slave->item(row, col)->flags();
            ui.tb_ecat_slave->item(row, col)->setFlags(currentFlags & (~(Qt::ItemFlag::ItemIsEditable)));
            _qItem = nullptr;
        }

}
#pragma endregion SystemTab


#pragma region ControlPanel
void winUI::setupControlPanel(void) {
    // EMG Push Button:
    ui.pb_emg_stop->setEnabled(true);
    ui.pb_emg_stop->setCheckable(true);
    QObject::connect(ui.pb_emg_stop, &QPushButton::toggled, this, &winUI::EMG_Actiion_Event);
    // Motion Halt/Resume Push Button:
    ui.pb_halt_resume->setEnabled(true);
    ui.pb_halt_resume->setCheckable(true);
    QObject::connect(ui.pb_halt_resume, &QPushButton::toggled, [&]() {
        if (ui.pb_halt_resume->isChecked()) { // halt-on state 
            ui.pb_speed_up->setEnabled(false);
            ui.pb_speed_down->setEnabled(false);
            ui.edit_speed_up_step->setEnabled(false);
            ui.pb_home->setEnabled(false);
            bool cmd = true;
            ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::Stop, 1, DataType::Bit, WRITE_CMD(cmd));
            RtSetEvent(h_ui_ackEvent);
        }
        else { // resume the motion
            ui.pb_speed_up->setEnabled(true);
            ui.pb_speed_down->setEnabled(true);
            ui.edit_speed_up_step->setEnabled(true);
            ui.pb_home->setEnabled(true);
        }
        });
    //
    QObject::connect(ui.pb_speed_up, &QPushButton::clicked, [&]() {
        if (ui.override_speed_ratio->value() < 100) {
            if (ui.override_speed_ratio->value() + ui.edit_speed_up_step->value() > 100)
                ui.override_speed_ratio->setValue(100);
            else
                ui.override_speed_ratio->setValue(ui.override_speed_ratio->value() + ui.edit_speed_up_step->value());
        }
        });
    QObject::connect(ui.pb_speed_down, &QPushButton::clicked, [&]() {
        if (ui.override_speed_ratio->value() > 1) {
            if (ui.override_speed_ratio->value() - ui.edit_speed_up_step->value() < 1)
                ui.override_speed_ratio->setValue(1);
            else
                ui.override_speed_ratio->setValue(ui.override_speed_ratio->value() - ui.edit_speed_up_step->value());
        }
        });
    // Go Home Push Button:
    ui.pb_home->setCheckable(true);
    QObject::connect(ui.pb_home, &QPushButton::pressed, [&]() {
        ui.pb_home->setChecked(true);

        double cmd = ui.override_speed_ratio->value();
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::GoHome, 1, DataType::F64, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
    QObject::connect(ui.pb_home, &QPushButton::released, [&]() {
        ui.pb_home->setChecked(false);
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::HaltAll, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });

    //Rest Axis Button clicked'
    QObject::connect(ui.pb_reset_axis, &QPushButton::clicked, [&](){
        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::RobotAxisReset, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
        });
}
//
void winUI::EMG_Actiion_Event(bool action) {
    if (action) {
        ui.pb_speed_up->setEnabled(false);
        ui.pb_speed_down->setEnabled(false);
        ui.edit_speed_up_step->setEnabled(false);
        ui.pb_halt_resume->setEnabled(false);
        ui.pb_home->setEnabled(false);

        bool cmd = true;
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::QuickStop, 1, DataType::Bit, WRITE_CMD(cmd));
        RtSetEvent(h_ui_ackEvent);
    }
    else {
        ui.pb_speed_up->setEnabled(true);
        ui.pb_speed_down->setEnabled(true);
        ui.edit_speed_up_step->setEnabled(true);
        ui.pb_halt_resume->setEnabled(true);
        ui.pb_halt_resume->setChecked(false);
        ui.pb_home->setEnabled(true);
    }
}
//
#pragma endregion ControlPanel

#pragma region JogTab
void winUI::setupJogTab(void) {
    // Jog PTP/LINE switch:
    QObject::connect(ui.switch_jog_mode, &QPushButton::toggled, [&]() {
        if (ui.switch_jog_mode->isChecked()) {
            ui.switch_jog_mode->setText("LINE");
        }
        else {
            ui.switch_jog_mode->setText("PTP");
        }
        });
    // Jog Move All (Cartesian):
    QObject::connect(ui.pb_Cartesian_move_all, &QPushButton::pressed, [&]() {
        ui.pb_Cartesian_move_all->setStyleSheet("background-color: rgb(0, 250, 0);");

        });
    QObject::connect(ui.pb_Cartesian_move_all, &QPushButton::released, [&]() {
        ui.pb_Cartesian_move_all->setStyleSheet("background-color: rgb(200, 200, 200);");

        });
}
#pragma endregion JogTab

#pragma region EncoderMonitor
EncoderMonitor::EncoderMonitor(bool* parentActiveStatus) {
    this->m_actived = parentActiveStatus;
    *m_actived = true;
    setupLayout();
}
void EncoderMonitor::setupLayout(void) {
    // Encoder monitor:
    this->setWindowIcon(QIcon("./resources/icons/encoder.jpg"));
    this->setGeometry(QRect(50, 50, 200, 200));
    this->setWindowTitle(QString("Servo Motor Encoder Internal Position"));
    static int tmp = 0;
    for (unsigned short i = 0; i < 6; ++i) {
        lcd[i].setParent(this);
        lcd[i].setGeometry(QRect(50, 15 + i * 30, 90, 30));
        lcd[i].setDigitCount(10);
        lcd[i].setMode(QLCDNumber::Mode::Dec);
        lcd[i].setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        label[i].setParent(this);
        label[i].setGeometry(QRect(27, 25 + i * 30, 21, 16));
        label[i].setLineWidth(2);
        QString str; str = "M" + QString::number(i + 1);
        label[i].setText(str);
    }
    //QString str = "Thread ID: " + QString::number(*threadid);
    QObject::connect(&this->timer, &QTimer::timeout, [&]() {
        for (unsigned short i = 0; i < 6; ++i) {
            lcd[i].display(m_motor_pulse[i]);
        }
        });
    timer.start(500);
    //
}
//
void EncoderMonitor::ConnectMotorDataBuffer(double* _pulse_data) {
    m_motor_pulse = _pulse_data;
}
#pragma endregion EncoderMonitor

#pragma region ForceControl

//
void winUI::setupPlots(void) {
    ui.bode_plot_magnitude->addGraph();
    ui.bode_plot_magnitude->xAxis->setLabel("Freq. (Hz)");
    ui.bode_plot_magnitude->xAxis->setScaleType(QCPAxis::stLogarithmic);
    ui.bode_plot_magnitude->yAxis->setLabel("dB");
    ui.bode_plot_magnitude->xAxis->grid()->setVisible(true);
    ui.bode_plot_magnitude->yAxis->grid()->setVisible(true);
    ui.bode_plot_magnitude->xAxis->setRange(0, 100);
    ui.bode_plot_magnitude->yAxis->setRange(-40, 10);
    ui.bode_plot_magnitude->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // add title layout element:
    ui.bode_plot_magnitude->plotLayout()->insertRow(0);
    ui.bode_plot_magnitude->plotLayout()->addElement(0, 0, new QCPTextElement(ui.bode_plot_magnitude, "Magnitude", QFont("sans", 12, QFont::Bold)));
    

    ui.bode_plot_phase->addGraph();
    ui.bode_plot_phase->xAxis->setLabel("Freq. (Hz)");
    ui.bode_plot_phase->xAxis->setScaleType(QCPAxis::stLogarithmic);
    ui.bode_plot_phase->yAxis->setLabel("degree");
    ui.bode_plot_phase->xAxis->grid()->setVisible(true);
    ui.bode_plot_phase->yAxis->grid()->setVisible(true);
    ui.bode_plot_phase->xAxis->setRange(0, 100);
    ui.bode_plot_phase->yAxis->setRange(-180, 180);
    ui.bode_plot_phase->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // add title layout element:
    ui.bode_plot_phase->plotLayout()->insertRow(0);
    ui.bode_plot_phase->plotLayout()->addElement(0, 0, new QCPTextElement(ui.bode_plot_phase, "Phase", QFont("sans", 12, QFont::Bold)));

    ui.joint_sweep_scope->addGraph();
    ui.joint_sweep_scope->addGraph();
    ui.joint_sweep_scope->graph(0)->setPen(QPen(Qt::blue));  // "ref" will be blue
    ui.joint_sweep_scope->graph(1)->setPen(QPen(Qt::red));   // "act" will be red

    ui.joint_sweep_scope->graph(0)->setName("ref");
    ui.joint_sweep_scope->graph(1)->setName("act");
    ui.joint_sweep_scope->xAxis->setLabel("Time (s)");
    ui.joint_sweep_scope->yAxis->setLabel("degree");
    ui.joint_sweep_scope->xAxis->grid()->setVisible(true);
    ui.joint_sweep_scope->yAxis->grid()->setVisible(true);
    ui.joint_sweep_scope->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui.joint_sweep_scope->legend->setVisible(true);
}

void winUI::plot_sinusoidalTest_result(char* log_path) {
    ui.sinusoidal_save_file_label->setText(QString(log_path));
    FILE* fs = nullptr;
    errno_t err = fopen_s(&fs, log_path, "r");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Error", "Error: Open sweep log file error");
        return;
    }


    //extract the sweep joint data q{idx} and qc{idx}
    char line[1024];
    char headers[1024];

    // Read the header line to skip it
    fgets(headers, 1024, fs);

    QVector<double> time;
    QVector<QVector<double>> qc, q;
    QVector<double> row(6);

    // Read each line of the CSV file
    while (fgets(line, 1024, fs)) {
        int i = 0;
        double values[48];  // Assuming 48 columns

        // Parse the line and store the values in the array
        char* token = strtok(line, ",");
        while (token != NULL && i < 48) {
            values[i] = atof(token);
            token = strtok(NULL, ",");
            i++;
        }

        time.push_back(values[0]);

        for (int i = 0; i < 6; i++) {
            row[i] = values[7 + i];
        }
        qc.push_back(row);

        for (int i = 0; i < 6; i++) {
            row[i] = values[13 + i];
        }
        q.push_back(row);
    }
    fclose(fs);
    int data_length = time.length();
    // delete previous plots
    while (QLayoutItem* item = ui.sinusoidal_layout->takeAt(0)) {
        if (QWidget* widget = item->widget()) {
            widget->deleteLater();
        }
        delete item;
    }


    // ------------------ Plot Joint data -------------------------
    for (int i = 0; i < 6; ++i) {
        QCustomPlot* plot = new QCustomPlot;

        // Add two graphs to the plot
        plot->addGraph();
        plot->addGraph();

        // Set graph names for legend
        plot->graph(0)->setName("act");
        plot->graph(1)->setName("ref");
        // Set colors for the two graphs
        plot->graph(0)->setPen(QPen(Qt::blue));  // "ref" will be blue
        plot->graph(1)->setPen(QPen(Qt::red));   // "act" will be red

        // Set axis labels
        plot->xAxis->setLabel("Time (s)");
        plot->yAxis->setLabel("degree");

        // Add legend to the plot
        plot->legend->setVisible(true);
        plot->legend->setBrush(QBrush(QColor(255, 255, 255, 150)));

        // Set plot interactions
        plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

        QVector<double> x(q.size());
        for (int j = 0; j < q.size(); ++j) {
            x[j] = q[j][i];
        }
        plot->graph(0)->setData(time, x);

        for (int j = 0; j < qc.size(); ++j) {
            x[j] = qc[j][i];
        }
        plot->graph(1)->setData(time, x);

        plot->xAxis->setRange(time.first(), time.last());
        plot->yAxis->rescale(true);
        plot->replot();

        // Add plot to the vertical layout
        ui.sinusoidal_layout->addWidget(plot);
    }
}

void winUI::plot_fft_result(char *log_path){
    ui.fft_save_file_label->setText(QString(log_path));
    FILE* fs = nullptr;
    errno_t err = fopen_s(&fs, log_path, "r");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Error", "Error: Open sweep log file error");
        return;
    }
    // extract the sweep joint index
    const char* sweep_pos = strstr(log_path, "sweep_");
    int sweep_joint_idx = -1;
    if (sweep_pos != NULL) {
        // Move the pointer right after "sweep_"
        sweep_pos += 6;

        // Extract the idx as an integer
        if (sscanf(sweep_pos, "%d", &sweep_joint_idx) != 1) {
            printf("Error: idx not found or incorrect format.\n");
            QMessageBox::warning(nullptr, "Error", "Error: Joint idx not found or incorrect format.");
            return;
        }
    }
    else {
        QMessageBox::warning(nullptr, "Error", "sweep_' not found in the filepath.");
        return;
    }

    //extract the sweep joint data q{idx} and qc{idx}
    char line[1024];
    char headers[1024];

    // Read the header line to skip it
    if (!fgets(headers, 1024, fs)) {
        QMessageBox::warning(nullptr, "Error", "Error: Failed to read header from file.");
        fclose(fs);
        return;
    }

    QVector<double> time, qc, q;

    // Read each line of the CSV file
    while (fgets(line, 1024, fs)) {
        int i = 0;
        double values[48];  // Assuming 48 columns

        // Parse the line and store the values in the array
        char* token = strtok(line, ",");
        while (token != NULL && i < 48) {
            values[i] = atof(token);
            token = strtok(NULL, ",");
            i++;
        }

        //// get the position value
        time.push_back(values[0]);
        qc.push_back(values[7 + sweep_joint_idx]);
        q.push_back(values[13 + sweep_joint_idx]);
    }
    fclose(fs);
    int data_length = time.length();


    // ------------------ Plot Joint data -------------------------
    // Set colors for the two graphs
    ui.joint_sweep_scope->graph(0)->setPen(QPen(Qt::blue));  // "ref" will be blue
    ui.joint_sweep_scope->graph(1)->setPen(QPen(Qt::red));   // "act" will be red

    ui.joint_sweep_scope->graph(0)->setName("ref");
    ui.joint_sweep_scope->graph(1)->setName("act");

    ui.joint_sweep_scope->graph(0)->setData(time, qc);
    ui.joint_sweep_scope->graph(1)->setData(time, q);

    ui.joint_sweep_scope->xAxis->setRange(time.first(), time.last());
    ui.joint_sweep_scope->yAxis->rescale(true);
    ui.joint_sweep_scope->legend->setVisible(true);
    ui.joint_sweep_scope->replot();

    //================== FFT ======================
    // Convert the signal to a complex Eigen vector
    Eigen::FFT<double> fft;
    Eigen::VectorXcd signal_vector(data_length);
    Eigen::VectorXcd signal_ref_vector(data_length);
    QVector<double> freq;
    QVector<double> magnitude;
    QVector<double> phase;
    double ts = 0.0005;

    freq.resize(data_length / 2);
    magnitude.resize(data_length / 2);
    phase.resize(data_length / 2);

    for (int i = 0; i < data_length; i++) {
        signal_vector[i] = std::complex<double>(q[i], 0.0);
        signal_ref_vector[i] = std::complex<double>(qc[i], 0.0);
    }

    // Perform FFT using Eigen
    Eigen::VectorXcd signal_fft = fft.fwd(signal_vector);
    Eigen::VectorXcd signal_ref_fft = fft.fwd(signal_ref_vector);

    for (int i = 0; i < data_length / 2; i++) {
        freq[i] = static_cast<double>(i) / (data_length * ts);
        magnitude[i] = 20 * log10(std::abs(signal_fft[i] / signal_ref_fft[i]));
        phase[i] = std::arg(signal_fft[i] / signal_ref_fft[i]) / M_PI * 180;
    }

    ui.bode_plot_magnitude->graph(0)->setData(freq, magnitude);
    ui.bode_plot_phase->graph(0)->setData(freq, phase);

    ui.bode_plot_magnitude->replot();
    ui.bode_plot_phase->replot();
    
    // compute and mark the bandwidth (-3dB) point
    double bw_pos;
    double min = 100000;
    for (int i = 0; i < data_length / 2; i++) {
        if (abs(magnitude[i] + 3) < min) {
            bw_pos = freq[i];
            min = abs(magnitude[i] + 3);
        }
    }
    ui.bode_plot_magnitude->clearItems();
    QCPItemLine* line1 = new QCPItemLine(ui.bode_plot_magnitude);

    line1->start->setCoords(0, -3); 
    line1->end->setCoords(bw_pos, -3);   

    // Optional: Customize the line appearance
    line1->setPen(QPen(Qt::red)); // Set line color

    QCPItemLine* line2 = new QCPItemLine(ui.bode_plot_magnitude);

    line2->start->setCoords(bw_pos, -3); 
    line2->end->setCoords(bw_pos, -40);    

    line2->setPen(QPen(Qt::red)); 
    char legend[64] = { 0 };
    sprintf(legend, "BW=%.2f\n", bw_pos);
    ui.bode_plot_magnitude->graph(0)->setName(QString(legend));
    ui.bode_plot_magnitude->legend->setVisible(TRUE);


    ui.bode_plot_magnitude->replot();
}
//
void winUI::plot_all_joints_fft(char* log_folder) {
    /* read all file under the folder and plot on the same plot in fft_layout*/
    struct dirent* entry;
    DIR* dp = opendir(log_folder);

    if (dp == NULL) {
        perror("opendir");
        return;
    }
    FILE* fs = nullptr;
    QVector<double> time, qc, q;
    int sweep_joint_idx = 0;

    while ((entry = readdir(dp))) {
        if (strstr(entry->d_name, ".csv")) {  // Check for .csv extension
            char filepath[1024];
            snprintf(filepath, sizeof(filepath), "%s/%s", log_folder, entry->d_name);
            ui.rtss_message->append(QString(filepath));
            fs = fopen(filepath, "r");

            char line[1024];
            
            time.clear();
            qc.clear();
            q.clear();

            // Read each line of the CSV file
            while (fgets(line, 1024, fs)) {
                int i = 0;
                double values[48];  // Assuming 48 columns

                // Parse the line and store the values in the array
                char* token = strtok(line, ",");
                while (token != NULL && i < 48) {
                    values[i] = atof(token);
                    token = strtok(NULL, ",");
                    i++;
                }

                //// get the position value
                time.push_back(values[0]);
                qc.push_back(values[7 + sweep_joint_idx]);
                q.push_back(values[13 + sweep_joint_idx]);
            }
            fclose(fs);
            int data_length = time.length();

            //================== FFT ======================
            // Convert the signal to a complex Eigen vector
            Eigen::FFT<double> fft;
            Eigen::VectorXcd signal_vector(data_length);
            Eigen::VectorXcd signal_ref_vector(data_length);
            QVector<double> freq;
            QVector<double> magnitude;
            QVector<double> phase;
            double ts = 0.0005;

            freq.resize(data_length / 2);
            magnitude.resize(data_length / 2);
            phase.resize(data_length / 2);

            for (int i = 0; i < data_length; i++) {
                signal_vector[i] = std::complex<double>(q[i], 0.0);
                signal_ref_vector[i] = std::complex<double>(qc[i], 0.0);
            }

            // Perform FFT using Eigen
            Eigen::VectorXcd signal_fft = fft.fwd(signal_vector);
            Eigen::VectorXcd signal_ref_fft = fft.fwd(signal_ref_vector);

            for (int i = 0; i < data_length / 2; i++) {
                freq[i] = static_cast<double>(i) / (data_length * ts);
                magnitude[i] = 20 * log10(std::abs(signal_fft[i] / signal_ref_fft[i]));
                phase[i] = std::arg(signal_fft[i] / signal_ref_fft[i]) / M_PI * 180;
            }

            // create plot
            // Create a QCustomPlot instance
            QCustomPlot* bode_plot_magnitude = new QCustomPlot;
            QCustomPlot* bode_plot_phase = new QCustomPlot;

            bode_plot_magnitude->addGraph();
            bode_plot_phase->addGraph();

            bode_plot_magnitude->graph(sweep_joint_idx)->setData(freq, magnitude);
            bode_plot_phase->graph(sweep_joint_idx)->setData(freq, phase);
            bode_plot_magnitude->xAxis->setRange(0, 100);
            bode_plot_phase->xAxis->setRange(0, 100);
            bode_plot_magnitude->yAxis->setRange(-40, 10);

            bode_plot_magnitude->replot();
            bode_plot_phase->replot();

            // compute and mark the bandwidth (-3dB) point
            double bw_pos;
            double min = 100000;
            for (int i = 0; i < data_length / 2; i++) {
                if (abs(magnitude[i] + 3) < min) {
                    bw_pos = freq[i];
                    min = abs(magnitude[i] + 3);
                }
            }
            bode_plot_magnitude->clearItems();

            char legend[64] = { 0 };
            sprintf(legend, "Joint %d BW=%.2f\n", sweep_joint_idx+1 ,bw_pos);
            bode_plot_magnitude->graph(sweep_joint_idx)->setName(QString(legend));
            bode_plot_magnitude->legend->setVisible(TRUE);


            bode_plot_magnitude->replot();

            sweep_joint_idx++;
            fclose(fs);

            ui.fft_layout->addWidget(bode_plot_magnitude);
            ui.fft_layout->addWidget(bode_plot_phase);
        }

    }

    closedir(dp);
}




//
void winUI::setupRcStateMessage(void) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 6; ++c) {
            ui.tb_rc_state_message->setItem(r, c, new QTableWidgetItem);
        }
}
//
void winUI::SystemStateMonitorRoutine(void) {
    ui.thread_id->setText(QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId())));
    RcSystemState system_state_tmp;
    system_state->GetInformation(RcSystemStateAction::copy, system_state_tmp);
    for (int i = 0; i < 6; ++i) {
        switch (system_state_tmp.servoMotion_state.at(i)) {
        case 0:
            ui.tb_rc_state_message->item(0, i)->setText("moving at a constant speed.");
            break;
        case 1:
            ui.tb_rc_state_message->item(0, i)->setText("accelerating");
            break;
        case 2:
            ui.tb_rc_state_message->item(0, i)->setText("decelerating");
            break;
        case 3:
            ui.tb_rc_state_message->item(0, i)->setText("stopped");
            break;
        default:
            ui.tb_rc_state_message->item(0, i)->setText("Unknown");
            break;
        }
    }

    for (int i = 0; i < 6; ++i) {

        switch (system_state_tmp.servo_status.at(i).State) {
        case 0: {
            ui.tb_rc_state_message->item(1, i)->setText("The axis is offline. It is in ErrorStop state.");
            break;
        }
        case 1: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "Communication Off. It is in ErrorStop state.", nullptr));
            break;
        }
        case 2: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "	Motion error. It is in ErrorStop state.", nullptr));
            break;
        }
        case 3: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "	Powered off.", nullptr));
            break;
        }
        case 4: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "	Stopping state.", nullptr));
            break;
        }
        case 5: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "StandStill state.", nullptr));
            break;
        }
        case 6: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "Discrete Motion state.", nullptr));
            break;
        }
        case 7: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "Continuous Motion state.", nullptr));
            break;
        }
        default: {
            ui.tb_rc_state_message->item(1, i)->setText(QCoreApplication::translate("WinUIClass", "Synchronized Motion state.", nullptr));
            break;
        }
        }
    }
}
//
void winUI::setupRobotParameterTabel(void) {
    for (int c = 0; c < 2; ++c) {
        for (int r1 = 0; r1 < 18; ++r1)
            ui.tb_joint_prt->setItem(r1, c, new QTableWidgetItem);
        for (int r2 = 0; r2 < 20; ++r2)
            ui.tb_motor_prt->setItem(r2, c, new QTableWidgetItem);
        for (int r3 = 0; r3 < 8; ++r3)
            ui.tb_kinematics_prt->setItem(r3, c, new QTableWidgetItem);
        //
        for (int r1 = 0; r1 < 18; ++r1)
            ui.tb_joint_prt->item(r1, 1)->setText(tr("degree"));
        ui.tb_motor_prt->item(0, 1)->setText(tr("pulse/revolute"));
        ui.tb_motor_prt->item(1, 1)->setText(tr("pulse/radian"));
        for (int i = 0; i < 6; ++i) {
            ui.tb_motor_prt->item(2 + i, 1)->setText(tr("pulse"));
            ui.tb_motor_prt->item(8 + i, 1)->setText(tr("ratio"));
            ui.tb_motor_prt->item(14 + i, 1)->setText(tr("N-m"));
        }
        ui.tb_kinematics_prt->item(1, 1)->setText(tr("mm"));
        ui.tb_kinematics_prt->item(2, 1)->setText(tr("degee"));
        ui.tb_kinematics_prt->item(3, 1)->setText(tr("mm"));
        ui.tb_kinematics_prt->item(4, 1)->setText(tr("degee"));
        ui.tb_kinematics_prt->item(5, 1)->setText(tr("degee"));
        ui.tb_kinematics_prt->item(6, 1)->setText(tr("mm"));
        ui.tb_kinematics_prt->item(7, 1)->setText(tr("mm"));
    }
    // Read Only:
    ui.tb_motor_prt->item(0, 0)->setText(QString::number(131072));
}
//
void winUI::ShowRobotParameters(void) {
    for (unsigned short i = 0; i < 2; ++i) {
        // Home:
        ui.tb_joint_prt->item(i, 0)->setText(QString::number(-90.0 + rad2deg(rt605.kinePrt.joint_home_position.at(i))));
        // NOT:
        ui.tb_joint_prt->item(6 + i * 2, 0)->setText(QString::number(-90.0 + rad2deg(rt605.kinePrt.joint_limit.at(0).at(i))));
        // POT:
        ui.tb_joint_prt->item(7 + i * 2, 0)->setText(QString::number(-90.0 + rad2deg(rt605.kinePrt.joint_limit.at(1).at(i))));
        // Motor Zero:
        ui.tb_motor_prt->item(2 + i, 0)->setText(QString::number((rt605.kinePrt.encoder_zero_pulse.at(i)), 15));
        // Reducer:
        ui.tb_motor_prt->item(8 + i, 0)->setText(QString::number((rt605.kinePrt.reduction_ratio.at(i))));
    }
    for (unsigned short i = 2; i < 6; ++i) {
        // Home:
        ui.tb_joint_prt->item(i, 0)->setText(QString::number(rad2deg(rt605.kinePrt.joint_home_position.at(i))));
        // NOT:
        ui.tb_joint_prt->item(6 + i * 2, 0)->setText(QString::number(rad2deg(rt605.kinePrt.joint_limit.at(0).at(i))));
        // POT:
        ui.tb_joint_prt->item(7 + i * 2, 0)->setText(QString::number(rad2deg(rt605.kinePrt.joint_limit.at(1).at(i))));
        // Motor Zero:
        ui.tb_motor_prt->item(2 + i, 0)->setText(QString::number((rt605.kinePrt.encoder_zero_pulse.at(i)), (char)103, 15));
        // Reducer:
        ui.tb_motor_prt->item(8 + i, 0)->setText(QString::number((rt605.kinePrt.reduction_ratio.at(i))));
    }
    // PPU:
    ui.tb_motor_prt->item(1, 0)->setText(QString::number(rt605.kinePrt.PPU, (char)103, 12));
}
//
void winUI::ExportRobotParameters(void) {
    char open_file[MAX_PATH] = { 0 };
    _strnset(open_file, 0, MAX_PATH);
    strncpy(open_file, app_dir, MAX_PATH);

    FILE* fs = nullptr;
    errno_t err;
    // Joint Home:
    strcat_s(open_file, "..\\..\\Parameters\\robotINF\\joint_home_pos.txt");
    err = fopen_s(&fs, open_file, "w");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Import Error",
            "Failed to import robot parameters.\nFile does not exist.",
            QMessageBox::Ok);
        return;
    }
    for (unsigned short i = 0; i < 6; ++i)
        fprintf_s(fs, "%g,", ui.tb_joint_prt->item(i, 0)->text().toDouble());
    fclose(fs);
    // Joint Limits:
    _strnset(open_file, 0, MAX_PATH);
    strncpy(open_file, app_dir, MAX_PATH);
    strcat_s(open_file, "..\\..\\Parameters\\robotINF\\joint_lim.txt");
    err = fopen_s(&fs, open_file, "w");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Import Error",
            "Failed to import robot parameters.\nFile does not exist.",
            QMessageBox::Ok);
        return;
    }
    // NOT:
    for (unsigned short i = 0; i < 6; ++i)
        fprintf_s(fs, "%g,", ui.tb_joint_prt->item(6 + (i * 2), 0)->text().toDouble());
    fprintf(fs, "\n");
    // POT:
    for (unsigned short i = 0; i < 6; ++i)
        fprintf_s(fs, "%g,", ui.tb_joint_prt->item(7 + (i * 2), 0)->text().toDouble());
    fclose(fs);
    // Motor Zero Pulse:
    _strnset(open_file, 0, MAX_PATH);
    strncpy(open_file, app_dir, MAX_PATH);
    strcat_s(open_file, "..\\..\\Parameters\\robotINF\\motor_zero_pulse.txt");

    err = fopen_s(&fs, open_file, "w");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Import Error",
            "Failed to import robot parameters.\nFile does not exist.",
            QMessageBox::Ok);
        return;
    }
    for (unsigned short i = 0; i < 6; ++i)
        fprintf(fs, "%lf,", ui.tb_motor_prt->item(2 + i, 0)->text().toDouble());
    fclose(fs);
    
     // Reducers:
     _strnset(open_file, 0, MAX_PATH);
     strncpy(open_file, app_dir, MAX_PATH);
     strcat_s(open_file, "..\\..\\Parameters\\robotINF\\reduction_ratio.txt");
     err = fopen_s(&fs, open_file, "w");
     if (err != 0) {
         QMessageBox::warning(nullptr, "Import Error",
             "Failed to import robot parameters.\nFile does not exist.",
             QMessageBox::Ok);
         return;
     }
     for (unsigned short i = 0; i < 6; ++i)
         fprintf(fs, "%i,", ui.tb_motor_prt->item(8 + i, 0)->text().toInt());
     fclose(fs);
    

     // Singularity Thresholds:
    _strnset(open_file, 0, MAX_PATH);
    strncpy(open_file, app_dir, MAX_PATH);
    strcat_s(open_file, "..\\..\\Parameters\\robotINF\\joint_singularity.txt");
    err = fopen_s(&fs, open_file, "w");
    if (err != 0) {
        QMessageBox::warning(nullptr, "Import Error",
            "Failed to import robot parameters.\nFile does not exist.",
            QMessageBox::Ok);
        return;
    }
    // Wrist:
    fprintf_s(fs, "wrist_threshold(deg.),%g,\n", ui.tb_kinematics_prt->item(5, 0)->text().toDouble());
    // Elbow:
    fprintf_s(fs, "elbow_threshold(mm),%g,\n", ui.tb_kinematics_prt->item(6, 0)->text().toDouble());
    // Shoulder:
    fprintf_s(fs, "shoulder_threshold(mm),%g,", ui.tb_kinematics_prt->item(7, 0)->text().toDouble());
    fclose(fs);

    ImportRobotParameters();

    emit signal_update_robot_parameters();
}
//
void winUI::ImportRobotParameters(void) {

    char open_file[MAX_PATH];
    _strnset(open_file, 0, MAX_PATH);
    strncpy(open_file, app_dir, MAX_PATH);

    FILE* fs = nullptr;

    rt605.kinePrt.ImportAllParameter(this->app_dir);
    
    emit signal_show_robot_parameters();
}
#pragma endregion ForceControl

void winUI::loadPID_JSON(void) {
    // Open file dialog to select JSON file
    QString fileName = QFileDialog::getOpenFileName(nullptr, "Open JSON File", "", "JSON Files (*.json);;All Files (*)");
    if (fileName.isEmpty()) {
        QMessageBox::warning(nullptr, "Waning", "No file selected");
        return;
    }

    QByteArray fileNameBytes = fileName.toLocal8Bit();
    const char* filename = fileNameBytes.data();
    FILE* file = fopen(filename, "r");
    if (!file) {
        QMessageBox::warning(nullptr, "Error", QString("Could not open file %1").arg(filename));
        return;
    }

    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);

    char* data = (char*)malloc(length + 1);
    if (!data) {
        QMessageBox::warning(nullptr, "Error", "Could not allocate memory");
        fclose(file);
        return;
    }

    fread(data, 1, length, file);
    data[length] = '\0';
    fclose(file);

    cJSON* json = cJSON_Parse(data);
    if (!json) {
        QMessageBox::warning(nullptr, "Error", "Could not parse JSON");
        free(data);
        return;
    }
    cJSON* robotPID = cJSON_GetObjectItem(json, "robotPID");
    if (robotPID) {
        for (int i = 0; i < 6; ++i) {
            char jointName[3];
            sprintf(jointName, "J%d", i + 1);
            cJSON* joint = cJSON_GetObjectItem(robotPID, jointName);
            if (joint) {
                cJSON* velocity = cJSON_GetObjectItem(joint, "velocity");
                if (velocity) {
                    cJSON* kp = cJSON_GetObjectItem(velocity, "kp");
                    cJSON* ki = cJSON_GetObjectItem(velocity, "ki");
                    if (kp && ki) {
                        rt605_pid->kvp[i] = (UINT16)(kp->valuedouble);
                        rt605_pid->kvi[i] = (UINT16)(ki->valuedouble);
                    }
                }
                cJSON* position = cJSON_GetObjectItem(joint, "position");
                if (position) {
                    cJSON* kp = cJSON_GetObjectItem(position, "kp");
                    cJSON* ki = cJSON_GetObjectItem(position, "ki");
                    if (kp && ki) {
                        rt605_pid->kpp[i] = (UINT16)(kp->valuedouble);
                        rt605_pid->kpi[i] = (UINT16)(ki->valuedouble);
                    }
                }
            }
        }
    }

    free(data);
}

void winUI::writePID_JSON(void) {
    // write PID TO JSON file
    cJSON* robotPID = cJSON_CreateObject();
    for (int i = 0; i < 6; ++i) {
        char jointName[3];
        sprintf(jointName, "J%d", i + 1);
        cJSON* joint = cJSON_CreateObject();

        cJSON* velocity = cJSON_CreateObject();
        cJSON_AddNumberToObject(velocity, "kp", rt605_pid->kvp[i]);
        cJSON_AddNumberToObject(velocity, "ki", rt605_pid->kvi[i]);
        cJSON_AddItemToObject(joint, "velocity", velocity);

        cJSON* position = cJSON_CreateObject();
        cJSON_AddNumberToObject(position, "kp", rt605_pid->kpp[i]);
        cJSON_AddNumberToObject(position, "ki", rt605_pid->kpi[i]);
        cJSON_AddItemToObject(joint, "position", position);

        cJSON_AddItemToObject(robotPID, jointName, joint);
    }

    cJSON* json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "robotPID", robotPID);

    // Open file dialog to save JSON file
    QString saveFileName = QFileDialog::getSaveFileName(nullptr, "Save JSON File", "", "JSON Files (*.json);;All Files (*)");
    if (saveFileName.isEmpty()) {
        QMessageBox::warning(nullptr, "Error", "No file selected for saving");
        return;
    }

    QByteArray saveFileNameBytes = saveFileName.toLocal8Bit();
    const char* output_filename = saveFileNameBytes.data();

    char* string = cJSON_Print(json);
    if (!string) {
        QMessageBox::warning(nullptr, "Error", "Could not print JSON");
        return;
    }

    FILE* file = fopen(output_filename, "w");
    if (!file) {
        QMessageBox::warning(nullptr, "Error", QString("Could not open file %1 for writing").arg(output_filename));
        free(string);
        return;
    }

    fprintf(file, "%s", string);
    fclose(file);
    free(string);
}
//
void winUI::JogOperation_PostProcess(void) {
    this->confirm_frequency_sweep_set = false;
}
//
void winUI::slot_DisableGUI(void) {
    this->GUI_enabled = false;
    this->setDisabled(true);
}
//
void winUI::slot_EnableGUI(void) {
    this->GUI_enabled = true;
    this->setEnabled(true);
    emit wait_rtss_dialog->signal_show(false);
}
//
void winUI::slot_show_wait_rtss_ack_dialog(void) {
    emit signal_DisableGUI();
    emit wait_rtss_dialog->signal_show(true);
}
//
void winUI::slot_motion_feedback_display_routine(void) {
    ui.lcd_q1->display(-90.0 + (double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(0)))) / 1000.0);
    ui.lcd_q2->display(-90.0 + (double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(1)))) / 1000.0);
    ui.lcd_q3->display((double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(2)))) / 1000.0);
    ui.lcd_q4->display((double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(3)))) / 1000.0);
    ui.lcd_q5->display((double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(4)))) / 1000.0);
    ui.lcd_q6->display((double)((int)(1000.0 * rad2deg(pShared_feedback->q.at(5)))) / 1000.0);

    ui.dq1_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(0)))) / 1000.0);
    ui.dq2_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(1)))) / 1000.0);
    ui.dq3_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(2)))) / 1000.0);
    ui.dq4_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(3)))) / 1000.0);
    ui.dq5_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(4)))) / 1000.0);
    ui.dq6_lcd->display((double)((int)(1000.0 * rad2deg(pShared_feedback->dq.at(5)))) / 1000.0);

    ui.lcd_torque1->display(pShared_feedback->tor.at(0));
    ui.lcd_torque2->display(pShared_feedback->tor.at(1));
    ui.lcd_torque3->display(pShared_feedback->tor.at(2));
    ui.lcd_torque4->display(pShared_feedback->tor.at(3));
    ui.lcd_torque5->display(pShared_feedback->tor.at(4));
    ui.lcd_torque6->display(pShared_feedback->tor.at(5));

    double x0[6]; memcpy(x0, pShared_feedback->X0.data(), 6 * sizeof(double));
    double x[6]; memcpy(x, pShared_feedback->X.data(), 6 * sizeof(double));
    for (unsigned short i = 0; i < 6; ++i) {
        if (i < 3) {
            x0[i] = x0[i] * 1000.0;
            x[i] = x[i] * 1000.0;
        }
        else {
            x0[i] = rad2deg(x0[i]);
            x[i] = rad2deg(x[i]);
        }
        x0[i] = ((double)((int)(x0[i] * 1000.0))) / 1000.0;
        x[i] = ((double)((int)(x[i] * 1000.0))) / 1000.0;
    }

    ui.lcd_xb->display(x0[0]);
    ui.lcd_yb->display(x0[1]);
    ui.lcd_zb->display(x0[2]);
    ui.lcd_rxb->display(x0[3]);
    ui.lcd_ryb->display(x0[4]);
    ui.lcd_rzb->display(x0[5]);

    ui.lcd_tcp_x->display(x[0]);
    ui.lcd_tcp_y->display(x[1]);
    ui.lcd_tcp_z->display(x[2]);
    ui.lcd_tcp_rx->display(x[3]);
    ui.lcd_tcp_ry->display(x[4]);
    ui.lcd_tcp_rz->display(x[5]);

    static double ov[3] = { 0.0, 0.0, 0.0 };
    double dx = (x[0] - ov[0]) / 0.4;
    double dy = (x[1] - ov[1]) / 0.4;
    double dz = (x[2] - ov[2]) / 0.4;
    ui.Override_lcd->display(sqrt(dx * dx + dy * dy + dz * dz));
    memcpy(ov, x, 3 * sizeof(double));
}
//
void winUI::slot_read_HRSS_intp(void) {
    QString file_str = QFileDialog::getOpenFileName(this, tr("Select the INTP file"), "./../../IntpLog", tr("log files (*.txt)"));

    if (!file_str.isNull()) {
        ui.text_intp_file_name->setText(file_str);
        emit signal_show_wait_rtss_ack_dialog();

        QByteArray byte_array = file_str.toLocal8Bit();
        char full_path[MAX_PATH] = "";
        memcpy(full_path, byte_array.data(), byte_array.size() + 1);

        char file[MAX_PATH];
        char* p1, * p2;
        p1 = strrchr(full_path, '/');
        p2 = strrchr(full_path, '.');
        memcpy(file, full_path + (p1 - full_path) + 1, (p2 - p1) - 1);

        unsigned long long Xr_length = 0;
        // ----------------------------- 開始計算軌跡的插補點數量：-----------------------------

        FILE* fs = nullptr;
        fopen_s(&fs, full_path, "r");
        if (fs == nullptr) {
            int ii = 0;
            return;
        }
        // 計算總資料長度：
        Xr_length = 0;
        double tmp[34];
        /* 一個 column 一個 column 掃描，計算一筆 sample 有幾個欄位的資料，
            如果結果為 col_num == 12 代表現在讀的 INTP_LOG 是從 HRSS_offline 產生的；結果為 col_num == 34 則代表現在讀的 INTP_LOG 是從 HRSS_online 產生的。
        */
        unsigned int col_num = 1;
        for (int c = getc(fs); c != '\n'; c = getc(fs)) {
            if (c == ',')
                ++col_num;
        }
        ++Xr_length;
        for (int c = getc(fs); !feof(fs); c = getc(fs)) {
            if (c == '\n')
                ++Xr_length;
        }
        fclose(fs);
        // ----------------------------- 動態配置記憶體：-----------------------------
        for (unsigned long long i = 0; i < 12; ++i) {
            Xr_buf.at(i).reserve(Xr_length);
            Xr_buf.at(i).resize(Xr_length);
        }
        intp_time.reserve(Xr_length); intp_time.resize(Xr_length);
        // 開始載入插補點：
        fopen_s(&fs, full_path, "r");
        for (unsigned long long i = 0; i < Xr_length; ++i) {
            if (col_num == 34) { // 所選擇的 HRSS INTP_LOG 檔是從實際機台 (HRSS_online) 產生的：
                fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
					%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
					%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
					%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                    tmp,
                    Xr_buf[0].data() + i,
                    Xr_buf[1].data() + i,
                    Xr_buf[2].data() + i,
                    Xr_buf[3].data() + i,
                    Xr_buf[4].data() + i,
                    Xr_buf[5].data() + i,
                    Xr_buf[6].data() + i,
                    Xr_buf[7].data() + i,
                    Xr_buf[8].data() + i,
                    Xr_buf[9].data() + i,
                    Xr_buf[10].data() + i,
                    Xr_buf[11].data() + i,
                    tmp + 13,
                    tmp + 14,
                    tmp + 15,
                    tmp + 16,
                    tmp + 17,
                    tmp + 18,
                    tmp + 19,
                    tmp + 20,
                    tmp + 21,
                    tmp + 22,
                    tmp + 23,
                    tmp + 24,
                    tmp + 25,
                    tmp + 26,
                    tmp + 27,
                    tmp + 28,
                    tmp + 29,
                    tmp + 30,
                    tmp + 31,
                    tmp + 32,
                    tmp + 33);
                intp_time[i] = static_cast<double>(i) * 0.0005;
            }
            else { // 所選擇的 HRSS INTP_LOG 檔是從軟體 HRSS_offline 產生的：
                fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", \
                    Xr_buf[0].data() + i,
                    Xr_buf[1].data() + i,
                    Xr_buf[2].data() + i,
                    Xr_buf[3].data() + i,
                    Xr_buf[4].data() + i,
                    Xr_buf[5].data() + i,
                    Xr_buf[6].data() + i,
                    Xr_buf[7].data() + i,
                    Xr_buf[8].data() + i,
                    Xr_buf[9].data() + i,
                    Xr_buf[10].data() + i,
                    Xr_buf[11].data() + i,
                    tmp + 12,
                    tmp + 13,
                    tmp + 14);
                intp_time[i] = static_cast<double>(i) * 0.0005;
            }
            // 將 HRSS INTP LOG 中定義的 (A, B, C) Command 與 Joint Command 之單位從 (m-degree) 轉換成 (degree)：
            for (unsigned short j = 0; j < 12; ++j) {
                if (j > 2) // 單位轉換
                    Xr_buf[j][i] = (Xr_buf[j][i] / 1000.0);
                else // 單位轉換成 mm
                    Xr_buf[j][i] = Xr_buf[j][i] / 1000.0;
            }
        }
        fclose(fs);


        //
        ui.tb_intp->clearContents();
        ui.tb_intp->setRowCount(0);
        ui.tb_intp->setColumnCount(0);
        ui.tb_intp->setRowCount(Xr_length);
        ui.tb_intp->setColumnCount(12);
        for (int r = 0; r < Xr_length; ++r) {
            for (int c = 0; c < 12; ++c) {
                ui.tb_intp->setItem(r, c, new QTableWidgetItem);
            }
        }

        for (int r = 0; r < Xr_length; ++r) {
            for (int c = 0; c < 12; ++c) {
                ui.tb_intp->item(r, c)->setText(QString::number(Xr_buf.at(c).at(r)));
            }
        }

        // headers:
        QStringList headers;
        headers << "Xr" << "Yr" << "Zr" << "Rxr" << "Ryr" << "Rzr" << "q1" << "q2" << "q3" << "q4" << "q5" << "q6";
        ui.tb_intp->setHorizontalHeaderLabels(headers);

        emit signal_select_intp_scope(getIntpScopeSelectChannel());
    
        // ----------------------------------------------------
        int ret = 0;
        char open_file_tmp[MAX_PATH] = "";
        fs = nullptr;
        // Read task information "workspace_frame.txt":
        memset(open_file_tmp, NULL, MAX_PATH);
        strncpy(open_file_tmp, this->app_dir, MAX_PATH);
        strcat(open_file_tmp, "Parameters\\task\\");
        strcat_s(open_file_tmp, 128, file);
        strcat(open_file_tmp, "\\");
        strcat_s(open_file_tmp, 128, "workspace_frame.txt");

        fopen_s(&fs, open_file_tmp, "r");
        double task_frame_tmp[6];
        fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", task_frame_tmp, task_frame_tmp + 1, task_frame_tmp + 2, task_frame_tmp + 3, task_frame_tmp + 4, task_frame_tmp + 5);
        fclose(fs);


        for (int i = 0; i < 6; ++i) {
            if (i < 3)
                task_frame_tmp[i] = task_frame_tmp[i] / 1000.0;
            else
                task_frame_tmp[i] = deg2rad(task_frame_tmp[i]);
        }
        rt605.set_WorkspaceFrame(task_frame_tmp);

        //_sleep(500);
        // Read task information "tool_frame.txt":
        memset(open_file_tmp, NULL, MAX_PATH);
        strncpy(open_file_tmp, this->app_dir, MAX_PATH);
        strcat(open_file_tmp, "Parameters\\task\\");
        strcat_s(open_file_tmp, 128, file);
        strcat(open_file_tmp, "\\");
        strcat_s(open_file_tmp, 128, "tool_frame.txt");

        fopen_s(&fs, open_file_tmp, "r");
        fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", task_frame_tmp, task_frame_tmp + 1, task_frame_tmp + 2, task_frame_tmp + 3, task_frame_tmp + 4, task_frame_tmp + 5);
        fclose(fs);

        for (int i = 0; i < 6; ++i) {
            if (i < 3)
                task_frame_tmp[i] = task_frame_tmp[i] / 1000.0;
            else
                task_frame_tmp[i] = deg2rad(task_frame_tmp[i]);
        }
        rt605.set_TcpFrame(task_frame_tmp);

        //_sleep(500);
        // *******************************************************************************************************************
        // 讀取 Task 之 critical interval：
        fs = nullptr;
        memset(open_file_tmp, NULL, MAX_PATH);
        strncpy(open_file_tmp, this->app_dir, MAX_PATH);
        strcat(open_file_tmp, "Parameters\\task\\");
        strcat_s(open_file_tmp, 128, file);
        strcat(open_file_tmp, "\\");
        strcat_s(open_file_tmp, 128, "fc_critical_interval.txt");
        fopen_s(&fs, open_file_tmp, "r");
        if (fs != nullptr) {
            unsigned long long fc_start, fc_stop, process_index[2];
            fscanf(fs, "%lli,%lli,%lli", &fc_start, process_index, process_index + 1);
            fclose(fs);
            //
            ui.edit_fc_intp_start->setEnabled(true);    ui.edit_fc_intp_start->setValue(fc_start);
            ui.edit_fc_intp_index0->setEnabled(true);   ui.edit_fc_intp_index0->setValue(process_index[0]);
            ui.edit_fc_intp_index1->setEnabled(true);   ui.edit_fc_intp_index1->setValue(process_index[1]);
            
        }
        else {
            ui.edit_fc_intp_start->setDisabled(true);
            ui.edit_fc_intp_index0->setDisabled(true);
            ui.edit_fc_intp_index1->setDisabled(true);
        }
        // enable pb_forceControl_taskStart
        ui.pb_start_intp_task->setEnabled(true);
        //
        ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::LoadTask, 1, DataType::PathString, WRITE_CMD(file));
        RtSetEvent(h_ui_ackEvent);
        //
    }
    else {
        ui.text_intp_file_name->setText(tr("null string"));
    }
}
//
void winUI::slot_stop_csp_task(void) {
    bool cmd = false;
    ui_mail->WriteMessage(h_ui_accessEvent, Node::Win_HMI, Node::RTSS_main, UI_Action::INTPtaskEnd, 1, DataType::Bit, WRITE_CMD(cmd));
    RtSetEvent(h_ui_ackEvent);
}
//
void winUI::setupIntpScope(void) {
    ui.scope_intp->addGraph();
    ui.scope_intp->xAxis->setLabel("time (sec.)");
    ui.scope_intp->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // setup select channel:
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 6; ++c) {
            select_intp_scope_channel[(r * 6) + c] = new QRadioButton();
            ui.gridLayout_select_intp_scope->addWidget(select_intp_scope_channel[(r * 6) + c], r, c);
        }
    select_intp_scope_channel[24] = new QRadioButton(); ui.gridLayout_select_intp_scope->addWidget(select_intp_scope_channel[24], 4, 0);
    select_intp_scope_channel[25] = new QRadioButton(); ui.gridLayout_select_intp_scope->addWidget(select_intp_scope_channel[25], 4, 1);
    select_intp_scope_channel[26] = new QRadioButton(); ui.gridLayout_select_intp_scope->addWidget(select_intp_scope_channel[26], 4, 2);
    select_intp_scope_channel[27] = new QRadioButton(); ui.gridLayout_select_intp_scope->addWidget(select_intp_scope_channel[27], 4, 3);

    select_intp_scope_channel[0]->setText("X");
    select_intp_scope_channel[1]->setText("Y");
    select_intp_scope_channel[2]->setText("Z");
    select_intp_scope_channel[3]->setText("Rx");
    select_intp_scope_channel[4]->setText("Ry");
    select_intp_scope_channel[5]->setText("Rz");

    select_intp_scope_channel[6]->setText("q1");
    select_intp_scope_channel[7]->setText("q2");
    select_intp_scope_channel[8]->setText("q3");
    select_intp_scope_channel[9]->setText("q4");
    select_intp_scope_channel[10]->setText("q5");
    select_intp_scope_channel[11]->setText("q6");

    select_intp_scope_channel[12]->setText("dX/dt");
    select_intp_scope_channel[13]->setText("dY/dt");
    select_intp_scope_channel[14]->setText("dZ/dt");
    select_intp_scope_channel[15]->setText("dRx/dt");
    select_intp_scope_channel[16]->setText("dRy/dt");
    select_intp_scope_channel[17]->setText("dRz/dt");

    select_intp_scope_channel[18]->setText("dq1/dt");
    select_intp_scope_channel[19]->setText("dq2/dt");
    select_intp_scope_channel[20]->setText("dq3/dt");
    select_intp_scope_channel[21]->setText("dq4/dt");
    select_intp_scope_channel[22]->setText("dq5/dt");
    select_intp_scope_channel[23]->setText("dq6/dt");

    select_intp_scope_channel[24]->setText("X-Y");
    select_intp_scope_channel[25]->setText("X-Z");
    select_intp_scope_channel[26]->setText("Y-Z");
    select_intp_scope_channel[27]->setText("Reserved");
}
//
void winUI::slot_select_intp_scope(int ch) {
    unsigned long long len = Xr_buf.at(0).size();
    dData.reserve(len); dData.resize(len);
    if (ch <= 2) {
        ui.scope_intp->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui.scope_intp->graph(0)->setData(intp_time, Xr_buf.at(ch));
        ui.scope_intp->xAxis->setLabel("second");
        ui.scope_intp->yAxis->setLabel("mm");
        ui.scope_intp->yAxis->setRange((*std::min_element(Xr_buf.at(ch).begin(), Xr_buf.at(ch).end())) - 25.0, (*std::max_element(Xr_buf.at(ch).begin(), Xr_buf.at(ch).end())) + 25.0);
        ui.scope_intp->xAxis->setRange(intp_time[0], intp_time[Xr_buf.at(0).size() - 1]);
    }
    else if (ch >= 3 && ch <= 11) { // Rx ~ q6:
        ui.scope_intp->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui.scope_intp->graph(0)->setData(intp_time, Xr_buf.at(ch));
        ui.scope_intp->xAxis->setLabel("second");
        ui.scope_intp->yAxis->setLabel("degree");
        ui.scope_intp->yAxis->setRange((*std::min_element(Xr_buf.at(ch).begin(), Xr_buf.at(ch).end())) - 20.0, (*std::max_element(Xr_buf.at(ch).begin(), Xr_buf.at(ch).end())) + 20.0);
        ui.scope_intp->xAxis->setRange(intp_time[0], intp_time[Xr_buf.at(0).size() - 1]);
    }
    else if (ch >= 12 && ch <= 14) { // dX:
        ui.scope_intp->graph(0)->setLineStyle(QCPGraph::lsNone);
        for (unsigned long long i = 1; i < len; ++i)
            dData[i] = (Xr_buf.at(ch - 12).at(i) - Xr_buf.at(ch - 12).at(i - 1)) / 0.0005;
        ui.scope_intp->graph(0)->setData(intp_time, dData);
        ui.scope_intp->xAxis->setLabel("second");
        ui.scope_intp->yAxis->setLabel("mm/sec.");
        ui.scope_intp->yAxis->setRange((*std::min_element(dData.begin(), dData.end())) - 25.0, (*std::max_element(dData.begin(), dData.end())) + 25.0);
        ui.scope_intp->xAxis->setRange(intp_time[0], intp_time[Xr_buf.at(0).size() - 1]);
    }
    else if (ch >= 15 && ch <= 23) { // Rx ~ dq6:
        ui.scope_intp->graph(0)->setLineStyle(QCPGraph::lsLine);
        for (unsigned long long i = 1; i < len; ++i)
            dData[i] = (Xr_buf.at(ch - 12).at(i) - Xr_buf.at(ch - 12).at(i - 1)) / 0.0005;
        ui.scope_intp->graph(0)->setData(intp_time, dData);
        ui.scope_intp->xAxis->setLabel("second");
        ui.scope_intp->yAxis->setLabel("degree/sec.");
        ui.scope_intp->yAxis->setRange((*std::min_element(dData.begin(), dData.end())) - 25.0, (*std::max_element(dData.begin(), dData.end())) + 25.0);
        ui.scope_intp->xAxis->setRange(intp_time[0], intp_time[Xr_buf.at(0).size() - 1]);
    }
    else if (ch >= 24) {
        ui.scope_intp->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
        switch (ch) {
        case 24: {
            ui.scope_intp->graph(0)->setData(Xr_buf.at(0), Xr_buf.at(1));
            ui.scope_intp->xAxis->setLabel("X (mm)");
            ui.scope_intp->yAxis->setLabel("Y (mm)");
            ui.scope_intp->xAxis->setRange((*std::min_element(Xr_buf.at(0).begin(), Xr_buf.at(0).end())) - 25.0, (*std::max_element(Xr_buf.at(0).begin(), Xr_buf.at(0).end())) + 25.0);
            ui.scope_intp->yAxis->setRange((*std::min_element(Xr_buf.at(1).begin(), Xr_buf.at(1).end())) - 25.0, (*std::max_element(Xr_buf.at(1).begin(), Xr_buf.at(1).end())) + 25.0);
            break;
        }
        case 25: {
            ui.scope_intp->graph(0)->setData(Xr_buf.at(0), Xr_buf.at(2));
            ui.scope_intp->xAxis->setLabel("X (mm)");
            ui.scope_intp->yAxis->setLabel("Z (mm)");
            ui.scope_intp->xAxis->setRange((*std::min_element(Xr_buf.at(0).begin(), Xr_buf.at(0).end())) - 25.0, (*std::max_element(Xr_buf.at(0).begin(), Xr_buf.at(0).end())) + 25.0);
            ui.scope_intp->yAxis->setRange((*std::min_element(Xr_buf.at(2).begin(), Xr_buf.at(2).end())) - 25.0, (*std::max_element(Xr_buf.at(2).begin(), Xr_buf.at(2).end())) + 25.0);
            break;
        }
        case 26: {
            ui.scope_intp->graph(0)->setData(Xr_buf.at(1), Xr_buf.at(2));
            ui.scope_intp->xAxis->setLabel("Y (mm)");
            ui.scope_intp->yAxis->setLabel("Z (mm)");
            ui.scope_intp->xAxis->setRange((*std::min_element(Xr_buf.at(1).begin(), Xr_buf.at(1).end())) - 25.0, (*std::max_element(Xr_buf.at(1).begin(), Xr_buf.at(1).end())) + 25.0);
            ui.scope_intp->yAxis->setRange((*std::min_element(Xr_buf.at(2).begin(), Xr_buf.at(2).end())) - 25.0, (*std::max_element(Xr_buf.at(2).begin(), Xr_buf.at(2).end())) + 25.0);
            break;
        }
        }
    }

    ui.scope_intp->replot();
    ui.scope_intp->update();
}
//
void winUI::waitstart(void) {
    ui.rtss_message->append("Start PID parameter setting");
    // start worker thread to check for busy state
    Worker* worker = new Worker();

    QThread* workerThread = new QThread;

    worker->moveToThread(workerThread);

    connect(workerThread, &QThread::started, this, [&]() {
        emit startwaiting(&(rt605_pid->busy));
        });

    connect(this, &winUI::startwaiting, worker, &Worker::checkBusy);
    connect(worker, &Worker::workFinished, this, &winUI::waitfinished);
    connect(worker, &Worker::workFinished, workerThread, &QThread::quit);
    connect(workerThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);

    ui.verticalLayout_9->setEnabled(FALSE);

    workerThread->start();
}

void winUI::waitfinished(void) {
    ui.rtss_message->append("PID setting Finshed");
    // read PID shared memory parameter and write to GUI
    int i = ui.select_joints->currentIndex();
    ui.kpp->setValue((double)this->rt605_pid->kpp[i]);
    ui.kpi->setValue((double)this->rt605_pid->kpi[i]);
    ui.kvp->setValue((double)this->rt605_pid->kvp[i]);
    ui.kvi->setValue((double)this->rt605_pid->kvi[i]);
    ui.verticalLayout_9->setEnabled(TRUE);
}

void winUI::waitRTSSTaskStart(void) {
    ui.rtss_message->append("Start RTSS Task");
    // start worker thread to check for busy state
    WaitRTSS* worker = new WaitRTSS();

    QThread* workerThread = new QThread;

    worker->moveToThread(workerThread);

    connect(workerThread, &QThread::started, this, [&]() {
        emit startRTSSwaiting(&(rtss_task_status->busy));
        });

    connect(this, &winUI::startRTSSwaiting, worker, &WaitRTSS::checkBusy);
    connect(worker, &WaitRTSS::RTSSTaskFinished, this, &winUI::waitRTSSTaskFinished);
    connect(worker, &WaitRTSS::RTSSTaskFinished, workerThread, &QThread::quit);
    connect(workerThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);
    //ui.pb_start_sweep->setEnabled(false);

    workerThread->start();
}

void winUI::waitRTSSTaskFinished(void) {
    ui.rtss_message->append("RTSS Task Finshed");
    ui.rtss_message->append(QString(rtss_task_status->file_name));
    if (strstr(rtss_task_status->file_name, "sinusoidal") != NULL) {
        this->plot_sinusoidalTest_result(rtss_task_status->file_name);
    }
    else if (strstr(rtss_task_status->file_name, "sweepALL") != NULL) {
        this->plot_all_joints_fft(rtss_task_status->file_name);
    }
    else if (strstr(rtss_task_status->file_name, "sweep") != NULL) {
        this->plot_fft_result(rtss_task_status->file_name);
    }
}

//
int winUI::getIntpScopeSelectChannel(void) {
    int ch = 0;
    for (int i = 0; i < 28; ++i) {
        if (select_intp_scope_channel[i]->isChecked()) {
            ch = i;
            break;
        }
    }
    return ch;
}
//
void winUI::setupWaitRtssDialog(void) {
    this->wait_rtss_dialog = new WaitRtssDialog;
    this->wait_rtss_dialog->SetMailBoxHandles(h_ui_ackEvent, h_ui_accessEvent, ui_mail);
    this->wait_rtss_thread = new QThread;
    //this->wait_rtss_dialog->moveToThread(wait_rtss_thread);
    //this->wait_rtss_thread->start();
}
//
#pragma region WaitRtssDialog
WaitRtssDialog::WaitRtssDialog(void) {
    SetLayout();
    QObject::connect(&polling_timer, &QTimer::timeout, this, &WaitRtssDialog::slot_check_ack);
    QObject::connect(this, &WaitRtssDialog::signal_show, this, &WaitRtssDialog::slot_show);
    QObject::connect(&pb_exit, &QPushButton::clicked, this, &WaitRtssDialog::slot_exit_pb);
    polling_timer.start(500);
}
//
void WaitRtssDialog::SetLayout(void) {
    this->dialog.setGeometry(200, 200, 300, 200);
    this->dialog.setWindowTitle("Wait RTSS Acknowledge");

    message.setText(tr("Waiting ...."));

    //pb_exit.setGeometry(100, 100, 100, 100);
    pb_exit.setText(tr("STOP"));

    this->window_hLayout.addWidget(&thread_id);
    this->window_hLayout.addWidget(&message);
    this->window_hLayout.addWidget(&pb_exit);

    this->window_hLayout.setStretch(0, 1);
    this->window_hLayout.setStretch(1, 3);
    this->window_hLayout.setStretch(2, 3);

    this->dialog.setLayout(&window_hLayout);
}
//
void WaitRtssDialog::SetMailBoxHandles(HANDLE ack, HANDLE access, MailBox* pmail) {
    this->gui_access = access;
    this->gui_ack = ack;
    this->ui_mail = pmail;
}
//
void WaitRtssDialog::slot_check_ack(void) {
    thread_id.setText(QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId())));
    //if (ui_mail->Receiver == Node::Win_HMI)
    if (1 == 1)
    {
        ui_mail->ReadMessage(gui_access, mail_copy);
        switch (mail_copy.Action) {
        case UI_Action::Rtss_Ack: {
            emit signal_gui_ack();
            break;
        }
        }
        ui_mail->Ack = false;
        ui_mail->Busy = false;
    }
}
//
void WaitRtssDialog::slot_show(bool show) {
    if (show)
        this->dialog.show();
    else
        this->dialog.hide();
}
//
void WaitRtssDialog::slot_exit_pb(void) {
    emit signal_gui_ack();
}
#pragma endregion WaitRtssDialog

#pragma region Plot3D
Plot3D::Plot3D(void) {
    figure.addGraph();
}
Plot3D::~Plot3D(void) {

}
void Plot3D::plot(QVector<double>& xData, QVector<double>& yData, QVector<double>& zData) {
    figure.graph(0)->setData(equivalent_data[0], equivalent_data[1]);
}
void Plot3D::setLabel(QString xLabel, QString yLabel, QString zLabel) {

}
void Plot3D::setSacatter(QCPScatterStyle::ScatterShape shape, double size) {
    figure.graph(0)->setScatterStyle(QCPScatterStyle(shape, size));
}
void Plot3D::xRange(double min, double max) {

}
void Plot3D::yRange(double min, double max) {

}
void Plot3D::zRange(double min, double max) {

}
void Plot3D::RotX(double rot) {

}
void Plot3D::RotY(double rot) {

}
void Plot3D::RotZ(double rot) {

}
void Plot3D::slot_show(bool show) {
    if (show)
        window.show();
    else
        window.hide();
}
#pragma endregion Plot3D