#pragma once

// std C:
#include<Windows.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include <tchar.h>
#include <dirent.h> // folder operation

// std C++
#include <iostream>
#include<thread>
#include<algorithm>
// Qt:
#include <QtWidgets/QMainWindow>
#include<QTimer>
#include<QtCore/QVector>
#include<QtCore/QThread>
#include<QtWidgets/QDialog>
#include<QtWidgets/QLabel>
#include<QFont>
#include<QtWidgets/QLayout>
#include"QSlider"
#include "ui_winUI.h"
   
// The Others:
#define WINDOWS
#include "../libs/shared.hpp"
#include"../libs/icalab/RT605/rt605Kinematics.h"
#include"../libs/eigen-3.4.0/Eigen/Dense"
#include"../libs/eigen-3.4.0/Eigen/QR"
#include"../libs/eigen-3.4.0/unsupported/Eigen/FFT"
#include "cJSON/cJSON.h"
 
#pragma region the_other_module
//
class Plot3D :public QObject {
    Q_OBJECT

    QWidget window;
    QVBoxLayout vLayout;
    QHBoxLayout hLayout;
    QGridLayout gLayout;

    QCustomPlot figure;
    QSlider slider_rotx, slider_roty, slider_rotz;
    QSlider slider_dx, slider_dy, slider_dz;
    QVector<double> equivalent_data[2];

public:
    Plot3D(void);
    ~Plot3D(void);

    void setLabel(QString xLabel, QString yLabel, QString zLabel);
    void setSacatter(QCPScatterStyle::ScatterShape shape, double size = 2.0);
    void xRange(double min, double max);
    void yRange(double min, double max);
    void zRange(double min, double max);
    void RotX(double rot);
    void RotY(double rot);
    void RotZ(double rot);
signals:
    void signal_show(bool show);
    void signal_plot(QVector<double>& xData, QVector<double>& yData, QVector<double>& zData);
public slots:
    void slot_show(bool show);
    void plot(QVector<double>& xData, QVector<double>& yData, QVector<double>& zData);
};
//
class WaitRtssDialog :public QObject {
    Q_OBJECT

    QDialog dialog;
    QVBoxLayout window_hLayout;
    QPushButton pb_exit;
    QLabel message;
    QLabel thread_id;
    QTimer polling_timer;

    HANDLE gui_ack, gui_access;
    MailBox* ui_mail;
    MailBox mail_copy;

    int clk;
public:
    WaitRtssDialog(void);
    void SetLayout(void);
    void SetMailBoxHandles(HANDLE ack, HANDLE access, MailBox* pmail);
signals:
    void signal_exit_pb(void);
    void signal_gui_ack(void);
    void signal_show(bool show);
    void signal_gui_read_servo_pid(void);
public slots:
    void slot_exit_pb(void);
    void slot_show(bool show);
    void slot_check_ack(void);
};
//
class EncoderMonitor :public QWidget {
    Q_OBJECT
    bool* m_actived;
    double* m_motor_pulse;
public:
    EncoderMonitor(bool* parentActiveStatus);
    QLCDNumber lcd[6];
    QLabel label[6];
    void setupLayout(void);
    void ConnectMotorDataBuffer(double* _pulse_data);
    QTimer timer;

    //void closeEvent(QCloseEvent* event) override;
signals:
    void closeSignal(void);
};


// Worker class to perform the PID eddit in a separate thread
// when user pressed read PID value, the WinUI start this worker thread
// to wait for RTSS finished write PID parameters to shared memory
// worker keep busy loop until
class Worker : public QObject {
    Q_OBJECT

public slots:
    void checkBusy(bool *busy_ptr) {
        // wait for PID parameters to finished writing
        while ((*busy_ptr)==TRUE) {
            QThread::msleep(100);
        }
        emit workFinished();
    }

signals:
    void workFinished();
};

class WaitRTSS : public QObject {
    Q_OBJECT

public slots:
    void checkBusy(bool* busy_ptr) {
        // wait for RTSS Task to finish
        while ((*busy_ptr) == TRUE) {
            QThread::msleep(100);
        }
        emit RTSSTaskFinished();
    }

signals:
    void RTSSTaskFinished();
};


#pragma endregion the_other_module

class winUI : public QMainWindow
{
    Q_OBJECT
        char app_dir[MAX_PATH];

    HANDLE h_ui_accessEvent;
    HANDLE h_ui_ackEvent;
    HANDLE h_ui_mailbox;
    pMailBox ui_mail;

    HANDLE hSystem_state;
    pRcSystemState system_state;
    // INTP:
    QVector<double> intp_time;
    QVector<double> dData;
    std::array<QVector<double>, 12> Xr_buf;
    QRadioButton* select_intp_scope_channel[28];
    // 定義獨立的 window:
    EncoderMonitor* m_encoder_monitor;

    // 定義獨立的 window 之狀態用：
    struct WindowStatus {
        bool encoder_monitor;
    }m_subWindow_started;
    Plot3D figure3d;
    // 其餘特殊用之物件狀態：
    bool uiButtonState_for_jointJog[1];
    bool uiButtonState_for_CartesianJog;
    // QTimer:
    QTimer timer_system_status;
    QTimer timer_motion_feedback;
    // Robot Information:
    icalab::RT605<double> rt605;
    MotionFeedback* pShared_feedback;
    HANDLE hShared_feedback;
    // force control tab:
    QHBoxLayout* fc_setup_horizontal_layout;
    // force sensor Monitor:
    //ForceSensorMonitor fc_monitor;

    // Mechanism Analyzer:
    bool confirm_frequency_sweep_set;
    QDialog* alarm_dialog_did_not_confirm_frequency_sweep;
    QLabel* alarm_message_did_not_confirm_frequency_sweep;

    bool comfirm_sinusoidalTest_set;

    // for slot_DisableGUI
    bool GUI_enabled;

    // PID parameters
    PID *rt605_pid;
    HANDLE h_ui_PID;

    // RTSS Task parameters
    // PID parameters
    RTSSTask_Status* rtss_task_status;
    HANDLE h_ui_RTSS_Task;

    // QThreads:
    QThread* wait_rtss_thread;
    //
    WaitRtssDialog* wait_rtss_dialog;
public:
    winUI(QWidget* parent = nullptr);
    ~winUI();
    void setupSystemTab(void);
    void setupJogTab(void);
    void setupControlPanel(void);
    void setupPlots(void);
    void setupRcStateMessage(void);
    void setupRobotParameterTabel(void);
    void setupIntpScope(void);
    int  getIntpScopeSelectChannel(void);
    void setupWaitRtssDialog(void);
    void plot_fft_result(char* file_path);
    void plot_sinusoidalTest_result(char* file_path);
    void plot_all_joints_fft(char *folder_path); // plot all six joints fft result
    
    // set controller parameters
    void loadPID_JSON(void); //load PID setting from JSON file
    void writePID_JSON(void); //write current PID to JSON file


public slots:
    void slot_motion_feedback_display_routine(void);
    void slot_read_HRSS_intp(void);
    void slot_select_intp_scope(int ch);
    void slot_stop_csp_task(void);
    void slot_show_wait_rtss_ack_dialog(void);
    void slot_DisableGUI(void);
    void slot_EnableGUI(void);
    void EMG_Actiion_Event(bool action);
    void SystemStateMonitorRoutine(void);
    void ShowRobotParameters(void);
    void ExportRobotParameters(void);
    void ImportRobotParameters(void);

    
    
    /*
    * "JogOperation_PostProcess"
    * 為一些經過JOG模式後進行的一些後處理設定，
    例如：
    JOG 後因為關節角度改變了，所以不能馬上進行 "Servo Tuning" 頁面中的掃頻的功能，必須再 confirm 一次掃頻的參數設定。
    */
    void JogOperation_PostProcess(void);

    // slot for waiting worker to start
    void waitstart(void);
    void waitfinished(void);

    void waitRTSSTaskStart(void);
    void waitRTSSTaskFinished(void);

signals:
    void signal_select_intp_scope(int ch);
    void signal_stop_csp_task(void);
    void signal_show_wait_rtss_ack_dialog(void);
    void signal_update_robot_parameters(void);
    void signal_show_robot_parameters(void);
    void signal_DisableGUI(void);
    void signal_EnableGUI(void);

    // signal to trigger waiting
    void startwaiting(bool *);
    void signal_waiting(void);

    void startRTSSwaiting(bool*);
    void signal_RTSSwaiting(void);

private:
    Ui::winUIClass ui;
    QLabel pidlabel;

    // events:
    void closeEvent(QCloseEvent* event) override;
};

extern struct AppDirectory* app_dir;
extern HANDLE hApp_dir;