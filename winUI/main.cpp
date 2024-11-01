#include "winUI.h"
#include <QtWidgets/QApplication>

void RunICRC_RTSS(void);

// Custom message handler
void customMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
    case QtDebugMsg:
        fprintf(stderr, "Debug: %s\n", localMsg.constData());
        break;
    case QtInfoMsg:
        fprintf(stderr, "Info: %s\n", localMsg.constData());
        break;
    case QtWarningMsg:
        fprintf(stderr, "Warning: %s\n", localMsg.constData());
        break;
    case QtCriticalMsg:
        fprintf(stderr, "Critical: %s\n", localMsg.constData());
        break;
    case QtFatalMsg:
        fprintf(stderr, "Fatal: %s\n", localMsg.constData());
        abort(); // Terminate the program on a fatal error
    }
}

int main(int argc, char *argv[])
{
    // open rtss:
    std::thread open_rtss(RunICRC_RTSS);
    open_rtss.detach();

    QApplication a(argc, argv);
    winUI w;
    // Install custom message handler
    qInstallMessageHandler(customMessageHandler);

    w.show();
    return a.exec();
}
void RunICRC_RTSS(void) {
    wchar_t app_directory[128];
    DWORD len = GetCurrentDirectory(128, app_directory);
    wchar_t PathStr[128] = L"\"";
    lstrcat(PathStr, app_directory);
    std::cout << "test" << std::endl;
    HINSTANCE nRet = ShellExecute(NULL, NULL, L"C:\\Program Files\\IntervalZero\\RTX64\\bin\\RtssRun", \
        L"..\\RtssDebug\\Rt_ICRC.rtss", NULL, SW_HIDE);
}
