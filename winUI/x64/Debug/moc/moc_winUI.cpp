/****************************************************************************
** Meta object code from reading C++ file 'winUI.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.5.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../winUI.h"
#include <QtGui/qtextcursor.h>
#include <QtGui/qscreen.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'winUI.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.5.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSPlot3DENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSPlot3DENDCLASS = QtMocHelpers::stringData(
    "Plot3D",
    "signal_show",
    "",
    "show",
    "signal_plot",
    "QList<double>&",
    "xData",
    "yData",
    "zData",
    "slot_show",
    "plot"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSPlot3DENDCLASS_t {
    uint offsetsAndSizes[22];
    char stringdata0[7];
    char stringdata1[12];
    char stringdata2[1];
    char stringdata3[5];
    char stringdata4[12];
    char stringdata5[15];
    char stringdata6[6];
    char stringdata7[6];
    char stringdata8[6];
    char stringdata9[10];
    char stringdata10[5];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSPlot3DENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSPlot3DENDCLASS_t qt_meta_stringdata_CLASSPlot3DENDCLASS = {
    {
        QT_MOC_LITERAL(0, 6),  // "Plot3D"
        QT_MOC_LITERAL(7, 11),  // "signal_show"
        QT_MOC_LITERAL(19, 0),  // ""
        QT_MOC_LITERAL(20, 4),  // "show"
        QT_MOC_LITERAL(25, 11),  // "signal_plot"
        QT_MOC_LITERAL(37, 14),  // "QList<double>&"
        QT_MOC_LITERAL(52, 5),  // "xData"
        QT_MOC_LITERAL(58, 5),  // "yData"
        QT_MOC_LITERAL(64, 5),  // "zData"
        QT_MOC_LITERAL(70, 9),  // "slot_show"
        QT_MOC_LITERAL(80, 4)   // "plot"
    },
    "Plot3D",
    "signal_show",
    "",
    "show",
    "signal_plot",
    "QList<double>&",
    "xData",
    "yData",
    "zData",
    "slot_show",
    "plot"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSPlot3DENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   38,    2, 0x06,    1 /* Public */,
       4,    3,   41,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       9,    1,   48,    2, 0x0a,    7 /* Public */,
      10,    3,   51,    2, 0x0a,    9 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5,    6,    7,    8,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, 0x80000000 | 5, 0x80000000 | 5, 0x80000000 | 5,    6,    7,    8,

       0        // eod
};

Q_CONSTINIT const QMetaObject Plot3D::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSPlot3DENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSPlot3DENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSPlot3DENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<Plot3D, std::true_type>,
        // method 'signal_show'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'signal_plot'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>,
        // method 'slot_show'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'plot'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QVector<double> &, std::false_type>
    >,
    nullptr
} };

void Plot3D::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Plot3D *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_show((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 1: _t->signal_plot((*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[3]))); break;
        case 2: _t->slot_show((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 3: _t->plot((*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QList<double>&>>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Plot3D::*)(bool );
            if (_t _q_method = &Plot3D::signal_show; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Plot3D::*)(QVector<double> & , QVector<double> & , QVector<double> & );
            if (_t _q_method = &Plot3D::signal_plot; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject *Plot3D::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Plot3D::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSPlot3DENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Plot3D::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void Plot3D::signal_show(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Plot3D::signal_plot(QVector<double> & _t1, QVector<double> & _t2, QVector<double> & _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS = QtMocHelpers::stringData(
    "WaitRtssDialog",
    "signal_exit_pb",
    "",
    "signal_gui_ack",
    "signal_show",
    "show",
    "signal_gui_read_servo_pid",
    "slot_exit_pb",
    "slot_show",
    "slot_check_ack"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS_t {
    uint offsetsAndSizes[20];
    char stringdata0[15];
    char stringdata1[15];
    char stringdata2[1];
    char stringdata3[15];
    char stringdata4[12];
    char stringdata5[5];
    char stringdata6[26];
    char stringdata7[13];
    char stringdata8[10];
    char stringdata9[15];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS_t qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS = {
    {
        QT_MOC_LITERAL(0, 14),  // "WaitRtssDialog"
        QT_MOC_LITERAL(15, 14),  // "signal_exit_pb"
        QT_MOC_LITERAL(30, 0),  // ""
        QT_MOC_LITERAL(31, 14),  // "signal_gui_ack"
        QT_MOC_LITERAL(46, 11),  // "signal_show"
        QT_MOC_LITERAL(58, 4),  // "show"
        QT_MOC_LITERAL(63, 25),  // "signal_gui_read_servo_pid"
        QT_MOC_LITERAL(89, 12),  // "slot_exit_pb"
        QT_MOC_LITERAL(102, 9),  // "slot_show"
        QT_MOC_LITERAL(112, 14)   // "slot_check_ack"
    },
    "WaitRtssDialog",
    "signal_exit_pb",
    "",
    "signal_gui_ack",
    "signal_show",
    "show",
    "signal_gui_read_servo_pid",
    "slot_exit_pb",
    "slot_show",
    "slot_check_ack"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSWaitRtssDialogENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   56,    2, 0x06,    1 /* Public */,
       3,    0,   57,    2, 0x06,    2 /* Public */,
       4,    1,   58,    2, 0x06,    3 /* Public */,
       6,    0,   61,    2, 0x06,    5 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       7,    0,   62,    2, 0x0a,    6 /* Public */,
       8,    1,   63,    2, 0x0a,    7 /* Public */,
       9,    0,   66,    2, 0x0a,    9 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject WaitRtssDialog::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSWaitRtssDialogENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<WaitRtssDialog, std::true_type>,
        // method 'signal_exit_pb'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_gui_ack'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_show'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'signal_gui_read_servo_pid'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_exit_pb'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_show'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'slot_check_ack'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void WaitRtssDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WaitRtssDialog *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_exit_pb(); break;
        case 1: _t->signal_gui_ack(); break;
        case 2: _t->signal_show((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 3: _t->signal_gui_read_servo_pid(); break;
        case 4: _t->slot_exit_pb(); break;
        case 5: _t->slot_show((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 6: _t->slot_check_ack(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (WaitRtssDialog::*)();
            if (_t _q_method = &WaitRtssDialog::signal_exit_pb; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (WaitRtssDialog::*)();
            if (_t _q_method = &WaitRtssDialog::signal_gui_ack; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (WaitRtssDialog::*)(bool );
            if (_t _q_method = &WaitRtssDialog::signal_show; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (WaitRtssDialog::*)();
            if (_t _q_method = &WaitRtssDialog::signal_gui_read_servo_pid; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject *WaitRtssDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WaitRtssDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSWaitRtssDialogENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int WaitRtssDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void WaitRtssDialog::signal_exit_pb()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void WaitRtssDialog::signal_gui_ack()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void WaitRtssDialog::signal_show(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void WaitRtssDialog::signal_gui_read_servo_pid()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSEncoderMonitorENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSEncoderMonitorENDCLASS = QtMocHelpers::stringData(
    "EncoderMonitor",
    "closeSignal",
    ""
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSEncoderMonitorENDCLASS_t {
    uint offsetsAndSizes[6];
    char stringdata0[15];
    char stringdata1[12];
    char stringdata2[1];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSEncoderMonitorENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSEncoderMonitorENDCLASS_t qt_meta_stringdata_CLASSEncoderMonitorENDCLASS = {
    {
        QT_MOC_LITERAL(0, 14),  // "EncoderMonitor"
        QT_MOC_LITERAL(15, 11),  // "closeSignal"
        QT_MOC_LITERAL(27, 0)   // ""
    },
    "EncoderMonitor",
    "closeSignal",
    ""
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSEncoderMonitorENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   20,    2, 0x06,    1 /* Public */,

 // signals: parameters
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject EncoderMonitor::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CLASSEncoderMonitorENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSEncoderMonitorENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSEncoderMonitorENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<EncoderMonitor, std::true_type>,
        // method 'closeSignal'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void EncoderMonitor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<EncoderMonitor *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->closeSignal(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (EncoderMonitor::*)();
            if (_t _q_method = &EncoderMonitor::closeSignal; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

const QMetaObject *EncoderMonitor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *EncoderMonitor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSEncoderMonitorENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int EncoderMonitor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void EncoderMonitor::closeSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSPID_EdditENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSPID_EdditENDCLASS = QtMocHelpers::stringData(
    "PID_Eddit",
    "workFinished",
    "",
    "checkBusy",
    "bool&",
    "busy_ptr"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSPID_EdditENDCLASS_t {
    uint offsetsAndSizes[12];
    char stringdata0[10];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[10];
    char stringdata4[6];
    char stringdata5[9];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSPID_EdditENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSPID_EdditENDCLASS_t qt_meta_stringdata_CLASSPID_EdditENDCLASS = {
    {
        QT_MOC_LITERAL(0, 9),  // "PID_Eddit"
        QT_MOC_LITERAL(10, 12),  // "workFinished"
        QT_MOC_LITERAL(23, 0),  // ""
        QT_MOC_LITERAL(24, 9),  // "checkBusy"
        QT_MOC_LITERAL(34, 5),  // "bool&"
        QT_MOC_LITERAL(40, 8)   // "busy_ptr"
    },
    "PID_Eddit",
    "workFinished",
    "",
    "checkBusy",
    "bool&",
    "busy_ptr"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSPID_EdditENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   26,    2, 0x06,    1 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       3,    1,   27,    2, 0x0a,    2 /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4,    5,

       0        // eod
};

Q_CONSTINIT const QMetaObject PID_Eddit::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSPID_EdditENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSPID_EdditENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSPID_EdditENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<PID_Eddit, std::true_type>,
        // method 'workFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'checkBusy'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool &, std::false_type>
    >,
    nullptr
} };

void PID_Eddit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PID_Eddit *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->workFinished(); break;
        case 1: _t->checkBusy((*reinterpret_cast< std::add_pointer_t<bool&>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PID_Eddit::*)();
            if (_t _q_method = &PID_Eddit::workFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject *PID_Eddit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PID_Eddit::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSPID_EdditENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int PID_Eddit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void PID_Eddit::workFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSwinUIENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSwinUIENDCLASS = QtMocHelpers::stringData(
    "winUI",
    "signal_select_intp_scope",
    "",
    "ch",
    "signal_stop_csp_task",
    "signal_show_wait_rtss_ack_dialog",
    "signal_update_robot_parameters",
    "signal_show_robot_parameters",
    "signal_DisableGUI",
    "signal_EnableGUI",
    "startwaiting",
    "slot_motion_feedback_display_routine",
    "slot_read_HRSS_intp",
    "slot_select_intp_scope",
    "slot_stop_csp_task",
    "slot_show_wait_rtss_ack_dialog",
    "slot_DisableGUI",
    "slot_EnableGUI",
    "EMG_Actiion_Event",
    "action",
    "SystemStateMonitorRoutine",
    "ShowRobotParameters",
    "ExportRobotParameters",
    "ImportRobotParameters",
    "JogOperation_PostProcess",
    "waitstart",
    "waitfinished"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSwinUIENDCLASS_t {
    uint offsetsAndSizes[54];
    char stringdata0[6];
    char stringdata1[25];
    char stringdata2[1];
    char stringdata3[3];
    char stringdata4[21];
    char stringdata5[33];
    char stringdata6[31];
    char stringdata7[29];
    char stringdata8[18];
    char stringdata9[17];
    char stringdata10[13];
    char stringdata11[37];
    char stringdata12[20];
    char stringdata13[23];
    char stringdata14[19];
    char stringdata15[31];
    char stringdata16[16];
    char stringdata17[15];
    char stringdata18[18];
    char stringdata19[7];
    char stringdata20[26];
    char stringdata21[20];
    char stringdata22[22];
    char stringdata23[22];
    char stringdata24[25];
    char stringdata25[10];
    char stringdata26[13];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSwinUIENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSwinUIENDCLASS_t qt_meta_stringdata_CLASSwinUIENDCLASS = {
    {
        QT_MOC_LITERAL(0, 5),  // "winUI"
        QT_MOC_LITERAL(6, 24),  // "signal_select_intp_scope"
        QT_MOC_LITERAL(31, 0),  // ""
        QT_MOC_LITERAL(32, 2),  // "ch"
        QT_MOC_LITERAL(35, 20),  // "signal_stop_csp_task"
        QT_MOC_LITERAL(56, 32),  // "signal_show_wait_rtss_ack_dialog"
        QT_MOC_LITERAL(89, 30),  // "signal_update_robot_parameters"
        QT_MOC_LITERAL(120, 28),  // "signal_show_robot_parameters"
        QT_MOC_LITERAL(149, 17),  // "signal_DisableGUI"
        QT_MOC_LITERAL(167, 16),  // "signal_EnableGUI"
        QT_MOC_LITERAL(184, 12),  // "startwaiting"
        QT_MOC_LITERAL(197, 36),  // "slot_motion_feedback_display_..."
        QT_MOC_LITERAL(234, 19),  // "slot_read_HRSS_intp"
        QT_MOC_LITERAL(254, 22),  // "slot_select_intp_scope"
        QT_MOC_LITERAL(277, 18),  // "slot_stop_csp_task"
        QT_MOC_LITERAL(296, 30),  // "slot_show_wait_rtss_ack_dialog"
        QT_MOC_LITERAL(327, 15),  // "slot_DisableGUI"
        QT_MOC_LITERAL(343, 14),  // "slot_EnableGUI"
        QT_MOC_LITERAL(358, 17),  // "EMG_Actiion_Event"
        QT_MOC_LITERAL(376, 6),  // "action"
        QT_MOC_LITERAL(383, 25),  // "SystemStateMonitorRoutine"
        QT_MOC_LITERAL(409, 19),  // "ShowRobotParameters"
        QT_MOC_LITERAL(429, 21),  // "ExportRobotParameters"
        QT_MOC_LITERAL(451, 21),  // "ImportRobotParameters"
        QT_MOC_LITERAL(473, 24),  // "JogOperation_PostProcess"
        QT_MOC_LITERAL(498, 9),  // "waitstart"
        QT_MOC_LITERAL(508, 12)   // "waitfinished"
    },
    "winUI",
    "signal_select_intp_scope",
    "",
    "ch",
    "signal_stop_csp_task",
    "signal_show_wait_rtss_ack_dialog",
    "signal_update_robot_parameters",
    "signal_show_robot_parameters",
    "signal_DisableGUI",
    "signal_EnableGUI",
    "startwaiting",
    "slot_motion_feedback_display_routine",
    "slot_read_HRSS_intp",
    "slot_select_intp_scope",
    "slot_stop_csp_task",
    "slot_show_wait_rtss_ack_dialog",
    "slot_DisableGUI",
    "slot_EnableGUI",
    "EMG_Actiion_Event",
    "action",
    "SystemStateMonitorRoutine",
    "ShowRobotParameters",
    "ExportRobotParameters",
    "ImportRobotParameters",
    "JogOperation_PostProcess",
    "waitstart",
    "waitfinished"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSwinUIENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,  152,    2, 0x06,    1 /* Public */,
       4,    0,  155,    2, 0x06,    3 /* Public */,
       5,    0,  156,    2, 0x06,    4 /* Public */,
       6,    0,  157,    2, 0x06,    5 /* Public */,
       7,    0,  158,    2, 0x06,    6 /* Public */,
       8,    0,  159,    2, 0x06,    7 /* Public */,
       9,    0,  160,    2, 0x06,    8 /* Public */,
      10,    0,  161,    2, 0x06,    9 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      11,    0,  162,    2, 0x0a,   10 /* Public */,
      12,    0,  163,    2, 0x0a,   11 /* Public */,
      13,    1,  164,    2, 0x0a,   12 /* Public */,
      14,    0,  167,    2, 0x0a,   14 /* Public */,
      15,    0,  168,    2, 0x0a,   15 /* Public */,
      16,    0,  169,    2, 0x0a,   16 /* Public */,
      17,    0,  170,    2, 0x0a,   17 /* Public */,
      18,    1,  171,    2, 0x0a,   18 /* Public */,
      20,    0,  174,    2, 0x0a,   20 /* Public */,
      21,    0,  175,    2, 0x0a,   21 /* Public */,
      22,    0,  176,    2, 0x0a,   22 /* Public */,
      23,    0,  177,    2, 0x0a,   23 /* Public */,
      24,    0,  178,    2, 0x0a,   24 /* Public */,
      25,    0,  179,    2, 0x0a,   25 /* Public */,
      26,    0,  180,    2, 0x0a,   26 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject winUI::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CLASSwinUIENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSwinUIENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSwinUIENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<winUI, std::true_type>,
        // method 'signal_select_intp_scope'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'signal_stop_csp_task'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_show_wait_rtss_ack_dialog'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_update_robot_parameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_show_robot_parameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_DisableGUI'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'signal_EnableGUI'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startwaiting'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_motion_feedback_display_routine'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_read_HRSS_intp'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_select_intp_scope'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'slot_stop_csp_task'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_show_wait_rtss_ack_dialog'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_DisableGUI'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'slot_EnableGUI'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'EMG_Actiion_Event'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'SystemStateMonitorRoutine'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ShowRobotParameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ExportRobotParameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ImportRobotParameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'JogOperation_PostProcess'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'waitstart'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'waitfinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void winUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<winUI *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signal_select_intp_scope((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 1: _t->signal_stop_csp_task(); break;
        case 2: _t->signal_show_wait_rtss_ack_dialog(); break;
        case 3: _t->signal_update_robot_parameters(); break;
        case 4: _t->signal_show_robot_parameters(); break;
        case 5: _t->signal_DisableGUI(); break;
        case 6: _t->signal_EnableGUI(); break;
        case 7: _t->startwaiting(); break;
        case 8: _t->slot_motion_feedback_display_routine(); break;
        case 9: _t->slot_read_HRSS_intp(); break;
        case 10: _t->slot_select_intp_scope((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 11: _t->slot_stop_csp_task(); break;
        case 12: _t->slot_show_wait_rtss_ack_dialog(); break;
        case 13: _t->slot_DisableGUI(); break;
        case 14: _t->slot_EnableGUI(); break;
        case 15: _t->EMG_Actiion_Event((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 16: _t->SystemStateMonitorRoutine(); break;
        case 17: _t->ShowRobotParameters(); break;
        case 18: _t->ExportRobotParameters(); break;
        case 19: _t->ImportRobotParameters(); break;
        case 20: _t->JogOperation_PostProcess(); break;
        case 21: _t->waitstart(); break;
        case 22: _t->waitfinished(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (winUI::*)(int );
            if (_t _q_method = &winUI::signal_select_intp_scope; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_stop_csp_task; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_show_wait_rtss_ack_dialog; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_update_robot_parameters; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_show_robot_parameters; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_DisableGUI; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::signal_EnableGUI; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (winUI::*)();
            if (_t _q_method = &winUI::startwaiting; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 7;
                return;
            }
        }
    }
}

const QMetaObject *winUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *winUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSwinUIENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int winUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 23)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
void winUI::signal_select_intp_scope(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void winUI::signal_stop_csp_task()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void winUI::signal_show_wait_rtss_ack_dialog()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void winUI::signal_update_robot_parameters()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void winUI::signal_show_robot_parameters()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void winUI::signal_DisableGUI()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void winUI::signal_EnableGUI()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void winUI::startwaiting()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}
QT_WARNING_POP
