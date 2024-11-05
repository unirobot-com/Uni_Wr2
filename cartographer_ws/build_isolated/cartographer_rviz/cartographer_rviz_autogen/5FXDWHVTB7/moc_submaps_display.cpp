/****************************************************************************
** Meta object code from reading C++ file 'submaps_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cartographer_rviz/submaps_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'submaps_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cartographer_rviz__Trajectory_t {
    QByteArrayData data[4];
    char stringdata0[75];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cartographer_rviz__Trajectory_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cartographer_rviz__Trajectory_t qt_meta_stringdata_cartographer_rviz__Trajectory = {
    {
QT_MOC_LITERAL(0, 0, 29), // "cartographer_rviz::Trajectory"
QT_MOC_LITERAL(1, 30, 17), // "AllEnabledToggled"
QT_MOC_LITERAL(2, 48, 0), // ""
QT_MOC_LITERAL(3, 49, 25) // "PoseMarkersEnabledToggled"

    },
    "cartographer_rviz::Trajectory\0"
    "AllEnabledToggled\0\0PoseMarkersEnabledToggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cartographer_rviz__Trajectory[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08 /* Private */,
       3,    0,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void cartographer_rviz::Trajectory::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Trajectory *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->AllEnabledToggled(); break;
        case 1: _t->PoseMarkersEnabledToggled(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject cartographer_rviz::Trajectory::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_cartographer_rviz__Trajectory.data,
    qt_meta_data_cartographer_rviz__Trajectory,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cartographer_rviz::Trajectory::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cartographer_rviz::Trajectory::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cartographer_rviz__Trajectory.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int cartographer_rviz::Trajectory::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
struct qt_meta_stringdata_cartographer_rviz__SubmapsDisplay_t {
    QByteArrayData data[6];
    char stringdata0[103];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cartographer_rviz__SubmapsDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cartographer_rviz__SubmapsDisplay_t qt_meta_stringdata_cartographer_rviz__SubmapsDisplay = {
    {
QT_MOC_LITERAL(0, 0, 33), // "cartographer_rviz::SubmapsDis..."
QT_MOC_LITERAL(1, 34, 5), // "Reset"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 17), // "AllEnabledToggled"
QT_MOC_LITERAL(4, 59, 25), // "PoseMarkersEnabledToggled"
QT_MOC_LITERAL(5, 85, 17) // "ResolutionToggled"

    },
    "cartographer_rviz::SubmapsDisplay\0"
    "Reset\0\0AllEnabledToggled\0"
    "PoseMarkersEnabledToggled\0ResolutionToggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cartographer_rviz__SubmapsDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x08 /* Private */,
       3,    0,   35,    2, 0x08 /* Private */,
       4,    0,   36,    2, 0x08 /* Private */,
       5,    0,   37,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void cartographer_rviz::SubmapsDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SubmapsDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->Reset(); break;
        case 1: _t->AllEnabledToggled(); break;
        case 2: _t->PoseMarkersEnabledToggled(); break;
        case 3: _t->ResolutionToggled(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject cartographer_rviz::SubmapsDisplay::staticMetaObject = { {
    &::rviz::MessageFilterDisplay< ::cartographer_ros_msgs::SubmapList>::staticMetaObject,
    qt_meta_stringdata_cartographer_rviz__SubmapsDisplay.data,
    qt_meta_data_cartographer_rviz__SubmapsDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cartographer_rviz::SubmapsDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cartographer_rviz::SubmapsDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cartographer_rviz__SubmapsDisplay.stringdata0))
        return static_cast<void*>(this);
    return ::rviz::MessageFilterDisplay< ::cartographer_ros_msgs::SubmapList>::qt_metacast(_clname);
}

int cartographer_rviz::SubmapsDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ::rviz::MessageFilterDisplay< ::cartographer_ros_msgs::SubmapList>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
