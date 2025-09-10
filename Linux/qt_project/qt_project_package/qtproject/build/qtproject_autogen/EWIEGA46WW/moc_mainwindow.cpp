/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.7.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../mainwindow.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.7.3. It"
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
struct qt_meta_stringdata_CLASSMainWindowENDCLASS_t {};
constexpr auto qt_meta_stringdata_CLASSMainWindowENDCLASS = QtMocHelpers::stringData(
    "MainWindow",
    "handleGameStart",
    "",
    "isHost",
    "handleGameOver",
    "result",
    "handleCountdown",
    "handleDatagramReadyRead",
    "sendDatagram",
    "data",
    "QHostAddress",
    "peerAddress",
    "peerPort",
    "handleRematchRequest",
    "handleOpponentQuit",
    "handleBindSocket",
    "localPort",
    "handleOpponentRematchVote"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSMainWindowENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   74,    2, 0x08,    1 /* Private */,
       4,    1,   77,    2, 0x08,    3 /* Private */,
       6,    0,   80,    2, 0x08,    5 /* Private */,
       7,    0,   81,    2, 0x08,    6 /* Private */,
       8,    1,   82,    2, 0x08,    7 /* Private */,
       8,    3,   85,    2, 0x08,    9 /* Private */,
      13,    0,   92,    2, 0x08,   13 /* Private */,
      14,    0,   93,    2, 0x08,   14 /* Private */,
      15,    1,   94,    2, 0x08,   15 /* Private */,
      17,    0,   97,    2, 0x08,   17 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QByteArray,    9,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 10, QMetaType::UShort,    9,   11,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::UShort,   16,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CLASSMainWindowENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSMainWindowENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSMainWindowENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,
        // method 'handleGameStart'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'handleGameOver'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'handleCountdown'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleDatagramReadyRead'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'sendDatagram'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        // method 'sendDatagram'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QHostAddress, std::false_type>,
        QtPrivate::TypeAndForceComplete<quint16, std::false_type>,
        // method 'handleRematchRequest'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleOpponentQuit'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleBindSocket'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<quint16, std::false_type>,
        // method 'handleOpponentRematchVote'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->handleGameStart((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 1: _t->handleGameOver((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->handleCountdown(); break;
        case 3: _t->handleDatagramReadyRead(); break;
        case 4: _t->sendDatagram((*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[1]))); break;
        case 5: _t->sendDatagram((*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QHostAddress>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<quint16>>(_a[3]))); break;
        case 6: _t->handleRematchRequest(); break;
        case 7: _t->handleOpponentQuit(); break;
        case 8: _t->handleBindSocket((*reinterpret_cast< std::add_pointer_t<quint16>>(_a[1]))); break;
        case 9: _t->handleOpponentRematchVote(); break;
        default: ;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSMainWindowENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
