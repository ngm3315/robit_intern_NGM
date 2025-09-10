/****************************************************************************
** Meta object code from reading C++ file 'homescreen.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.7.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../homescreen.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'homescreen.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_CLASSHomeScreenENDCLASS_t {};
constexpr auto qt_meta_stringdata_CLASSHomeScreenENDCLASS = QtMocHelpers::stringData(
    "HomeScreen",
    "startGame",
    "",
    "isHost",
    "sendDatagram",
    "data",
    "QHostAddress",
    "peerAddress",
    "peerPort",
    "rematchRequested",
    "opponentVotedForRematch",
    "opponentQuit",
    "Reset",
    "bindSocket",
    "localPort",
    "onDatagramReceived",
    "sender",
    "senderPort",
    "isConnected",
    "onConnectClicked",
    "onQuitClicked",
    "onRematchClicked",
    "onExitToHomeClicked",
    "sendReadyStatus",
    "on_reset_botton_clicked",
    "onInputChanged"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSHomeScreenENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,  104,    2, 0x06,    1 /* Public */,
       4,    3,  107,    2, 0x06,    3 /* Public */,
       9,    0,  114,    2, 0x06,    7 /* Public */,
      10,    0,  115,    2, 0x06,    8 /* Public */,
      11,    0,  116,    2, 0x06,    9 /* Public */,
      12,    0,  117,    2, 0x06,   10 /* Public */,
      13,    1,  118,    2, 0x06,   11 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      15,    4,  121,    2, 0x0a,   13 /* Public */,
      19,    0,  130,    2, 0x08,   18 /* Private */,
      20,    0,  131,    2, 0x08,   19 /* Private */,
      21,    0,  132,    2, 0x08,   20 /* Private */,
      22,    0,  133,    2, 0x08,   21 /* Private */,
      23,    0,  134,    2, 0x08,   22 /* Private */,
      24,    0,  135,    2, 0x08,   23 /* Private */,
      25,    0,  136,    2, 0x08,   24 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 6, QMetaType::UShort,    5,    7,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::UShort,   14,

 // slots: parameters
    QMetaType::Void, QMetaType::QByteArray, 0x80000000 | 6, QMetaType::UShort, QMetaType::Bool,    5,   16,   17,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject HomeScreen::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CLASSHomeScreenENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSHomeScreenENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSHomeScreenENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<HomeScreen, std::true_type>,
        // method 'startGame'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'sendDatagram'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QHostAddress, std::false_type>,
        QtPrivate::TypeAndForceComplete<quint16, std::false_type>,
        // method 'rematchRequested'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'opponentVotedForRematch'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'opponentQuit'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'Reset'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'bindSocket'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<quint16, std::false_type>,
        // method 'onDatagramReceived'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QByteArray &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QHostAddress, std::false_type>,
        QtPrivate::TypeAndForceComplete<quint16, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'onConnectClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onQuitClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onRematchClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onExitToHomeClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'sendReadyStatus'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'on_reset_botton_clicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onInputChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void HomeScreen::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<HomeScreen *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->startGame((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 1: _t->sendDatagram((*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QHostAddress>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<quint16>>(_a[3]))); break;
        case 2: _t->rematchRequested(); break;
        case 3: _t->opponentVotedForRematch(); break;
        case 4: _t->opponentQuit(); break;
        case 5: _t->Reset(); break;
        case 6: _t->bindSocket((*reinterpret_cast< std::add_pointer_t<quint16>>(_a[1]))); break;
        case 7: _t->onDatagramReceived((*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QHostAddress>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<quint16>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[4]))); break;
        case 8: _t->onConnectClicked(); break;
        case 9: _t->onQuitClicked(); break;
        case 10: _t->onRematchClicked(); break;
        case 11: _t->onExitToHomeClicked(); break;
        case 12: _t->sendReadyStatus(); break;
        case 13: _t->on_reset_botton_clicked(); break;
        case 14: _t->onInputChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (HomeScreen::*)(bool );
            if (_t _q_method = &HomeScreen::startGame; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)(const QByteArray & , QHostAddress , quint16 );
            if (_t _q_method = &HomeScreen::sendDatagram; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)();
            if (_t _q_method = &HomeScreen::rematchRequested; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)();
            if (_t _q_method = &HomeScreen::opponentVotedForRematch; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)();
            if (_t _q_method = &HomeScreen::opponentQuit; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)();
            if (_t _q_method = &HomeScreen::Reset; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (HomeScreen::*)(quint16 );
            if (_t _q_method = &HomeScreen::bindSocket; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject *HomeScreen::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *HomeScreen::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSHomeScreenENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int HomeScreen::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void HomeScreen::startGame(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void HomeScreen::sendDatagram(const QByteArray & _t1, QHostAddress _t2, quint16 _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void HomeScreen::rematchRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void HomeScreen::opponentVotedForRematch()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void HomeScreen::opponentQuit()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void HomeScreen::Reset()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void HomeScreen::bindSocket(quint16 _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_WARNING_POP
