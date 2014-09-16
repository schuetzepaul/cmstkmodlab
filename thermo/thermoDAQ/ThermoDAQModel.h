#ifndef THERMODAQMODEL_H
#define THERMODAQMODEL_H

#include <QObject>
#include <QVector>
#include <QDateTime>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>

#include <qwt_date.h>

#include "HuberPetiteFleurModel.h"
#include "KeithleyModel.h"
#include "HamegModel.h"
#include "PfeifferModel.h"
#include "IotaModel.h"
#include "ArduinoPresModel.h"

//template <typename value_type> class ThermoDAQValue
//{
//public:
//    ThermoDAQValue(unsigned int time, value_type value) {
//        time_ = time;
//        value_ = value;
//    }

//    unsigned int& time() { return time_; }
//    const unsigned int& time() const { return time_; }
//    value_type& value() { return value_; }
//    const value_type& value() const { return value_; }

//protected:

//    unsigned int time_;
//    value_type value_;
//};

template <typename value_type> class ThermoDAQValueVector : public QVector<double>
{
public:
    ThermoDAQValueVector() {

    }

    void clear() {
        QVector<double>::clear();
        values_.clear();
    }

    bool push(const QDateTime& time, value_type value) {
        bool ret = QVector<double>::size()>0 && lastValue()==value;
        QVector<double>::append(QwtDate::toDouble(time));
        values_.append(value);
        return ret;
    }

    bool pushIfChanged(const QDateTime& time, value_type value) {
        if (QVector<double>::size()>0 && lastValue()==value) return false;
        QVector<double>::append(QwtDate::toDouble(time));
        values_.append(value);
        return true;
    }

    double& lastTime() { return QVector<double>::last(); }
    const double& lastTime() const { return QVector<double>::last(); }
    value_type& lastValue() { return values_.last(); }

    const double* constTimes() const { return QVector<double>::constData(); }
    const value_type* constValues() const { return values_.constData(); }

protected:
    QVector<value_type> values_;
};

class ThermoDAQModel : public QObject
{
    Q_OBJECT
public:
    explicit ThermoDAQModel(HuberPetiteFleurModel* huberModel,
                            KeithleyModel* keithleyModel,
                            HamegModel* hamegModel,
                            PfeifferModel* pfeifferModel,
			    IotaModel* iotaModel,
			    ArduinoPresModel* arduinoPresModel,
                            QObject *parent = 0);

    QDateTime& currentTime();

    void customDAQMessage(const QString & message);
    void createDAQStatusMessage(QString & buffer);

    void myMoveToThread(QThread *thread);

    bool daqState() const { return daqState_; }

public slots:

    void startMeasurement();
    void stopMeasurement();

protected slots:

    void huberInfoChanged();
    void keithleySensorStateChanged(unsigned int sensor, State newState);
    void keithleyTemperatureChanged(unsigned int sensor, double temperature);
    void hamegInfoChanged();
    void pfeifferInfoChanged();
    void iotaInfoChanged();
    void arduinoPresInfoChanged();

    void clearHistory();

protected:

  bool daqState_;

  HuberPetiteFleurModel* huberModel_;
  KeithleyModel* keithleyModel_;
  HamegModel* hamegModel_;
  PfeifferModel* pfeifferModel_;
  IotaModel* iotaModel_;
  ArduinoPresModel* arduinoPresModel_;

  QMutex mutex_;

  QDateTime currentTime_;

  template <typename T> bool updateIfChanged(T &variable, T newValue) {
      if (variable==newValue) return false;
      variable = newValue;
      return true;
  }

  // HUBER DATA
  bool huberCirculator_;
  float huberWorkingTemperature_;
  float huberBathTemperature_;

  // KEITHLEY DATA
  State keithleySensorState_[10];
  double keithleyTemperature_[10];

  // HAMEG DATA
  bool hamegRemoteMode_;
  bool hamegOutputsEnabled_;
  bool hamegCVMode_[2];
  float hamegSetVoltage_[2];
  float hamegSetCurrent_[2];
  float hamegVoltage_[2];
  float hamegCurrent_[2];

  // PFEIFFER DATA
  int pfeifferStatus1_;
  double pfeifferPressure1_;
  int pfeifferStatus2_;
  double pfeifferPressure2_;

  // IOTA DATA
  bool iotaPumpEnabled_;
  float iotaActPressure_;
  float iotaSetPressure_;
  float iotaActFlow_;
  float iotaSetFlow_;
  
  // ARDUINO PRES DATA
  float arduinoPressureA_;
  float arduinoPressureB_;
  
signals:

  void daqMessage(const QString & message);
  void daqStateChanged(bool running);
  void newDataAvailable();
};

#endif // THERMODAQMODEL_H
