
#include "DefoMainWindow.h"



///
///
///
DefoMainWindow::DefoMainWindow( QWidget* parent ) : QWidget( parent ) {

  setupUi();
  setupSignalsAndSlots();

  pollingDelay_ = new QTimer();

  // set default measurement id
  defaultMeasurementId();

  isRefImage_ = false;

  baseFolderName_ = basefolderTextedit_->toPlainText(); // LOAD DEFAULT

}



///
///
///
void DefoMainWindow::setupUi( void ) {

  //  if (MainWindow->objectName().isEmpty()) MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
  //  MainWindow->resize(1120, 900);
  resize(1120, 900);
  QFont font;
  font.setPointSize(10);
//   MainWindow->setFont(font);
  setFont(font);
  //  centralwidget = new QWidget(MainWindow);
  centralwidget = new QWidget(this);
  centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
  mainTabWidget_ = new QTabWidget(centralwidget);
  mainTabWidget_->setObjectName(QString::fromUtf8("mainTabWidget_"));
  mainTabWidget_->setGeometry(QRect(30, 20, 1061, 851));
  mainTabWidget_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  online_tab = new QWidget();
  online_tab->setObjectName(QString::fromUtf8("online_tab"));
  rawimageGroupBox_ = new QGroupBox(online_tab);
  rawimageGroupBox_->setObjectName(QString::fromUtf8("rawimageGroupBox_"));
  rawimageGroupBox_->setGeometry(QRect(10, 10, 671, 791));
  QFont font1;
  font1.setPointSize(12);
  rawimageGroupBox_->setFont(font1);
  rawimageGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  rawimageGroupBox_->setAlignment(Qt::AlignCenter);
  rawimageLabel_ = new DefoImageLabel( rawimageGroupBox_ );
  rawimageLabel_->setObjectName(QString::fromUtf8("rawimageLabel_"));
  rawimageLabel_->setGeometry(QRect(20, 50, 525, 700));
  rawimageLabel_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
//   rawimageWidget_ = new QWidget(rawimageGroupBox_);
//   rawimageWidget_->setObjectName(QString::fromUtf8("rawimageWidget_"));
//   rawimageWidget_->setGeometry(QRect(20, 50, 525, 700));
//   rawimageWidget_->setAutoFillBackground(false);
//   rawimageWidget_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
//   rawimageZoomButton_ = new QToolButton(rawimageGroupBox_);
//   rawimageZoomButton_->setObjectName(QString::fromUtf8("rawimageZoomButton_"));
//   rawimageZoomButton_->setGeometry(QRect(560, 520, 91, 41));
//   rawimageZoomButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  QIcon icon;
  icon.addPixmap(QPixmap(QString::fromUtf8("icons/48px-Icon-Lupe.png")), QIcon::Normal, QIcon::Off);
//   rawimageZoomButton_->setIcon(icon);
//   rawimageZoomButton_->setPopupMode(QToolButton::DelayedPopup);
//   rawimageZoomButton_->setToolButtonStyle(Qt::ToolButtonIconOnly);
//   rawimageZoomButton_->setArrowType(Qt::NoArrow);
  areaGroupBox_ = new QGroupBox(rawimageGroupBox_);
  areaGroupBox_->setObjectName(QString::fromUtf8("areaGroupBox_"));
  areaGroupBox_->setGeometry(QRect(560, 50, 91, 160));
  areaGroupBox_->setFont(font);
  areaGroupBox_->setAlignment(Qt::AlignCenter);
  areaNewButton_ = new QPushButton(areaGroupBox_);
  areaNewButton_->setObjectName(QString::fromUtf8("areaNewButton_"));
  areaNewButton_->setEnabled(true);
  areaNewButton_->setGeometry(QRect(20, 90, 51, 26));
  areaNewButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  areaNewButton_->setFlat(false);
  areaDeleteButton_ = new QPushButton(areaGroupBox_);
  areaDeleteButton_->setObjectName(QString::fromUtf8("areaDeleteButton_"));
  areaDeleteButton_->setEnabled(false);
  areaDeleteButton_->setGeometry(QRect(20, 120, 51, 26));
  areaDeleteButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
//   areaDoneButton_ = new QPushButton(areaGroupBox_);
//   areaDoneButton_->setObjectName(QString::fromUtf8("areaDoneButton_"));
//   areaDoneButton_->setEnabled(false);
//   areaDoneButton_->setGeometry(QRect(20, 180, 51, 26));
  QFont font2;
  font2.setBold(true);
  font2.setWeight(75);
//   areaDoneButton_->setFont(font2);
//   areaDoneButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 255, 0);"));
//   areaCancelButton_ = new QPushButton(areaGroupBox_);
//   areaCancelButton_->setObjectName(QString::fromUtf8("areaCancelButton_"));
//   areaCancelButton_->setEnabled(false);
//   areaCancelButton_->setGeometry(QRect(20, 120, 51, 26));
//   areaCancelButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  areaComboBox_ = new QComboBox(areaGroupBox_);
  areaComboBox_->setObjectName(QString::fromUtf8("areaComboBox_"));
  areaComboBox_->setEnabled(false);
  areaComboBox_->setGeometry(QRect(20, 50, 51, 26));
  areaComboBox_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  areaLabel_ = new QLabel(areaGroupBox_);
  areaLabel_->setObjectName(QString::fromUtf8("areaLabel_"));
  areaLabel_->setGeometry(QRect(20, 30, 51, 20));
  areaLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  areaLabel_->setAlignment(Qt::AlignCenter);
//   rawimageSaveButton_ = new QPushButton(rawimageGroupBox_);
//   rawimageSaveButton_->setObjectName(QString::fromUtf8("rawimageSaveButton_"));
//   rawimageSaveButton_->setGeometry(QRect(560, 450, 91, 41));
//   rawimageSaveButton_->setFont(font1);
//   rawimageSaveButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  displayGroupBox_ = new QGroupBox(rawimageGroupBox_);
  displayGroupBox_->setObjectName(QString::fromUtf8("displayGroupBox_"));
  displayGroupBox_->setGeometry(QRect(560, 230, 91, 91));
  displayGroupBox_->setFont(font);
  displayGroupBox_->setAlignment(Qt::AlignCenter);
  displayGroupBox_->setCheckable(false);
  displayGroupBox_->setChecked(false);
  displayAreasButton_ = new QRadioButton(displayGroupBox_);
  displayAreasButton_->setObjectName(QString::fromUtf8("displayAreasButton_"));
  displayAreasButton_->setEnabled(false);
  displayAreasButton_->setGeometry(QRect(10, 40, 51, 22));
  displayAreasButton_->setAutoExclusive( false ); // all may be ticked independently
  displaySplinesButton_ = new QRadioButton(displayGroupBox_);
  displaySplinesButton_->setObjectName(QString::fromUtf8("displaySplinesButton_"));
  displaySplinesButton_->setEnabled(false);
  displaySplinesButton_->setGeometry(QRect(10, 60, 61, 22));
  displaySplinesButton_->setAutoExclusive( false ); // all may be ticked independently
  displayRecoitemButton_ = new QRadioButton(displayGroupBox_);
  displayRecoitemButton_->setObjectName(QString::fromUtf8("displayRecoitemButton_"));
  displayRecoitemButton_->setGeometry(QRect(10, 20, 71, 22));
  displayRecoitemButton_->setAutoExclusive( false ); // all may be ticked independently

  scheduleGroupBox_ = new QGroupBox(online_tab);
  scheduleGroupBox_->setObjectName(QString::fromUtf8("scheduleGroupBox_"));
  scheduleGroupBox_->setGeometry(QRect(690, 340, 351, 461));
  scheduleGroupBox_->setFont(font1);
  scheduleGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  scheduleGroupBox_->setAlignment(Qt::AlignCenter);
  scheduleClearButton_ = new QPushButton(scheduleGroupBox_);
  scheduleClearButton_->setObjectName(QString::fromUtf8("scheduleClearButton_"));
  scheduleClearButton_->setGeometry(QRect(100, 360, 71, 31));
  QFont font3;
  font3.setPointSize(9);
  scheduleClearButton_->setFont(font3);
  scheduleClearButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  scheduleVerifyButton_ = new QPushButton(scheduleGroupBox_);
  scheduleVerifyButton_->setObjectName(QString::fromUtf8("scheduleVerifyButton_"));
  scheduleVerifyButton_->setGeometry(QRect(20, 360, 71, 31));
  scheduleVerifyButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  scheduleVerifyButton_->setEnabled( false );

  scheduleLoadButton_ = new QPushButton(scheduleGroupBox_);
  scheduleLoadButton_->setObjectName(QString::fromUtf8("scheduleLoadButton_"));
  scheduleLoadButton_->setGeometry(QRect(180, 360, 71, 31));
  scheduleLoadButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));

  scheduleSaveButton_ = new QPushButton(scheduleGroupBox_);
  scheduleSaveButton_->setObjectName(QString::fromUtf8("scheduleSaveButton_"));
  scheduleSaveButton_->setGeometry(QRect(260, 360, 71, 31));
  scheduleSaveButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));

  scheduleTableview_ = new QTableView( scheduleGroupBox_ );
  schedule_ = new DefoSchedule;
  scheduleTableview_->setObjectName(QString::fromUtf8("scheduleTableview_"));
  scheduleTableview_->setGeometry(QRect(10, 30, 331, 311));
  scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 200,200,200 ); "));
  scheduleTableview_->setModel( schedule_->getModel() );
  scheduleTableview_->setShowGrid( true );
  for( unsigned int i = 0; i < 20; ++i ) scheduleTableview_->setRowHeight( i, 20 );
  scheduleTableview_->setColumnWidth( 0, 80 );
  scheduleTableview_->setColumnWidth( 1, 180 );
  scheduleTableview_->setColumnWidth( 2, 30 );
  QFont fontMonospace;
  fontMonospace.setFamily(QString::fromUtf8("Monospace"));
  scheduleTableview_->setFont(fontMonospace);


  scheduleStopButton_ = new QPushButton(scheduleGroupBox_);
  scheduleStopButton_->setObjectName(QString::fromUtf8("scheduleStopButton_"));
  scheduleStopButton_->setGeometry(QRect(240, 400, 91, 41));
  scheduleStopButton_->setFont(font2);
  scheduleStopButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 0, 0);"));
  scheduleStartButton_ = new QPushButton(scheduleGroupBox_);
  scheduleStartButton_->setObjectName(QString::fromUtf8("scheduleStartButton_"));
  scheduleStartButton_->setGeometry(QRect(20, 400, 91, 41));
  scheduleStartButton_->setFont(font2);
  scheduleStartButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 255, 0);"));
  schedulePauseButton_ = new QPushButton(scheduleGroupBox_);
  schedulePauseButton_->setObjectName(QString::fromUtf8("schedulePauseButton_"));
  schedulePauseButton_->setGeometry(QRect(130, 400, 91, 41));
  schedulePauseButton_->setFont(font2);
  schedulePauseButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 127);"));
  measurementidGroupBox_ = new QGroupBox(online_tab);
  measurementidGroupBox_->setObjectName(QString::fromUtf8("measurementidGroupBox_"));
  measurementidGroupBox_->setGeometry(QRect(690, 10, 351, 121));
  QFont font4;
  font4.setPointSize(11);
  measurementidGroupBox_->setFont(font4);
  measurementidGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  measurementidGroupBox_->setAlignment(Qt::AlignCenter);
  measurementidEditButton_ = new QPushButton(measurementidGroupBox_);
  measurementidEditButton_->setObjectName(QString::fromUtf8("measurementidEditButton_"));
  measurementidEditButton_->setGeometry(QRect(10, 80, 61, 31));
  measurementidEditButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  measurementidDefaultButton_ = new QPushButton(measurementidGroupBox_);
  measurementidDefaultButton_->setObjectName(QString::fromUtf8("measurementidDefaultButton_"));
  measurementidDefaultButton_->setGeometry(QRect(80, 80, 61, 31));
  measurementidDefaultButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  measurementidTextedit_ = new QPlainTextEdit(measurementidGroupBox_);
  measurementidTextedit_->setObjectName(QString::fromUtf8("plainTextEdit_4"));
  measurementidTextedit_->setGeometry(QRect(10, 20, 331, 51));
  measurementidTextedit_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  measurementidTextedit_->setReadOnly(true);
  QFont font5;
  font5.setFamily(QString::fromUtf8("Monospace"));
  font5.setPointSize(10);
  measurementidTextedit_->setFont(font5);

  imageinfoGroupBox_ = new QGroupBox(online_tab);
  imageinfoGroupBox_->setObjectName(QString::fromUtf8("imageinfoGroupBox_"));
  imageinfoGroupBox_->setGeometry(QRect(690, 140, 351, 191));
  imageinfoGroupBox_->setFont(font4);
  imageinfoGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  imageinfoGroupBox_->setAlignment(Qt::AlignCenter);
  imageinfoTextedit_ = new QPlainTextEdit(imageinfoGroupBox_);
  imageinfoTextedit_->setObjectName(QString::fromUtf8("imageinfoTextedit_"));
  imageinfoTextedit_->setGeometry(QRect(10, 30, 331, 151));
  QFont font6;
  font6.setFamily(QString::fromUtf8("Monospace"));
  imageinfoTextedit_->setFont(font6);
  imageinfoTextedit_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  imageinfoTextedit_->setReadOnly(true);
  mainTabWidget_->addTab(online_tab, QString());
  tab_2 = new QWidget();
  tab_2->setObjectName(QString::fromUtf8("tab_2"));
  historyGroupBox_ = new QGroupBox(tab_2);
  historyGroupBox_->setObjectName(QString::fromUtf8("historyGroupBox_"));
  historyGroupBox_->setGeometry(QRect(20, 380, 1021, 421));
  historyGroupBox_->setFont(font1);
  historyGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  historyGroupBox_->setAlignment(Qt::AlignCenter);
  historyQwtPlot_ = new QwtPlot(historyGroupBox_);
  historyQwtPlot_->setObjectName(QString::fromUtf8("historyQwtPlot_"));
  historyQwtPlot_->setGeometry(QRect(10, 30, 991, 321));
  historyQwtPlot_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  historyZoomButton_ = new QToolButton(historyGroupBox_);
  historyZoomButton_->setObjectName(QString::fromUtf8("historyZoomButton_"));
  historyZoomButton_->setGeometry(QRect(680, 370, 61, 41));
  historyZoomButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  historyZoomButton_->setIcon(icon);
  historyDisplayGroupBox_ = new QGroupBox(historyGroupBox_);
  historyDisplayGroupBox_->setObjectName(QString::fromUtf8("historyDisplayGroupBox_"));
  historyDisplayGroupBox_->setGeometry(QRect(100, 360, 531, 51));
  historyDisplayGroupBox_->setAlignment(Qt::AlignCenter);

  historyDisplay0Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay0Button_->setObjectName(QString::fromUtf8("historyDisplay0Button_"));
  historyDisplay0Button_->setGeometry(QRect(10, 20, 31, 22));
  historyDisplay0Button_->setFont(font2);
  historyDisplay0Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay0Button_->setAutoExclusive( false );

  historyDisplay1Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay1Button_->setObjectName(QString::fromUtf8("historyDisplay1Button_"));
  historyDisplay1Button_->setGeometry(QRect(50, 20, 31, 22));
  historyDisplay1Button_->setFont(font2);
  historyDisplay1Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay1Button_->setAutoExclusive( false );

  historyDisplay2Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay2Button_->setObjectName(QString::fromUtf8("historyDisplay2Button_"));
  historyDisplay2Button_->setGeometry(QRect(90, 20, 31, 22));
  historyDisplay2Button_->setFont(font2);
  historyDisplay2Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay2Button_->setAutoExclusive( false );

  historyDisplay3Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay3Button_->setObjectName(QString::fromUtf8("historyDisplay3Button_"));
  historyDisplay3Button_->setGeometry(QRect(130, 20, 31, 22));
  historyDisplay3Button_->setFont(font2);
  historyDisplay3Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay3Button_->setAutoExclusive( false );

  historyDisplay4Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay4Button_->setObjectName(QString::fromUtf8("historyDisplay4Button_"));
  historyDisplay4Button_->setGeometry(QRect(170, 20, 31, 22));
  historyDisplay4Button_->setFont(font2);
  historyDisplay4Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay4Button_->setAutoExclusive( false );

  historyDisplay5Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay5Button_->setObjectName(QString::fromUtf8("historyDisplay5Button_"));
  historyDisplay5Button_->setGeometry(QRect(210, 20, 31, 22));
  historyDisplay5Button_->setFont(font2);
  historyDisplay5Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay5Button_->setAutoExclusive( false );

  historyDisplay6Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay6Button_->setObjectName(QString::fromUtf8("historyDisplay6Button_"));
  historyDisplay6Button_->setGeometry(QRect(250, 20, 31, 22));
  historyDisplay6Button_->setFont(font2);
  historyDisplay6Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay6Button_->setAutoExclusive( false );

  historyDisplay7Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay7Button_->setObjectName(QString::fromUtf8("historyDisplay7Button_"));
  historyDisplay7Button_->setGeometry(QRect(290, 20, 31, 22));
  historyDisplay7Button_->setFont(font2);
  historyDisplay7Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay7Button_->setAutoExclusive( false );

  historyDisplay8Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay8Button_->setObjectName(QString::fromUtf8("historyDisplay8Button_"));
  historyDisplay8Button_->setGeometry(QRect(330, 20, 31, 22));
  historyDisplay8Button_->setFont(font2);
  historyDisplay8Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay8Button_->setAutoExclusive( false );

  historyDisplay9Button_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplay9Button_->setObjectName(QString::fromUtf8("historyDisplay9Button_"));
  historyDisplay9Button_->setGeometry(QRect(370, 20, 31, 22));
  historyDisplay9Button_->setFont(font2);
  historyDisplay9Button_->setLayoutDirection(Qt::RightToLeft);
  historyDisplay9Button_->setAutoExclusive( false );

  historyDisplayRefButton_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplayRefButton_->setObjectName(QString::fromUtf8("historyDisplayRefButton_"));
  historyDisplayRefButton_->setGeometry(QRect(410, 20, 41, 22));
  historyDisplayRefButton_->setFont(font2);
  historyDisplayRefButton_->setLayoutDirection(Qt::RightToLeft);
  historyDisplayRefButton_->setAutoExclusive( false );

  historyDisplayBathButton_ = new QRadioButton(historyDisplayGroupBox_);
  historyDisplayBathButton_->setObjectName(QString::fromUtf8("historyDisplayBathButton_"));
  historyDisplayBathButton_->setGeometry(QRect(450, 20, 61, 22));
  historyDisplayBathButton_->setFont(font2);
  historyDisplayBathButton_->setLayoutDirection(Qt::RightToLeft);
  historyDisplayBathButton_->setAutoExclusive( false );

  historyClearButton_ = new QPushButton(historyGroupBox_);
  historyClearButton_->setObjectName(QString::fromUtf8("historyClearButton_"));
  historyClearButton_->setGeometry(QRect(760, 370, 61, 41));
  historyClearButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  historySaveButton_ = new QPushButton(historyGroupBox_);
  historySaveButton_->setObjectName(QString::fromUtf8("historySaveButton_"));
  historySaveButton_->setGeometry(QRect(840, 370, 61, 41));
  historySaveButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  chillerGroupBox_ = new QGroupBox(tab_2);
  chillerGroupBox_->setObjectName(QString::fromUtf8("chillerGroupBox_"));
  chillerGroupBox_->setGeometry(QRect(20, 20, 181, 341));
  chillerGroupBox_->setFont(font1);
  chillerGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerGroupBox_->setAlignment(Qt::AlignCenter);
  chillerTargetLcd_ = new QLCDNumber(chillerGroupBox_);
  chillerTargetLcd_->setObjectName(QString::fromUtf8("chillerTargetLcd_"));
  chillerTargetLcd_->setGeometry(QRect(30, 170, 121, 41));
  chillerTargetLcd_->setLayoutDirection(Qt::RightToLeft);
  chillerTargetLcd_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  chillerTargetLcd_->setFrameShadow(QFrame::Raised);
  chillerTargetLcd_->setSmallDecimalPoint(true);
  chillerTargetLcd_->setMode(QLCDNumber::Dec);
  chillerTargetLcd_->setSegmentStyle(QLCDNumber::Flat);
  chillerTargetLcd_->setProperty("value", QVariant(0));
  chillerTargetLcd_->setProperty("intValue", QVariant(0));
  chillerTargetLabel_ = new QLabel(chillerGroupBox_);
  chillerTargetLabel_->setObjectName(QString::fromUtf8("chillerTargetLabel_"));
  chillerTargetLabel_->setGeometry(QRect(50, 150, 71, 20));
  chillerTargetLabel_->setFont(font1);
  chillerTargetLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerTargetLabel_->setAlignment(Qt::AlignCenter);
  chillerPowerLcd_ = new QLCDNumber(chillerGroupBox_);
  chillerPowerLcd_->setObjectName(QString::fromUtf8("chillerPowerLcd_"));
  chillerPowerLcd_->setGeometry(QRect(30, 250, 121, 41));
  chillerPowerLcd_->setLayoutDirection(Qt::RightToLeft);
  chillerPowerLcd_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  chillerPowerLcd_->setFrameShadow(QFrame::Raised);
  chillerPowerLcd_->setSmallDecimalPoint(true);
  chillerPowerLcd_->setMode(QLCDNumber::Dec);
  chillerPowerLcd_->setSegmentStyle(QLCDNumber::Flat);
  chillerPowerLcd_->setProperty("value", QVariant(0));
  chillerPowerLcd_->setProperty("intValue", QVariant(0));
  chillerPowerLabel_ = new QLabel(chillerGroupBox_);
  chillerPowerLabel_->setObjectName(QString::fromUtf8("chillerPowerLabel_"));
  chillerPowerLabel_->setGeometry(QRect(50, 230, 71, 20));
  chillerPowerLabel_->setFont(font1);
  chillerPowerLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerPowerLabel_->setAlignment(Qt::AlignCenter);
  chillerBathLcd_ = new QLCDNumber(chillerGroupBox_);
  chillerBathLcd_->setObjectName(QString::fromUtf8("chillerBathLcd_"));
  chillerBathLcd_->setGeometry(QRect(30, 50, 121, 41));
  chillerBathLcd_->setLayoutDirection(Qt::RightToLeft);
  chillerBathLcd_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  chillerBathLcd_->setFrameShadow(QFrame::Raised);
  chillerBathLcd_->setSmallDecimalPoint(true);
  chillerBathLcd_->setMode(QLCDNumber::Dec);
  chillerBathLcd_->setSegmentStyle(QLCDNumber::Flat);
  chillerBathLcd_->setProperty("value", QVariant(0));
  chillerBathLcd_->setProperty("intValue", QVariant(0));
  chillerBathLabel_ = new QLabel(chillerGroupBox_);
  chillerBathLabel_->setObjectName(QString::fromUtf8("chillerBathLabel_"));
  chillerBathLabel_->setGeometry(QRect(50, 30, 71, 20));
  chillerBathLabel_->setFont(font1);
  chillerBathLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerBathLabel_->setAlignment(Qt::AlignCenter);
  chillerBathMaxLabel_ = new QLabel(chillerGroupBox_);
  chillerBathMaxLabel_->setObjectName(QString::fromUtf8("chillerBathMaxLabel_"));
  chillerBathMaxLabel_->setGeometry(QRect(100, 120, 41, 20));
  chillerBathMaxLabel_->setFont(font3);
  chillerBathMaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerBathMaxLabel_->setAlignment(Qt::AlignCenter);
  chillerBathMaxLcd_ = new QLCDNumber(chillerGroupBox_);
  chillerBathMaxLcd_->setObjectName(QString::fromUtf8("chillerBathMaxLcd_"));
  chillerBathMaxLcd_->setGeometry(QRect(90, 100, 61, 21));
  chillerBathMaxLcd_->setLayoutDirection(Qt::RightToLeft);
  chillerBathMaxLcd_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  chillerBathMaxLcd_->setFrameShadow(QFrame::Raised);
  chillerBathMaxLcd_->setSmallDecimalPoint(true);
  chillerBathMaxLcd_->setMode(QLCDNumber::Dec);
  chillerBathMaxLcd_->setSegmentStyle(QLCDNumber::Flat);
  chillerBathMaxLcd_->setProperty("value", QVariant(0));
  chillerBathMaxLcd_->setProperty("intValue", QVariant(0));
  chillerBathMinLcd_ = new QLCDNumber(chillerGroupBox_);
  chillerBathMinLcd_->setObjectName(QString::fromUtf8("chillerBathMinLcd_"));
  chillerBathMinLcd_->setGeometry(QRect(30, 100, 61, 21));
  chillerBathMinLcd_->setLayoutDirection(Qt::RightToLeft);
  chillerBathMinLcd_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  chillerBathMinLcd_->setFrameShadow(QFrame::Raised);
  chillerBathMinLcd_->setSmallDecimalPoint(true);
  chillerBathMinLcd_->setMode(QLCDNumber::Dec);
  chillerBathMinLcd_->setSegmentStyle(QLCDNumber::Flat);
  chillerBathMinLcd_->setProperty("value", QVariant(0));
  chillerBathMinLcd_->setProperty("intValue", QVariant(0));
  chillerBathMinLabel_ = new QLabel(chillerGroupBox_);
  chillerBathMinLabel_->setObjectName(QString::fromUtf8("chillerBathMinLabel_"));
  chillerBathMinLabel_->setGeometry(QRect(40, 120, 41, 20));
  chillerBathMinLabel_->setFont(font3);
  chillerBathMinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerBathMinLabel_->setAlignment(Qt::AlignCenter);
  chillerClearButton_ = new QPushButton(chillerGroupBox_);
  chillerClearButton_->setObjectName(QString::fromUtf8("chillerClearButton_"));
  chillerClearButton_->setGeometry(QRect(30, 300, 111, 31));
  chillerClearButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  sensorsGroupBox_ = new QGroupBox(tab_2);
  sensorsGroupBox_->setObjectName(QString::fromUtf8("sensorsGroupBox_"));
  sensorsGroupBox_->setGeometry(QRect(210, 20, 831, 341));
  sensorsGroupBox_->setFont(font1);
  sensorsGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsGroupBox_->setAlignment(Qt::AlignCenter);
  sensorsLcd5_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd5_->setObjectName(QString::fromUtf8("sensorsLcd5_"));
  sensorsLcd5_->setGeometry(QRect(50, 190, 121, 41));
  sensorsLcd5_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd5_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd5_->setFrameShadow(QFrame::Raised);
  sensorsLcd5_->setSmallDecimalPoint(true);
  sensorsLcd5_->setMode(QLCDNumber::Dec);
  sensorsLcd5_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd5_->setProperty("value", QVariant(0));
  sensorsLcd5_->setProperty("intValue", QVariant(0));
  sensorsLcd8_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd8_->setObjectName(QString::fromUtf8("sensorsLcd8_"));
  sensorsLcd8_->setGeometry(QRect(500, 190, 121, 41));
  sensorsLcd8_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd8_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd8_->setFrameShadow(QFrame::Raised);
  sensorsLcd8_->setSmallDecimalPoint(true);
  sensorsLcd8_->setMode(QLCDNumber::Dec);
  sensorsLcd8_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd8_->setProperty("value", QVariant(0));
  sensorsLcd8_->setProperty("intValue", QVariant(0));
  sensorsLcd4_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd4_->setObjectName(QString::fromUtf8("sensorsLcd4_"));
  sensorsLcd4_->setGeometry(QRect(650, 60, 121, 41));
  sensorsLcd4_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd4_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd4_->setFrameShadow(QFrame::Raised);
  sensorsLcd4_->setSmallDecimalPoint(true);
  sensorsLcd4_->setMode(QLCDNumber::Dec);
  sensorsLcd4_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd4_->setProperty("value", QVariant(0));
  sensorsLcd4_->setProperty("intValue", QVariant(0));
  sensorsLcd1_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd1_->setObjectName(QString::fromUtf8("sensorsLcd1_"));
  sensorsLcd1_->setGeometry(QRect(200, 60, 121, 41));
  sensorsLcd1_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd1_->setFrameShadow(QFrame::Raised);
  sensorsLcd1_->setSmallDecimalPoint(true);
  sensorsLcd1_->setMode(QLCDNumber::Dec);
  sensorsLcd1_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd1_->setProperty("value", QVariant(0));
  sensorsLcd1_->setProperty("intValue", QVariant(0));
  sensorsLcd3_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd3_->setObjectName(QString::fromUtf8("sensorsLcd3_"));
  sensorsLcd3_->setGeometry(QRect(500, 60, 121, 41));
  sensorsLcd3_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd3_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd3_->setFrameShadow(QFrame::Raised);
  sensorsLcd3_->setSmallDecimalPoint(true);
  sensorsLcd3_->setMode(QLCDNumber::Dec);
  sensorsLcd3_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd3_->setProperty("value", QVariant(0));
  sensorsLcd3_->setProperty("intValue", QVariant(0));
  sensorsLcd6_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd6_->setObjectName(QString::fromUtf8("sensorsLcd6_"));
  sensorsLcd6_->setGeometry(QRect(200, 190, 121, 41));
  sensorsLcd6_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd6_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd6_->setFrameShadow(QFrame::Raised);
  sensorsLcd6_->setSmallDecimalPoint(true);
  sensorsLcd6_->setMode(QLCDNumber::Dec);
  sensorsLcd6_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd6_->setProperty("value", QVariant(0));
  sensorsLcd6_->setProperty("intValue", QVariant(0));
  sensorsLcd7_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd7_->setObjectName(QString::fromUtf8("sensorsLcd7_"));
  sensorsLcd7_->setGeometry(QRect(350, 190, 121, 41));
  sensorsLcd7_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd7_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd7_->setFrameShadow(QFrame::Raised);
  sensorsLcd7_->setSmallDecimalPoint(true);
  sensorsLcd7_->setMode(QLCDNumber::Dec);
  sensorsLcd7_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd7_->setProperty("value", QVariant(0));
  sensorsLcd7_->setProperty("intValue", QVariant(0));
  sensorsLcd0_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd0_->setObjectName(QString::fromUtf8("sensorsLcd0_"));
  sensorsLcd0_->setGeometry(QRect(50, 60, 121, 41));
  sensorsLcd0_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd0_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd0_->setFrameShadow(QFrame::Raised);
  sensorsLcd0_->setSmallDecimalPoint(true);
  sensorsLcd0_->setMode(QLCDNumber::Dec);
  sensorsLcd0_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd0_->setProperty("value", QVariant(0));
  sensorsLcd0_->setProperty("intValue", QVariant(0));
  sensorsLcd2_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd2_->setObjectName(QString::fromUtf8("sensorsLcd2_"));
  sensorsLcd2_->setGeometry(QRect(350, 60, 121, 41));
  sensorsLcd2_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd2_->setFrameShadow(QFrame::Raised);
  sensorsLcd2_->setSmallDecimalPoint(true);
  sensorsLcd2_->setMode(QLCDNumber::Dec);
  sensorsLcd2_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd2_->setProperty("value", QVariant(0));
  sensorsLcd2_->setProperty("intValue", QVariant(0));
  sensorsLcd9_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcd9_->setObjectName(QString::fromUtf8("sensorsLcd9_"));
  sensorsLcd9_->setGeometry(QRect(650, 190, 121, 41));
  sensorsLcd9_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcd9_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcd9_->setFrameShadow(QFrame::Raised);
  sensorsLcd9_->setSmallDecimalPoint(true);
  sensorsLcd9_->setMode(QLCDNumber::Dec);
  sensorsLcd9_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcd9_->setProperty("value", QVariant(0));
  sensorsLcd9_->setProperty("intValue", QVariant(0));
  sensorsSensor0Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor0Label_->setObjectName(QString::fromUtf8("sensorsSensor0Label_"));
  sensorsSensor0Label_->setGeometry(QRect(60, 40, 101, 20));
  sensorsSensor0Label_->setFont(font1);
  sensorsSensor0Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor0Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor1Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor1Label_->setObjectName(QString::fromUtf8("sensorsSensor1Label_"));
  sensorsSensor1Label_->setGeometry(QRect(210, 40, 101, 20));
  sensorsSensor1Label_->setFont(font1);
  sensorsSensor1Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor1Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor2Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor2Label_->setObjectName(QString::fromUtf8("sensorsSensor2Label_"));
  sensorsSensor2Label_->setGeometry(QRect(360, 40, 101, 20));
  sensorsSensor2Label_->setFont(font1);
  sensorsSensor2Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor2Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor3Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor3Label_->setObjectName(QString::fromUtf8("sensorsSensor3Label_"));
  sensorsSensor3Label_->setGeometry(QRect(510, 40, 101, 20));
  sensorsSensor3Label_->setFont(font1);
  sensorsSensor3Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor3Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor4Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor4Label_->setObjectName(QString::fromUtf8("sensorsSensor4Label_"));
  sensorsSensor4Label_->setGeometry(QRect(660, 40, 101, 20));
  sensorsSensor4Label_->setFont(font1);
  sensorsSensor4Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor4Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor6Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor6Label_->setObjectName(QString::fromUtf8("sensorsSensor6Label_"));
  sensorsSensor6Label_->setGeometry(QRect(210, 170, 101, 20));
  sensorsSensor6Label_->setFont(font1);
  sensorsSensor6Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor6Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor7Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor7Label_->setObjectName(QString::fromUtf8("sensorsSensor7Label_"));
  sensorsSensor7Label_->setGeometry(QRect(360, 170, 101, 20));
  sensorsSensor7Label_->setFont(font1);
  sensorsSensor7Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor7Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor5Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor5Label_->setObjectName(QString::fromUtf8("sensorsSensor5Label_"));
  sensorsSensor5Label_->setGeometry(QRect(60, 170, 101, 20));
  sensorsSensor5Label_->setFont(font1);
  sensorsSensor5Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor5Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor9Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor9Label_->setObjectName(QString::fromUtf8("sensorsSensor9Label_"));
  sensorsSensor9Label_->setGeometry(QRect(660, 170, 101, 20));
  sensorsSensor9Label_->setFont(font1);
  sensorsSensor9Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor9Label_->setAlignment(Qt::AlignCenter);
  sensorsSensor8Label_ = new QLabel(sensorsGroupBox_);
  sensorsSensor8Label_->setObjectName(QString::fromUtf8("sensorsSensor8Label_"));
  sensorsSensor8Label_->setGeometry(QRect(510, 170, 101, 20));
  sensorsSensor8Label_->setFont(font1);
  sensorsSensor8Label_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor8Label_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin5_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin5_->setObjectName(QString::fromUtf8("sensorsLcdMin5_"));
  sensorsLcdMin5_->setGeometry(QRect(50, 240, 61, 21));
  sensorsLcdMin5_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin5_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin5_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin5_->setSmallDecimalPoint(true);
  sensorsLcdMin5_->setMode(QLCDNumber::Dec);
  sensorsLcdMin5_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin5_->setProperty("value", QVariant(0));
  sensorsLcdMin5_->setProperty("intValue", QVariant(0));
  sensorsLcdMax5_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax5_->setObjectName(QString::fromUtf8("sensorsLcdMax5_"));
  sensorsLcdMax5_->setGeometry(QRect(110, 240, 61, 21));
  sensorsLcdMax5_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax5_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax5_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax5_->setSmallDecimalPoint(true);
  sensorsLcdMax5_->setMode(QLCDNumber::Dec);
  sensorsLcdMax5_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax5_->setProperty("value", QVariant(0));
  sensorsLcdMax5_->setProperty("intValue", QVariant(0));
  sensorsSensor5MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor5MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor5MinLabel_"));
  sensorsSensor5MinLabel_->setGeometry(QRect(60, 260, 41, 20));
  sensorsSensor5MinLabel_->setFont(font3);
  sensorsSensor5MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor5MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor5MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor5MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor5MaxLabel_"));
  sensorsSensor5MaxLabel_->setGeometry(QRect(120, 260, 41, 20));
  sensorsSensor5MaxLabel_->setFont(font3);
  sensorsSensor5MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor5MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor6MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor6MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor6MinLabel_"));
  sensorsSensor6MinLabel_->setGeometry(QRect(210, 260, 41, 20));
  sensorsSensor6MinLabel_->setFont(font3);
  sensorsSensor6MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor6MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax6_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax6_->setObjectName(QString::fromUtf8("sensorsLcdMax6_"));
  sensorsLcdMax6_->setGeometry(QRect(260, 240, 61, 21));
  sensorsLcdMax6_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax6_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax6_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax6_->setSmallDecimalPoint(true);
  sensorsLcdMax6_->setMode(QLCDNumber::Dec);
  sensorsLcdMax6_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax6_->setProperty("value", QVariant(0));
  sensorsLcdMax6_->setProperty("intValue", QVariant(0));
  sensorsSensor6MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor6MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor6MaxLabel_"));
  sensorsSensor6MaxLabel_->setGeometry(QRect(270, 260, 41, 20));
  sensorsSensor6MaxLabel_->setFont(font3);
  sensorsSensor6MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor6MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin6_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin6_->setObjectName(QString::fromUtf8("sensorsLcdMin6_"));
  sensorsLcdMin6_->setGeometry(QRect(200, 240, 61, 21));
  sensorsLcdMin6_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin6_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin6_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin6_->setSmallDecimalPoint(true);
  sensorsLcdMin6_->setMode(QLCDNumber::Dec);
  sensorsLcdMin6_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin6_->setProperty("value", QVariant(0));
  sensorsLcdMin6_->setProperty("intValue", QVariant(0));
  sensorsLcdMin7_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin7_->setObjectName(QString::fromUtf8("sensorsLcdMin7_"));
  sensorsLcdMin7_->setGeometry(QRect(350, 240, 61, 21));
  sensorsLcdMin7_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin7_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin7_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin7_->setSmallDecimalPoint(true);
  sensorsLcdMin7_->setMode(QLCDNumber::Dec);
  sensorsLcdMin7_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin7_->setProperty("value", QVariant(0));
  sensorsLcdMin7_->setProperty("intValue", QVariant(0));
  sensorsSensor7MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor7MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor7MinLabel_"));
  sensorsSensor7MinLabel_->setGeometry(QRect(360, 260, 41, 20));
  sensorsSensor7MinLabel_->setFont(font3);
  sensorsSensor7MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor7MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax7_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax7_->setObjectName(QString::fromUtf8("sensorsLcdMax7_"));
  sensorsLcdMax7_->setGeometry(QRect(410, 240, 61, 21));
  sensorsLcdMax7_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax7_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax7_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax7_->setSmallDecimalPoint(true);
  sensorsLcdMax7_->setMode(QLCDNumber::Dec);
  sensorsLcdMax7_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax7_->setProperty("value", QVariant(0));
  sensorsLcdMax7_->setProperty("intValue", QVariant(0));
  sensorsSensor7MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor7MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor7MaxLabel_"));
  sensorsSensor7MaxLabel_->setGeometry(QRect(420, 260, 41, 20));
  sensorsSensor7MaxLabel_->setFont(font3);
  sensorsSensor7MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor7MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin8_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin8_->setObjectName(QString::fromUtf8("sensorsLcdMin8_"));
  sensorsLcdMin8_->setGeometry(QRect(500, 240, 61, 21));
  sensorsLcdMin8_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin8_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin8_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin8_->setSmallDecimalPoint(true);
  sensorsLcdMin8_->setMode(QLCDNumber::Dec);
  sensorsLcdMin8_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin8_->setProperty("value", QVariant(0));
  sensorsLcdMin8_->setProperty("intValue", QVariant(0));
  sensorsSensor8MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor8MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor8MinLabel_"));
  sensorsSensor8MinLabel_->setGeometry(QRect(510, 260, 41, 20));
  sensorsSensor8MinLabel_->setFont(font3);
  sensorsSensor8MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor8MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax8_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax8_->setObjectName(QString::fromUtf8("sensorsLcdMax8_"));
  sensorsLcdMax8_->setGeometry(QRect(560, 240, 61, 21));
  sensorsLcdMax8_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax8_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax8_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax8_->setSmallDecimalPoint(true);
  sensorsLcdMax8_->setMode(QLCDNumber::Dec);
  sensorsLcdMax8_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax8_->setProperty("value", QVariant(0));
  sensorsLcdMax8_->setProperty("intValue", QVariant(0));
  sensorsSensor8MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor8MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor8MaxLabel_"));
  sensorsSensor8MaxLabel_->setGeometry(QRect(570, 260, 41, 20));
  sensorsSensor8MaxLabel_->setFont(font3);
  sensorsSensor8MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor8MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin9_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin9_->setObjectName(QString::fromUtf8("sensorsLcdMin9_"));
  sensorsLcdMin9_->setGeometry(QRect(650, 240, 61, 21));
  sensorsLcdMin9_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin9_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin9_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin9_->setSmallDecimalPoint(true);
  sensorsLcdMin9_->setMode(QLCDNumber::Dec);
  sensorsLcdMin9_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin9_->setProperty("value", QVariant(0));
  sensorsLcdMin9_->setProperty("intValue", QVariant(0));
  sensorsSensor9MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor9MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor9MinLabel_"));
  sensorsSensor9MinLabel_->setGeometry(QRect(660, 260, 41, 20));
  sensorsSensor9MinLabel_->setFont(font3);
  sensorsSensor9MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor9MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax9_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax9_->setObjectName(QString::fromUtf8("sensorsLcdMax9_"));
  sensorsLcdMax9_->setGeometry(QRect(710, 240, 61, 21));
  sensorsLcdMax9_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax9_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax9_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax9_->setSmallDecimalPoint(true);
  sensorsLcdMax9_->setMode(QLCDNumber::Dec);
  sensorsLcdMax9_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax9_->setProperty("value", QVariant(0));
  sensorsLcdMax9_->setProperty("intValue", QVariant(0));
  sensorsSensor9MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor9MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor9MaxLabel_"));
  sensorsSensor9MaxLabel_->setGeometry(QRect(720, 260, 41, 20));
  sensorsSensor9MaxLabel_->setFont(font3);
  sensorsSensor9MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor9MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin3_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin3_->setObjectName(QString::fromUtf8("sensorsLcdMin3_"));
  sensorsLcdMin3_->setGeometry(QRect(500, 110, 61, 21));
  sensorsLcdMin3_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin3_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin3_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin3_->setSmallDecimalPoint(true);
  sensorsLcdMin3_->setMode(QLCDNumber::Dec);
  sensorsLcdMin3_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin3_->setProperty("value", QVariant(0));
  sensorsLcdMin3_->setProperty("intValue", QVariant(0));
  sensorsLcdMin2_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin2_->setObjectName(QString::fromUtf8("sensorsLcdMin2_"));
  sensorsLcdMin2_->setGeometry(QRect(350, 110, 61, 21));
  sensorsLcdMin2_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin2_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin2_->setSmallDecimalPoint(true);
  sensorsLcdMin2_->setMode(QLCDNumber::Dec);
  sensorsLcdMin2_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin2_->setProperty("value", QVariant(0));
  sensorsLcdMin2_->setProperty("intValue", QVariant(0));
  sensorsSensor2MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor2MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor2MaxLabel_"));
  sensorsSensor2MaxLabel_->setGeometry(QRect(420, 130, 41, 20));
  sensorsSensor2MaxLabel_->setFont(font3);
  sensorsSensor2MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor2MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin1_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin1_->setObjectName(QString::fromUtf8("sensorsLcdMin1_"));
  sensorsLcdMin1_->setGeometry(QRect(200, 110, 61, 21));
  sensorsLcdMin1_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin1_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin1_->setSmallDecimalPoint(true);
  sensorsLcdMin1_->setMode(QLCDNumber::Dec);
  sensorsLcdMin1_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin1_->setProperty("value", QVariant(0));
  sensorsLcdMin1_->setProperty("intValue", QVariant(0));
  sensorsSensor4MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor4MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor4MinLabel_"));
  sensorsSensor4MinLabel_->setGeometry(QRect(660, 130, 41, 20));
  sensorsSensor4MinLabel_->setFont(font3);
  sensorsSensor4MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor4MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor0MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor0MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor0MaxLabel_"));
  sensorsSensor0MaxLabel_->setGeometry(QRect(120, 130, 41, 20));
  sensorsSensor0MaxLabel_->setFont(font3);
  sensorsSensor0MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor0MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor1MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor1MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor1MaxLabel_"));
  sensorsSensor1MaxLabel_->setGeometry(QRect(270, 130, 41, 20));
  sensorsSensor1MaxLabel_->setFont(font3);
  sensorsSensor1MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor1MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMin4_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin4_->setObjectName(QString::fromUtf8("sensorsLcdMin4_"));
  sensorsLcdMin4_->setGeometry(QRect(650, 110, 61, 21));
  sensorsLcdMin4_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin4_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin4_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin4_->setSmallDecimalPoint(true);
  sensorsLcdMin4_->setMode(QLCDNumber::Dec);
  sensorsLcdMin4_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin4_->setProperty("value", QVariant(0));
  sensorsLcdMin4_->setProperty("intValue", QVariant(0));
  sensorsSensor3MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor3MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor3MaxLabel_"));
  sensorsSensor3MaxLabel_->setGeometry(QRect(570, 130, 41, 20));
  sensorsSensor3MaxLabel_->setFont(font3);
  sensorsSensor3MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor3MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor2MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor2MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor2MinLabel_"));
  sensorsSensor2MinLabel_->setGeometry(QRect(360, 130, 41, 20));
  sensorsSensor2MinLabel_->setFont(font3);
  sensorsSensor2MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor2MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsSensor0MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor0MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor0MinLabel_"));
  sensorsSensor0MinLabel_->setGeometry(QRect(60, 130, 41, 20));
  sensorsSensor0MinLabel_->setFont(font3);
  sensorsSensor0MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor0MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax4_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax4_->setObjectName(QString::fromUtf8("sensorsLcdMax4_"));
  sensorsLcdMax4_->setGeometry(QRect(710, 110, 61, 21));
  sensorsLcdMax4_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax4_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax4_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax4_->setSmallDecimalPoint(true);
  sensorsLcdMax4_->setMode(QLCDNumber::Dec);
  sensorsLcdMax4_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax4_->setProperty("value", QVariant(0));
  sensorsLcdMax4_->setProperty("intValue", QVariant(0));
  sensorsSensor4MaxLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor4MaxLabel_->setObjectName(QString::fromUtf8("sensorsSensor4MaxLabel_"));
  sensorsSensor4MaxLabel_->setGeometry(QRect(720, 130, 41, 20));
  sensorsSensor4MaxLabel_->setFont(font3);
  sensorsSensor4MaxLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor4MaxLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax2_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax2_->setObjectName(QString::fromUtf8("sensorsLcdMax2_"));
  sensorsLcdMax2_->setGeometry(QRect(410, 110, 61, 21));
  sensorsLcdMax2_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax2_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax2_->setSmallDecimalPoint(true);
  sensorsLcdMax2_->setMode(QLCDNumber::Dec);
  sensorsLcdMax2_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax2_->setProperty("value", QVariant(0));
  sensorsLcdMax2_->setProperty("intValue", QVariant(0));
  sensorsSensor3MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor3MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor3MinLabel_"));
  sensorsSensor3MinLabel_->setGeometry(QRect(510, 130, 41, 20));
  sensorsSensor3MinLabel_->setFont(font3);
  sensorsSensor3MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor3MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax0_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax0_->setObjectName(QString::fromUtf8("sensorsLcdMax0_"));
  sensorsLcdMax0_->setGeometry(QRect(110, 110, 61, 21));
  sensorsLcdMax0_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax0_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax0_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax0_->setSmallDecimalPoint(true);
  sensorsLcdMax0_->setMode(QLCDNumber::Dec);
  sensorsLcdMax0_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax0_->setProperty("value", QVariant(0));
  sensorsLcdMax0_->setProperty("intValue", QVariant(0));
  sensorsLcdMin0_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMin0_->setObjectName(QString::fromUtf8("sensorsLcdMin0_"));
  sensorsLcdMin0_->setGeometry(QRect(50, 110, 61, 21));
  sensorsLcdMin0_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMin0_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMin0_->setFrameShadow(QFrame::Raised);
  sensorsLcdMin0_->setSmallDecimalPoint(true);
  sensorsLcdMin0_->setMode(QLCDNumber::Dec);
  sensorsLcdMin0_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMin0_->setProperty("value", QVariant(0));
  sensorsLcdMin0_->setProperty("intValue", QVariant(0));
  sensorsLcdMax1_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax1_->setObjectName(QString::fromUtf8("sensorsLcdMax1_"));
  sensorsLcdMax1_->setGeometry(QRect(260, 110, 61, 21));
  sensorsLcdMax1_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax1_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax1_->setSmallDecimalPoint(true);
  sensorsLcdMax1_->setMode(QLCDNumber::Dec);
  sensorsLcdMax1_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax1_->setProperty("value", QVariant(0));
  sensorsLcdMax1_->setProperty("intValue", QVariant(0));
  sensorsSensor1MinLabel_ = new QLabel(sensorsGroupBox_);
  sensorsSensor1MinLabel_->setObjectName(QString::fromUtf8("sensorsSensor1MinLabel_"));
  sensorsSensor1MinLabel_->setGeometry(QRect(210, 130, 41, 20));
  sensorsSensor1MinLabel_->setFont(font3);
  sensorsSensor1MinLabel_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  sensorsSensor1MinLabel_->setAlignment(Qt::AlignCenter);
  sensorsLcdMax3_ = new QLCDNumber(sensorsGroupBox_);
  sensorsLcdMax3_->setObjectName(QString::fromUtf8("sensorsLcdMax3_"));
  sensorsLcdMax3_->setGeometry(QRect(560, 110, 61, 21));
  sensorsLcdMax3_->setLayoutDirection(Qt::RightToLeft);
  sensorsLcdMax3_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  sensorsLcdMax3_->setFrameShadow(QFrame::Raised);
  sensorsLcdMax3_->setSmallDecimalPoint(true);
  sensorsLcdMax3_->setMode(QLCDNumber::Dec);
  sensorsLcdMax3_->setSegmentStyle(QLCDNumber::Flat);
  sensorsLcdMax3_->setProperty("value", QVariant(0));
  sensorsLcdMax3_->setProperty("intValue", QVariant(0));
  sensorsClearButton_ = new QPushButton(sensorsGroupBox_);
  sensorsClearButton_->setObjectName(QString::fromUtf8("sensorsClearButton_"));
  sensorsClearButton_->setGeometry(QRect(10, 300, 111, 31));
  sensorsClearButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  mainTabWidget_->addTab(tab_2, QString());
  offline_tab = new QWidget();
  offline_tab->setObjectName(QString::fromUtf8("offline_tab"));
  surfaceGroupBox_ = new QGroupBox(offline_tab);
  surfaceGroupBox_->setObjectName(QString::fromUtf8("surfaceGroupBox_"));
  surfaceGroupBox_->setGeometry(QRect(10, 10, 1031, 801));
  surfaceGroupBox_->setFont(font1);
  surfaceGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  surfaceGroupBox_->setAlignment(Qt::AlignCenter);
  surfacesTabWidget_ = new QTabWidget(surfaceGroupBox_);
  surfacesTabWidget_->setObjectName(QString::fromUtf8("surfacesTabWidget_"));
  surfacesTabWidget_->setGeometry(QRect(20, 20, 831, 761));
  displaytypeGroupbox_ = new QGroupBox(surfaceGroupBox_);
  displaytypeGroupbox_->setObjectName(QString::fromUtf8("displaytypeGroupbox_"));
  displaytypeGroupbox_->setGeometry(QRect(869, 120, 141, 51));
  displaytypeGroupbox_->setFont(font4);
  displaytypeGroupbox_->setAlignment(Qt::AlignCenter);
  displaytype2DButton_ = new QRadioButton(displaytypeGroupbox_);
  displaytype2DButton_->setObjectName(QString::fromUtf8("displaytype2DButton_"));
  displaytype2DButton_->setGeometry(QRect(20, 20, 41, 22));
  displaytype2DButton_->setChecked( false );
  displaytype3DButton_ = new QRadioButton(displaytypeGroupbox_);
  displaytype3DButton_->setObjectName(QString::fromUtf8("displaytype3DButton_"));
  displaytype3DButton_->setGeometry(QRect(80, 20, 41, 22));
  displaytype3DButton_->setChecked( true );
  zscaleGroupbox_ = new QGroupBox(surfaceGroupBox_);
  zscaleGroupbox_->setObjectName(QString::fromUtf8("zscaleGroupbox_"));
  zscaleGroupbox_->setGeometry(QRect(870, 280, 141, 71));
  zscaleGroupbox_->setFont(font4);
  zscaleGroupbox_->setAlignment(Qt::AlignCenter);
  zscalePlusButton_ = new QPushButton(zscaleGroupbox_);
  zscalePlusButton_->setObjectName(QString::fromUtf8("zscalePlusButton_"));
  zscalePlusButton_->setGeometry(QRect(10, 30, 51, 26));
  QFont font13;
  font13.setPointSize(14);
  zscalePlusButton_->setFont(font13);
  zscaleMinusButton_ = new QPushButton(zscaleGroupbox_);
  zscaleMinusButton_->setObjectName(QString::fromUtf8("zscaleMinusButton_"));
  zscaleMinusButton_->setGeometry(QRect(80, 30, 51, 26));
  zscaleMinusButton_->setFont(font13);

  //  surfaceResetButton_ = new QPushButton(tab);
  //  surfaceResetButton_->setObjectName(QString::fromUtf8("surfaceResetButton_"));
  //  surfaceResetButton_->setGeometry(QRect(749, 650, 51, 26));

  displayitemsGroupbox_ = new QGroupBox(surfaceGroupBox_);
  displayitemsGroupbox_->setObjectName(QString::fromUtf8("displayitemsGroupbox_"));
  displayitemsGroupbox_->setGeometry(QRect(869, 180, 141, 91));
  displayitemsGroupbox_->setFont(font4);
  displayitemsGroupbox_->setAlignment(Qt::AlignCenter);
  displayitemsAxesButton_ = new QCheckBox(displayitemsGroupbox_);
  displayitemsAxesButton_->setObjectName(QString::fromUtf8("displayitemsAxesButton_"));
  displayitemsAxesButton_->setGeometry(QRect(10, 20, 61, 22));
  displayitemsAxesButton_->setFont(font);
  displayitemsMeshButton_ = new QCheckBox(displayitemsGroupbox_);
  displayitemsMeshButton_->setObjectName(QString::fromUtf8("displayitemsMeshButton_"));
  displayitemsMeshButton_->setGeometry(QRect(70, 20, 51, 22));
  displayitemsMeshButton_->setFont(font);
  displayitemsShadeButton_ = new QCheckBox(displayitemsGroupbox_);
  displayitemsShadeButton_->setObjectName(QString::fromUtf8("displayitemsShadeButton_"));
  displayitemsShadeButton_->setGeometry(QRect(10, 40, 61, 22));
  displayitemsShadeButton_->setFont(font);
  displayitemsLegendButton_ = new QCheckBox(displayitemsGroupbox_);
  displayitemsLegendButton_->setObjectName(QString::fromUtf8("displayitemsLegendButton_"));
  displayitemsLegendButton_->setGeometry(QRect(70, 40, 71, 22));
  displayitemsLegendButton_->setFont(font);
  displayitemsIsolinesButton_ = new QCheckBox(displayitemsGroupbox_);
  displayitemsIsolinesButton_->setObjectName(QString::fromUtf8("displayitemsIsolinesButton_"));
  displayitemsIsolinesButton_->setGeometry(QRect(10, 60, 71, 22));
  displayitemsIsolinesButton_->setFont(font);
  displayitemsIsolinesSpinbox_ = new QSpinBox(displayitemsGroupbox_);
  displayitemsIsolinesSpinbox_->setObjectName(QString::fromUtf8("displayitemsIsolinesSpinbox_"));
  displayitemsIsolinesSpinbox_->setGeometry(QRect(80, 60, 41, 21));
  displayitemsIsolinesSpinbox_->setMinimum(1);
  displayitemsIsolinesSpinbox_->setMaximum(20);
  displayitemsIsolinesSpinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  displayitemsIsolinesSpinbox_->setValue( 10 );

  tab = new QWidget();
  tab->setObjectName(QString::fromUtf8("tab"));
  surfacePlot_ =  new DefoSurfacePlot( tab );
  surfacePlot_->setObjectName(QString::fromUtf8("surfacePlot_"));
  surfacePlot_->setGeometry(QRect(20, 20, 791, 691));//setGeometry(QRect(20, 20, 751, 650));
  surfacePlot_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
  //  surfacesQwtPlot_ = new QwtPlot(tab);
  //  surfacesQwtPlot_->setObjectName(QString::fromUtf8("surfacesQwtPlot_"));
  //  surfacesQwtPlot_->setGeometry(QRect(20, 20, 751, 650));
  //  surfacesQwtPlot_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
//   surfacesZoomButton_ = new QToolButton(tab);
//   surfacesZoomButton_->setObjectName(QString::fromUtf8("surfacesZoomButton_"));
//   surfacesZoomButton_->setGeometry(QRect(790, 620, 51, 41));
//   surfacesZoomButton_->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234)"));
//   surfacesZoomButton_->setIcon(icon);
//   surfacesZoomButton_->setPopupMode(QToolButton::DelayedPopup);
//   surfacesZoomButton_->setToolButtonStyle(Qt::ToolButtonIconOnly);
//   surfacesZoomButton_->setArrowType(Qt::NoArrow);
  surfacesTabWidget_->addTab(tab, QString());
  
  mainTabWidget_->addTab(offline_tab, QString());
  advanced_tab = new QWidget();
  advanced_tab->setObjectName(QString::fromUtf8("advanced_tab"));
  basefolderGroupBox_ = new QGroupBox(advanced_tab);
  basefolderGroupBox_->setObjectName(QString::fromUtf8("basefolderGroupBox_"));
  basefolderGroupBox_->setGeometry(QRect(40, 30, 581, 91));
  basefolderGroupBox_->setFont(font1);
  basefolderGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  basefolderGroupBox_->setAlignment(Qt::AlignCenter);
  basefolderTextedit_ = new QPlainTextEdit(basefolderGroupBox_);
  basefolderTextedit_->setObjectName(QString::fromUtf8("basefolderTextedit_"));
  basefolderTextedit_->setGeometry(QRect(10, 30, 511, 51));
  basefolderTextedit_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  QFont font7;
  font7.setFamily(QString::fromUtf8("Monospace"));
  font7.setPointSize(11);
  basefolderTextedit_->setFont(font7);
  basefolderTextedit_->setReadOnly(true);
  basefolderEditButton_ = new QPushButton(basefolderGroupBox_);
  basefolderEditButton_->setObjectName(QString::fromUtf8("basefolderEditButton_"));
  basefolderEditButton_->setGeometry(QRect(530, 40, 41, 31));

  chillerparametersGroupBox_ = new QGroupBox(advanced_tab);
  chillerparametersGroupBox_->setObjectName(QString::fromUtf8("chillerparametersGroupBox_"));
  chillerparametersGroupBox_->setGeometry(QRect(830, 360, 201, 241));
  chillerparametersGroupBox_->setFont(font1);
  chillerparametersGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  chillerparametersGroupBox_->setAlignment(Qt::AlignCenter);
  chillerparametersGroupBox_->setFlat(false);
  chillerParametersSpinbox1_ = new QDoubleSpinBox(chillerparametersGroupBox_);
  chillerParametersSpinbox1_->setObjectName(QString::fromUtf8("chillerParametersSpinbox1_"));
  chillerParametersSpinbox1_->setGeometry(QRect(80, 40, 91, 41));
  QFont font8;
  font8.setPointSize(12);
  font8.setBold(false);
  font8.setWeight(50);
  chillerParametersSpinbox1_->setFont(font8);
  chillerParametersSpinbox1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  chillerParametersSpinbox2_ = new QSpinBox(chillerparametersGroupBox_);
  chillerParametersSpinbox2_->setObjectName(QString::fromUtf8("chillerParametersSpinbox2_"));
  chillerParametersSpinbox2_->setGeometry(QRect(80, 100, 91, 41));
  chillerParametersSpinbox2_->setFont(font1);
  chillerParametersSpinbox2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 225)"));
  chillerParametersSpinbox3_ = new QSpinBox(chillerparametersGroupBox_);
  chillerParametersSpinbox3_->setObjectName(QString::fromUtf8("chillerParametersSpinbox3_"));
  chillerParametersSpinbox3_->setGeometry(QRect(80, 160, 91, 41));
  chillerParametersSpinbox3_->setFont(font1);
  chillerParametersSpinbox3_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  chillerParametersLabel1_ = new QLabel(chillerparametersGroupBox_);
  chillerParametersLabel1_->setObjectName(QString::fromUtf8("chillerParametersLabel1_"));
  chillerParametersLabel1_->setGeometry(QRect(10, 50, 61, 16));
  QFont font9;
  font9.setPointSize(10);
  font9.setBold(true);
  font9.setWeight(75);
  chillerParametersLabel1_->setFont(font9);
  chillerParametersLabel1_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  chillerParametersLabel1_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
  chillerParametersLabel2_ = new QLabel(chillerparametersGroupBox_);
  chillerParametersLabel2_->setObjectName(QString::fromUtf8("chillerParametersLabel2_"));
  chillerParametersLabel2_->setGeometry(QRect(10, 110, 61, 16));
  chillerParametersLabel2_->setFont(font9);
  chillerParametersLabel2_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  chillerParametersLabel2_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
  chillerParametersLabel3_ = new QLabel(chillerparametersGroupBox_);
  chillerParametersLabel3_->setObjectName(QString::fromUtf8("chillerParametersLabel3_"));
  chillerParametersLabel3_->setGeometry(QRect(10, 170, 61, 16));
  chillerParametersLabel3_->setFont(font9);
  chillerParametersLabel3_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  chillerParametersLabel3_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
  rawimagerecoGroupBox_ = new QGroupBox(advanced_tab);
  rawimagerecoGroupBox_->setObjectName(QString::fromUtf8("rawimagerecoGroupBox_"));
  rawimagerecoGroupBox_->setGeometry(QRect(40, 360, 371, 241));
  rawimagerecoGroupBox_->setFont(font1);
  rawimagerecoGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  rawimagerecoGroupBox_->setAlignment(Qt::AlignCenter);
  seedingthresholdsGroupBox_ = new QGroupBox(rawimagerecoGroupBox_);
  seedingthresholdsGroupBox_->setObjectName(QString::fromUtf8("seedingthresholdsGroupBox_"));
  seedingthresholdsGroupBox_->setGeometry(QRect(20, 40, 161, 171));
  seedingthresholdsGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  seedingThresholdsStep1Spinbox_ = new QDoubleSpinBox(seedingthresholdsGroupBox_);
  seedingThresholdsStep1Spinbox_->setObjectName(QString::fromUtf8("seedingThresholdsStep1Spinbox_"));
  seedingThresholdsStep1Spinbox_->setGeometry(QRect(60, 40, 81, 31));
  seedingThresholdsStep1Spinbox_->setFont(font8);
  seedingThresholdsStep1Spinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  seedingThresholdsStep1Spinbox_->setDecimals(0);
  seedingThresholdsStep2Spinbox_ = new QDoubleSpinBox(seedingthresholdsGroupBox_);
  seedingThresholdsStep2Spinbox_->setObjectName(QString::fromUtf8("seedingThresholdsStep2Spinbox_"));
  seedingThresholdsStep2Spinbox_->setGeometry(QRect(60, 80, 81, 31));
  seedingThresholdsStep2Spinbox_->setFont(font8);
  seedingThresholdsStep2Spinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  seedingThresholdsStep2Spinbox_->setDecimals(0);
  seedingThresholdsStep3Spinbox_ = new QDoubleSpinBox(seedingthresholdsGroupBox_);
  seedingThresholdsStep3Spinbox_->setObjectName(QString::fromUtf8("seedingThresholdsStep3Spinbox_"));
  seedingThresholdsStep3Spinbox_->setGeometry(QRect(60, 120, 81, 31));
  seedingThresholdsStep3Spinbox_->setFont(font8);
  seedingThresholdsStep3Spinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  seedingThresholdsStep3Spinbox_->setDecimals(0);
  seedingThresholdsStep1Label_ = new QLabel(seedingthresholdsGroupBox_);
  seedingThresholdsStep1Label_->setObjectName(QString::fromUtf8("seedingThresholdsStep1Label_"));
  seedingThresholdsStep1Label_->setGeometry(QRect(10, 40, 41, 31));
  seedingThresholdsStep1Label_->setFont(font2);
  seedingThresholdsStep1Label_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  seedingThresholdsStep1Label_->setAlignment(Qt::AlignCenter);
  seedingThresholdsStep2Label_ = new QLabel(seedingthresholdsGroupBox_);
  seedingThresholdsStep2Label_->setObjectName(QString::fromUtf8("seedingThresholdsStep2Label_"));
  seedingThresholdsStep2Label_->setGeometry(QRect(10, 80, 41, 31));
  seedingThresholdsStep2Label_->setFont(font2);
  seedingThresholdsStep2Label_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  seedingThresholdsStep2Label_->setAlignment(Qt::AlignCenter);
  seedingThresholdsStep3Label_ = new QLabel(seedingthresholdsGroupBox_);
  seedingThresholdsStep3Label_->setObjectName(QString::fromUtf8("seedingThresholdsStep3Label_"));
  seedingThresholdsStep3Label_->setGeometry(QRect(10, 120, 41, 31));
  seedingThresholdsStep3Label_->setFont(font2);
  seedingThresholdsStep3Label_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  seedingThresholdsStep3Label_->setAlignment(Qt::AlignCenter);
  blueishnessGroupBox_ = new QGroupBox(rawimagerecoGroupBox_);
  blueishnessGroupBox_->setObjectName(QString::fromUtf8("blueishnessGroupBox_"));
  blueishnessGroupBox_->setGeometry(QRect(200, 40, 151, 81));
  blueishnessGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  blueishnessSpinBox_ = new QDoubleSpinBox(blueishnessGroupBox_);
  blueishnessSpinBox_->setObjectName(QString::fromUtf8("blueishnessSpinBox_"));
  blueishnessSpinBox_->setGeometry(QRect(20, 30, 111, 31));
  blueishnessSpinBox_->setFont(font8);
  blueishnessSpinBox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
  blueishnessSpinBox_->setDecimals(2);
  hswGroupBox_ = new QGroupBox(rawimagerecoGroupBox_);
  hswGroupBox_->setObjectName(QString::fromUtf8("hswGroupBox_"));
  hswGroupBox_->setGeometry(QRect(200, 130, 151, 81));
  hswGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  hswSpinBox_ = new QDoubleSpinBox(hswGroupBox_);
  hswSpinBox_->setObjectName(QString::fromUtf8("hswSpinBox_"));
  hswSpinBox_->setGeometry(QRect(20, 30, 101, 31));
  QFont font10;
  font10.setFamily(QString::fromUtf8("Courier New"));
  font10.setPointSize(14);
  font10.setBold(false);
  font10.setWeight(50);
  hswSpinBox_->setFont(font10);
  hswSpinBox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  hswSpinBox_->setDecimals(0);
  groupBox_14 = new QGroupBox(advanced_tab);
  groupBox_14->setObjectName(QString::fromUtf8("groupBox_14"));
  groupBox_14->setGeometry(QRect(640, 40, 391, 61));
  groupBox_14->setStyleSheet(QString::fromUtf8("background-color: rgb(236, 235, 234);"));
  pushButton_12 = new QPushButton(groupBox_14);
  pushButton_12->setObjectName(QString::fromUtf8("pushButton_12"));
  pushButton_12->setGeometry(QRect(10, 10, 91, 41));
  QFont font11;
  font11.setPointSize(14);
  font11.setBold(true);
  font11.setWeight(75);
  pushButton_12->setFont(font11);
  pushButton_12->setStyleSheet(QString::fromUtf8("background-color: rgb(212, 212, 212);\n"
						 "color: rgb(170, 0, 0);"));
  pushButton_13 = new QPushButton(groupBox_14);
  pushButton_13->setObjectName(QString::fromUtf8("pushButton_13"));
  pushButton_13->setGeometry(QRect(110, 10, 61, 41));
  pushButton_13->setFont(font2);
  pushButton_13->setStyleSheet(QString::fromUtf8("background-color: rgb(212, 212, 212);"));
  pushButton_14 = new QPushButton(groupBox_14);
  pushButton_14->setObjectName(QString::fromUtf8("pushButton_14"));
  pushButton_14->setGeometry(QRect(180, 10, 61, 41));
  pushButton_14->setFont(font2);
  pushButton_14->setStyleSheet(QString::fromUtf8("background-color: rgb(212, 212, 212);"));
  pushButton_15 = new QPushButton(groupBox_14);
  pushButton_15->setObjectName(QString::fromUtf8("pushButton_15"));
  pushButton_15->setGeometry(QRect(250, 10, 61, 41));
  pushButton_15->setFont(font2);
  pushButton_15->setStyleSheet(QString::fromUtf8("background-color: rgb(212, 212, 212);"));
  pushButton_18 = new QPushButton(groupBox_14);
  pushButton_18->setObjectName(QString::fromUtf8("pushButton_18"));
  pushButton_18->setGeometry(QRect(320, 10, 61, 41));
  pushButton_18->setFont(font2);
  pushButton_18->setStyleSheet(QString::fromUtf8("background-color: rgb(212, 212, 212);"));
  commentGroupBox_ = new QGroupBox(advanced_tab);
  commentGroupBox_->setObjectName(QString::fromUtf8("commentGroupBox_"));
  commentGroupBox_->setGeometry(QRect(40, 130, 581, 221));
  commentGroupBox_->setFont(font1);
  commentGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  commentGroupBox_->setAlignment(Qt::AlignCenter);
  commentTextEdit_ = new QPlainTextEdit(commentGroupBox_);
  commentTextEdit_->setObjectName(QString::fromUtf8("commentTextEdit_"));
  commentTextEdit_->setGeometry(QRect(10, 30, 561, 181));
  commentTextEdit_->setFont(font3);
  commentTextEdit_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

  surfaceRecoGroupBox_ = new QGroupBox(advanced_tab);
  surfaceRecoGroupBox_->setObjectName(QString::fromUtf8("surfaceRecoGroupBox_"));
  surfaceRecoGroupBox_->setGeometry(QRect(430, 360, 381, 241));
  surfaceRecoGroupBox_->setFont(font1);
  surfaceRecoGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  surfaceRecoGroupBox_->setAlignment(Qt::AlignCenter);
  surfaceRecoSpacingGroupBox_ = new QGroupBox(surfaceRecoGroupBox_);
  surfaceRecoSpacingGroupBox_->setObjectName(QString::fromUtf8("surfaceRecoSpacingGroupBox_"));
  surfaceRecoSpacingGroupBox_->setGeometry(QRect(20, 30, 131, 61));
  surfaceRecoSpacingGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  surfaceRecoSpacingGroupBox_->setAlignment(Qt::AlignCenter);
  surfaceRecoSpacingSpinbox_ = new QDoubleSpinBox(surfaceRecoSpacingGroupBox_);
  surfaceRecoSpacingSpinbox_->setObjectName(QString::fromUtf8("surfaceRecoSpacingSpinbox_"));
  surfaceRecoSpacingSpinbox_->setGeometry(QRect(20, 20, 91, 31));
  surfaceRecoSpacingSpinbox_->setFont(font8);
  surfaceRecoSpacingSpinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  surfaceRecoSpacingSpinbox_->setDecimals(0);
  surfaceRecoSearchpathGroupBox_ = new QGroupBox(surfaceRecoGroupBox_);
  surfaceRecoSearchpathGroupBox_->setObjectName(QString::fromUtf8("surfaceRecoSearchpathGroupBox_"));
  surfaceRecoSearchpathGroupBox_->setGeometry(QRect(20, 100, 131, 61));
  surfaceRecoSearchpathGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  surfaceRecoSearchpathGroupBox_->setAlignment(Qt::AlignCenter);
  surfaceRecoSearchpathSpinbox_ = new QDoubleSpinBox(surfaceRecoSearchpathGroupBox_);
  surfaceRecoSearchpathSpinbox_->setObjectName(QString::fromUtf8("surfaceRecoSearchpathSpinbox_"));
  surfaceRecoSearchpathSpinbox_->setGeometry(QRect(20, 20, 91, 31));
  surfaceRecoSearchpathSpinbox_->setFont(font8);
  surfaceRecoSearchpathSpinbox_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  surfaceRecoSearchpathSpinbox_->setDecimals(0);
  groupBox_20 = new QGroupBox(advanced_tab);
  groupBox_20->setObjectName(QString::fromUtf8("groupBox_20"));
  groupBox_20->setGeometry(QRect(640, 110, 391, 241));
  groupBox_20->setFont(font1);
  groupBox_20->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  groupBox_20->setAlignment(Qt::AlignCenter);
  geometryGroupBox_ = new QGroupBox(advanced_tab);
  geometryGroupBox_->setObjectName(QString::fromUtf8("geometryGroupBox_"));
  geometryGroupBox_->setGeometry(QRect(40, 610, 371, 161));
  geometryGroupBox_->setFont(font1);
  geometryGroupBox_->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 127);"));
  geometryGroupBox_->setAlignment(Qt::AlignCenter);
  geometrySpinbox1_ = new QSpinBox(geometryGroupBox_);
  geometrySpinbox1_->setObjectName(QString::fromUtf8("geometrySpinbox1_"));
  geometrySpinbox1_->setGeometry(QRect(70, 30, 81, 26));
  geometrySpinbox1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  geometrySpinbox2_ = new QSpinBox(geometryGroupBox_);
  geometrySpinbox2_->setObjectName(QString::fromUtf8("geometrySpinbox2_"));
  geometrySpinbox2_->setGeometry(QRect(70, 60, 81, 26));
  geometrySpinbox2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  geometrySpinbox3_ = new QSpinBox(geometryGroupBox_);
  geometrySpinbox3_->setObjectName(QString::fromUtf8("geometrySpinbox3_"));
  geometrySpinbox3_->setGeometry(QRect(70, 90, 81, 26));
  geometrySpinbox3_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  geometrySpinbox4_ = new QDoubleSpinBox(geometryGroupBox_);
  geometrySpinbox4_->setObjectName(QString::fromUtf8("geometrySpinbox4_"));
  geometrySpinbox4_->setGeometry(QRect(70, 120, 81, 26));
  geometrySpinbox4_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  geometryLabel1_ = new QLabel(geometryGroupBox_);
  geometryLabel1_->setObjectName(QString::fromUtf8("geometryLabel1_"));
  geometryLabel1_->setGeometry(QRect(10, 30, 41, 31));
  geometryLabel1_->setFont(font9);
  geometryLabel1_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryLabel1_->setAlignment(Qt::AlignCenter);
  geometryLabel2_ = new QLabel(geometryGroupBox_);
  geometryLabel2_->setObjectName(QString::fromUtf8("geometryLabel2_"));
  geometryLabel2_->setGeometry(QRect(10, 60, 51, 31));
  geometryLabel2_->setFont(font9);
  geometryLabel2_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryLabel2_->setTextFormat(Qt::AutoText);
  geometryLabel2_->setAlignment(Qt::AlignCenter);
  geometryLabel3_ = new QLabel(geometryGroupBox_);
  geometryLabel3_->setObjectName(QString::fromUtf8("geometryLabel3_"));
  geometryLabel3_->setGeometry(QRect(10, 90, 51, 31));
  geometryLabel3_->setFont(font9);
  geometryLabel3_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryLabel3_->setAlignment(Qt::AlignCenter);
  geometryLabel4_ = new QLabel(geometryGroupBox_);
  geometryLabel4_->setObjectName(QString::fromUtf8("geometryLabel4_"));
  geometryLabel4_->setGeometry(QRect(10, 120, 41, 31));
  geometryLabel4_->setFont(font9);
  geometryLabel4_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryLabel4_->setAlignment(Qt::AlignCenter);
  geometryPitchGroupBox_ = new QGroupBox(geometryGroupBox_);
  geometryPitchGroupBox_->setObjectName(QString::fromUtf8("geometryPitchGroupBox_"));
  geometryPitchGroupBox_->setGeometry(QRect(190, 30, 141, 111));
  geometryPitchGroupBox_->setAlignment(Qt::AlignCenter);
  geometryPitchLabel1_ = new QLabel(geometryPitchGroupBox_);
  geometryPitchLabel1_->setObjectName(QString::fromUtf8("geometryPitchLabel1_"));
  geometryPitchLabel1_->setGeometry(QRect(10, 30, 31, 31));
  QFont font12;
  font12.setPointSize(12);
  font12.setBold(true);
  font12.setWeight(75);
  geometryPitchLabel1_->setFont(font12);
  geometryPitchLabel1_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryPitchLabel1_->setAlignment(Qt::AlignCenter);
  geometryPitchLabel2_ = new QLabel(geometryPitchGroupBox_);
  geometryPitchLabel2_->setObjectName(QString::fromUtf8("geometryPitchLabel2_"));
  geometryPitchLabel2_->setGeometry(QRect(10, 70, 31, 31));
  geometryPitchLabel2_->setFont(font12);
  geometryPitchLabel2_->setStyleSheet(QString::fromUtf8("color: rgb(1, 89, 23);"));
  geometryPitchLabel2_->setAlignment(Qt::AlignCenter);
  geometryPitchSpinbox1_ = new QDoubleSpinBox(geometryPitchGroupBox_);
  geometryPitchSpinbox1_->setObjectName(QString::fromUtf8("geometryPitchSpinbox1_"));
  geometryPitchSpinbox1_->setGeometry(QRect(40, 30, 71, 26));
  geometryPitchSpinbox1_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  geometryPitchSpinbox2_ = new QDoubleSpinBox(geometryPitchGroupBox_);
  geometryPitchSpinbox2_->setObjectName(QString::fromUtf8("geometryPitchSpinbox2_"));
  geometryPitchSpinbox2_->setGeometry(QRect(40, 70, 71, 26));
  geometryPitchSpinbox2_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
  mainTabWidget_->addTab(advanced_tab, QString());
  //  MainWindow->setCentralWidget(centralwidget);
  // setCentralWidget(centralwidget);
  //  statusbar = new QStatusBar(this);
  //  statusbar->setObjectName(QString::fromUtf8("statusbar"));
  //MainWindow->setStatusBar(statusbar);
  //  setStatusBar(statusbar);

  retranslateUi();

  mainTabWidget_->setCurrentIndex(0);
  surfacesTabWidget_->setCurrentIndex(0);


  //  QMetaObject::connectSlotsByName(MainWindow);
} // setupUi


void DefoMainWindow::retranslateUi( void ) {

  //  MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
  setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
  rawimageGroupBox_->setTitle(QApplication::translate("MainWindow", "Current raw image", 0, QApplication::UnicodeUTF8));
  //  rawimageZoomButton_->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
  areaGroupBox_->setTitle(QApplication::translate("MainWindow", "Area", 0, QApplication::UnicodeUTF8));
  areaNewButton_->setText(QApplication::translate("MainWindow", "New", 0, QApplication::UnicodeUTF8));
  areaDeleteButton_->setText(QApplication::translate("MainWindow", "Delete", 0, QApplication::UnicodeUTF8));
//   areaDoneButton_->setText(QApplication::translate("MainWindow", "DONE", 0, QApplication::UnicodeUTF8));
//   areaCancelButton_->setText(QApplication::translate("MainWindow", "Cancel", 0, QApplication::UnicodeUTF8));
  areaLabel_->setText(QApplication::translate("MainWindow", "Select", 0, QApplication::UnicodeUTF8));
  //  rawimageSaveButton_->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
  displayGroupBox_->setTitle(QApplication::translate("MainWindow", "Display", 0, QApplication::UnicodeUTF8));
  displayAreasButton_->setText(QApplication::translate("MainWindow", "Areas", 0, QApplication::UnicodeUTF8));
  displaySplinesButton_->setText(QApplication::translate("MainWindow", "Splines", 0, QApplication::UnicodeUTF8));
  displayRecoitemButton_->setText(QApplication::translate("MainWindow", "Recoitems", 0, QApplication::UnicodeUTF8));
  scheduleGroupBox_->setTitle(QApplication::translate("MainWindow", "Schedule", 0, QApplication::UnicodeUTF8));
  scheduleClearButton_->setText(QApplication::translate("MainWindow", "Clear", 0, QApplication::UnicodeUTF8));
  scheduleVerifyButton_->setText(QApplication::translate("MainWindow", "Verify", 0, QApplication::UnicodeUTF8));
  scheduleLoadButton_->setText(QApplication::translate("MainWindow", "Load", 0, QApplication::UnicodeUTF8));
  scheduleSaveButton_->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
  scheduleStopButton_->setText(QApplication::translate("MainWindow", "STOP", 0, QApplication::UnicodeUTF8));
  scheduleStartButton_->setText(QApplication::translate("MainWindow", "START", 0, QApplication::UnicodeUTF8));
  schedulePauseButton_->setText(QApplication::translate("MainWindow", "PAUSE/RES", 0, QApplication::UnicodeUTF8));
  measurementidGroupBox_->setTitle(QApplication::translate("MainWindow", "Measurement ID", 0, QApplication::UnicodeUTF8));
  measurementidEditButton_->setText(QApplication::translate("MainWindow", "Edit", 0, QApplication::UnicodeUTF8));
  measurementidDefaultButton_->setText(QApplication::translate("MainWindow", "Default", 0, QApplication::UnicodeUTF8));
  measurementidTextedit_->setPlainText(QApplication::translate("MainWindow", "", 0, QApplication::UnicodeUTF8));
  imageinfoGroupBox_->setTitle(QApplication::translate("MainWindow", "Image info", 0, QApplication::UnicodeUTF8));
//   imageinfoTextedit_->setPlainText(QApplication::translate("MainWindow", "date:\n"
// 							   "size:\n"
// 							   "type:\n"
// 							   "fetched-from:\n"
// 							   "grayscale-average:", 0, QApplication::UnicodeUTF8));
  mainTabWidget_->setTabText(mainTabWidget_->indexOf(online_tab), QApplication::translate("MainWindow", "Online", 0, QApplication::UnicodeUTF8));
  historyGroupBox_->setTitle(QApplication::translate("MainWindow", "History", 0, QApplication::UnicodeUTF8));
  historyZoomButton_->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
  historyDisplayGroupBox_->setTitle(QApplication::translate("MainWindow", "Display select", 0, QApplication::UnicodeUTF8));
  historyDisplay0Button_->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
  historyDisplay1Button_->setText(QApplication::translate("MainWindow", "1", 0, QApplication::UnicodeUTF8));
  historyDisplay2Button_->setText(QApplication::translate("MainWindow", "2", 0, QApplication::UnicodeUTF8));
  historyDisplay3Button_->setText(QApplication::translate("MainWindow", "3", 0, QApplication::UnicodeUTF8));
  historyDisplay4Button_->setText(QApplication::translate("MainWindow", "4", 0, QApplication::UnicodeUTF8));
  historyDisplay5Button_->setText(QApplication::translate("MainWindow", "5", 0, QApplication::UnicodeUTF8));
  historyDisplay6Button_->setText(QApplication::translate("MainWindow", "6", 0, QApplication::UnicodeUTF8));
  historyDisplay7Button_->setText(QApplication::translate("MainWindow", "7", 0, QApplication::UnicodeUTF8));
  historyDisplay8Button_->setText(QApplication::translate("MainWindow", "8", 0, QApplication::UnicodeUTF8));
  historyDisplay9Button_->setText(QApplication::translate("MainWindow", "9", 0, QApplication::UnicodeUTF8));
  historyDisplayRefButton_->setText(QApplication::translate("MainWindow", "ref", 0, QApplication::UnicodeUTF8));
  historyDisplayBathButton_->setText(QApplication::translate("MainWindow", "bath", 0, QApplication::UnicodeUTF8));
  historyClearButton_->setText(QApplication::translate("MainWindow", "Clear", 0, QApplication::UnicodeUTF8));
  historySaveButton_->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
  chillerGroupBox_->setTitle(QApplication::translate("MainWindow", "Chiller", 0, QApplication::UnicodeUTF8));
  chillerTargetLabel_->setText(QApplication::translate("MainWindow", "Target (\302\260C)", 0, QApplication::UnicodeUTF8));
  chillerPowerLabel_->setText(QApplication::translate("MainWindow", "Power (%)", 0, QApplication::UnicodeUTF8));
  chillerBathLabel_->setText(QApplication::translate("MainWindow", "Bath (\302\260C)", 0, QApplication::UnicodeUTF8));
  chillerBathMaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  chillerBathMinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  chillerClearButton_->setText(QApplication::translate("MainWindow", "Clear min/max", 0, QApplication::UnicodeUTF8));
  sensorsGroupBox_->setTitle(QApplication::translate("MainWindow", "Pt-100 Sensors", 0, QApplication::UnicodeUTF8));
  sensorsSensor0Label_->setText(QApplication::translate("MainWindow", "Sensor 0 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor1Label_->setText(QApplication::translate("MainWindow", "Sensor 1 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor2Label_->setText(QApplication::translate("MainWindow", "Sensor 2 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor3Label_->setText(QApplication::translate("MainWindow", "Sensor 3 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor4Label_->setText(QApplication::translate("MainWindow", "Sensor 4 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor6Label_->setText(QApplication::translate("MainWindow", "Sensor 6 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor7Label_->setText(QApplication::translate("MainWindow", "Sensor 7 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor5Label_->setText(QApplication::translate("MainWindow", "Sensor 5 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor9Label_->setText(QApplication::translate("MainWindow", "Sensor 9 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor8Label_->setText(QApplication::translate("MainWindow", "Sensor 8 (\302\260C)", 0, QApplication::UnicodeUTF8));
  sensorsSensor5MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor5MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor6MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor6MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor7MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor7MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor8MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor8MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor9MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor9MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor2MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor4MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor0MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor1MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor3MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor2MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor0MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor4MaxLabel_->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
  sensorsSensor3MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsSensor1MinLabel_->setText(QApplication::translate("MainWindow", "min", 0, QApplication::UnicodeUTF8));
  sensorsClearButton_->setText(QApplication::translate("MainWindow", "Clear min/max", 0, QApplication::UnicodeUTF8));
  mainTabWidget_->setTabText(mainTabWidget_->indexOf(tab_2), QApplication::translate("MainWindow", "Cooling", 0, QApplication::UnicodeUTF8));
  surfaceGroupBox_->setTitle(QApplication::translate("MainWindow", "Current surfaces", 0, QApplication::UnicodeUTF8));
  //  surfacesZoomButton_->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
  surfacesTabWidget_->setTabText(surfacesTabWidget_->indexOf(tab), QApplication::translate("MainWindow", "Area 1", 0, QApplication::UnicodeUTF8));
  displaytypeGroupbox_->setTitle(QApplication::translate("MainWindow", "Display type", 0, QApplication::UnicodeUTF8));
  displaytype2DButton_->setText(QApplication::translate("MainWindow", "2D", 0, QApplication::UnicodeUTF8));
  displaytype3DButton_->setText(QApplication::translate("MainWindow", "3D", 0, QApplication::UnicodeUTF8));
  displayitemsGroupbox_->setTitle(QApplication::translate("MainWindow", "Display items", 0, QApplication::UnicodeUTF8));
  displayitemsAxesButton_->setText(QApplication::translate("MainWindow", "Axes", 0, QApplication::UnicodeUTF8));
  displayitemsMeshButton_->setText(QApplication::translate("MainWindow", "Mesh", 0, QApplication::UnicodeUTF8));
  displayitemsShadeButton_->setText(QApplication::translate("MainWindow", "Shade", 0, QApplication::UnicodeUTF8));
  displayitemsLegendButton_->setText(QApplication::translate("MainWindow", "Legend", 0, QApplication::UnicodeUTF8));
  displayitemsIsolinesButton_->setText(QApplication::translate("MainWindow", "Isolines", 0, QApplication::UnicodeUTF8));
  zscaleGroupbox_->setTitle(QApplication::translate("MainWindow", "z scale", 0, QApplication::UnicodeUTF8));
  zscalePlusButton_->setText(QApplication::translate("MainWindow", "+", 0, QApplication::UnicodeUTF8));
  zscaleMinusButton_->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
  mainTabWidget_->setTabText(mainTabWidget_->indexOf(offline_tab), QApplication::translate("MainWindow", "Offline", 0, QApplication::UnicodeUTF8));
  basefolderGroupBox_->setTitle(QApplication::translate("MainWindow", "Output base folder", 0, QApplication::UnicodeUTF8));
  basefolderTextedit_->setPlainText(QApplication::translate("MainWindow", "out", 0, QApplication::UnicodeUTF8));
  basefolderEditButton_->setText(QApplication::translate("MainWindow", "Edit", 0, QApplication::UnicodeUTF8));
  chillerparametersGroupBox_->setTitle(QApplication::translate("MainWindow", "Chiller parameters", 0, QApplication::UnicodeUTF8));
  chillerParametersLabel1_->setText(QApplication::translate("MainWindow", "Prop (Xp)", 0, QApplication::UnicodeUTF8));
  chillerParametersLabel2_->setText(QApplication::translate("MainWindow", "Int (Tn)", 0, QApplication::UnicodeUTF8));
  chillerParametersLabel3_->setText(QApplication::translate("MainWindow", "Diff (Tv)", 0, QApplication::UnicodeUTF8));
  rawimagerecoGroupBox_->setTitle(QApplication::translate("MainWindow", "Raw image reconstruction", 0, QApplication::UnicodeUTF8));
  seedingthresholdsGroupBox_->setTitle(QApplication::translate("MainWindow", "Seeding thresholds (adc)", 0, QApplication::UnicodeUTF8));
  seedingThresholdsStep1Label_->setText(QApplication::translate("MainWindow", "Step 1", 0, QApplication::UnicodeUTF8));
  seedingThresholdsStep2Label_->setText(QApplication::translate("MainWindow", "Step 2", 0, QApplication::UnicodeUTF8));
  seedingThresholdsStep3Label_->setText(QApplication::translate("MainWindow", "Step 3", 0, QApplication::UnicodeUTF8));
  blueishnessGroupBox_->setTitle(QApplication::translate("MainWindow", "Blueishness threshold", 0, QApplication::UnicodeUTF8));
  hswGroupBox_->setTitle(QApplication::translate("MainWindow", "Half square width (px)", 0, QApplication::UnicodeUTF8));
  groupBox_14->setTitle(QString());
  pushButton_12->setText(QApplication::translate("MainWindow", "APPLY", 0, QApplication::UnicodeUTF8));
  pushButton_13->setText(QApplication::translate("MainWindow", "READ", 0, QApplication::UnicodeUTF8));
  pushButton_14->setText(QApplication::translate("MainWindow", "LOAD", 0, QApplication::UnicodeUTF8));
  pushButton_15->setText(QApplication::translate("MainWindow", "SAVE", 0, QApplication::UnicodeUTF8));
  pushButton_18->setText(QApplication::translate("MainWindow", "DEFAULT", 0, QApplication::UnicodeUTF8));
  commentGroupBox_->setTitle(QApplication::translate("MainWindow", "Comment", 0, QApplication::UnicodeUTF8));
  commentTextEdit_->setPlainText(QApplication::translate("MainWindow", "# ADD COMMENT", 0, QApplication::UnicodeUTF8));
  surfaceRecoGroupBox_->setTitle(QApplication::translate("MainWindow", "Surface reconstruction", 0, QApplication::UnicodeUTF8));
  surfaceRecoSpacingGroupBox_->setTitle(QApplication::translate("MainWindow", "Spacing estimate (px)", 0, QApplication::UnicodeUTF8));
  surfaceRecoSearchpathGroupBox_->setTitle(QApplication::translate("MainWindow", "Searchpath width (px)", 0, QApplication::UnicodeUTF8));
  groupBox_20->setTitle(QApplication::translate("MainWindow", "Output options", 0, QApplication::UnicodeUTF8));
  geometryGroupBox_->setTitle(QApplication::translate("MainWindow", "Geometry parameters", 0, QApplication::UnicodeUTF8));
  geometryLabel1_->setText(QApplication::translate("MainWindow", "f [mm]", 0, QApplication::UnicodeUTF8));
  geometryLabel2_->setText(QApplication::translate("MainWindow", "L<sub>g</sub> [mm]", 0, QApplication::UnicodeUTF8));
  geometryLabel3_->setText(QApplication::translate("MainWindow", "L<sub>c</sub> [mm]", 0, QApplication::UnicodeUTF8));
  geometryLabel4_->setText(QApplication::translate("MainWindow", "\316\264 [\302\260]", 0, QApplication::UnicodeUTF8));
  geometryPitchGroupBox_->setTitle(QApplication::translate("MainWindow", "Pixel pitch (\302\265m)", 0, QApplication::UnicodeUTF8));
  geometryPitchLabel1_->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
  geometryPitchLabel2_->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
  mainTabWidget_->setTabText(mainTabWidget_->indexOf(advanced_tab), QApplication::translate("MainWindow", "Advanced", 0, QApplication::UnicodeUTF8));
  //  Q_UNUSED(DefoMainWindow);

}





///
///
///
void DefoMainWindow::setupSignalsAndSlots( void ) {

  // rawimage & areas
  connect( areaNewButton_, SIGNAL(clicked()), rawimageLabel_, SLOT(defineArea()) );

  // action polling
  connect( this, SIGNAL(pollAction()), schedule_, SLOT(pollAction()) );
  connect( schedule_, SIGNAL(newAction(DefoSchedule::scheduleItem)), this, SLOT( handleAction(DefoSchedule::scheduleItem)) );
  connect( schedule_, SIGNAL(unableToDeliverAction()), this, SLOT(stopPolling()) );
  connect( schedule_, SIGNAL(newRow(int)), scheduleTableview_, SLOT(selectRow(int)) );

  // schedule buttons & misc
  connect( scheduleStartButton_, SIGNAL(clicked()), this, SLOT(startPolling()) );
  connect( schedulePauseButton_, SIGNAL(clicked()), this, SLOT(pausePolling()) );
  connect( scheduleStopButton_, SIGNAL(clicked()), this, SLOT(stopPolling()) );
  connect( scheduleClearButton_, SIGNAL(clicked()), schedule_, SLOT(clear()) );
  connect( scheduleLoadButton_, SIGNAL(clicked()), schedule_, SLOT(loadFromFile()) );
  connect( scheduleSaveButton_, SIGNAL(clicked()), schedule_, SLOT(saveToFile()) );

  // measurement id
  connect( measurementidEditButton_, SIGNAL(clicked()), this, SLOT(editMeasurementId()) );
  connect( measurementidDefaultButton_, SIGNAL(clicked()), this, SLOT(defaultMeasurementId()) );

  // offline display
  connect( displaytype3DButton_, SIGNAL(toggled( bool ) ), surfacePlot_, SLOT( toggleView( bool ) ) );
  connect( displayitemsAxesButton_, SIGNAL( toggled( bool ) ), surfacePlot_, SLOT( setIsAxes( bool ) ) );
  connect( displayitemsMeshButton_, SIGNAL( toggled( bool ) ), surfacePlot_, SLOT( setIsMesh( bool ) ) );
  connect( displayitemsShadeButton_, SIGNAL( toggled( bool ) ), surfacePlot_, SLOT( setIsShade( bool ) ) );
  connect( displayitemsLegendButton_, SIGNAL( toggled( bool ) ), surfacePlot_, SLOT( setIsLegend( bool ) ) );
  connect( displayitemsIsolinesButton_, SIGNAL( toggled( bool ) ), surfacePlot_, SLOT( setIsIsolines( bool ) ) );
  connect( displayitemsIsolinesSpinbox_, SIGNAL( valueChanged( int ) ), surfacePlot_, SLOT( setNIsolines( int ) ) );
  connect( surfacePlot_, SIGNAL( shadeEnabled( bool ) ), displayitemsShadeButton_, SLOT( setChecked( bool ) ) );
  connect( surfacePlot_, SIGNAL( meshEnabled( bool ) ), displayitemsMeshButton_, SLOT( setChecked( bool ) ) );
  connect( surfacePlot_, SIGNAL( legendEnabled( bool ) ), displayitemsLegendButton_, SLOT( setChecked( bool ) ) );
  connect( surfacePlot_, SIGNAL( axesEnabled( bool ) ), displayitemsAxesButton_, SLOT( setChecked( bool ) ) );
  connect( surfacePlot_, SIGNAL( isolinesEnabled( bool ) ), displayitemsIsolinesButton_, SLOT( setChecked( bool ) ) );
  connect( surfacePlot_, SIGNAL( nIsolines( int ) ), displayitemsIsolinesSpinbox_, SLOT( setValue( int ) ) );
  connect( displayitemsIsolinesButton_, SIGNAL( toggled( bool ) ), displayitemsIsolinesSpinbox_, SLOT( setEnabled( bool ) ) );

  connect( zscalePlusButton_, SIGNAL( clicked() ), surfacePlot_, SLOT( increaseZScale() ) );
  connect( zscaleMinusButton_, SIGNAL( clicked() ), surfacePlot_, SLOT( decreaseZScale() ) );

  // advanced
  connect( basefolderEditButton_, SIGNAL( clicked() ), this, SLOT( editBaseFolder() ) );

}



///
/// to be called upon a DefoSchedule::newAction signal
///
void DefoMainWindow::handleAction( DefoSchedule::scheduleItem item ) {

  bool isContinuePolling = true; // local decision

  switch( item.first ) {

    
  case DefoSchedule::REF:
    {
      std::cout << " [DefoMainWindow::handleAction] =2= received REF" << std::endl;

      isRefImage_ = true;
    }
    break;




  case DefoSchedule::DEFO:
    {
      std::cout << " [DefoMainWindow::handleAction] =2= received DEFO" << std::endl;
    }
    break;





  case DefoSchedule::FILE_REF:
    {
      std::cout << " [DefoMainWindow::handleAction] =2= received FILE_REF" << std::endl;

      // check file name/path
      QFileInfo fileInfo( item.second );
      if( !fileInfo.exists() ) {
	QMessageBox::critical( this, tr("[DefoMainWindow::handleAction]"),
			       QString("[FILE_REF]: file \'%1\' not found.").arg(item.second),
			       QMessageBox::Ok );
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_REF] cannot open file: " << item.second.toStdString() << std::endl;
	imageinfoTextedit_->appendPlainText( QString( "ERROR: [FILE_REF] cannot open file: \'%1\'" ).arg( item.second ) );
	scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 255,0,0 ); "));
	stopPolling();
	isContinuePolling = false;
	break;
      }

      // create output folder
      QDateTime datime = QDateTime::currentDateTime();
      QDir currentDir( currentFolderName_ );
      QString subdir = datime.toString( QString( "ddMMyy-hhmmss" ) );
      if( !currentDir.mkdir( subdir ) ) {
	QMessageBox::critical( this, tr("[DefoMainWindow::handleAction]"),
			       QString("[FILE_DEFO]: cannot create output dir: \'%1\'").arg(subdir),
			       QMessageBox::Ok );
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_DEFO]: cannot create output dir: " 
		  << subdir.toStdString() << std::endl;
      }

      QDir outputDir = QDir( currentFolderName_ + "/" + subdir );

      // get the image & save it
      DefoRawImage refImage( item.second.toStdString().c_str() );
      QString rawImageFileName = outputDir.path() + "/refimage_raw.jpg";
      refImage.getImage().save( rawImageFileName, 0, 100 );

      // point reconstruction
      referenceImageOutput_ = defoRecoImage_.reconstruct( refImage );
      QImage& qImage =  referenceImageOutput_.second.getImage();

      // save the updated file
      QString recoImageFileName = outputDir.path() + "/refimage_reco.jpg";
      qImage.save( recoImageFileName, 0, 100 );

      // display info
      imageinfoTextedit_->clear();
      datime = QDateTime::currentDateTime(); // resuse!!
      imageinfoTextedit_->appendPlainText( QString( "Raw image size: %1 x %2 pixel" ).arg(qImage.width()).arg(qImage.height()) );
      imageinfoTextedit_->appendPlainText( QString( "Fetched: %1" ).arg( datime.toString( QString( "dd.MM.yy hh:mm:ss" ) ) ) );
      imageinfoTextedit_->appendPlainText( QString( "Type: from disk [%1]" ).arg( item.second ) );

//       // scale image
//       QSize newSize( 700, 525 ); 
//       QImage *transformedImage = new QImage( qImage.scaled( newSize ) );

//       // rotate 90deg
//       QTransform rotateTransform;
//       rotateTransform.rotate(90);
//       *transformedImage = transformedImage->transformed( rotateTransform );

//       // display
//       rawimageLabel_->setPixmap( QPixmap::fromImage( *transformedImage ) );

      // tell the label to rotate the image and display
      rawimageLabel_->setRotation( true );
      rawimageLabel_->displayImageToSize( qImage );

      isRefImage_ = true;

    }
    break;

  case DefoSchedule::FILE_DEFO:
    {
      std::cout << " [DefoMainWindow::handleAction] =2= received FILE_DEFO" << std::endl;

      // check file name/path
      QFileInfo fileInfo( item.second );
      if( !fileInfo.exists() ) {
	QMessageBox::critical( this, tr("[DefoMainWindow::handleAction]"),
			       QString("[FILE_DEFO]: file \'%1\' not found.").arg(item.second),
			       QMessageBox::Ok );
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_DEFO] cannot open file: " << item.second.toStdString() << std::endl;
	imageinfoTextedit_->appendPlainText( QString( "ERROR: [FILE_DEFO] cannot open file: \'%1\'" ).arg( item.second ) );
	scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 255,0,0 ); "));
	stopPolling();
	isContinuePolling = false;
	break;
      }

      if( !isRefImage_ ) { // check if we have a reference for the reconstruction
	QMessageBox::critical( this, tr("[DefoMainWindow::handleAction]"),
			       QString("[FILE_DEFO]: no reference image taken."),//.arg(item.second),
			       QMessageBox::Ok );
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_DEFO]: no reference image taken." << item.second.toStdString() << std::endl;
	scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 255,0,0 ); "));
	stopPolling();
	isContinuePolling = false;
	break;
      }

      // create output folder
      QDateTime datime = QDateTime::currentDateTime();
      QDir currentDir( currentFolderName_ );
      QString subdir = datime.toString( QString( "ddMMyy-hhmmss" ) );
      if( !currentDir.mkdir( subdir ) ) {
	QMessageBox::critical( this, tr("[DefoMainWindow::handleAction]"),
			       QString("[FILE_DEFO]: cannot create output dir: \'%1\'").arg(subdir),
			       QMessageBox::Ok );
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_DEFO]: cannot create output dir: " 
		  << subdir.toStdString() << std::endl;
      }

      QDir outputDir = QDir( currentFolderName_ + "/" + subdir );


      // get the image & save the raw version
      DefoRawImage defoImage( item.second.toStdString().c_str() );
      QString rawImageFileName = outputDir.path() + "/defoimage_raw.jpg";
      defoImage.getImage().save( rawImageFileName, 0, 100 );
      
      // point reconstruction
      defoImageOutput_ = defoRecoImage_.reconstruct( defoImage );
      QImage& qImage =  defoImageOutput_.second.getImage();

      // save the updated file
      QString recoImageFileName = outputDir.path() + "/defoimage_reco.jpg";
      qImage.save( recoImageFileName, 0, 100 );

      // display info
      imageinfoTextedit_->clear();
      datime = QDateTime::currentDateTime(); // resuse!
      imageinfoTextedit_->appendPlainText( QString( "Raw image size: %1 x %2 pixel" ).arg(qImage.width()).arg(qImage.height()) );
      imageinfoTextedit_->appendPlainText( QString( "Fetched: %1" ).arg( datime.toString( QString( "dd.MM.yy hh:mm:ss" ) ) ) );
      imageinfoTextedit_->appendPlainText( QString( "Type: from disk [%1]" ).arg( item.second ) );

//       // scale image
//       QSize newSize( 700, 525 ); 
//       QImage *transformedImage = new QImage( qImage.scaled( newSize ) );

//       // rotate 90deg
//       QTransform rotateTransform;
//       rotateTransform.rotate(90);
//       *transformedImage = transformedImage->transformed( rotateTransform );

//       // display
//       rawimageLabel_->setPixmap( QPixmap::fromImage( *transformedImage ) );

      // tell the label to rotate the image and display
      rawimageLabel_->setRotation( true );
      rawimageLabel_->displayImageToSize( qImage );

      // surface reconstruction
      const DefoSurface defoSurface = defoRecoSurface_.reconstruct( defoImageOutput_.first, referenceImageOutput_.first );

      surfacePlot_->setDisplayTitle( measurementId_ );
      surfacePlot_->setData( defoSurface );
      surfacePlot_->draw();
      
      { // serialize the output
	QString surfaceFileName = outputDir.path() + "/surface.defosurface";
	std::ofstream ofs( surfaceFileName.toStdString().c_str() );
	boost::archive::binary_oarchive oa( ofs );
	oa << defoSurface;
      }
      
    }
    break;

  case DefoSchedule::TEMP:
    std::cout << " TEMP action yet unsupported." << std::endl;
    break;



  case DefoSchedule::SLEEP:
    {
      std::cout << " [DefoMainWindow::handleAction] =2= received SLEEP " << item.second.toStdString() << std::endl;
      bool isOk = false;
      unsigned int time = 0;
      time = item.second.toUInt( &isOk, 10 );
      if( !isOk ) {
	std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [SLEEP] bad conversion to uint: " << item.second.toStdString() << std::endl;
	stopPolling();
	isContinuePolling = false;
	break;
      }
      isContinuePolling = false; // further polling done by timer
      if( isPolling_ ) QTimer::singleShot( time*1000, schedule_, SLOT(pollAction()) );
    }
    break;

  case DefoSchedule::GOTO:
    std::cerr << " SHIT.. GOTO SHOULD NEVER ARRIVE HERE!" << std::endl;
    throw;
    break;

  case DefoSchedule::END:
    std::cout << " [DefoMainWindow::handleAction] =2= Stop polling on END" << std::endl;
    stopPolling();
    isContinuePolling = false;
    break;

  default:
    break;

  }

  // we poll with a delay to give everybody time to update
  if( isContinuePolling && isPolling_ ) QTimer::singleShot( 1000, schedule_, SLOT(pollAction()) );

}



///
///
///
void DefoMainWindow::startPolling( void ) {

  schedule_->setIndex( 0 ); // start from beginning
  
  // create output folder
  QDir baseDir( baseFolderName_ );
  QString subdir = measurementidTextedit_->toPlainText();
  if( !baseDir.mkdir( subdir ) ) {
    QMessageBox::critical( this, tr("[DefoMainWindow::startPolling]"),
			   QString("[FILE_DEFO]: cannot create measurement dir: \'%1\'").arg(subdir),
			   QMessageBox::Ok );
    std::cerr << " [DefoMainWindow::handleAction] ** ERROR: [FILE_DEFO]: cannot create measurement dir: " 
	      << subdir.toStdString() << std::endl;
  }
  
  currentFolderName_ = baseFolderName_ + "/" + subdir;

  // highlight color for table view to green
  scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 0,255,0 ); "));

  scheduleStartButton_->setEnabled( false ); // toggle some buttons enable
  schedulePauseButton_->setEnabled( true );
  scheduleStopButton_->setEnabled( true );
  measurementidEditButton_->setEnabled( false );
  measurementidDefaultButton_->setEnabled( false );
  basefolderEditButton_->setEnabled( false );

  imageinfoTextedit_->clear(); // clear the imageinfo

  isPolling_ = true;

  emit pollAction(); // and go...
  
}



///
/// stop polling actions from the scheduler
///
void DefoMainWindow::stopPolling( void ) {

  std::cout << " [DefoMainWindow::stopPolling] =2= Stop polling." << std::endl;

  // free the buttons etc.
  scheduleStartButton_->setEnabled( true );
  schedulePauseButton_->setEnabled( false );
  scheduleStopButton_->setEnabled( false );
  measurementidEditButton_->setEnabled( true );
  measurementidDefaultButton_->setEnabled( true );
  basefolderEditButton_->setEnabled( true );

  // gray out
  scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 200,200,200 ); "));

  isPolling_ = false;

}



///
/// pause polling actions from the scheduler
///
void DefoMainWindow::pausePolling( void ) {

  if( isPolling_ ) {
    std::cout << " [DefoMainWindow::pausePolling] =2= Pause polling." << std::endl;
    scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 255,255,0 ); "));
    isPolling_ = false;
  }
  else {
    std::cout << " [DefoMainWindow::pausePolling] =2= Resume polling." << std::endl;
    scheduleTableview_->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);\n selection-background-color: rgb( 0,255,0 ); "));
    isPolling_ = true;
    emit pollAction();
  }

}



///
/// set the measurementid string
///
void DefoMainWindow::editMeasurementId( void ) {
  
  bool ok;
  measurementId_ = QInputDialog::getText( 0, "MeasurementID", "MeasurementID: ", QLineEdit::Normal, measurementidTextedit_->toPlainText(), &ok);
  
  if( ok && !measurementId_.isEmpty() ) measurementidTextedit_->setPlainText( measurementId_ );

}



///
/// set the measurementid string
///
void DefoMainWindow::defaultMeasurementId( void ) {
  
  QDateTime datime = QDateTime::currentDateTime();
  measurementId_ = QString( "defomeasurement-%1" ).arg( datime.toString( QString( "ddMMyy-hhmmss" ) ) );
  measurementidTextedit_->setPlainText( measurementId_ );

}



///
///
///
void DefoMainWindow::editBaseFolder( void ) {
  
  QString dirName = QFileDialog::getExistingDirectory( this,
							QString("Base folder"),
							QString("defo"),
							QFileDialog::ShowDirsOnly
						      );

  if( dirName.isEmpty() ) return;

  baseFolderName_ = dirName;
  basefolderTextedit_->setPlainText( baseFolderName_ );
  
}
