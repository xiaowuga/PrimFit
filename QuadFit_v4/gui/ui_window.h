/********************************************************************************
** Form generated from reading UI file 'window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOW_H
#define UI_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Window
{
public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionExit;
    QAction *actionAbout;
    QAction *actionClearRecentFiles;
    QAction *actionSnapshot;
    QAction *actionSetBackgroundColor;
    QAction *actionTopologyStatistics;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuRecentFiles;
    QMenu *menuView;
    QMenu *menuHelp;
    QMenu *menuTopology;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Window)
    {
        if (Window->objectName().isEmpty())
            Window->setObjectName(QString::fromUtf8("Window"));
        Window->resize(800, 600);
        Window->setMinimumSize(QSize(800, 600));
        actionOpen = new QAction(Window);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resources/icons/fileopen.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionSave = new QAction(Window);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resources/icons/filesave.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon1);
        actionExit = new QAction(Window);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resources/icons/exit.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExit->setIcon(icon2);
        actionAbout = new QAction(Window);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resources/icons/help.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAbout->setIcon(icon3);
        actionClearRecentFiles = new QAction(Window);
        actionClearRecentFiles->setObjectName(QString::fromUtf8("actionClearRecentFiles"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resources/icons/clear.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionClearRecentFiles->setIcon(icon4);
        actionSnapshot = new QAction(Window);
        actionSnapshot->setObjectName(QString::fromUtf8("actionSnapshot"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resources/icons/snap_shot.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSnapshot->setIcon(icon5);
        actionSetBackgroundColor = new QAction(Window);
        actionSetBackgroundColor->setObjectName(QString::fromUtf8("actionSetBackgroundColor"));
        actionTopologyStatistics = new QAction(Window);
        actionTopologyStatistics->setObjectName(QString::fromUtf8("actionTopologyStatistics"));
        centralWidget = new QWidget(Window);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        Window->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Window);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 24));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuRecentFiles = new QMenu(menuFile);
        menuRecentFiles->setObjectName(QString::fromUtf8("menuRecentFiles"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuTopology = new QMenu(menuBar);
        menuTopology->setObjectName(QString::fromUtf8("menuTopology"));
        Window->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Window);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        Window->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Window);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        Window->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuTopology->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(menuRecentFiles->menuAction());
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuRecentFiles->addSeparator();
        menuRecentFiles->addAction(actionClearRecentFiles);
        menuView->addAction(actionSnapshot);
        menuView->addSeparator();
        menuView->addAction(actionSetBackgroundColor);
        menuHelp->addAction(actionAbout);
        menuTopology->addAction(actionTopologyStatistics);
        menuTopology->addSeparator();
        mainToolBar->addAction(actionOpen);
        mainToolBar->addAction(actionSave);
        mainToolBar->addAction(actionSnapshot);
        mainToolBar->addSeparator();
        mainToolBar->addAction(actionAbout);

        retranslateUi(Window);

        QMetaObject::connectSlotsByName(Window);
    } // setupUi

    void retranslateUi(QMainWindow *Window)
    {
        Window->setWindowTitle(QApplication::translate("Window", "MainWindow", nullptr));
        actionOpen->setText(QApplication::translate("Window", "Open", nullptr));
#ifndef QT_NO_SHORTCUT
        actionOpen->setShortcut(QApplication::translate("Window", "Ctrl+O", nullptr));
#endif // QT_NO_SHORTCUT
        actionSave->setText(QApplication::translate("Window", "Save", nullptr));
#ifndef QT_NO_SHORTCUT
        actionSave->setShortcut(QApplication::translate("Window", "Ctrl+S", nullptr));
#endif // QT_NO_SHORTCUT
        actionExit->setText(QApplication::translate("Window", "Exit", nullptr));
#ifndef QT_NO_SHORTCUT
        actionExit->setShortcut(QApplication::translate("Window", "Ctrl+Q", nullptr));
#endif // QT_NO_SHORTCUT
        actionAbout->setText(QApplication::translate("Window", "About Easy3D", nullptr));
#ifndef QT_NO_TOOLTIP
        actionAbout->setToolTip(QApplication::translate("Window", "About Easy3D", nullptr));
#endif // QT_NO_TOOLTIP
        actionClearRecentFiles->setText(QApplication::translate("Window", "Clear", nullptr));
        actionSnapshot->setText(QApplication::translate("Window", "Snapshot", nullptr));
#ifndef QT_NO_SHORTCUT
        actionSnapshot->setShortcut(QApplication::translate("Window", "S", nullptr));
#endif // QT_NO_SHORTCUT
        actionSetBackgroundColor->setText(QApplication::translate("Window", "Background Color", nullptr));
        actionTopologyStatistics->setText(QApplication::translate("Window", "Show Statistics", nullptr));
        menuFile->setTitle(QApplication::translate("Window", "File", nullptr));
        menuRecentFiles->setTitle(QApplication::translate("Window", "Recent Files", nullptr));
        menuView->setTitle(QApplication::translate("Window", "View", nullptr));
        menuHelp->setTitle(QApplication::translate("Window", "Help", nullptr));
        menuTopology->setTitle(QApplication::translate("Window", "Topology", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Window: public Ui_Window {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOW_H
