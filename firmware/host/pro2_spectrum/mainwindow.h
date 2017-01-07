/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QQueue>
#include <QThread>
#include <QSemaphore>
#include <QDebug>
#include <QTimer>
#include <QVector>

class QCustomPlot;

namespace Ui {
class MainWindow;
}

class MainWindow;

class PlotThread : public QThread
{
public:
    explicit PlotThread() { runStatus = true; func=NULL;}
    ~PlotThread() {
        runStatus = false;
        if (!this->isFinished())
            this->wait();
        qDebug() << "PlotThread isFinished";
    }

    void setFunc(void (MainWindow::*f)(void)) {func = f;}

private:
    void (MainWindow::*func)(void);
    QSemaphore mSem;
    volatile bool runStatus;
    void run();
};

const static int FREQ_WIDTH = 84;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setupItemDemo(QCustomPlot *customPlot);
    void pushUberPacket(quint16 *buf, int len);
    void updataPlot(void);

private:
    Ui::MainWindow *ui;
    QQueue<quint16> mUberQueue;
    PlotThread mPlotThread;
    QTimer dataTimer;

private slots:
    void uberOpen();
    void uberClose();
    void bracketDataSlot();
    void uberRefresh();
};

#endif // MAINWINDOW_H
