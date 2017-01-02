/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "mainwindow.h"
#include <QApplication>

#include <QThread>
#include <QSemaphore>

#include "uberusb.h"

class QCustomPlot;

#include <QDebug>

class UberThread : public QThread
{
public:
    explicit UberThread() { runStatus = true;}
    ~UberThread() {
        runStatus = false;
        if (!this->isFinished())
            this->wait();
        qDebug() << "UberThread isFinished";
    }

private:
    QSemaphore mSem;
    volatile bool runStatus;
    void run();
};

void UberThread::run()
{
    while(runStatus) {
        uber_handle_events();
        //mSem.tryAcquire(1, 100); // sem.available() --
    }
}

MainWindow *wp;

extern "C" void UberPacket(unsigned short *buf, int len)
{
    wp->pushUberPacket((quint16 *)buf, len);
    qDebug() << "UberPacket: " << len;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    wp = &w;

    w.show();

    uberusb();

    UberThread mUberThread;
    mUberThread.start();

    return a.exec();
}
