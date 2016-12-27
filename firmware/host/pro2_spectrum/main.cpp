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

class UberThread : public QThread
{
    Q_OBJECT

public:
    explicit UberThread();
    ~UberThread(){}

private:
    bool runStatus;
    void run();
};

void UberThread::run()
{
    while(1);

}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    uberusb();

    return a.exec();
}
