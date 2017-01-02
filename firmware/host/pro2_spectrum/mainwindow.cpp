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
#include "ui_mainwindow.h"

#include "uberusb.h"

extern MainWindow *wp;

void PlotThread::run()
{
    while(runStatus) {
        mSem.tryAcquire(1, 20); // sem.available() --
        //qDebug() << "PlotThread";
        //if (func) (wp->*func)();
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setupItemDemo(ui->mQCustomPlot);

    ui->mQCustomPlot->replot();

    mPlotThread.setFunc(&MainWindow::updataPlot);
    mPlotThread.start();

    connect(ui->mButtonOpen, SIGNAL(clicked()), this, SLOT(uberOpen()));
    connect(ui->mButtonClose, SIGNAL(clicked()), this, SLOT(uberClose()));

    // setup a timer that repeatedly calls MainWindow::bracketDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(bracketDataSlot()));
    dataTimer.start(10); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::bracketDataSlot()
{
    updataPlot();
}

void MainWindow::updataPlot(void)
{
    //qDebug() << "updataPlot";
    if(mUberQueue.size() > 84) {
        int n = 84;
        QVector<double> x(n), y(n);
        for (int i=0; i<n; ++i) {
            quint16 data = mUberQueue.dequeue();
            //quint16 data = 0;
            int freq = (int) (data & 0xff);
            signed char dbm = (signed char) (data >> 8) ;
            y[freq] = (double)dbm;
            x[freq] = 2400 + freq;

            //qDebug() << "x " << freq << "y " << y[freq];

//            y[i] = i;
//            x[i] = 2400 + i;
        }

        ui->mQCustomPlot->graph()->setData(x, y);
        ui->mQCustomPlot->replot();
    }

    // calculate frames per second:
    double secs = QCPAxisTickerDateTime::dateTimeToKey(QDateTime::currentDateTime());
    double key = secs;
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
      ui->statusBar->showMessage(
            QString("%1 FPS, Total Data points: %2")
            .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
            .arg(ui->mQCustomPlot->graph(0)->data()->size()), 0);
      lastFpsKey = key;
      frameCount = 0;
    }
}

void MainWindow::pushUberPacket(quint16 *buf, int len)
{
    for (int i = 0; i < len; ++i) {
        mUberQueue.enqueue(buf[i]);
    }
}

void MainWindow::uberOpen()
{
    qDebug() << "uberOpen";
    uberopen();
}

void MainWindow::uberClose()
{
    qDebug() << "uberClose";
    uberclose();
}

void MainWindow::setupItemDemo(QCustomPlot *customPlot)
{
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    QCPGraph *graph = customPlot->addGraph();

    int n = 84;

    QVector<double> x(n), y(n);
    for (int i=0; i<n; ++i) {
        x[i] = 2400 + i;
        y[i] = i + 10;
    }

    graph->setData(x, y);
    graph->setPen(QPen(Qt::blue));
    graph->rescaleKeyAxis();

    customPlot->yAxis->setRange(-60, 50);
    customPlot->xAxis->grid()->setZeroLinePen(Qt::NoPen);

    // add the bracket at the top:
    QCPItemBracket *bracket = new QCPItemBracket(customPlot);
    bracket->left->setCoords(-8, 1.1);
    bracket->right->setCoords(8, 1.1);
    bracket->setLength(13);

}

