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
    connect(ui->mButtonRefresh, SIGNAL(clicked()), this, SLOT(uberRefresh()));

    // setup a timer that repeatedly calls MainWindow::bracketDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(bracketDataSlot()));
    dataTimer.start(11); // Interval 0 means to refresh as fast as possible
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
    int n = FREQ_WIDTH;

    static QVector<double> sMax(n);
    static QVector<int> sDisTime(n);

    if(mUberQueue.size() > n) {

        //qDebug() << "updataPlot";
        ui->mQCustomPlot->removeGraph(1);
        ui->mQCustomPlot->addGraph();
        QPen pen;
        pen.setStyle(Qt::DotLine);
        pen.setWidth(2);
        pen.setColor(Qt::red);
        ui->mQCustomPlot->graph(1)->setPen(pen);

        QVector<double> x(n), y(n);

        for (int i=0; i<n; ++i) {
            quint16 data = mUberQueue.dequeue();
            //quint16 data = 0;
            int freq = (int) (data & 0xff);
            signed char dbm = (signed char) (data >> 8) ;
            y[freq] = (double)dbm - 54;
            x[freq] = 2400 + freq;

            {
                //qDebug() << "x " << freq << "y " << y[freq] << "yx " << yyMax[freq] << " " << xxDisTime[freq];
                int *pDis = sDisTime.data();
                double *pMax = sMax.data();

                if (pDis[freq]-- < 0) {
                    pMax[freq] = -120.0;
                    pDis[freq] = -1;
                }

                if (y[freq] > pMax[freq]) {
                    pMax[freq] = y[freq];
                    pDis[freq] = 60;
                    //ui->mQCustomPlot->graph(1)->addData(x[freq], sMax[freq]);
                }

                if (pDis[freq] > 0) {
                    ui->mQCustomPlot->graph(1)->addData(x[freq], pMax[freq]);
                }
            }
        }

        ui->mQCustomPlot->graph(0)->setData(x, y);

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
            QString("%1 FPS, Total Data points: %2, Queue.size %3")
            .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
            .arg(ui->mQCustomPlot->graph(0)->data()->size())
            .arg(mUberQueue.size()), 0);
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

void MainWindow::uberRefresh()
{
    qDebug() << "uberRefresh";
    mUberQueue.clear();
}

void MainWindow::setupItemDemo(QCustomPlot *customPlot)
{
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    int n = FREQ_WIDTH;

    QVector<double> x(n), y(n);
    for (int i=0; i<n; ++i) {
        x[i] = 2400 + i;
        y[i] = -100;
    }

    QPen pen;

    customPlot->axisRect()->setBackground(QPixmap("./AppoTech_logo_B_H.jpg"));
    customPlot->axisRect()->setBackgroundScaled(0);
    customPlot->axisRect()->setBackgroundScaledMode(Qt::KeepAspectRatioByExpanding);

    // add two new graphs and set their look:
    customPlot->addGraph();
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(1);
    pen.setColor(Qt::blue);
    customPlot->graph(0)->setPen(pen); // line color blue for first graph
    customPlot->graph(0)->setData(x, y);

    customPlot->addGraph();
    pen.setStyle(Qt::DashLine);
    pen.setWidth(2);
    pen.setColor(QColor(0xff,0xff,0));
    customPlot->graph(1)->setPen(pen);
    customPlot->graph(1)->setData(x, y);

    customPlot->yAxis->setRange(-100, -80);
    customPlot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    customPlot->xAxis->setRange(2400, 2490);

    customPlot->xAxis->grid()->setSubGridVisible(true);
    customPlot->xAxis->setScaleType(QCPAxis::stLinear);

    customPlot->yAxis->grid()->setSubGridVisible(true);
    customPlot->yAxis->setScaleType(QCPAxis::stLinear);

}

