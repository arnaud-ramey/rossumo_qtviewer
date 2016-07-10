/*!
  \file        mainwindow.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/7/8

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________
 */
#include "mainwindow.h"

MainWindow::MainWindow(QNode *node, QWidget *parent) :
  QMainWindow(parent),
  qnode(node),
  ui(new Ui::MainWindow) {
  ui->setupUi(this);
  // https://doc.qt.io/qt-4.8/signalsandslots.html
  QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(alertReceived(QString)), this, SLOT(refresh_alert(QString)));
  QObject::connect(qnode, SIGNAL(batteryReceived(uchar)), this, SLOT(refresh_battery(uchar)));
  QObject::connect(qnode, SIGNAL(linkReceived(uchar)), this, SLOT(refresh_link(uchar)));
  QObject::connect(qnode, SIGNAL(postureReceived(QString)), this, SLOT(refresh_posture(QString)));
  QObject::connect(qnode, SIGNAL(robot_nameReceived(QString)), this, SLOT(refresh_robot_name(QString)));
  QObject::connect(qnode, SIGNAL(rgbReceived()), this, SLOT(refresh_rgb()));
  qnode->init(this);
}

MainWindow::~MainWindow()
{
  qnode->terminate();
  delete ui;
}

void MainWindow::refresh_rgb() {
  qnode->_mutex.lock();
  ui->rgbDisplay->setPixmap(QPixmap::fromImage(qnode->_rgb));
  qnode->_mutex.unlock();
}
