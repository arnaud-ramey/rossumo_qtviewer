/*!
  \\file        mainwindow.cpp
  \\author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \\date        2016/7/8

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
  QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(imageReceived()), this, SLOT(refresh_rgb()));
  qnode->init(this);
}

MainWindow::~MainWindow()
{
  qnode->terminate();
  delete ui;
}

void MainWindow::refresh_rgb() {
  //ROS_INFO("refresh_rgb()");
  qnode->_mutex.lock();
  ui->rgbDisplay->setPixmap(QPixmap::fromImage(qnode->_rgb));
  qnode->_mutex.unlock();
}
