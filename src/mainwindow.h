/*!
  \\file        mainwindow.h
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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QMutex>
#ifndef Q_MOC_RUN
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "ui_mainwindow.h"
#endif

class QNode; // forward declaration

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QNode *node, QWidget *parent = 0);
  virtual ~MainWindow();

  //private:
  Ui::MainWindow *ui;
  QNode *qnode;

public Q_SLOTS:
  void refresh_rgb();
};

////////////////////////////////////////////////////////////////////////////////

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int &argc, char **argv) {
    _init_argc = argc;
    _init_argv = argv;
  }

  ~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    delete _it;
  }

  void init(MainWindow* parent) {
    _parent = parent;
    ros::init(_init_argc, _init_argv, "foo");
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros::NodeHandle nh_public;
    _it = new image_transport::ImageTransport(nh_public);
    _rgb_sub = _it->subscribe("rgb", 1, &QNode::rgb_cb, this);
    start();
  }

  virtual void run() {
    ros::spin();
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

  void rgb_cb(const sensor_msgs::ImageConstPtr& msg) {
    //ROS_INFO("rgb_cb(%ix%i, %s)", msg->width, msg->height, msg->encoding.c_str());
    _mutex.lock();
    _mydata = msg->data; // deep copy
    _rgb = QImage(_mydata.data(), msg->width, msg->height, QImage::Format_RGB888);
    _mutex.unlock();
    Q_EMIT imageReceived();
  }

  // ROS stuff
  image_transport::ImageTransport* _it;
  image_transport::Subscriber _rgb_sub;
  ros::Subscriber _batt_sub;

  // ROS - Qt interfaces
  std::vector<uchar> _mydata;
  QMutex _mutex;
  int _init_argc;
  char** _init_argv;

  // Qt stuff
  MainWindow* _parent;
  QImage _rgb;

Q_SIGNALS:
  void imageReceived();
  void rosShutdown();
};

#endif // MAINWINDOW_H
