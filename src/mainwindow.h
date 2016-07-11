/*!
  \file        mainwindow.h
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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QMutex>
#ifndef Q_MOC_RUN
#include <image_transport/image_transport.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
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
  void refresh_alert(QString alert) {
    ui->alerts->setText(alert);
  }
  void refresh_battery(uchar battery) {
    ui->battery->setValue(battery);
  }
  void refresh_link(uchar link) {
    ui->linkQuality->setValue(link);
  }
  void refresh_posture(QString posture) {
    ROS_INFO("refresh_posture(%s)", posture.toStdString().c_str());
    ui->posture->setText(posture);
  }
  void refresh_robot_name(QString robot_name) {
    ROS_INFO("refresh_robot_name(%s)", robot_name.toStdString().c_str());
    ui->robotName->setText(robot_name);
  }
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
    _alert_sub = nh_public.subscribe("alert", 1, &QNode::alert_cb, this);
    _battery_sub = nh_public.subscribe("battery_percentage", 1, &QNode::battery_cb, this);
    _link_sub = nh_public.subscribe("link_quality", 1, &QNode::link_cb, this);
    _posture_sub = nh_public.subscribe("posture", 1, &QNode::posture_cb, this);
    _it = new image_transport::ImageTransport(nh_public);
    _rgb_sub = _it->subscribe("rgb", 1, &QNode::rgb_cb, this);

    Q_EMIT robot_nameReceived(QString::fromStdString(nh_public.getNamespace()));
    start();
  }

  virtual void run() {
    ros::spin();
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

  void alert_cb(const std_msgs::StringConstPtr& msg) {
    ROS_INFO("alert_cb()");
    Q_EMIT alertReceived(QString::fromStdString(msg->data));
  }
  void battery_cb(const std_msgs::UInt8ConstPtr& msg) {
    ROS_INFO("battery_cb()");
    Q_EMIT batteryReceived(msg->data);
  }
  void link_cb(const std_msgs::UInt8ConstPtr& msg) {
    ROS_INFO("link_cb()");
    Q_EMIT linkReceived(msg->data);
  }
  void posture_cb(const std_msgs::StringConstPtr& msg) {
    ROS_INFO("posture_cb()");
    Q_EMIT postureReceived(QString::fromStdString(msg->data));
  }
  void rgb_cb(const sensor_msgs::ImageConstPtr& msg) {
    //ROS_INFO("rgb_cb(%ix%i, %s)", msg->width, msg->height, msg->encoding.c_str());
    _mutex.lock();
    _rgbdata = msg->data; // deep copy
    // https://doc.qt.io/qt-4.8/qimage.htmlhttps://doc.qt.io/qt-4.8/qimage.html
    if (msg->encoding == "bgr8")
      _rgb = QImage(_rgbdata.data(), msg->width, msg->height, QImage::Format_RGB888).rgbSwapped();
    else
      _rgb = QImage(_rgbdata.data(), msg->width, msg->height, QImage::Format_RGB888);
    _mutex.unlock();
    Q_EMIT rgbReceived();
  }

  // ROS stuff
  image_transport::ImageTransport* _it;
  image_transport::Subscriber _rgb_sub;
  ros::Subscriber _alert_sub;
  ros::Subscriber _battery_sub;
  ros::Subscriber _link_sub;
  ros::Subscriber _posture_sub;

  // ROS - Qt interfaces
  std::vector<uchar> _rgbdata;
  QImage _rgb;
  QMutex _mutex;
  int _init_argc;
  char** _init_argv;

  // Qt stuff
  MainWindow* _parent;

Q_SIGNALS:
  void alertReceived(QString alert);
  void batteryReceived(uchar battery);
  void linkReceived(uchar link);
  void postureReceived(QString posture);
  void robot_nameReceived(QString robot_name);
  void rgbReceived();
  void rosShutdown();
};

#endif // MAINWINDOW_H
