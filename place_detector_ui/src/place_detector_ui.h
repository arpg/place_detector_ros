#ifndef PLACEDETECTORUI_H
#define PLACEDETECTORUI_H

#include "ros/ros.h"
#include "place_detector/PlaceLabel.h"

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

class place_detector_ui : public rviz::Panel {
  Q_OBJECT
 public:
  place_detector_ui(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:
  void on_label_junction_click();
  void on_label_room_click();
  void on_label_corridor_click();
  void on_label_bend_click();
  void on_label_skip_click();
  void on_label_undo_click();
  void on_label_done_click();
 protected Q_SLOTS:

 protected:
  QPushButton* button_label_junction;
  ros::ServiceClient client_label;

  QPushButton* button_label_room;
  QPushButton* button_label_corridor;
  QPushButton* button_label_bend;
  QPushButton* button_label_skip;
  QPushButton* button_label_undo;
  QPushButton* button_label_done;

  ros::NodeHandle nh;
};



#endif  
