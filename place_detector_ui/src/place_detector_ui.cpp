#include "place_detector_ui.h"

place_detector_ui::place_detector_ui(QWidget* parent) : rviz::Panel(parent) 
{
  client_label = nh.serviceClient<place_detector::PlaceLabel>("place_detector/label_in");

  QVBoxLayout* v_box_layout = new QVBoxLayout;

  button_label_junction = new QPushButton;
  button_label_room = new QPushButton;
  button_label_corridor = new QPushButton;
  button_label_bend = new QPushButton;
  button_label_skip = new QPushButton;
  button_label_undo = new QPushButton;
  button_label_done = new QPushButton;

  button_label_junction->setText("Junction");
  button_label_room->setText("Room");
  button_label_corridor->setText("Corridor");
  button_label_bend->setText("Bend");
  button_label_skip->setText("Skip");
  button_label_undo->setText("Undo");
  button_label_done->setText("Done");

  v_box_layout->addWidget(button_label_junction);
  v_box_layout->addWidget(button_label_room);
  v_box_layout->addWidget(button_label_corridor);
  v_box_layout->addWidget(button_label_bend);
  v_box_layout->addWidget(button_label_skip);
  v_box_layout->addWidget(button_label_undo);
  v_box_layout->addWidget(button_label_done);

  //QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  //QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  //QLabel* text_label_ptr = new QLabel("Frontier ID:");

  //global_id_line_edit = new QLineEdit();

  //global_hbox_layout->addWidget(text_label_ptr);
  //global_hbox_layout->addWidget(global_id_line_edit);
  //global_hbox_layout->addWidget(button_global_planner);
  //global_vbox_layout->addLayout(global_hbox_layout);
  //v_box_layout->addLayout(global_vbox_layout);

  setLayout(v_box_layout);

  connect(button_label_junction, SIGNAL(clicked()), this, SLOT(on_label_junction_click()));
  connect(button_label_room, SIGNAL(clicked()), this, SLOT(on_label_room_click()));
  connect(button_label_corridor, SIGNAL(clicked()), this, SLOT(on_label_corridor_click()));
  connect(button_label_bend, SIGNAL(clicked()), this, SLOT(on_label_bend_click()));
  connect(button_label_skip, SIGNAL(clicked()), this, SLOT(on_label_skip_click()));
  connect(button_label_undo, SIGNAL(clicked()), this, SLOT(on_label_undo_click()));
  connect(button_label_done, SIGNAL(clicked()), this, SLOT(on_label_done_click()));

}

void place_detector_ui::on_label_junction_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "junction";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_room_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "room";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_corridor_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "corridor";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_bend_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "bend";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_skip_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "skip";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_undo_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "undo";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}

void place_detector_ui::on_label_done_click() 
{
  place_detector::PlaceLabel srv;
  srv.request.label = "done";
  if (!client_label.call(srv)) 
  {
    ROS_ERROR("place_detector_ui: Service call failed: %s", client_label.getService().c_str());
  }
}


void place_detector_ui::save(rviz::Config config) const {
  rviz::Panel::save(config);
}
void place_detector_ui::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(place_detector_ui, rviz::Panel)