#ifndef ROAD_LINES_DISPLAY_H
#define ROAD_LINES_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>
#include <custom_msgs/RoadLines.h>
#endif

namespace rviz
{
class ColorProperty;
class FloatProperty;
}  // namespace rviz

namespace selfie_rviz
{
class RoadLinesVisual;

class RoadLinesDisplay : public rviz::MessageFilterDisplay<custom_msgs::RoadLines>
{
  Q_OBJECT
public:
  RoadLinesDisplay();
  virtual ~RoadLinesDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorsAndAlpha();
  void updateRenderingRange();

private:
  void processMessage(const custom_msgs::RoadLines::ConstPtr& msg);

  RoadLinesVisual* visual_;

  // User-editable property variables.
  rviz::ColorProperty* boundaries_color_property_;
  rviz::ColorProperty* centerline_color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* rendering_start_property_;
  rviz::FloatProperty* rendering_end_property_;
  rviz::FloatProperty* rendering_step_property_;
};

}  // end namespace selfie_rviz

#endif  // ROAD_LINES_DISPLAY_H
