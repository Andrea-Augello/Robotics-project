#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>

int main() {
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  WbDeviceTag display = wb_robot_get_device("display");

  WbImageRef ir = wb_display_image_load(display, "../../../Media/Image/warning.jpg");
  wb_display_image_paste(display, ir, 0, 0, false);
 
  wb_robot_cleanup();
  return 0;
}
