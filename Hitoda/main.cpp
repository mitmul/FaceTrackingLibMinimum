#include "KinectControl.h"

int main(void)
{
  try
  {
    KinectControl kinect;
    kinect.initialize();
    kinect.run();
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}