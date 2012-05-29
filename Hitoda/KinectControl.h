#pragma once

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <FaceTrackLib.h>
#include <NuiApi.h>
#include <opencv2\opencv.hpp>

#include "FaceTrack.h"

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

class KinectControl
{
public:
  KinectControl(void);
  ~KinectControl(void);

  void initialize();
  void run();

  

private:
  INuiSensor *kinect;
  HANDLE imageStreamHandle;
  HANDLE depthStreamHandle;
  HANDLE streamEvent;

  DWORD width;
  DWORD height;

  FaceTrack *face;
  bool skeleton_tracked[NUI_SKELETON_COUNT];
  IFTImage* color_image;
  IFTImage* depth_image;
  FT_VECTOR3D neck_point[NUI_SKELETON_COUNT];
  FT_VECTOR3D head_point[NUI_SKELETON_COUNT];
  FT_VECTOR3D hint_points[2];

  void createInstance();
  void setRgbImage(cv::Mat& image);
  void setDepthImage(cv::Mat& image);
  void setSkeleton(cv::Mat& image);
  void setJoint(cv::Mat& image, int joint, Vector4 position);
  HRESULT setClosestHint(FT_VECTOR3D* hint_3d);

  cv::Mat rgbImage;
  cv::Mat depthImage;
  cv::Mat clothImage;
  cv::Mat userMask;

  inline void ERROR_CHECK(HRESULT ret)
  {
    std::stringstream ss;
    ss << ret << std::endl;

    if(ret != S_OK)
      throw std::runtime_error(ss.str().c_str());
  }
};

