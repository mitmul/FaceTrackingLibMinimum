#pragma once

#define _WINDOWS

#include <iostream>
#include <sstream>
#include <FaceTrackLib.h>
#include <NuiApi.h>
#include <opencv2\opencv.hpp>

class FaceTrack
{
public:
  FaceTrack(const int width, const int height);
  ~FaceTrack(void);

  void initialize();
  IFTResult* tracking(IFTImage* color_image, IFTImage* depth_image, FT_VECTOR3D* hint);

private:
  int w, h;

  IFTFaceTracker* face;
  IFTResult* face_result;
  bool last_track;

};

