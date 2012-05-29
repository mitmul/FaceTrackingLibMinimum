#include "FaceTrack.h"


FaceTrack::FaceTrack(int width, int height)
  : w(width)
  , h(height)
  , face_result(NULL)
  , last_track(false)
{

}


FaceTrack::~FaceTrack(void)
{
  face_result->Release();
  face->Release();
}

void FaceTrack::initialize(void)
{
  try
  {
    HRESULT hr;
    
    face = FTCreateFaceTracker();
    if(!face)
      std::cerr << "CreateFaceTracker Failed" << std::endl;

    FT_CAMERA_CONFIG color_config;
    color_config.Width = w;
    color_config.Height = h;
    color_config.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;
   
    FT_CAMERA_CONFIG depth_config;
    depth_config.Width = w;
    depth_config.Height = h;
    depth_config.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;


    hr = face->Initialize(&color_config, &depth_config, NULL, NULL);
    if(FAILED(hr))
      std::cerr << "FaceTracker Initialize Failed" << std::endl;
    
    hr = face->CreateFTResult(&face_result);
    if(FAILED(hr) || !face_result)
      std::cerr << "FaceTracker Result Create Failed" << std::endl;
  }
  catch(std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }
}

IFTResult* FaceTrack::tracking(IFTImage* color_image, IFTImage* depth_image, FT_VECTOR3D* hint)
{
  try
  {
    HRESULT hr;
    FT_SENSOR_DATA sensor_data;
    sensor_data.pVideoFrame = color_image;
    sensor_data.pDepthFrame = depth_image;
    sensor_data.ZoomFactor = 1.0f;

    POINT view_offset = {0, 0};
    sensor_data.ViewOffset = view_offset;

    FT_VECTOR3D hints[2] = {hint[0], hint[1]};

    SYSTEMTIME time;
    GetLocalTime(&time);

    std::stringstream time_stamp;
    time_stamp << time.wMonth << ":"
               << time.wDay << ":" 
               << time.wHour << ":"
               << time.wMinute << ":" 
               << time.wSecond << ":"
               << time.wMilliseconds;

    if(last_track)
    {
      hr = face->ContinueTracking(&sensor_data, hints, face_result);
      if(FAILED(hr) || FAILED(face_result->GetStatus()))
      {
        std::cerr << time_stamp.str() << " Face tracking cannot continue" << std::endl;
        last_track = false;
      }
    }
    else
    {
      hr = face->StartTracking(&sensor_data, NULL, hints, face_result);
      if(SUCCEEDED(hr) && SUCCEEDED(face_result->GetStatus()))
      {
        std::cout << time_stamp.str() << " Face tracking started successfully" << std::endl;
        last_track = true;
      }
      else
      {
        std::cerr << time_stamp.str() << " Face tracking cannot started" << std::endl;
        last_track = false;
      }
    }

    // Tracking‚Å‚«‚Ä‚¢‚½‚ç
    if(SUCCEEDED(hr) && SUCCEEDED(face_result->GetStatus()))
      return face_result;
    else
      return NULL;
  }
  catch(std::exception &e)
  {
    std::cerr << "FaceTrack::tracking" << std::endl << e.what() << std::endl;
  }
}