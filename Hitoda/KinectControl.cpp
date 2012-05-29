#include "KinectControl.h"


KinectControl::KinectControl()
{
}


KinectControl::~KinectControl()
{
  if(kinect != 0){
    kinect->NuiShutdown();
    kinect->Release();
  }
}

void KinectControl::initialize()
{
  createInstance();

  // Kinect�̐ݒ������������
  ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR |
                                    NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |
                                    NUI_INITIALIZE_FLAG_USES_SKELETON));

  // RGB�J����������������
  ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle));

  // �����J����������������
  ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));

  // �X�P���g��������������
  ERROR_CHECK(kinect->NuiSkeletonTrackingEnable(0,
    NUI_SKELETON_TRACKING_FLAG_SUPPRESS_NO_FRAME_DATA |
    NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE |
    NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT));

  // Near���[�h
  ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE));

  // �t���[���X�V�C�x���g�̃n���h�����쐬����
  streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
  ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

  // �w�肵���𑜓x�́A��ʃT�C�Y���擾����
  NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height);

  // FaceTrack���̉�
  face = new FaceTrack(width, height);
  face->initialize();
  color_image = FTCreateImage();
  depth_image = FTCreateImage();
  color_image->Allocate(width, height, FTIMAGEFORMAT_UINT8_B8G8R8X8);
  depth_image->Allocate(width, height, FTIMAGEFORMAT_UINT16_D13P3);
  hint_points[0] = hint_points[1] = FT_VECTOR3D(0, 0, 0);
}

void KinectControl::run()
{
  // ���C�����[�v
  while(1)
  {
    try
    {
      // �f�[�^�̍X�V��҂�
      DWORD ret = ::WaitForSingleObject(streamEvent, INFINITE);
      ResetEvent(streamEvent);

      setRgbImage(rgbImage);
      setDepthImage(depthImage);
      setSkeleton(depthImage);

      // FaceTrackingLib
      setClosestHint(hint_points);
      IFTResult* result = face->tracking(color_image, depth_image, hint_points);

      if(result != NULL)
      {
        FT_VECTOR2D* points;
        UINT point_num;
        result->Get2DShapePoints(&points, &point_num);

        for(int i = 0; i < point_num; ++i)
        {
          cv::circle(rgbImage, cv::Point(points[i].x, points[i].y), 1, cv::Scalar(0, 0, 255), -1);
        }
      }

      cv::imshow("RGB Camera", rgbImage);
      cv::imshow("Depth Camera", depthImage);

      // �I���̂��߂̃L�[���̓`�F�b�N���A�\���̂��߂̃E�F�C�g
      int key = cv::waitKey(10);
      if(key == 'q'){
        break;
      }
    }
    catch(std::exception &e)
    {
      std::cerr << e.what() << std::endl;
    }
  }
}

HRESULT KinectControl::setClosestHint(FT_VECTOR3D* hint_3d)
{
  int selectedSkeleton = -1;
  float smallestDistance = 0;

  if(!hint_3d)
  {
    return(E_POINTER);
  }

  if(hint_3d[1].x == 0 && hint_3d[1].y == 0 && hint_3d[1].z == 0)
  {
    // Get the skeleton closest to the camera
    for(int i = 0 ; i < NUI_SKELETON_COUNT ; i++)
    {
      if (skeleton_tracked[i] && (smallestDistance == 0 || head_point[i].z < smallestDistance))
      {
        smallestDistance = head_point[i].z;
        selectedSkeleton = i;
      }
    }
  }
  else
  {
    // Get the skeleton closest to the previous position
    for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
    {
      if (skeleton_tracked[i])
      {
        float d = abs(head_point[i].x - hint_3d[1].x) +
          abs(head_point[i].y - hint_3d[1].y) +
          abs(head_point[i].z - hint_3d[1].z);
        if (smallestDistance == 0 || d < smallestDistance)
        {
          smallestDistance = d;
          selectedSkeleton = i;
        }
      }
    }
  }
  if (selectedSkeleton == -1)
  {
    return E_FAIL;
  }

  hint_3d[0] = neck_point[selectedSkeleton];
  hint_3d[1] = head_point[selectedSkeleton];

  return S_OK;
}

void KinectControl::createInstance()
{
  // �ڑ�����Ă���Kinect�̐����擾����
  int count = 0;
  ERROR_CHECK(NuiGetSensorCount(&count));
  if(count == 0){
    throw std::runtime_error("Kinect ��ڑ����Ă�������");
  }

  // �ŏ���Kinect�̃C���X�^���X���쐬����
  ERROR_CHECK(NuiCreateSensorByIndex(0, &kinect));

  // Kinect�̏�Ԃ��擾����
  HRESULT status = kinect->NuiStatus();
  if(status != S_OK){
    throw std::runtime_error("Kinect �����p�\�ł͂���܂���");
  }
}

void KinectControl::setRgbImage(cv::Mat& image)
{
  try
  {
    // RGB�J�����̃t���[���f�[�^���擾����
    NUI_IMAGE_FRAME imageFrame = { 0 };
    ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame));

    // �摜�f�[�^���擾����
    NUI_LOCKED_RECT colorData;
    imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

    // �摜�f�[�^���R�s�[����
    image = cv::Mat(height, width, CV_8UC4, colorData.pBits);

    // FaceTrack�փR�s�[
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &colorData, NULL, 0);
    memcpy(color_image->GetBuffer(), PBYTE(colorData.pBits), MIN(color_image->GetBufferSize(), UINT(texture->BufferLen())));

    // �t���[���f�[�^���������
    ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame));
  }
  catch(std::exception& ex)
  {
    std::cout << "KinectControl::setRgbImage" << std::endl << ex.what() << std::endl;
  }
}

void KinectControl::setDepthImage(cv::Mat& image)
{
  try
  {
    userMask = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    image = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));

    // �����J�����̃t���[���f�[�^���擾����
    NUI_IMAGE_FRAME depthFrame = { 0 };
    ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame));

    // �����f�[�^���擾����
    NUI_LOCKED_RECT depthData = { 0 };
    depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

    USHORT* depth = (USHORT*)depthData.pBits;
    for(int i = 0; i < (depthData.size / sizeof(USHORT)); ++i){
      USHORT distance = NuiDepthPixelToDepth(depth[i]);
      USHORT player = NuiDepthPixelToPlayerIndex(depth[i]);

      LONG depthX = i % width;
      LONG depthY = i / width;
      LONG colorX = depthX;
      LONG colorY = depthY;

      // �����J�����̍��W���ARGB�J�����̍��W�ɕϊ�����
      kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, depthX , depthY, 0, &colorX, &colorY);

      // ���[�U�s�N�Z�����ǂ������}�X�N�摜�ɋL�^
      if(player != 0){
        int index = (colorY * width) + colorX;
        userMask.data[index] = 255;
      }

      // 8bit�ɂ��ăf�v�X�摜�Ɋi�[
      int index = (colorY * width) + colorX;
      image.data[index] = distance / 8192.0 * 255;
    }

    // FaceTrack�փR�s�[
    INuiFrameTexture* texture = depthFrame.pFrameTexture;
    texture->LockRect(0, &depthData, NULL, 0);
    memcpy(depth_image->GetBuffer(), PBYTE(depthData.pBits), MIN(depth_image->GetBufferSize(), UINT(texture->BufferLen())));

    // �t���[���f�[�^���������
    ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));
  }
  catch(std::exception& ex)
  {
    std::cout << "KinectControl::setDepthImage" << std::endl << ex.what() << std::endl;
  }
}

void KinectControl::setSkeleton(cv::Mat& image)
{ 
  try
  {
    // �X�P���g���̃t���[�����擾����
    NUI_SKELETON_FRAME skeletonFrame = {0};
    ERROR_CHECK(kinect->NuiSkeletonGetNextFrame(0, &skeletonFrame));

    for(int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
      head_point[i] = neck_point[i] = FT_VECTOR3D(0, 0, 0);
      skeleton_tracked[i] = false;

      NUI_SKELETON_DATA& skeletonData = skeletonFrame.SkeletonData[i];
      if(skeletonData.eTrackingState == NUI_SKELETON_TRACKED)
      {
        skeleton_tracked[i] = true;

        // �e�W���C���g���Ƃ�
        for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
        {
          if(skeletonData.eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED)
          {
            setJoint(image, j, skeletonData.SkeletonPositions[j]);

            if(j == NUI_SKELETON_POSITION_SHOULDER_CENTER)
            {
              neck_point[i].x = skeletonFrame.SkeletonData[i].SkeletonPositions[j].x;
              neck_point[i].y = skeletonFrame.SkeletonData[i].SkeletonPositions[j].y;
              neck_point[i].z = skeletonFrame.SkeletonData[i].SkeletonPositions[j].z;
            }

            if(j == NUI_SKELETON_POSITION_HEAD)
            {
              head_point[i].x = skeletonFrame.SkeletonData[i].SkeletonPositions[j].x;
              head_point[i].y = skeletonFrame.SkeletonData[i].SkeletonPositions[j].y;
              head_point[i].z = skeletonFrame.SkeletonData[i].SkeletonPositions[j].z;
            }
          }
        }
      }
      else if(skeletonData.eTrackingState == NUI_SKELETON_POSITION_ONLY)
      {
        setJoint(image, -1, skeletonData.Position);
      }
    }
  }
  catch(std::exception& ex)
  {
    std::cout << "KinectControl::setSkeleton" << std::endl << ex.what() << std::endl;
  }
}

void KinectControl::setJoint(cv::Mat& image, int joint, Vector4 position)
{
  try {
    FLOAT depthX = 0, depthY = 0;
    NuiTransformSkeletonToDepthImage(position, &depthX, &depthY, CAMERA_RESOLUTION);

    LONG colorX = 0;
    LONG colorY = 0;

    kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX , (LONG)depthY, 0, &colorX, &colorY);

    cv::circle(image, cv::Point(colorX, colorY), 5, cv::Scalar(255), -1);
  }
  catch(std::exception& ex){
    std::cout << "KinectControl::setJoint" << std::endl << ex.what() << std::endl;
  }
}