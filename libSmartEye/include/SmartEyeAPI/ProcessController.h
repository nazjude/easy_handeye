/** 
 * @file ProcessController.h
 * @brief controller of 3D camera device
 */

#pragma once
#include "common_type.h"
#include <vector>
#include <string>


class ProjectorController;
class CameraController;
class Reconstruction;
class Suite3DVector;
class Parameter;
class ProcessControllerImpl;

namespace cv    // use the data structure of opencv
{
    class Mat;
    template <class T>
    class Point_;
    template <class T>
    class Point3_;
    typedef Point3_<float> Point3f;
    typedef Point_<int> Point2i;
}

class ProcessController
{
public:
    SMARTEYE_API ProcessController();
    SMARTEYE_API ~ProcessController();

    /**
     * @brief       initialize Device
     * @param       deviceIndex the index to devices. It's used to choose between devices when using multiple ones. The default is zero.
     * @return      result of the initialization. Please refer to common_type.h for meanings of the enumerations.
     * @attention   please call this function before operating the device.
     */
    SMARTEYE_API int initDevice(const unsigned int deviceIndex=0);

    /**
     * @brief       initialize Device by device's serial number
     * @param       deviceSN device's serial number. It's used to choose between devices when using multiple ones.
     * @return      Result of the initialization. Please refer to common_type.h for meanings of the enumerations.
     * @attention   please call this function before operating the device.
     */
    SMARTEYE_API int initDeviceBySN(const std::string &deviceSN);

    /**
     * @brief       count the number of the devices. 
     *              When using multiple sets of devices, you can call this function to count the number of the devices.
     * @return      quantity of the devices
     */
    SMARTEYE_API int countDevice();

    /**
     * @brief       Traverse all devices and output the serial number of all devices. 
     *              When using multiple sets of devices, you can call this function to get all device serial numbers of connected devices.
     * @param       deviceSNVector, the vector that will hold all device serial numbers
     */
    SMARTEYE_API void getDeviceSNList(std::vector<std::string> &deviceSNVector);

    /**
     * @brief       capture a 3D model
     * @return      result of the capturing. Please refer to common_type.h for specific meanings.
     */
    SMARTEYE_API int captureThreeModel();

    /**
     * @brief       capture multi exposure 3D model and merge
     * @return      result of the capturing. Please refer to comm_type.h for specific meanings.
     */
    SMARTEYE_API int captureThreeModelMultiExposure(std::vector<int> exposureTimes = {500, 5000, 15000});

    /**
     * @brief       get the 2D image
     * @return      2D image data, using OpenCV data structure.
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     *              Please pay attention to discern it from captureOneImage() which is to get another instant one. The picture get from get2Dimage() is to get a 2D image that is already taken for 3D capturing.
     */
    SMARTEYE_API cv::Mat get2DImage();

    /**
     * @brief       get the 2D image
     * @return      2D image data, using vector structure.
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     *              Please pay attention to discern it from captureOneImage() which is to get another instant one. The picture get from get2Dimage() is to get a 2D image that is already taken for 3D capturing.
     */
    SMARTEYE_API void get2DImage(std::vector<std::vector<unsigned char> > &image2D);

    /**
     * @brief       get the color image. only for color camera
     * @return      2D image data, using OpenCV data structure.
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API cv::Mat getColorImage();

    /**
     * @brief       get the color image. only for color camera
     * @return      2D image data, using vector structure.
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API void getColorImage(std::vector<std::vector<unsigned char> > &colorImage);

    /**
     * @brief       get unrectified images
     * @param       imageLeft: left image; imageRight: right image. using OpenCV data structure.
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API void getUnrectifiedImages(cv::Mat &imageLeft, cv::Mat &imageRight);

    /**
     * @brief       get unrectified images
     * @param       imageLeft: left image; imageRight: right image. using vector structure.
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API void getUnrectifiedImages(std::vector<std::vector<unsigned char> > &imageLeft, std::vector<std::vector<unsigned char> > &imageRight);

    /**
     * @brief       get the depth image
     * @return      depth image data, using OpenCV data structure. CV_32FC1
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API cv::Mat getDepthImage();

    /**
     * @brief       get the depth image
     * @return      depth image data, using vector structure.
     * @attention   This function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API void getDepthImage(std::vector<std::vector<float> > &depthImage);

    /**
     * @brief       the mapping from 2D image points to 3D point cloud
     * @param       entered 2D coordinates
     * @return      3D coordinates corresponding to 2D points. Unit: m. 3D coordinate (0,0,0) indicates the point is invalid.
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API cv::Point3f map2DImagePointTo3DSpace(const cv::Point2i& point2D);

    /**
     * @brief       get 3D point cloud
     * @param[out]  poinCloud point cloud data
     * @return      void
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     */
    SMARTEYE_API void getPointCloud(PointCloud_SE_Ptr& pointCloud);

    /**
     * @brief       capture an image with left camera.
     * @param[out]  image  output image, using OpenCV data structure.
     * @param       needLight   true : projector will projecting light when capturing image
     * @return      void
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     *              please pay attention to discern it from get2DImage() which is to get a 2D image that is already taken for 3D imaging. The 2D image get by captureOneImage() is another instant one.
     *              image can be color image when using color camera
     */
    SMARTEYE_API void captureOneImage(cv::Mat& image, bool needLight = true);

    /**
     * @brief       capture an image with left camera.
     * @param[out]  image  output image, using vector structure.
     * @param       needLight   true : projector will projecting light when capturing image
     * @return      void
     * @attention   this function is only valid when called after capturing a 3D model successfully.
     *              please pay attention to discern it from get2DImage() which is to get a 2D image that is already taken for 3D imaging. The 2D image get by captureOneImage() is another instant one.
     *              image in vector struction is always gray image even using color camera
     */
    SMARTEYE_API void captureOneImage(std::vector<std::vector<unsigned char> > &image, bool needLight = true);

    /**
     * @brief       capture images.
     * @param[out]  imageLeft  output left image, using OpenCV data structure.
     * @param[out]  imageRight  output right image, using OpenCV data structure.
     * @return      void
     * @attention   please pay attention to discern it from getUnrectifiedImages() which is to get a 2D image that is already taken for 3D imaging. The 2D image get by captureUnrectifiedImages() is another instance.
     *              imageLeft and imageRight can be color image when using color camera
     */
    SMARTEYE_API void captureUnrectifiedImages(cv::Mat &imageLeft, cv::Mat &imageRight);

    /**
     * @brief       capture images.
     * @param[out]  imageLeft  output left image, using vector structure.
     * @param[out]  imageRight  output right image, using vector structure.
     * @return      void
     * @attention   please pay attention to discern it from getUnrectifiedImages() which is to get a 2D image that is already taken for 3D imaging. The 2D image get by captureUnrectifiedImages() is another instance.
     *              imageLeft and imageRight in vector struction are always gray image even using color camera
     */
    SMARTEYE_API void captureUnrectifiedImages(std::vector<std::vector<unsigned char> > &imageLeft, std::vector<std::vector<unsigned char> > &imageRight);

    /**
     * @brief       set the exposure time for capturing 2D image.
     * @param       exposureTime2D  set the exposure time. Unit: Usec (no more than 1,000,000)
     * @return      the result of exposure time setting. True stands for success while false for failure.
     * @attention   please note that this exposure time setting is not for 3D.
     */
    SMARTEYE_API bool setExposureTime2D(int exposureTime2D);

    /**
     * @brief       set the exposure time for getting 3D capturing.
     * @param       exposureTime3D  set the exposure time. Unit: Usec (no more than 1,000,000)
     * @return      the result of exposure time setting. True stands for success while false for failure.
     * @attention   please note that this exposure time setting is not for 2D.
     */
    SMARTEYE_API bool setExposureTime3D(int exposureTime3D);

    /**
     * @brief       set the maxCoeff for reconstruction.
     * @param       maxCoeff. Range from 0.1 to 1.0
     * @return      the result of maxCoeff setting. True stands for success while false for failure.
     */
    SMARTEYE_API bool setMaxCoeff(float maxCoeff);

    /**
     * @brief       set the filter flag
     * @param       canFilter: true : add filter operation. false : no filter operation.
     */
    SMARTEYE_API void setFilterFlag(bool canFilter);
    
    void receiveCaptureImages();

    /**
     * @brief       stop 3D capturing
     * @param       void
     * @return      void
     * @attention   no need to call for user
     */
    SMARTEYE_API void stopCaptureThreeModel();

    /**
     * @brief       switch device
     * @param       deviceIndex  index of the device to choose from.
     * @return      the result of switching. Please refer to common_type.h for meanings of the enumerations.
     */
    SMARTEYE_API int changeDevice(unsigned int deviceIndex);

    /**
     * @brief       switch device by serial number
     * @param       deviceSN  serial number of the device to choose from.
     * @return      the result of switching. Please refer to common_type.h for meanings of the enumerations.
     */
    SMARTEYE_API int changeDeviceBySN(std::string &deviceSN);

    /**
    * @brief       connect projector again
    * @return      true: operation is successful. false: operation is failed
    */
    SMARTEYE_API bool connectProjectorAgain();


    //////////////////////////////////////////////////////////////////////////
    //// following getting functions should be called after initDevice()
    
    /** 
     * @return      exposureTime2D. Unit: Usec (no more than 1,000,000)
     */
    SMARTEYE_API int getExposureTime2D();
    
    /**
     * @return      exposureTime3D. Unit: Usec (no more than 1,000,000)
     */
    SMARTEYE_API int getExposureTime3D();
    
    SMARTEYE_API float getMaxCoeff();
    SMARTEYE_API void getZRange(float &minZ, float &maxZ);
    SMARTEYE_API void getXYRange(int &minX, int &maxX, int &minY, int &maxY);
    SMARTEYE_API void getResolution(int &resolutionWidth, int &resolutionHeight);
    
    SMARTEYE_API std::string getCalibrationFile();
    /**
     * @brief       set calibration file.
     * @param       calibrationFilePath: the path of calibration file.
     * @return      false: calibrationFilePath does not exist. true: operation executes successfully.
     * @attention   This function should be called after initDevice()
     */
    SMARTEYE_API bool setCalibrationFile(std::string calibrationFilePath);

    SMARTEYE_API void setNonTriggerMode(bool isNonTriggerMode=true);
    /**
     * @brief       get new images in non-trigger mode.
     * @param       leftImage: image of left camera; rightImage: image of right camera.
     * @return      false: no new images. true: has new images
     * @attention   this function should be called after setNonTriggerMode(true)
     */
    SMARTEYE_API bool getNonTriggerModeImages(cv::Mat &leftImage, cv::Mat &rightImage);
    

private:    
    void timeoutOperation();
    int generateSerialNumber();
    int connectDevice(unsigned int deviceIndex);
    void disConnectDevice();
    
    ProjectorController*    pProjector;
    CameraController*       pCamera;
    Reconstruction*         pReconstructor;
    Suite3DVector*          pSuite3DVector;
    Parameter*              pParameter; 

    ProcessControllerImpl   *impl;
};

