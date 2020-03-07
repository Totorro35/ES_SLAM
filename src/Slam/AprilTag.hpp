/**
 * @file Homography.hpp
 * @author S'tout mo vie (gilou.assistant@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <visp/vpKeyPoint.h>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <Slam/Slam.hpp>

namespace Slam
{

template <class T>
class AprilTag : public Slam<T>
{
private:
    const vpDetectorAprilTag::vpAprilTagFamily m_tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpCameraParameters m_cam;
    vpDetectorAprilTag m_detector;
    const double m_tagSize=0.1;

public:
    AprilTag(const vpImage<T> &I,const vpCameraParameters& cam) : m_cam(cam),m_detector(m_tagFamily)
    {
        this->addPoses(vpHomogeneousMatrix());
    }

    vpHomogeneousMatrix update(const vpImage<T> &I)
    {
        std::vector<vpHomogeneousMatrix> cMo_vec;
        m_detector.detect(I, m_tagSize, m_cam, cMo_vec);

        std::cout<<"Nb tag : "<<cMo_vec.size()<<std::endl;

        vpHomogeneousMatrix curTw;
        if(cMo_vec.size()==0){
            curTw = this->last();
        }else{
            curTw=cMo_vec[0];
        }
        this->addPoses(curTw.inverse());

        return curTw;
    }
};

}