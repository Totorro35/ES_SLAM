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

#include <Slam/Slam.hpp>

namespace Slam
{

template <class T>
class Homography : public Slam<T>
{
private:

public:
    Homography() {}

    void update(const vpImage<T>& i)
    {
        vpPoseVector p;
        this->addPoses(p);
    }

};

} // namespace Slam
