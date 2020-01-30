/**
 * @file Slam.hpp
 * @author S'tout mo vie (gilou.assistant@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <visp/vpImage.h>
#include <Loader/Tools.hpp>
#include <visp/vpPoseVector.h>

namespace Slam
{

template <class T>
class Slam
{
private:
    std::vector<vpHomogeneousMatrix> m_poses;

protected:

    void addPoses(const vpPoseVector &pose)
    {
        m_poses.push_back(vpHomogeneousMatrix(pose));
    }

    void addPoses(const vpHomogeneousMatrix &pose)
    {
        m_poses.push_back(pose);
    }

    const vpHomogeneousMatrix& last() const{
        return m_poses[m_poses.size()-1];
    }

public:
    /**
     * @brief Update function (Called for each frame of video)
     * 
     * @param i 
     */
    virtual vpHomogeneousMatrix update(const vpImage<T> &I) = 0;

    virtual ~Slam() {}

    /**
     * @brief Save all poses
     * 
     * @return std::string 
     */
    virtual void saveJson(std::string filename) const
    {
        std::string file = "";
        int cpt = 0;
        for (auto var : m_poses)
        {
            file += "#" + std::to_string(cpt) + "\n";
            for (size_t i = 0; i < 4; i++)
            {
                for (size_t j = 0; j < 4; j++)
                {
                    file+=std::to_string(var[i][j])+" ";
                }
                file+="\n";
            }

            cpt++;
        }

        Loader::saveFile(filename, file);
    }
};

} // namespace Slam
