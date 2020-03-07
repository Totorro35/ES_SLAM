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
#include <Slam/Slam.hpp>
#include <visp/vpImageIo.h>

namespace Slam
{

template <class T>
class Pokemon : public Slam<T>
{
private:
    const std::string detectorName = "ORB";
    const std::string extractorName = "ORB";
    const std::string matcherName = "BruteForce-Hamming";

    vpKeyPoint::vpFilterMatchingType filterType;
    vpKeyPoint keypoint;

    vpImage<T> ref;
    vpCameraParameters m_cam;

    static vpHomogeneousMatrix pose_from_homography(const vpHomography& homography)
    {
        vpMatrix oHw = homography.convert();

        // Normalization to ensure that ||c1|| = 1
        double norm = sqrt(vpMath::sqr(oHw[0][0]) + vpMath::sqr(oHw[1][0]) + vpMath::sqr(oHw[2][0]));
        oHw /= norm;
        vpColVector c1 = oHw.getCol(0);
        vpColVector c2 = oHw.getCol(1);
        vpColVector c3 = vpColVector::crossProd(c1, c2);
        vpHomogeneousMatrix oTw;
        for (int i = 0; i < 3; i++)
        {
            oTw[i][0] = c1[i];
            oTw[i][1] = c2[i];
            oTw[i][2] = c3[i];
            oTw[i][3] = oHw[i][2];
        }
        return oTw;
    }

public:
    Pokemon(const vpImage<T> &I,const vpCameraParameters& cam) : m_cam(cam),
                                        filterType(vpKeyPoint::ratioDistanceThreshold),
                                        keypoint(detectorName, extractorName, matcherName, filterType)
    {
        vpImageIo::read(ref,"../data/tag.png");
        keypoint.buildReference(ref);
        this->addPoses(vpHomogeneousMatrix());
    }

    vpHomogeneousMatrix update(const vpImage<T> &I)
    {
        unsigned int nbMatch = keypoint.matchPoint(I);
        std::cout << "Nb matches: " << nbMatch << std::endl;

        std::vector<vpImagePoint> iPref(nbMatch),
            iPcur(nbMatch); // Coordinates in pixels (for display only)
        std::vector<double> mPref_x(nbMatch), mPref_y(nbMatch);
        std::vector<double> mPcur_x(nbMatch), mPcur_y(nbMatch);
        std::vector<bool> inliers(nbMatch);

        for (unsigned int i = 0; i < nbMatch; i++)
        {
            keypoint.getMatchedPoints(i, iPref[i], iPcur[i]);
            vpPixelMeterConversion::convertPoint(m_cam, iPref[i], mPref_x[i], mPref_y[i]);
            vpPixelMeterConversion::convertPoint(m_cam, iPcur[i], mPcur_x[i], mPcur_y[i]);
        }

        vpHomography curHref;

        try
        {
            double residual;
            vpHomography::ransac(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
                                 (unsigned int)(mPref_x.size() * 0.25), 2.0 / m_cam.get_px(), true);
        }
        catch (...)
        {
            std::cout << "Cannot compute homography from matches..." << std::endl;
        }

        vpHomogeneousMatrix curTref = pose_from_homography(curHref);
        this->addPoses(curTref.inverse());

        return curTref;
    }
};

} // namespace Slam
