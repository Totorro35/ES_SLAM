/**
 * @file main.cpp
 * @author S'tout mo vie (gilou.assistant@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpVideoReader.h>

#include <Slam/Homography.hpp>

#include <memory>

typedef unsigned char ImgType;

int main(int argc, char **argv)
{

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
    try
    {

        

        std::string videoname = "";
        for (int i = 0; i < argc; i++)
        {
            if (std::string(argv[i]) == "--name")
                videoname = std::string(argv[i + 1]);
            else if (std::string(argv[i]) == "--help")
            {
                std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n"
                          << std::endl;
                return 0;
            }
        }

        vpImage<ImgType> I;
        vpVideoReader g;
        g.setFileName(videoname);
        g.open(I);
        std::cout << "video name: " << videoname << std::endl;
        std::cout << "video framerate: " << g.getFramerate() << "Hz" << std::endl;
        std::cout << "video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

        std::shared_ptr<Slam::Slam<ImgType>> slam(new Slam::Homography<ImgType>(I));

#ifdef VISP_HAVE_X11
        vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I);
#else
        std::cout << "No image viewer is available..." << std::endl;
#endif

        vpDisplay::setTitle(I, "Video reader");
        while (!g.end())
        {
            double t = vpTime::measureTimeMs();
            g.acquire(I);
            slam->update(I);
            vpDisplay::display(I);
            vpDisplay::flush(I);
            if (vpDisplay::getClick(I, false))
                break;
            vpTime::wait(t, 1000. / g.getFramerate());
        }

        slam->saveJson("out.txt");
    }
    catch (vpException e)
    {
        std::cout << e.getMessage() << std::endl;
    }
#else
    (void)argc;
    (void)argv;
    std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif
}