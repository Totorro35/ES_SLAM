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
#include <Slam/AprilTag.hpp>
#include <Slam/Pokemon.hpp>
#include <visp/vpRGBa.h>

#include <memory>

#include <visp/vpVideoWriter.h>

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
        vpImage<vpRGBa> Ioverlay;
        vpVideoReader g;
        g.setFileName(videoname);
        g.open(I);
        std::cout << "video name: " << videoname << std::endl;
        std::cout << "video framerate: " << g.getFramerate() << "Hz" << std::endl;
        std::cout << "video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

        vpCameraParameters cam(1512, 1512, I.getWidth() / 2, I.getHeight() / 2);

        //std::shared_ptr<Slam::Slam<ImgType>> slam(new Slam::Homography<ImgType>(I,cam));
        //std::shared_ptr<Slam::Slam<ImgType>> slam(new Slam::AprilTag<ImgType>(I,cam));
        std::shared_ptr<Slam::Slam<ImgType>> slam(new Slam::Pokemon<ImgType>(I,cam));

        vpVideoWriter writer;
        writer.setFileName("./test.avi");
        writer.open(I);

#ifdef VISP_HAVE_X11
        vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I);
#else
        std::cout << "No image viewer is available..." << std::endl;
#endif

        vpTranslationVector objPos(0.0,0.0,0.5);
        vpHomogeneousMatrix wTo(objPos,vpRotationMatrix());

        vpDisplay::setTitle(I, "Video reader");
        while (!g.end())
        {
            double t = vpTime::measureTimeMs();
            g.acquire(I);
            vpHomogeneousMatrix cTw = slam->update(I);
            vpDisplay::display(I);
            vpDisplay::displayFrame(I, cTw, cam, 0.05, vpColor::none, 3);
            vpDisplay::flush(I);
            vpDisplay::getImage(I,Ioverlay);
            writer.saveFrame(Ioverlay);
            if (vpDisplay::getClick(I, false))
                break;
            vpTime::wait(t, 1000. / g.getFramerate());
            //vpDisplay::getClick(I);
        }

        slam->saveJson("../data/out.txt");
        slam->saveJson2("../data/out2.txt");
        writer.close();
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