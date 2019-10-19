#include <iostream>
#include "opencv2/opencv.hpp"
#include "WorkerClass.h"
#include "CameraInput.h"
#include "FaceDetector.h"
#include "HungarianKalmanTracker.h"
#include "OutputWindow.h"
// ----------------------------------------------------------------------
// -----------------------------
//
// -----------------------------
// Application: our Observer.
class Application
{
public:
    // -----------------------------
    //
    // -----------------------------
    explicit Application(BaseWorkerClass& worker,IOutput& output,ITracker& tracker,IDetector& detector, IInput& input) :
        worker_(worker),
        output_(output),
        tracker_(tracker),
        detector_(detector),
        input_(input)
    {
        finished = false;

        worker_.Register<BaseWorkerClassObservers::OnWorkerEvent>([this](double& worker_FPS)
            {
                OnWorker(worker_FPS);
            });

        worker_.Register<BaseWorkerClassObservers::OnRunEvent>([this](void)
            {
                OnRun();
            });

        worker_.Register<BaseWorkerClassObservers::OnCloseEvent>([this](void)
            {
                OnClose();
            });

        std::cout << "Events - registered" << std::endl;

        worker_.Run();

        while (!finished)
        {
            Sleep(100);
            std::cin.ignore();
            finished = true;
        }

        worker_.Stop();
    }
    // -----------------------------
    //
    // -----------------------------
    ~Application()
    {

    }

private:
    bool finished;

    void OnRun()
    {
        std::cout << "On run event." << std::endl;
    }

    void OnClose()
    {
        std::cout << "On close event." << std::endl;
    }

    void OnWorker(double worker_FPS)
    {
        std::cout << "On worker. FPS=" << worker_FPS << std::endl;
        cv::UMat frame;
        input_.Get(frame);
        detector_.Detect(frame);
        regions_t regs = detector_.Get();
        std::cout << "regs " << regs.size() << std::endl;
        tracker_.Update(regs, frame, worker_FPS);
        std::vector<TrackingObject> tracks=tracker_.GetTracks();
        std::cout << "tracks " << tracks.size() << std::endl;
        output_.Update(regs, frame, worker_FPS);
    }

    BaseWorkerClass& worker_;
    IOutput& output_;
    IDetector& detector_;
    ITracker& tracker_;
    IInput& input_;
};
// -----------------------------
//
// -----------------------------
int main()
{
    CameraInput input("../../data/smuglanka.mp4");
    FaceDetector detector;
    HungarianKalmanTracker tracker;
    OutputWindow output;

    cxxproperties::Properties detector_config;
    detector_config.add("cascadeFileName","./../../data/haarcascade_frontalface_alt2.xml");
    detector_config.add("minObjectWidth", 100);
    detector_config.add("minObjectHeight", 100);
    detector.Init(detector_config);

    cxxproperties::Properties tracker_config;
    tracker_config.add("kalmanType", "KalmanLinear");
    tracker_config.add("filterGoal", "FilterRect");
    tracker_config.add("matchType", "MatchHungrian");
    tracker.Init(tracker_config);

    WorkerClass worker;
    Application application{ worker,output,tracker,detector,input };
}
