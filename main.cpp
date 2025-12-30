// main.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include <vtkSmartPointer.h>
#include <vtkImageImport.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>

#include <windows.h> 
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <filesystem>
#include <chrono>
#include <cmath>

namespace fs = std::filesystem;

// functions
static void cleanMask(cv::Mat& m)
{
    // Reduce noise and fill holes
    cv::GaussianBlur(m, m, cv::Size(5, 5), 0);
    cv::threshold(m, m, 200, 255, cv::THRESH_BINARY); // remove shadows
    cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, { 3,3 }));
    cv::morphologyEx(m, m, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, { 7,7 }));
    cv::medianBlur(m, m, 3);
}

// Draw rounded rectangle
static void drawRoundedRect(cv::Mat& img, const cv::Rect& r, const cv::Scalar& color, int thickness = 2, int radius = 8)
{
    // clamp radius
    int rad = std::min(radius, std::min(r.width / 4, r.height / 4));
    // straight edges
    cv::line(img, { r.x + rad, r.y }, { r.x + r.width - rad - 1, r.y }, color, thickness);
    cv::line(img, { r.x + rad, r.y + r.height - 1 }, { r.x + r.width - rad - 1, r.y + r.height - 1 }, color, thickness);
    cv::line(img, { r.x, r.y + rad }, { r.x, r.y + r.height - rad - 1 }, color, thickness);
    cv::line(img, { r.x + r.width - 1, r.y + rad }, { r.x + r.width - 1, r.y + r.height - rad - 1 }, color, thickness);
    // corners (use ellipse/circle)
    cv::ellipse(img, cv::Point(r.x + rad, r.y + rad), cv::Size(rad, rad), 180.0, 0, 90, color, thickness);
    cv::ellipse(img, cv::Point(r.x + r.width - rad - 1, r.y + rad), cv::Size(rad, rad), 270.0, 0, 90, color, thickness);
    cv::ellipse(img, cv::Point(r.x + rad, r.y + r.height - rad - 1), cv::Size(rad, rad), 90.0, 0, 90, color, thickness);
    cv::ellipse(img, cv::Point(r.x + r.width - rad - 1, r.y + r.height - rad - 1), cv::Size(rad, rad), 0.0, 0, 90, color, thickness);
}

// centroid tracker 
struct Track {
    int id;
    cv::Point centroid;
    int missed; // consecutive missed detections
    std::vector<cv::Point> trail;
};

static double dist2(const cv::Point& a, const cv::Point& b) {
    double dx = double(a.x - b.x);
    double dy = double(a.y - b.y);
    return dx * dx + dy * dy;
}

// main 
int main()
{
    // filenames
    const std::string file1 = "VASTChallenge2009-M3-VIDEOPART1.mov";
    const std::string file2 = "VASTChallenge2009-M3-VIDEOPART2.mov";

    // output
    const std::string outVideoName = "CombineVideosOutput.avi";
    const std::string outScreensDir = "OutputScreens";
    fs::create_directories(outScreensDir);

    // open captures
    cv::VideoCapture cap1(file1, cv::CAP_FFMPEG);
    cv::VideoCapture cap2(file2, cv::CAP_FFMPEG);
    if (!cap1.isOpened() || !cap2.isOpened()) {
        std::cerr << "Error: can not open input videos\n";
        return -1;
    }

    // get fps
    double fps1 = cap1.get(cv::CAP_PROP_FPS); if (fps1 <= 0) fps1 = 25;
    double fps2 = cap2.get(cv::CAP_PROP_FPS); if (fps2 <= 0) fps2 = 25;
    double outFps = std::min(fps1, fps2);

    // read first frames
    cv::Mat f1, f2;
    cap1 >> f1;
    cap2 >> f2;
    if (f1.empty() || f2.empty()) {
        std::cerr << "Error: can not read first frames\n";
        return -1;
    }

    if (f1.channels() == 1) cv::cvtColor(f1, f1, cv::COLOR_GRAY2BGR);
    if (f2.channels() == 1) cv::cvtColor(f2, f2, cv::COLOR_GRAY2BGR);

    if (f1.size() != f2.size()) cv::resize(f2, f2, f1.size());

    int w = f1.cols;
    int h = f1.rows;
    int outW = w * 2;
    int outH = h * 2;

    // VideoWriter for processed output (BGR)
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter writer(outVideoName, fourcc, outFps, cv::Size(outW, outH));
    if (!writer.isOpened()) {
        std::cerr << "Warning: cannot open VideoWriter, output video will not be saved\n";
    }

    // background subtractors
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg1 = cv::createBackgroundSubtractorMOG2();
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg2 = cv::createBackgroundSubtractorMOG2();
    bg1->setHistory(200); bg1->setDetectShadows(true);
    bg2->setHistory(200); bg2->setDetectShadows(true);

    // tracker state
    std::map<int, Track> tracks;
    int nextTrackId = 1;
    const double maxAssignDist2 = 60.0 * 60.0;
    const int maxMissed = 10;
    const int maxTrail = 30;

    // persistent vtk buffer
    std::vector<unsigned char> vtkBuffer(outW * outH * 3);

    // VTK importer/viewer
    vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New();
    importer->SetDataSpacing(1, 1, 1);
    importer->SetDataOrigin(0, 0, 0);
    importer->SetWholeExtent(0, outW - 1, 0, outH - 1, 0, 0);
    importer->SetDataExtentToWholeExtent();
    importer->SetDataScalarTypeToUnsignedChar();
    importer->SetNumberOfScalarComponents(3);
    importer->SetImportVoidPointer(vtkBuffer.data());
    importer->Modified();

    vtkSmartPointer<vtkImageViewer2> viewer = vtkSmartPointer<vtkImageViewer2>::New();
    viewer->SetInputConnection(importer->GetOutputPort());
    viewer->SetColorWindow(255);
    viewer->SetColorLevel(127);
    viewer->Render();

    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    viewer->GetRenderWindow()->SetInteractor(interactor);
    interactor->Initialize();

    // combined BGR image that we will write to file
    cv::Mat out(outH, outW, CV_8UC3, vtkBuffer.data());

    int frameCount = 0;
    int savedScreens = 0;
    auto startTime = std::chrono::steady_clock::now();

    // rewind to start 
    cap1.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap2.set(cv::CAP_PROP_POS_FRAMES, 0);

    while (true)
    {
        cap1 >> f1;
        cap2 >> f2;
        if (f1.empty() || f2.empty()) break;

        // Timestamp
        double t_ms = cap1.get(cv::CAP_PROP_POS_MSEC);
        std::string timeText = "Time: " + std::to_string(t_ms / 1000.0f) + " s";

        if (f1.channels() == 1) cv::cvtColor(f1, f1, cv::COLOR_GRAY2BGR);
        if (f2.channels() == 1) cv::cvtColor(f2, f2, cv::COLOR_GRAY2BGR);
        if (f1.size() != f2.size()) cv::resize(f2, f2, f1.size());

        // masks
        cv::Mat m1, m2;
        bg1->apply(f1, m1);
        bg2->apply(f2, m2);

        cleanMask(m1);
        cleanMask(m2);

        // now find contours and detections for both feeds
        std::vector<cv::Rect> dets;
        std::vector<cv::Point> centers;

        auto collectDetections = [&](const cv::Mat& mask, const cv::Mat& frame, int xOffset) {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (auto& c : contours) {
                double area = cv::contourArea(c);
                if (area < 250) continue;
                cv::Rect r = cv::boundingRect(c);

                r &= cv::Rect(0, 0, frame.cols, frame.rows);
                if (r.width <= 0 || r.height <= 0) continue;
                // store detection as global coordinates for combined image
                cv::Rect rGlobal(r.x + xOffset, r.y, r.width, r.height);
                dets.push_back(rGlobal);
                centers.push_back(cv::Point(rGlobal.x + rGlobal.width / 2, rGlobal.y + rGlobal.height / 2));
            }
            };

        collectDetections(m1, f1, 0);
        collectDetections(m2, f2, w);

        // update tracks
        std::vector<int> assignedDet(dets.size(), -1);
        // find nearest detection
        for (auto& kv : tracks) {
            kv.second.missed++;
        }

        for (size_t d = 0; d < dets.size(); ++d) {
            double best = 1e12; int bestId = -1;
            for (auto& kv : tracks) {
                int id = kv.first;
                Track& t = kv.second;
                double D = dist2(t.centroid, centers[d]);
                if (D < best) { best = D; bestId = id; }
            }
            if (bestId != -1 && best <= maxAssignDist2) {
                // assign detection d to bestId
                Track& t = tracks[bestId];
                t.centroid = centers[d];
                t.missed = 0;
                t.trail.push_back(centers[d]);
                if ((int)t.trail.size() > maxTrail) t.trail.erase(t.trail.begin());
                assignedDet[d] = bestId;
            }
        }


        for (size_t d = 0; d < dets.size(); ++d) {
            if (assignedDet[d] == -1) {
                Track nt;
                nt.id = nextTrackId++;
                nt.centroid = centers[d];
                nt.missed = 0;
                nt.trail.push_back(centers[d]);
                tracks[nt.id] = nt;
                assignedDet[d] = nt.id;
            }
        }

        // Remove tracks missed too long
        std::vector<int> toRemove;
        for (auto& kv : tracks) {
            if (kv.second.missed > maxMissed) toRemove.push_back(kv.first);
        }
        for (int id : toRemove) tracks.erase(id);

        // 2x2 output
        f1.copyTo(out(cv::Rect(0, 0, w, h)));
        f2.copyTo(out(cv::Rect(w, 0, w, h)));
        cv::Mat m1c; cv::cvtColor(m1, m1c, cv::COLOR_GRAY2BGR);
        m1c.copyTo(out(cv::Rect(0, h, w, h)));
        cv::Mat m2c; cv::cvtColor(m2, m2c, cv::COLOR_GRAY2BGR);
        m2c.copyTo(out(cv::Rect(w, h, w, h)));

        // Top left Video 1
        cv::putText(out, timeText,
            cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        // Top right Video 2
        cv::putText(out, timeText,
            cv::Point(w + 20, 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        // Bottom left Mask 1
        cv::putText(out, timeText,
            cv::Point(20, h + 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        // Bottom right Mask 2
        cv::putText(out, timeText,
            cv::Point(w + 20, h + 40),
            cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

        // Draw rounded boxes + IDs + trails
        for (auto& kv : tracks) {
            int id = kv.first;
            Track& t = kv.second;
            // find the detection rect associated with this id
            // choose nearest det center
            int bestIdx = -1; double bestD = 1e12;
            for (size_t d = 0; d < dets.size(); ++d) {
                if (assignedDet[d] != id) continue;
                double D = dist2(centers[d], t.centroid);
                if (D < bestD) { bestD = D; bestIdx = (int)d; }
            }
            cv::Scalar color(0, 255, 0);
            if (bestIdx != -1) {
                cv::Rect r = dets[bestIdx];
                if (r.y < h) {
                    drawRoundedRect(out, r, color, 2, 10);
                    // label
                    std::string txt = "ID:" + std::to_string(id);
                    int tx = r.x;
                    int ty = std::max(12, r.y - 6);
                    cv::putText(out, txt, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
                }
            }
            // draw trail
            for (size_t k = 1; k < t.trail.size(); ++k) {
                cv::Point a = t.trail[k - 1];
                cv::Point b = t.trail[k];
                // skip if trail point not in top area
                if (a.y >= h || b.y >= h) continue;
                cv::line(out, a, b, cv::Scalar(255, 0, 0), 2);
            }
            // small circle at centroid if in top area
            if (t.centroid.y < h) cv::circle(out, t.centroid, 4, cv::Scalar(0, 0, 255), -1);
        }

        // write combined frame to video
        if (writer.isOpened()) writer.write(out);

        // save screenshot every N frames
        if (frameCount % 150 == 0) {
            std::string fname = outScreensDir + "/screen_" + std::to_string(savedScreens++) + ".png";
            cv::Mat toSave = out.clone();
            cv::imwrite(fname, toSave);
        }

        cv::Mat flipped;
        cv::flip(out, flipped, 0);
        if ((size_t)vtkBuffer.size() == (size_t)(flipped.total() * flipped.elemSize()))
            memcpy(vtkBuffer.data(), flipped.data, vtkBuffer.size());
        else {
            std::cerr << "Buffer size mismatch\n";
        }

        importer->Modified();
        viewer->Render();
        interactor->ProcessEvents();

        // exit on ESC
        if (GetAsyncKeyState(VK_ESCAPE)) break;

        frameCount++;
    }

    // cleanup and timing info
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
    std::cout << "Processed frames: " << frameCount << " in " << duration << "s\n";
    if (writer.isOpened()) writer.release();
    std::cout << "Saved video: " << outVideoName << " and screenshots in " << outScreensDir << "\n";

    interactor->Start();
    return 0;
}
