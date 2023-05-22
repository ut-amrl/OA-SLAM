/**
 * This file is part of OA-SLAM.
 *
 * Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
 * (Inria, LORIA, Université de Lorraine)
 * OA-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OA-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h> /* srand, rand */

#include <opencv2/core/core.hpp>

#include <Timestamp.h>
#include <Converter.h>
#include <ImageDetections.h>
#include <System.h>
#include "Osmap.h"
#include <nlohmann/json.hpp>
#include <experimental/filesystem>
#include "Utils.h"

using json = nlohmann::json;

namespace fs = std::experimental::filesystem;

using namespace std;

void ReadNodesByTimestamps(const string &strPathToNodesByTimestamps,
                           vector<ORB_SLAM2::Timestamp> &timestamps)
{
    std::ifstream ifile(strPathToNodesByTimestamps);
    if (!ifile.is_open())
    {
        std::cout << "Failed to open file " << strPathToNodesByTimestamps << std::endl;
        exit(1);
    }
    std::string line;
    std::getline(ifile, line);
    while (std::getline(ifile, line))
    {
        std::stringstream sstream(line);
        std::string word;
        std::getline(sstream, word, ',');
        std::getline(sstream, word, ',');
        uint32_t sec = (uint32_t)std::stoul(word);
        std::getline(sstream, word, ',');
        uint32_t nsec = (uint32_t)std::stoul(word);
        timestamps.emplace_back(sec, nsec);
    }
}

void LoadImages(const string &strPathInputDataRootDir,
                const string &strPathToSequenceCam1,
                const string &strPathToSequenceCam2,
                const string &strPathToNodesByTimestamps,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<string> &vstrImageLeftAbsolute,
                vector<string> &vstrImageRightAbsolute,
                vector<ORB_SLAM2::Timestamp> &vTimestamps)
{
    std::string line;

    ifstream imgPathsLeft;
    imgPathsLeft.open(strPathToSequenceCam1);
    if (!imgPathsLeft.is_open())
    {
        std::cout << "Failed to open file " << strPathToSequenceCam1 << std::endl;
        exit(1);
    }
    while (std::getline(imgPathsLeft, line))
    {
        vstrImageLeft.push_back(line);
        fs::path pathToImage = fs::path(strPathInputDataRootDir) / line;
        vstrImageLeftAbsolute.push_back(pathToImage.string());
    }
    imgPathsLeft.close();

    ifstream imgPathsRight;
    imgPathsRight.open(strPathToSequenceCam2);
    if (!imgPathsRight.is_open())
    {
        std::cout << "Failed to open file " << strPathToSequenceCam2 << std::endl;
        exit(1);
    }
    while (std::getline(imgPathsRight, line))
    {
        vstrImageRight.push_back(line);
        fs::path pathToImage = fs::path(strPathInputDataRootDir) / line;
        vstrImageRightAbsolute.push_back(pathToImage.string());
    }
    imgPathsRight.close();

    ReadNodesByTimestamps(strPathToNodesByTimestamps, vTimestamps);
}

int main(int argc, char **argv)
{
    srand(time(nullptr));
    std::cout << "C++ version: " << __cplusplus << std::endl;

    if (argc != 12)
    {
        cerr << "argc = " << argc << endl;
        cerr << endl
             << "Usage:\n"
                " ./oa-slam\n"
                "      vocabulary_file\n"
                "      camera_file\n"
                "      input_data_root_dir (root data directory; path_to_image_sequence stores paths relative to it)\n"
                "      path_to_image_sequence_for_cam_1 (.txt file listing the images for the first camera)\n"
                "      path_to_image_sequence_for_cam_2 (.txt file listing the images for the second camera)\n"
                "      path_to_nodes_by_timestamps (.txt file listing timestamps that correspond to each frame)\n"
                "      detections_file_for_cam_1 (.json file with detections or .onnx yolov5 weights)\n"
                "      detections_file_for_cam_2 (.json file with detections or .onnx yolov5 weights)\n"
                "      categories_to_ignore_file (file containing the categories to ignore (one category_id per line))\n"
                "      relocalization_mode ('points', 'objects' or 'points+objects')\n"
                "      output_path \n";
        return 1;
    }

    std::string vocabulary_file = string(argv[1]);
    std::string parameters_file = string(argv[2]);
    string input_data_root_dir = string(argv[3]);
    string path_to_images_cam_1 = string(argv[4]);
    string path_to_images_cam_2 = string(argv[5]);
    string path_to_nodes_by_timestamps = string(argv[6]);
    std::string detections_file_for_cam_1(argv[7]);
    std::string detections_file_for_cam_2(argv[8]);
    std::string categories_to_ignore_file(argv[9]);
    string reloc_mode = string(argv[10]);
    string output_path = string(argv[11]);

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrImageLeftAbsolute;
    vector<string> vstrImageRightAbsolute;
    vector<std::pair<uint32_t, uint32_t>> vTimestamps;
    LoadImages(input_data_root_dir,
               path_to_images_cam_1,
               path_to_images_cam_2,
               path_to_nodes_by_timestamps,
               vstrImageLeft,
               vstrImageRight,
               vstrImageLeftAbsolute,
               vstrImageRightAbsolute,
               vTimestamps);
    size_t nFrames = vTimestamps.size();
    std::cout << "nFrames: " << nFrames << std::endl;

    // Load categories to ignore
    std::ifstream fin(categories_to_ignore_file);
    vector<int> classes_to_ignore;
    if (!fin.is_open())
    {
        std::cout << "Warning !! Failed to open the file with ignore classes. No class will be ignore.\n";
    }
    else
    {
        int cat;
        while (fin >> cat)
        {
            std::cout << "Ignore category: " << cat << "\n";
            classes_to_ignore.push_back(cat);
        }
    }
    fin.close();

    cout << "Start loading detectors" << endl;
    std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector_left, detector_right;
    detector_left = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file_for_cam_1, classes_to_ignore);
    detector_right = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file_for_cam_2, classes_to_ignore);
    cout << "Finish loading detectors" << endl;

    // Relocalization mode
    ORB_SLAM2::enumRelocalizationMode relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    if (reloc_mode == string("points"))
        relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    else if (reloc_mode == std::string("objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
    else if (reloc_mode == std::string("points+objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
    else
    {
        std::cerr << "Error: Invalid parameter for relocalization mode. "
                     "It should be 'points', 'objects' or 'points+objects'.\n";
        return 1;
    }
    cout << "Finish setting relocalization mode" << endl;

    ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::STEREO, false, false, false);
    SLAM.SetRelocalizationMode(relocalization_mode);

    ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);
    cv::Mat imLeft, imRight;

    std::vector<double> vTimesTrack(nFrames);

    for (size_t ni = 0; ni < nFrames; ++ni)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        imLeft = cv::imread(vstrImageLeftAbsolute[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRightAbsolute[ni], cv::IMREAD_UNCHANGED);
        std::vector<ORB_SLAM2::Detection::Ptr> detectionsLeft, detectionsRight;
        detectionsLeft = detector_left->detect(vstrImageLeft[ni]);
        detectionsRight = detector_right->detect(vstrImageRight[ni]);
        cv::Mat m = SLAM.TrackStereo(imLeft, imRight, vTimestamps[ni], detectionsLeft, detectionsRight, false);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack.push_back(ttrack);
        // std::cout << "time = " << ttrack << "\n";
        if (SLAM.ShouldQuit())
            break;
    }
    SLAM.SaveLatestTrajectoryOVSlam(output_path);
    osmap.mapSave("map_test");

    return 0;
}