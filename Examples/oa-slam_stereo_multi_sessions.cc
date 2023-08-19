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

void LoadSequenceFile(const string &strPathToSequenceFile,
                      vector<string> &bagnames,
                      string &sequence_id)
{
    json data;
    std::ifstream fin(strPathToSequenceFile);
    if (!fin.is_open())
    {
        std::cerr << "Warning failed to open file: " << strPathToSequenceFile << std::endl;
        return;
    }
    fin >> data;
    fin.close();

    sequence_id = data["seq_id"];
    for (const auto &bagname : data["sequence"])
    {
        bagnames.push_back(bagname);
    }
}

void LoadSequenceFileWithWaypoints(const string &strPathToSequenceFile,
                                   vector<string> &bagnames,
                                   string &sequence_id)
{
    json data;
    std::ifstream fin(strPathToSequenceFile);
    if (!fin.is_open())
    {
        std::cerr << "Warning failed to open file: " << strPathToSequenceFile << std::endl;
        return;
    }
    fin >> data;
    fin.close();

    sequence_id = data["sequence_info"]["seq_id"];
    for (const auto &baginfo : data["sequence_info"]["sequence"])
    {
        bagnames.push_back(baginfo["bag_base_name"]);
    }
}

// This executable is specific for UT VSLAM evaluation
int main(int argc, char **argv)
{
    srand(time(nullptr));
    std::cout << "C++ version: " << __cplusplus << std::endl;

    if (argc != 7)
    {
        cerr << "argc = " << argc << endl;
        cerr << endl
             << "Usage:\n"
                " ./oa-slam\n"
                "      vocabulary_file\n"
                "      camera_file\n"
                "      data_root_dir (root data directory; path_to_image_sequence stores paths relative to data_root_dir/oa_slam_in/<bagname>)\n"
                "      sequence_file \n"
                "      categories_to_ignore_file (file containing the categories to ignore (one category_id per line))\n"
                "      relocalization_mode ('points', 'objects' or 'points+objects')\n";
        return 1;
    }

    std::string vocabulary_file = string(argv[1]);
    std::string parameters_file = string(argv[2]);
    string data_root_dir = string(argv[3]);
    string strPathToSequenceFile = string(argv[4]);
    std::string categories_to_ignore_file(argv[5]);
    string reloc_mode = string(argv[6]);

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

    // Load sequence file
    vector<string> bagnames;
    string sequence_id;
    LoadSequenceFileWithWaypoints(strPathToSequenceFile, bagnames, sequence_id);
    // LoadSequenceFile(strPathToSequenceFile, bagnames, sequence_id);

    // naming assumptions of the root data directory
    const string ORB_OUT = "orb_out";
    const string OA_SLAM_IN = "oa_slam_in";
    const string OA_SLAM_OUT = "oa_slam_out";

    fs::path oa_slam_in_root_dir = fs::path(data_root_dir) / OA_SLAM_IN;
    if (!fs::exists(oa_slam_in_root_dir))
    {
        std::cout << "Path " << oa_slam_in_root_dir.string() << " doesn't exist. "
                  << "You may forget to run the data generator script." << std::endl;
        exit(0);
    }
    fs::path orb_out_rood_dir = fs::path(data_root_dir) / ORB_OUT;
    if (!fs::exists(orb_out_rood_dir))
    {
        std::cout << "Path " << orb_out_rood_dir.string() << " doesn't exist. "
                  << "You need to run ORB_SLAM2 visual frontend to obtain the nodes_by_timestamps.txt file." << std::endl;
        exit(0);
    }
    fs::path oa_slam_out_root_dir = fs::path(data_root_dir) / OA_SLAM_OUT;
    if (!fs::exists(oa_slam_out_root_dir))
    {
        if (!fs::create_directory(oa_slam_out_root_dir))
        {
            std::cout << "Failed to create directory " << oa_slam_out_root_dir.string() << std::endl;
            exit(0);
        }
    }
    fs::path oa_slam_out_seq_dir = oa_slam_out_root_dir / sequence_id;
    if (!fs::exists(oa_slam_out_seq_dir))
    {
        if (!fs::create_directory(oa_slam_out_seq_dir))
        {
            std::cout << "Failed to create directory " << oa_slam_out_seq_dir.string() << std::endl;
            exit(0);
        }
    }

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

    // ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::STEREO, false, false, false);
    ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::STEREO, true, true, false);
    // ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::STEREO, true, true, 1);
    SLAM.SetRelocalizationMode(relocalization_mode);
    ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);

    size_t bag_id = 0;
    for (const string &bagname : bagnames)
    {
        fs::path oa_slam_in_dir = oa_slam_in_root_dir / bagname;
        if (!fs::exists(oa_slam_in_dir))
        {
            std::cout << "Path " << oa_slam_in_dir.string() << " doesn't exist. "
                      << "You may forget to run the data generator script." << std::endl;
            exit(0);
        }
        fs::path path_to_images_cam_1 = oa_slam_in_dir / "1" / "cam_1_images.txt";
        fs::path path_to_images_cam_2 = oa_slam_in_dir / "2" / "cam_2_images.txt";
        fs::path detections_file_for_cam_1 = oa_slam_in_dir / "1" / "detections" / "detections.json";
        fs::path detections_file_for_cam_2 = oa_slam_in_dir / "2" / "detections" / "detections.json";

        fs::path orb_out_for_bagname = orb_out_rood_dir / bagname;
        if (!fs::exists(orb_out_for_bagname))
        {
            std::cout << "Path " << orb_out_for_bagname.string() << " doesn't exist. "
                      << "You need to run ORB_SLAM2 visual frontend to obtain the nodes_by_timestamps.txt file." << std::endl;
            exit(0);
        }
        fs::path path_to_nodes_by_timestamps = orb_out_for_bagname / "timestamps" / "node_ids_and_timestamps.txt";
        if (!fs::exists(path_to_nodes_by_timestamps))
        {
            std::cout << "Path " << path_to_nodes_by_timestamps.string() << " doesn't exist. "
                      << "You need to run ORB_SLAM2 visual frontend to obtain the nodes_by_timestamps.txt file." << std::endl;
            exit(0);
        }

        vector<string> vstrImageLeft;
        vector<string> vstrImageRight;
        vector<string> vstrImageLeftAbsolute;
        vector<string> vstrImageRightAbsolute;
        vector<std::pair<uint32_t, uint32_t>> vTimestamps;
        LoadImages(oa_slam_in_dir,
                   path_to_images_cam_1,
                   path_to_images_cam_2,
                   path_to_nodes_by_timestamps,
                   vstrImageLeft,
                   vstrImageRight,
                   vstrImageLeftAbsolute,
                   vstrImageRightAbsolute,
                   vTimestamps);

        cout << "Start loading detectors" << endl;
        std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector_left, detector_right;
        detector_left = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file_for_cam_1.string(), classes_to_ignore);
        detector_right = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file_for_cam_2.string(), classes_to_ignore);
        cout << "Finish loading detectors" << endl;

        cv::Mat imLeft, imRight;

        size_t nFrames = vTimestamps.size();
        std::cout << "nFrames: " << nFrames << std::endl;
        SLAM.MarkNewTrajectoryStart();
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
            // std::cout << "time = " << ttrack << "\n";
            if (SLAM.ShouldQuit())
                break;
        }
        fs::path output_path = oa_slam_out_seq_dir / (std::to_string(bag_id) + "_" + bagname);
        if (!fs::exists(output_path))
        {
            if (!fs::create_directory(output_path))
            {
                std::cout << "Failed to create directory " << output_path.string() << std::endl;
                exit(0);
            }
        }
        fs::path traj_output_path = output_path / "trajectory.csv";
        SLAM.SaveLatestTrajectoryOVSlam(traj_output_path);
        fs::path ellipsoid_output_path = output_path / "ellipsoids.csv";
        SLAM.SaveObjectMapOVSLAM(ellipsoid_output_path.string());
        ++bag_id;
    }

    return 0;
}