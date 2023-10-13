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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <experimental/filesystem>

#include <System.h>
// #include "Osmap.h"
#include <nlohmann/json.hpp>
#include "Utils.h"
#include "Timestamp.h"
#include "TimestampConversion.h"

using json = nlohmann::json;
namespace fs = std::experimental::filesystem;

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    srand(time(nullptr));
    std::cout << "C++ version: " << __cplusplus << std::endl;

    if(argc != 10)
    {
        cerr << endl << "Usage:\n"
                        " ./oa-slam_localization\n"
                        "      vocabulary_file\n"
                        "      camera_file\n"
                        "      path_to_image_sequence (.txt file listing the images or a folder with rgb.txt)\n"
                        "      detections_file (.json file with detections or .onnx yolov5 weights)\n"
                        "      categories_to_ignore_file (file containing the categories to ignore (one category_id per line))\n"
                        "      map_file (.yaml)\n"
                        "      relocalization_mode ('points', 'objects' or 'points+objects')\n"
                        "      output_name \n"
                        "      force_relocalization_on_each_frame (0 or 1)\n";
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::string vocabulary_file = string(argv[1]);
    std::string parameters_file = string(argv[2]);
    string path_to_images = string(argv[3]);
    std::string detections_file(argv[4]);
    std::string categories_to_ignore_file(argv[5]);
    string map_file = string(argv[6]);
    string reloc_mode = string(argv[7]);
    string output_name = string(argv[8]);
    bool force_reloc = std::stoi(argv[9]);


    // possible to pass 'webcam_X' where 'X' is the webcam id
    bool use_webcam = false;
    int webcam_id = 0;
    if (path_to_images.size() >= 6 && path_to_images.substr(0, 6) == "webcam") {
        use_webcam = true;
        if (path_to_images.size() > 7) {
            webcam_id = std::stoi(path_to_images.substr(7));
        }
    }

    // Possible to pass a file listing images instead of a folder containing a file rgb.txt
    std::string image_list_file = "rgb.txt";
    int nn = path_to_images.size();
    if (!use_webcam && get_file_extension(path_to_images) == "txt") {
        int pos = path_to_images.find_last_of('/');
        image_list_file = path_to_images.substr(pos+1);
        path_to_images = path_to_images.substr(0, pos+1);
    }

    if (!use_webcam && path_to_images.back() != '/')
        path_to_images += "/";

    string output_folder = output_name;
    if (output_folder.back() != '/')
        output_folder += "/";
    fs::create_directories(output_folder);

    // Get map folder absolute path
    int l = map_file.find_last_of('/') + 1;
    std::string map_folder = map_file.substr(0, l);
    if (map_folder[0] != '/') {
        fs::path map_folder_abs = fs::current_path() / map_folder;
        map_folder = map_folder_abs.string();
    }

    // Load categories to ignore
    std::ifstream fin(categories_to_ignore_file);
    vector<int> classes_to_ignore;
    if (!fin.is_open()) {
        std::cout << "Warning !! Failed to open the file with ignore classes. No class will be ignore.\n";
    } else {
        int cat;
        while (fin >> cat) {
            std::cout << "Ignore category: " << cat << "\n";
            classes_to_ignore.push_back(cat);
        }
    }

    // Load object detections
    auto extension = get_file_extension(detections_file);
    std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector = nullptr;
    bool detect_from_file = false;
    if (extension == "onnx") { // load network
        detector = std::make_shared<ORB_SLAM2::ObjectDetector>(detections_file, classes_to_ignore);
        detect_from_file = false;
    } else if (extension == "json") { // load from external detections file
        detector = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file, classes_to_ignore);
        detect_from_file = true;
    } else {
        std::cout << "Invalid detection file. It should be .json or .onnx\n"
                      "No detections will be obtained.\n";
    }


    ORB_SLAM2::enumRelocalizationMode relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    if (reloc_mode == string("points"))
        relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    else if (reloc_mode == std::string("objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
    else if (reloc_mode == std::string("points+objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
    else {
        std::cerr << "Error: Invalid parameter for relocalization mode. "
                     "It should be 'points', 'objects' or 'points+objects'.\n";
        return 1;
    }

    // Load images
    cv::VideoCapture cap;
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    int nImages = 10000;
    if (!use_webcam) {
        string strFile = path_to_images + image_list_file;
        LoadImages(strFile, vstrImageFilenames, vTimestamps);
        nImages = vstrImageFilenames.size();
    } else {
        cap.open(0);
        std::cout << "Open webcam" << std::endl;
    }

    ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::MONOCULAR, true, true, 0);
    SLAM.SetRelocalizationMode(relocalization_mode);
    SLAM.map_folder = map_folder;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);
    std::cout << "Start loading map" << std::endl;
    // osmap.mapLoad(map_file);
    std::cout << "End of loading map" << std::endl;
    SLAM.ActivateLocalizationMode();

    // SLAM.remove_nth_object_by_cat(71, 2); // remove some objects from the loaded map

    // Main loop
    cv::Mat im;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    poses.reserve(nImages);
    std::vector<std::string> filenames;
    filenames.reserve(nImages);
    std::vector<double> reloc_times;
    reloc_times.reserve(nImages);
    std::vector<bool> reloc_status;
    reloc_status.reserve(nImages);
    int ni = 0;
    while (1)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::string filename;
        if (use_webcam) {
            cap >> im;  // get image from webcam
            filename = "frame_" + std::to_string(ni) + ".png";
        }
        else
        {
            filename = path_to_images + vstrImageFilenames[ni];
            im = cv::imread(filename, cv::IMREAD_UNCHANGED);  // read image from disk
        }
        double tframe = ni < vTimestamps.size() ? vTimestamps[ni] : std::time(nullptr);
        if(im.empty())
        {
            cerr << endl << "Failed to load image: "
                 << filename << endl;
            return 1;
        }
        filenames.push_back(filename);

        // Get object detections
        std::vector<ORB_SLAM2::Detection::Ptr> detections;
        if (detector) {
            if (detect_from_file)
                detections = detector->detect(filename); // from detections file
            else
                detections = detector->detect(im);  // from neural network
        }

        // Pass the image and detections to the SLAM system
        cv::Mat m = SLAM.TrackMonocular(im, ORB_SLAM2::toTimestampPair(tframe), detections, force_reloc);
        reloc_times.push_back(SLAM.relocalization_duration);
        reloc_status.push_back(SLAM.relocalization_status);

        if (m.rows && m.cols)
            poses[ni] = ORB_SLAM2::cvToEigenMatrix<double, float, 4, 4>(m);
        else
            poses.push_back(Eigen::Matrix4d::Identity());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack.push_back(ttrack);

        if (SLAM.ShouldQuit())
            break;

        ++ni;
        if (ni >= nImages)
            break;
    }

    // Stop all threads
    SLAM.Shutdown();
  
    // Save camera trajectory
    json json_data;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d m = poses[i];
        json R({{m(0, 0), m(0, 1), m(0, 2)},
                {m(1, 0), m(1, 1), m(1, 2)},
                {m(2, 0), m(2, 1), m(2, 2)}});
        json t({m(0, 3), m(1, 3), m(2, 3)});
        json image_data;
        image_data["file_name"] = filenames[i];
        image_data["R"] = R;
        image_data["t"] = t;
        json_data.push_back(image_data);
    }

    std::ofstream json_file(output_folder + "camera_poses_" + output_name + ".json");
    json_file << json_data;
    json_file.close();
    std::cout << "Saved " << poses.size() << " poses.\n";


    // Relocalization time statistics
    std::ofstream file_times(output_folder + "relocalization_times.txt");
    for (int i = 0; i < reloc_times.size(); ++i) {
        file_times << reloc_times[i] << " " << (int)reloc_status[i] << "\n";
    }
    file_times.close();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    string s0;
    double t = 0;
    int n = 0;
    bool found_timestamps = false;
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty() && s[0] != '#')
        {
            stringstream ss;
            ss << s;
            string sRGB;
            if (ss.str().find(' ') != std::string::npos) {
                ss >> t;
                found_timestamps = true;
            }
            ss >> sRGB;

            vTimestamps.push_back(t);
            vstrImageFilenames.push_back(sRGB);
            if (!found_timestamps)
                t += 0.033;
            ++n;
        }
    }
    f.close();
}