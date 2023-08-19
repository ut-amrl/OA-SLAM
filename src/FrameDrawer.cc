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


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "ImageDetections.h"
#include "Tracking.h"
#include "MapObject.h"
#include "ColorManager.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
    {
        cvtColor(im,im,cv::COLOR_GRAY2BGR);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0 && i < vIniKeys.size() && vMatches[i] < vCurrentKeys.size())
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK || state==Tracking::LOST) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    s << " | FRAME "  << frame_id_;

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    // pTracker->mImGray.copyTo(mIm);
    pTracker->im_rgb_.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    frame_id_ = pTracker->GetCurrentFrameIdx();


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK || pTracker->mLastProcessedState==Tracking::LOST)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);

    auto tracks = pTracker->GetObjectTracks();
    detections_widgets_.clear();

    const auto& detections = pTracker->GetCurrentFrameDetections();
    for (auto det : detections) {
        detections_widgets_.push_back(DetectionWidget(det->bbox, 0, det->category_id,
                                        det->score, cv::Scalar(0, 0, 0),
                                        2, false)); //1.5
    }
    auto current_frame_id = pTracker->GetCurrentFrameIdx();
    for (auto tr : tracks) {
        if (current_frame_id == tr->GetLastObsFrameId()) {
            auto bb = tr->GetLastBbox();
            detections_widgets_.push_back(DetectionWidget(bb, tr->GetId(), tr->GetCategoryId(),
                                                            tr->GetLastObsScore(), tr->GetColor(),
                                                            3, true));
        }
    }

    object_projections_widgets_.clear();
    cv::Mat cv_Rt = pTracker->mCurrentFrame.mTcw;
    cv::Mat cv_K = pTracker->mCurrentFrame.mK;
    if (cv_Rt.rows == 4 && cv_Rt.cols == 4) {

        Eigen::Matrix<double, 3, 4> Rt;
        Eigen::Matrix3d K;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                Rt(i, j) = cv_Rt.at<float>(i, j);
                if (j < 3)
                    K(i, j) = cv_K.at<float>(i, j);
            }
        }
        Eigen::Matrix<double, 3, 4> P = K * Rt;

        for (auto& tr : tracks) {
            // if (tr->GetLastObsFrameId() == pTracker->GetCurrentFrameIdx()) {
                // if (tr->GetCategoryId() == 1 || tr->GetCategoryId() == 2 || tr->GetCategoryId() == 3 || tr->GetCategoryId() == 9 ||tr->GetCategoryId() == 10 || tr->GetCategoryId() == 11) continue;

                const auto* obj = tr->GetMapObject();
                if (obj) {
                    Eigen::Vector3d c = obj->GetEllipsoid().GetCenter();
                    double z = Rt.row(2).dot(c.homogeneous());
                    if (z < 0) {
                        continue;
                    }
                    auto proj = obj->GetEllipsoid().project(P);
                    object_projections_widgets_.push_back(ObjectProjectionWidget(proj, tr->GetId(),
                                                                                 tr->GetCategoryId(), tr->GetColor(),
                                                                                 tr->GetStatus() == ObjectTrackStatus::IN_MAP,
                                                                                 tr->unc_));
                }
            // }
        }
    }
}

cv::Mat FrameDrawer::DrawDetections(cv::Mat img)
{
    std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> detections;
    {
        unique_lock<mutex> lock(mMutex);
        detections = detections_widgets_;
    }
    const auto& manager = CategoryColorsManager::GetInstance();
    cv::Scalar color;
    for (auto d : detections) {
        const auto& bb = d.bbox;
        if (use_category_cols_) {
            color = manager[d.category_id];
        } else {
            color = d.color;
        }
        cv::rectangle(img, cv::Point2i(bb[0], bb[1]),
                           cv::Point2i(bb[2], bb[3]),
                           color,
                           d.thickness);
        if (d.display_info) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << d.score;
            // cv::putText(img, std::to_string(d.id) + "(" + std::to_string(d.category_id) + ") | " + ss.str(),
            cv::putText(img, std::to_string(d.id) + "|" + ss.str() + "|" + std::to_string(d.category_id),
                        cv::Point2i(bb[0]-10, bb[1]-5), cv::FONT_HERSHEY_DUPLEX,
                        0.55, cv::Scalar(255, 255, 255), 1, false);
        }
    }
    return img;
}

void draw_ellipse_dashed(cv::Mat img, const Ellipse& ell, const cv::Scalar& color, int thickness)
{
    int size = 8;
    int space = 16;
    const auto& c = ell.GetCenter();
    const auto& axes = ell.GetAxes();
    double angle = ell.GetAngle();
    for (int i = 0; i < 360; i += space) {
        cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]),
                    TO_DEG(angle), i, i+size, color, thickness);
    }
}

cv::Mat FrameDrawer::DrawProjections(cv::Mat img)
{
    std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> projections;
    {
        unique_lock<mutex> lock(mMutex);
        projections = object_projections_widgets_;
    }

    const auto& manager= CategoryColorsManager::GetInstance();
    cv::Scalar color;
    for (auto w : projections)
    {
        const auto& ell = w.ellipse;
        const auto& c = ell.GetCenter();
        const auto& axes = ell.GetAxes();
        double angle = ell.GetAngle();
//        auto bb = ell.ComputeBbox();
        // auto [status_, bb] = find_on_image_bbox(ell, img.cols, img.rows);
        if (use_category_cols_) {
            color = manager[w.category_id];
        } else {
            color = w.color;
        }
        if (w.in_map) {
            cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, color, 2);
            // stringstream ss;
            // ss << std::fixed << std::setprecision(3) << w.uncertainty;
            // cv::putText(img, ss.str(), cv::Point2f(c[0], c[1]), cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(255, 255, 255), 1, false);
        }
        else
            draw_ellipse_dashed(img, ell, color, 2);
        // cv::rectangle(img, cv::Point2i(bb[0], bb[1]), cv::Point2i(bb[2], bb[3]), color, 2);
    }

    return img;
}

} //namespace ORB_SLAM
