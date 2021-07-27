//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "pch.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>  // cv::Canny()
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

BYTE ConvertDepthPixel_this(USHORT v, BYTE bSigma, USHORT mask, USHORT maxshort, const int vmin, const int vmax)
{
    if ((mask != 0) && (bSigma & mask) > 0)
    {
        v = 0;
    }

    if ((maxshort != 0) && (v > maxshort))
    {
        v = 0;
    }

    float colorValue = 0.0f;
    if (v <= vmin)
    {
        colorValue = 0.0f;
    }
    else if (v >= vmax)
    {
        colorValue = 1.0f;
    }
    else
    {
        colorValue = (float)(v - vmin) / (float)(vmax - vmin);
    }

    return (BYTE)(colorValue * 255);
}

void ProcessRmFrameWithAruco(IResearchModeSensorFrame* pSensorFrame, cv::Mat& cvResultMat, std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners)
{
    HRESULT hr = S_OK;
    ResearchModeSensorResolution resolution;
    size_t outBufferCount = 0;
    const BYTE *pImage = nullptr;
    IResearchModeSensorVLCFrame *pVLCFrame = nullptr;
    pSensorFrame->GetResolution(&resolution);
    static cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    if (SUCCEEDED(hr))
    {
        pVLCFrame->GetBuffer(&pImage, &outBufferCount);

        cv::Mat processed(resolution.Height, resolution.Width, CV_8U, (void*)pImage);
        cv::aruco::detectMarkers(processed, dictionary, corners, ids);

        cvResultMat = processed;

        // if at least one marker detected
        if (ids.size() > 0)
            cv::aruco::drawDetectedMarkers(cvResultMat, corners, ids);
    }

    if (pVLCFrame)
    {
        pVLCFrame->Release();
    }
}


void ProcessAHATData(IResearchModeSensorFrame* pSensorFrame, cv::Mat& cvResultMat,cv::Mat& cvResultMask, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners)
{
    HRESULT hr = S_OK;
    ResearchModeSensorResolution resolution;
    size_t outBufferCount = 0;
    const BYTE* pImage = nullptr;
    const UINT16* pDepth = nullptr;
    const UINT16* pAbImage = nullptr;
    
    IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
    pSensorFrame->GetResolution(&resolution);
    //BYTE* pDepth_byte = new BYTE[resolution.Height * resolution.Width];
    BYTE* pAbImage_byte = new BYTE[resolution.Height * resolution.Width];
    hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));
    DirectX::XMFLOAT4X4 cameraPose;
    int maxClampDepth = 0;
    USHORT maxshort = 0;
    USHORT mask = 0;
    const BYTE* pSigma = nullptr;

    mask = 0x0;
    maxClampDepth = 3000;
    maxshort = 4090;
    
    if (SUCCEEDED(hr))
    {
        //pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
        pDepthFrame->GetAbDepthBuffer(&pAbImage, &outBufferCount);
        

        for (int i = 0; i < resolution.Width * resolution.Height; i++) {
            /*pDepth_byte[i]= ConvertDepthPixel_this(
                pDepth[i],
                pSigma ? pSigma[i] : 0,
                mask,
                maxshort,
                0,
                maxClampDepth);*/
            pAbImage_byte[i] = ConvertDepthPixel_this(
                pAbImage[i],
                pSigma ? pSigma[i] : 0,
                mask,
                maxshort,
                0,
                maxClampDepth);
        }
        //cv::Mat depth_mat(resolution.Height, resolution.Width, CV_8U, (void*)pDepth_byte);

        cv::Mat depthAb_mat(resolution.Height, resolution.Width, CV_8U, (void*)pAbImage_byte);
        cvResultMask = cv::Mat::zeros(resolution.Height, resolution.Width, CV_8U);
        cv::threshold(depthAb_mat, depthAb_mat, 200, 255, cv::THRESH_BINARY);
        std::vector<cv::Vec3f> circles;
        cv::Mat Blurred;
        cv::GaussianBlur(depthAb_mat, Blurred, cv::Size(5, 5), 2, 2);
        //cv::Canny(Blurred, Blurred, 5, 15);
        

        cv::HoughCircles(Blurred, circles, cv::HOUGH_GRADIENT, 15, 10, 20, 15, 3, 30);
        int a = 1;
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            cv::Point shiftx(0, 12);
            cv::Point shifty(12, 0);
            int radius = cvRound(circles[i][2]);
            // draw the circle center
            //cv::circle(cvResultMask, center, 3, cv::Scalar(255, 255, 255), -1, 8, 0);
            // draw the circle outline
            //cv::circle(cvResultMask, center, radius, cv::Scalar(255, 255, 255), 3, 8, 0);
            cv::line(cvResultMask, center - shiftx, center + shiftx, cv::Scalar(255, 255, 255), 3, 8, 0);
            cv::line(cvResultMask, center - shifty, center + shifty, cv::Scalar(255, 255, 255), 3, 8, 0);
        }
        cvResultMat = Blurred;
        
        // if at least one marker detected
    }

    if (pDepthFrame) {
        pDepthFrame->Release();;
        //delete(pDepth_byte);
        //下面这片内存在画图的时候还要用，因此在画图的时候再回收
        //delete(pAbImage_byte);
        //DELETE(pDepth_byte);
    }
}

void ProcessRmFrameWithCanny(IResearchModeSensorFrame* pSensorFrame, cv::Mat& cvResultMat)
{
    HRESULT hr = S_OK;
    ResearchModeSensorResolution resolution;
    size_t outBufferCount = 0;
    const BYTE *pImage = nullptr;
    IResearchModeSensorVLCFrame *pVLCFrame = nullptr;
    pSensorFrame->GetResolution(&resolution);

    hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    if (SUCCEEDED(hr))
    {
        pVLCFrame->GetBuffer(&pImage, &outBufferCount);

        cv::Mat processed(resolution.Height, resolution.Width, CV_8U, (void*)pImage);

        cv::Canny(processed, cvResultMat, 400, 1000, 5);
    }

    if (pVLCFrame)
    {
        pVLCFrame->Release();
        
    }
}


