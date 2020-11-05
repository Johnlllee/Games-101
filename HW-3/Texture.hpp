//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        float x = u * width;
        float y = (1 - v) * height;
        int left = std::max(0, std::min((int)std::floor(x), width-1));
        int right = std::max(0, std::min((int)std::ceil(x), width-1));
        int top = std::max(0, std::min((int)std::floor(y), height-1));
        int bottom = std::max(0, std::min((int)std::ceil(y), height-1));
        auto ltc = image_data.at<cv::Vec3b>(top, left);
        auto lbc = image_data.at<cv::Vec3b>(bottom, left);
        auto rtc = image_data.at<cv::Vec3b>(top, right);
        auto rbc = image_data.at<cv::Vec3b>(bottom, right);
        auto ltcf = Eigen::Vector3f(ltc[0], ltc[1], ltc[2]);
        auto lbcf = Eigen::Vector3f(lbc[0], lbc[1], lbc[2]);
        auto rtcf = Eigen::Vector3f(rtc[0], rtc[1], rtc[2]);
        auto rbcf = Eigen::Vector3f(rbc[0], rbc[1], rbc[2]);
        auto xbcb = (lbcf*(right-x) + rbcf*(x-left))/(right-left); // x_bilinear_color_bottom
        auto xbct = (ltcf*(right-x) + rtcf*(x-left))/(right-left); // x_bilinear_color_top
        Eigen::Vector3f ybc = (xbct*(bottom-y) + xbcb*(y-top))/(bottom - top); // y_bilinear_color
        ybc[0] = std::max(0.f, std::min(ybc[0], 255.f));
        ybc[1] = std::max(0.f, std::min(ybc[1], 255.f));
        ybc[2] = std::max(0.f, std::min(ybc[2], 255.f));
        return ybc;
    }

};
#endif //RASTERIZER_TEXTURE_H
