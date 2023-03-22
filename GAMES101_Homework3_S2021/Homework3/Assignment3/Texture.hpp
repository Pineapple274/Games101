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

    Eigen::Vector3f getColorBilinear(float u, float v){
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u0 = (int)u_img;
        auto v0 = (int)v_img;
        auto u1 = u0 + 1;
        auto v1 = v0 + 1;
        auto s = u_img - u0;
        auto t = v_img - v0;
        auto c00 = image_data.at<cv::Vec3b>(v0, u0);
        auto c01 = image_data.at<cv::Vec3b>(v0, u1);
        auto c10 = image_data.at<cv::Vec3b>(v1, u0);
        auto c11 = image_data.at<cv::Vec3b>(v1, u1);
        auto c0 = (1 - s) * Eigen::Vector3f(c00[0], c00[1], c00[2]) + s * Eigen::Vector3f(c01[0], c01[1], c01[2]);
        auto c1 = (1 - s) * Eigen::Vector3f(c10[0], c10[1], c10[2]) + s * Eigen::Vector3f(c11[0], c11[1], c11[2]);
        auto c = (1 - t) * c0 + t * c1;
        return Eigen::Vector3f(c[0], c[1], c[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
