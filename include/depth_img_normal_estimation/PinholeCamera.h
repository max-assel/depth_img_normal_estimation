#pragma once

// #include <ros/ros.h>
// #include "rclcpp/rclcpp.hpp"

struct PinholeCamera
{
    float fx; // focal length x
    float fy; // focal length y
    float inv_fx; // 1 / focal length x (for efficiency)
    float inv_fy; // 1 / focal length y (for efficiency)
    float u_0; // optical center x
    float v_0; // optical center y
    // float k1; // radial distortion coefficient
    // float k2; // radial distortion coefficient
    // float k3; // radial distortion coefficient
    // float p1; // tangential distortion coefficient
    // float p2; // tangential distortion coefficient
    int16_t width; // image width (in pixels)
    int16_t height; // image height (in pixels)

    PinholeCamera()
    {
        // K =  [fx, 0, u_0;]
        //      [0, fy, v_0;]
        //      [0, 0, 1]
        fx = 194.46678161621094;
        fy = 194.46678161621094;
        inv_fx = 1.0 / fx;
        inv_fy = 1.0 / fy;
        width = 320;
        height = 240;
        u_0 = 161.86795043945312;
        v_0 = 120.33906555175781;
        // k1 = 0.0;
        // k2 = 0.0;
        // k3 = 0.0;
        // p1 = 0.0;
        // p2 = 0.0;
    }

    PinholeCamera(float fx, float fy, 
                    float u_0, float v_0, 
                    // float k1, float k2, float k3, float p1, float p2,
                    float width, float height)
    {
        this->fx = fx;
        this->fy = fy;
        this->u_0 = u_0;
        this->v_0 = v_0;
        // this->k1 = k1;
        // this->k2 = k2;
        // this->k3 = k3;
        // this->p1 = p1;
        // this->p2 = p2;
        this->width = width;
        this->height = height;
    }
};