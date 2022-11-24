#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "curve_gen/cubic_curve.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
struct gline
{
    gline(double x1, double y1, double x2, double y2): x1(x1), y1(y1), x2(x2), y2(y2){}
    double x1;
    double y1;
    double x2;
    double y2;
};
// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    ros::Publisher wayPointsPub;
    ros::Publisher curvePub;
    ros::Publisher diskPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        curvePub = nh.advertise<visualization_msgs::Marker>("/visualizer/curve", 10);
        diskPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/disk", 1000);
    }

    inline void visualize(const CubicCurve &curve)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "odom";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.20;

        if (curve.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = curve.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = 0.0;
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (curve.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector2d lastX = curve.getPos(0.0);
            for (double t = T; t < curve.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector2d X = curve.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = 0.0;
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = 0.0;
                trajMarker.points.push_back(point);
                lastX = X;
            }
            curvePub.publish(trajMarker);
        }
    }

    inline void visualizeDisks(const Eigen::Matrix2Xd &disk)
    {
        visualization_msgs::Marker diskMarker;
        visualization_msgs::MarkerArray diskMarkers;
//
//        diskMarker.type = visualization_msgs::Marker::CYLINDER;
//        diskMarker.header.stamp = ros::Time::now();
//        diskMarker.header.frame_id = "odom";
//        diskMarker.pose.orientation.w = 1.00;
//        diskMarker.action = visualization_msgs::Marker::ADD;
//        diskMarker.ns = "disk";
//        diskMarker.color.r = 1.00;
//        diskMarker.color.g = 0.00;
//        diskMarker.color.b = 0.00;
//        diskMarker.color.a = 1.00;
//
//        for (int i = 0; i < 1; ++i)
//        {
//            diskMarker.id = i;
//            diskMarker.pose.position.x = disk(0);
//            diskMarker.pose.position.y = disk(1);
//            diskMarker.pose.position.z = 0.5;
//            diskMarker.scale.x = disk(2) * 2.0;
//            diskMarker.scale.y = disk(2) * 2.0;
//            diskMarker.scale.z = 1.0;
//            diskMarkers.markers.push_back(diskMarker);
//        }
//
//        diskPub.publish(diskMarkers);

        geometry_msgs::Point p;
        p.x = 1;
        p.y = 1;
        p.z = 1;
        geometry_msgs::Point p1;
        p1.x = 3;
        p1.y = 1;
        p1.z = 1;
        geometry_msgs::Point p2;
        p2.x = 3;
        p2.y = 2;
        p2.z = 1;
        geometry_msgs::Point p3;
        p3.x = 1;
        p3.y = 2;
        p3.z = 1;
        geometry_msgs::Point p4;
        p4.x = 1;
        p4.y = 1;
        p4.z = 7;
        geometry_msgs::Point p5;
        p5.x = 3;
        p5.y = 1;
        p5.z = 7;
        geometry_msgs::Point p6;
        p6.x = 3;
        p6.y = 2;
        p6.z = 7;
        geometry_msgs::Point p7;
        p7.x = 1;
        p7.y = 2;
        p7.z = 7;
        visualization_msgs::Marker line_list;

        diskMarker.header.frame_id = "odom";
        diskMarker.lifetime = ros::Duration(0.5);
        diskMarker.ns = "lines";
        diskMarker.action = visualization_msgs::Marker::ADD;
        diskMarker.pose.orientation.w = 1.0;
        diskMarker.id = 2;
        diskMarker.type = visualization_msgs::Marker::LINE_LIST;
        diskMarker.scale.x = 0.1;
        // Line list is red
        diskMarker.color.r = 1.0;
        diskMarker.color.a = 1.0;
//        diskMarker.points.push_back(p);
//        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p3);
//        diskMarker.points.push_back(p3);
//        diskMarker.points.push_back(p);
//
//        diskMarker.points.push_back(p4);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p4);
//
//        diskMarker.points.push_back(p4);
//        diskMarker.points.push_back(p);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p3);
//        diskMarkers.markers.push_back(diskMarker);
//        diskPub.publish(diskMarkers);
        gline line1(0,0, 200, 200);
        gline line2(200,200, 100, 300);
        gline line3(100,300, 200, 200);
        std::vector<gline> m_gline;
        m_gline.push_back(line1);
        m_gline.push_back(line2);
        m_gline.push_back(line3);
//        diskMarker.header.frame_id = "odom";
//        diskMarker.lifetime = ros::Duration(0.5);
//        diskMarker.ns = "lines";
//        diskMarker.action = visualization_msgs::Marker::ADD;
//        diskMarker.pose.orientation.w = 1.0;
//        diskMarker.id = 2;
//        diskMarker.type = visualization_msgs::Marker::LINE_LIST;
//        diskMarker.scale.x = 0.1;
//        // Line list is red
//        diskMarker.color.r = 1.0;
//        diskMarker.color.a = 1.0;
        for (std::vector<gline>::const_iterator cit = m_gline.begin(); cit != m_gline.end(); ++cit)
        {
            geometry_msgs::Point p_start;
            p_start.x = cit->x1;
            p_start.y = cit->y1;
            p_start.z = 10;
            diskMarker.points.push_back(p_start);
            geometry_msgs::Point p_end;
            p_end.x = cit->x2;
            p_end.y = cit->y2;
            p_end.z = 10;
            diskMarker.points.push_back(p_end);
        }
        diskMarker.points.push_back(p);
        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p3);
//        diskMarker.points.push_back(p3);
//        diskMarker.points.push_back(p);

//        diskMarker.points.push_back(p4);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p4);
//
//        diskMarker.points.push_back(p4);
//        diskMarker.points.push_back(p);
//        diskMarker.points.push_back(p5);
//        diskMarker.points.push_back(p1);
//        diskMarker.points.push_back(p6);
//        diskMarker.points.push_back(p2);
//        diskMarker.points.push_back(p7);
//        diskMarker.points.push_back(p3);
        diskMarkers.markers.push_back(diskMarker);
        diskPub.publish(diskMarkers);
    }
};

#endif