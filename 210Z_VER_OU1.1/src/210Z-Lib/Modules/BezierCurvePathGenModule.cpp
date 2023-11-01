/**
 * @file BezierCurvePathGenModule.cpp
 * @author Zechariah Wang
 * @brief Path Gen for PP with Bezier Curves
 * @version 0.1
 * @date 2023-10-31
 * 
 */

#include "main.h"

using namespace Eclipse;

std::vector<CurvePoint> Eclipse::BezierCurve::linear_interpolation(Point p0, Point p1, double t, const uint8_t num_points){
    std::vector<CurvePoint> Path;                                                
    for (double i = 0; i < num_points; i++){
        t = i / (num_points - 1);
        double x = (p1.getX() - p0.getX()) * t + p0.getX();
        double y = (p1.getY() - p0.getY()) * t + p0.getY();
        CurvePoint GlobalPoint(x, y, 4, 2, 10, 5, 1);
        Path.push_back(GlobalPoint);
    }
    return Path;
}

std::vector<CurvePoint> Eclipse::BezierCurve::quadratic_bezier_curve(Point p0, Point p1, Point p2, double t, const uint8_t num_points){
    std::vector<CurvePoint> Path;     
    for (double i = 0; i < num_points; i++){
        t = i / (num_points - 1);
        double x = pow(1 - t, 2) * p0.getX() + 2 * (1 - t) * (t  * p1.getX()) + t;
        double y = pow(1 - t, 2) * p0.getY() + 2 * (1 - t) * (t  * p1.getY()) + t;
        CurvePoint GlobalPoint(x, y, 4, 2, 10, 5, 1);
        Path.push_back(GlobalPoint);
    }
    return Path;
}

std::vector<CurvePoint> Eclipse::BezierCurve::cubic_bezier_curve(Point p0, Point p1, Point p2, Point p3, double t, const uint8_t num_points){                          
    std::vector<CurvePoint> Path;     
    for (double i = 0; i < num_points; i++){
        t = i / (num_points - 1);
        double x = pow(1 - t, 3) * p0.getX() + 3 * t * pow(1 - t, 2) * p1.getX() +  3 * pow(t, 2) * p2.getX() + pow(t, 3) * p3.getX();
        double y = pow(1 - t, 3) * p0.getY() + 3 * t * pow(1 - t, 2) * p1.getY() +  3 * pow(t, 2) * p2.getY() + pow(t, 3) * p3.getY();
        CurvePoint GlobalPoint(x, y, 4, 2, 10, 5, 1);
        Path.push_back(GlobalPoint);
    }
    return Path;
}

std::vector<CurvePoint> Eclipse::BezierCurve::quartic_bezier_curve(Point p0, Point p1, Point p2, Point p3, Point p4, double t, const uint8_t num_points){
    std::vector<CurvePoint> Path;     
    for (double i = 0; i < num_points; i++){
        t = i / (num_points - 1);
        double x = (pow(1 - t, 4) * p0.getX()) + (4 * t * pow(1 - t, 3) * p1.getX());
        x = x + (6 * pow(t, 2) * pow(1 - t, 2) * p2.getX()) + (4 * pow(t, 3) * (1-t) * p3.getX()) + (pow(t, 3) * p4.getX());
        double y = (pow(1 - t, 4) * p0.getY()) + (4 * t * pow(1 - t, 3) * p1.getY());
        y = y + (6 * pow(t, 2) * pow(1 - t, 2) * p2.getY()) + (4 * pow(t, 3) * (1-t) * p3.getY()) + (pow(t, 3) * p4.getY());
        CurvePoint GlobalPoint(x, y, 4, 2, 10, 5, 1);
        Path.push_back(GlobalPoint);
    }
    return Path;
}

std::vector<CurvePoint> Eclipse::BezierCurve::quintic_bezier_curve(Point p0, Point p1, Point p2, Point p3, Point p4, Point p5, double t, const uint8_t num_points){
    std::vector<CurvePoint> Path;   
    return Path;
}