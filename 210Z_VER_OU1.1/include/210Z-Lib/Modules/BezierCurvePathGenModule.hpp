#include "iostream"
#include "api.h"
#pragma once
#include "main.h"

namespace Eclipse{
    class BezierCurve {
        private:
            bool init;
        public:
            std::vector<CurvePoint> linear_interpolation(Point p0, Point p1, double t, const uint8_t num_points);
            std::vector<CurvePoint> quadratic_bezier_curve(Point p0, Point p1, Point p2, double t, const uint8_t num_points);
            std::vector<CurvePoint> cubic_bezier_curve(Point p0, Point p1, Point p2, Point p3, double t, const uint8_t num_points);
            std::vector<CurvePoint> quartic_bezier_curve(Point p0, Point p1, Point p2, Point p3, Point p4, double t, const uint8_t num_points);
            std::vector<CurvePoint> quintic_bezier_curve(Point p0, Point p1, Point p2, Point p3, Point p4, Point p5, double t, const uint8_t num_points);


    };
}
