/**
 * @file AlgorithmModule.cpp
 * @author Zechariah Wang
 * @brief Helper functions for path-tracking algorithms (Pure Pursuit) and MTP algorithms
 * @version 0.1
 * @date 2023-02-13
 */

#include "main.h"

using namespace Eclipse;

/**
 * @brief setting constructors and default vals 
 * 
 */

CurvePoint::CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
    this->x = x;
    this->y = y;
    this->moveSpeed = moveSpeed;
    this->turnSpeed = turnSpeed;
    this->followDistance = followDistance;
    this->slowDownTurnRadians = slowDownTurnRadians;
    this->slowDownTurnAmount = slowDownTurnAmount;
}

CurvePoint::CurvePoint(const CurvePoint &thisPoint){ // Assigns values to the class
    x = thisPoint.x;
    y = thisPoint.y;
    moveSpeed = thisPoint.moveSpeed;
    turnSpeed = thisPoint.turnSpeed;
    followDistance = thisPoint.followDistance;
    slowDownTurnRadians = thisPoint.slowDownTurnRadians;
    slowDownTurnAmount = thisPoint.slowDownTurnAmount;
}

Point CurvePoint::toPoint(){ // Sets a new point to the current x and y val
    Point newPoint;
    newPoint.setX(x);
    newPoint.setY(y);
    return newPoint;
}

int Point::AngleWrap(double angle){ // Wrap angle to 2 PI (360 degrees)
    while (angle < -M_PI){ angle += 2 * M_PI; }
    while (angle > M_PI){ angle -= 2 * M_PI; }
    return angle;
}

/**
 * @brief Given two points, calculate all possible intersections between look ahead distance
 * 
 */

std::vector<Point> LineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
    if (fabs(linePoint1.getY() - linePoint2.getY()) < 0.003){ linePoint1.setY(linePoint2.getY() + 0.003); }
    if (fabs(linePoint1.getX() - linePoint2.getX()) < 0.003){ linePoint1.setX(linePoint2.getX() + 0.003); }

    double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());
    double b = (linePoint1.getY()) - m1 * (linePoint1.getX());
    double x1 = linePoint1.getX() - circleCenter.getX();
    double y1 = linePoint1.getY() - circleCenter.getY();
    double quadraticA = 1.0 + pow(m1, 2);
    double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);
    double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

    quadraticB = (-2 * utility::get_x()) + (2.0 * m1 * b) - (2 * utility::get_y() * m1);
    quadraticC = pow(utility::get_x(), 2) + pow(b, 2) - (2 * b * utility::get_y()) + pow(utility::get_y(), 2) - pow(radius, 2);

    std::vector<Point> allPoints; double minX; double maxX;
    if (linePoint1.getX() < linePoint2.getX()){ minX = linePoint1.getX(); maxX = linePoint2.getX(); }
    else{ maxX = linePoint1.getX(); minX = linePoint2.getX(); }
    try
    {
        // Solution 1
        double xRoot1 = (-quadraticB + sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot1 = m1 * (xRoot1) + b;
        if (xRoot1 > minX && xRoot1 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot1);
            newPoint.setY(yRoot1);
            allPoints.push_back(newPoint);
        }
        else{} // No Points
        // Solution 2
        double xRoot2 = (-quadraticB - sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot2 = m1 * (xRoot2) + b;
        if (xRoot2 > minX && xRoot2 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot2);
            newPoint.setY(yRoot2);
            allPoints.push_back(newPoint);
        }
        else{} // No points 
    }
    catch (std::exception e){} // No intersection, time to throw exception
    return allPoints;
}

void CurvePoint::setPoint(Point point){ x = point.getX(); y = point.getY(); moveSpeed = point.getMoveSpeed(); turnSpeed = point.getTurnSpeed(); }
double CurvePoint::getFollowDistance(){ return followDistance; }
double CurvePoint::getX(){ return x; }
double CurvePoint::getY(){ return y; }
double CurvePoint::getMoveSpeed(){ return moveSpeed; }
double CurvePoint::getTurnSpeed(){ return turnSpeed; }

/**
 * @brief determines the closest point from the individual point class representing local robot pos, and signals driver to follow path
 * 
 */

CurvePoint getFollowPointPath(std::vector<CurvePoint> pathPoints, Point robotLocation, double followRadius){
    CurvePoint followMe(pathPoints.at(0));
    std::vector<Point> intersections;
    std::vector<Point> intersections2;
    for (int i = 0; i < pathPoints.size() - 1; i++){ 
        CurvePoint startLine = pathPoints.at(i);
        CurvePoint endLine = pathPoints.at(i + 1);
        intersections = LineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
        if (intersections.size() == 1){ followMe.setPoint(intersections.at(0)); }
        else if (intersections.size() == 2){
            Point one = intersections.at(0);
            Point two = intersections.at(1);
            double distanceOne = sqrtf(pow((endLine.getX() - one.getX()), 2) + pow((endLine.getY() - one.getY()), 2));
            double distanceTwo = sqrtf(pow((endLine.getX() - two.getX()), 2) + pow((endLine.getY() - two.getY()), 2));
            if (distanceOne < distanceTwo){ followMe.setPoint(one);}
            else{ followMe.setPoint(two); }
        }
    }
    return followMe;
}

/**
 * @brief Pure Pursuit driver function, determining the closest point to follow as part of the point trajectory defined
 * 
 */

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle, double lkp, double akp, bool reverse){
    Point robotPosition; 
    robotPosition.setX(utility::get_x());
    robotPosition.setY(utility::get_y());

    CurvePoint followMe = getFollowPointPath(allPoints, robotPosition, 15);
    mtp.set_mtp_constants(5, 35, 150, 0, lkp, 110);
    mimic_move_to_point(followMe.getX(), followMe.getY(), reverse);
}


