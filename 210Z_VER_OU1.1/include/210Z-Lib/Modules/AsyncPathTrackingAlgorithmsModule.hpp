#include "iostream"
#include "vector"

class Point{
    double xCoord;
    double yCoord;
    double local_moveSpeed;
    double local_turnSpeed;
    public:
        int AngleWrap(double angle);
        void setX(double x) {xCoord = x;}
        void setY(double y) {yCoord = y;}

        void setMoveSpeed(double moveSpeed) {local_moveSpeed = moveSpeed;}
        void setTurnSpeed(double turnSpeed) {local_turnSpeed = turnSpeed;}

        double getX(){
            return xCoord;
        }
        double getY(){
            return yCoord;
        }
        double getMoveSpeed(){
            return local_moveSpeed;
        }
        double getTurnSpeed(){
            return local_turnSpeed;
        }
};

class CurvePoint{
    private:
        bool init;
    public:

        double x;
        double y;
        double moveSpeed;
        double turnSpeed;
        double followDistance;
        double slowDownTurnRadians;
        double slowDownTurnAmount;
        
        CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount);
        CurvePoint(const CurvePoint &thisPoint);

        Point toPoint();
        void setPoint(Point point);
        double getFollowDistance();
        double getX();
        double getY();
        double getMoveSpeed();
        double getTurnSpeed();

};

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle, double move_speed, double turn_speed);