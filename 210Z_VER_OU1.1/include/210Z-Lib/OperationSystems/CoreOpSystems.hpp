#include "iostream"

namespace Eclipse{
    class MatchMovement{
        private:
            bool init = true;
        public:
            double p_set = 0.8; // Power set
            double launch_iterator = 0; // Launcher counter
            bool l_stat = true; // Launch status
            double it_ps = 1;
            bool robotBrakeType = false;

            const double maxPower = 100;
            const double halfPower = 75;
            const short int lowPower = 50;
            const double DriveTrainMultiplier = 94;
            const double FrontDriveTrainMultiplier = 94;
            const double BackDriveTrainMultiplier = 94;

            void dt_Control();
            void exponential_curve_accelerator();
            void x_drive_dt_Control();
    };
}