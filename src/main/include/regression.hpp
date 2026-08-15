#pragma once
#include <vector>

class Regression {
    public:
        static double PerformAngleRegression(double distance){
            double angle = 0.0;



            for (double coeff : angleCoefficients) {
                angle = angle * distance + coeff;
            }

            return angle;
        }
        private:
        static std::vector<double> angleCoefficients;
};
inline std::vector<double> Regression::angleCoefficients = {36.53318, -15.62686};
