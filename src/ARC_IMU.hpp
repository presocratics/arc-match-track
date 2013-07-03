#ifndef  ARC_IMU_INC
#define  ARC_IMU_INC
#include <cv.h>
#include <iostream>
#include "config.hpp"
using namespace cv;
using namespace std;

// =====================================================================================
//        Class:  ARC_IMU
//  Description:  Calculates slopes and transforms using IMU data.
// =====================================================================================
class ARC_IMU
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        //ARC_IMU ( );                             // constructor 

        // ====================  ACCESSORS     ======================================= 
        Mat get_A()
        {
            return A;
        }

        // ====================  MUTATORS      ======================================= 
        void set_A( Mat am )
        {
            A = am;
        }

        // ====================  OPERATORS     ======================================= 

        // ====================  METHODS       ======================================= 
        Point3f poToCr( Point2f obj2d, Mat rot_mat );
        Point2f CrTopo( Point3f obj3d, Mat rot_mat );
        float calc_slope( Point2f obj2d, Mat rot_mat );
        Mat calc_rotation_matrix( Point3f imu_data );
    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 
        Mat A;

}; // -----  end of class ARC_IMU  ----- 

#endif   // ----- #ifndef ARC_IMU_INC  ----- 
