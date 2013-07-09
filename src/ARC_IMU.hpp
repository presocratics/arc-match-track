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
        Matx33d get_A()
        {
            return A;
        }

        // ====================  MUTATORS      ======================================= 
        void set_A( Matx33d am )
        {
            A = am;
        }

        // ====================  OPERATORS     ======================================= 

        // ====================  METHODS       ======================================= 
        Point3f poToCr( Point2f obj2d, Matx33d rot_mat );
        Point2f CrTopo( Point3f obj3d, Matx33d rot_mat );
        void poEndpoints( Point2f obj2d, Matx33d rot_mat, Point2f* out_line );
        float calc_slope( Point2f obj2d, Matx33d rot_mat );
        Matx33d calc_rotation_matrix( Point3f imu_data );
    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 
        Matx33d A;

}; // -----  end of class ARC_IMU  ----- 

#endif   // ----- #ifndef ARC_IMU_INC  ----- 
