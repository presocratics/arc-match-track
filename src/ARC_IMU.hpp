#ifndef  arc_imu_INC
#define  ARC_IMU_INC
#include "config.hpp"

// =====================================================================================
//        Class:  ARC_IMU
//  Description:  Calculates slopes and transforms using IMU data.
// =====================================================================================
class ARC_IMU
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_IMU ();                             // constructor 

        // ====================  ACCESSORS     ======================================= 

        // ====================  MUTATORS      ======================================= 
        void set_A( Mat am )
        {
            A = am;
        }

        // ====================  OPERATORS     ======================================= 

        // ====================  METHODS       ======================================= 
        Point3f 2dToCr( Point2f obj2d, Point3f imu_data );
        Point2f CrTo2d( Point3f obj3d, Point3f imu_data );
        float calc_slope( Point2f obj2d, Point3f imu_data );
    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 
        Mat A;

}; // -----  end of class ARC_IMU  ----- 

#endif   // ----- #ifndef ARC_IMU_INC  ----- 
