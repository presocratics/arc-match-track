#ifndef  arc_sensors_INC
#define  ARC_Sensors_INC
#include <cv.h>
#include <cstring>
#include <cerrno>
#include "config.hpp"
/*
 * =====================================================================================
 *        Class:  ARC_Sensors
 *  Description:  
 * =====================================================================================
 */
class ARC_Sensors
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ARC_Sensors ( const char *camsrc, const char *imusrc );         /* constructor */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void set_camera_source( const char *fn );
        void set_imu_source( const char *fn );
        int update();

        /* ====================  OPERATORS     ======================================= */

        /*  ===================  PUBLIC DATA   ============================= */
        cv::Mat image;
        cv::Vec3d imu;

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        int update_imu();

        /* ====================  DATA MEMBERS  ======================================= */
        char camera_fn[MAXLINE], imu_fn[MAXLINE];



}; /* -----  end of class ARC_Sensors  ----- */

#endif   /* ----- #ifndef ARC_Sensors_INC  ----- */
