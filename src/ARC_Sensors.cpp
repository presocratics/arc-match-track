/*
 * =====================================================================================
 *
 *       Filename:  ARC_Sensors.cpp
 *
 *    Description:  Sensors class. For camera and IMU
 *
 *        Version:  1.0
 *        Created:  06/11/2014 11:23:07 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */
#include "ARC_Sensors.hpp"

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Sensors
 *      Method:  ARC_Sensors :: ARC_Sensors
 * Description:  Constructor
 *--------------------------------------------------------------------------------------
 */
    
ARC_Sensors::ARC_Sensors ( const char *camsrc, const char *imusrc )
{
    set_camera_source( camsrc );
    set_imu_source( imusrc );
    return ;
}		/* -----  end of method ARC_Sensors::ARC_Sensors  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Sensors
 *      Method:  ARC_Sensors :: set_camera_source
 * Description:  Sets filename for camera data.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_Sensors::set_camera_source ( const char *fn )
{
    // TODO mkfifo
    strncpy(camera_fn, fn, MAXLINE);
    return ;
}		/* -----  end of method ARC_Sensors::set_camera_source  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Sensors
 *      Method:  ARC_Sensors :: set_imu_source
 * Description:  Sets filename for imu data.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_Sensors::set_imu_source ( const char *fn )
{
    // TODO mkfifo
    strncpy(imu_fn, fn, MAXLINE);
    return ;
}		/* -----  end of method ARC_Sensors::set_imu_source  ----- */


    int
ARC_Sensors::update ( )
{
    return update_imu();
}		/* -----  end of method ARC_Sensors::update  ----- */


    int
ARC_Sensors::update_imu ( )
{
    FILE *imu_fd;
    imu_fd = fopen( imu_fn, "r" );
    fscanf( imu_fd, "%lf,%lf,%lf", &imu[0], &imu[1], &imu[2] );
    fclose(imu_fd);
    return 0;
}		/* -----  end of method ARC_Sensors::update_imu  ----- */

