#ifndef  ARC_WRITE_INC
#define  ARC_WRITE_INC
#include <string>
#include <ostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cv.h>

/*
 * =====================================================================================
 *        Class:  ARC_Write
 *  Description:  Class for writing file output and maintaining IDs.
 * =====================================================================================
 */
class ARC_Write
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ARC_Write ( std::string filename );                             /* constructor */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void set_outfile( std::string f )
        {
            filename = f;
        }

        /* ====================  OPERATORS     ======================================= */
        /* ====================  METHODS       ======================================= */
        void write_matches( std::string frame_name, 
                std::vector<cv::KeyPoint>& source_kpt, std::vector<cv::KeyPoint>& reflection_kpt,
                std::vector<cv::DMatch>& matches );

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        unsigned int get_id( cv::KeyPoint kpt );

        /* ====================  DATA MEMBERS  ======================================= */
        std::string filename;
        unsigned int current_id;
        std::map<std::string, unsigned int> dict;

}; /* -----  end of class ARC_Write  ----- */

#endif   /* ----- #ifndef ARC_Write_INC  ----- */
