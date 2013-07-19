/*
 * =====================================================================================
 *
 *       Filename:  ARC_Write.cpp
 *
 *    Description:  ARC Class for writing matched pair coordinates.
 *
 *        Version:  1.0
 *        Created:  06/25/2013 09:19:49 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), miller7@illinois.edu
 *   Organization:  ARC Lab
 *
 * =====================================================================================
 */
#include "ARC_Write.hpp"
#include "config.hpp"


/*
ARC_Write::ARC_Write ( std::string f )
{
    current_id = 0;
    filename = f;

    std::ofstream  ofs;                                // create ofstream object 

    ofs.open ( filename.c_str() );           // open ofstream 
    if (!ofs) {
        cerr << "\nERROR : failed to open output file " << filename << endl;
        exit (EXIT_FAILURE);
    }
    // This tests that we can open the file, and clear any existing, so we can
    // later write with append without appending to some existing thing.
    ofs.close ();                                 // close ofstream 
    return ;
}*/		/* -----  end of method ARC_Write::ARC_Write  ----- */

    void
ARC_Write::write_matches ( string frame_name,
        std::vector<cv::KeyPoint>& source_kpt, std::vector<cv::KeyPoint>& reflection_kpt,
        std::vector<cv::DMatch>& matches, Rect roi )
{
//TODO support directions.
    std::string    ofs_file_name = filename;                 /* output file name */
    std::ofstream  ofs;                                /* create ofstream object */

    ofs.open ( ofs_file_name.c_str(), std::ofstream::out | std::ofstream::app );           /* open ofstream */
    if (!ofs) {
        std::cerr << "\nERROR : failed to open output file " << ofs_file_name << std::endl;
        exit (EXIT_FAILURE);
    }
    PPC good_points;
    /*
    keypoints_to_goodpoints( source_kpt, reflection_kpt,
            &good_points.source, &good_points.reflection,
            matches, roi, DOWN );
    size_t i=0;
    const char* c_str = frame_name.c_str();
    int frame_num = atoi( basename(c_str) );
    for( std::vector<cv::DMatch>::iterator it=matches.begin() ;
            it!=matches.end(); ++it, ++i )
    {
        unsigned int id;
        cv::KeyPoint sk = source_kpt[ it->trainIdx ];
        cv::Point2f sp = good_points.source[ i ];
        cv::Point2f rp = good_points.reflection[ i ];
        id = get_id( sk );
        ofs << frame_num << spc
             << id << spc
             << sp.x << spc << sp.y << spc
             << rp.x << spc << rp.y << std::endl;
    }
        */
    ofs.close ();                                 /* close ofstream */
    return ;
}		/* -----  end of method ARC_Write::write_matches  ----- */

/*
    unsigned int
ARC_Write::get_id ( cv::KeyPoint kpt )
{
    std::stringstream key_ss;
    std::string key;
    std::map<std::string, unsigned int>::iterator id_ptr;
    std::stringstream hash_input;
    key_ss << kpt.size << kpt.angle << kpt.response << kpt.octave;
    key = key_ss.str();
    if( (id_ptr=dict.find( key )) != dict.end() )
        return id_ptr->second;
    dict.insert( make_pair(key, ++current_id) );
    return current_id;
}*/		/* -----  end of method ARC_Write::get_id  ----- */

