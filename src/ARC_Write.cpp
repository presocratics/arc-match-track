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


ARC_Write::ARC_Write ( std::string f )
{
    current_id = 0;
    filename = f;
    return ;
}		/* -----  end of method ARC_Write::ARC_Write  ----- */

    void
ARC_Write::write_matches ( std::string frame_name,
        std::vector<cv::KeyPoint>& source_kpt, std::vector<cv::KeyPoint>& reflection_kpt,
        std::vector<cv::DMatch>& matches )
{

    std::string    ofs_file_name = filename;                 /* output file name */
    std::ofstream  ofs;                                /* create ofstream object */

    ofs.open ( ofs_file_name.c_str(), std::ofstream::out | std::ofstream::app );           /* open ofstream */
    if (!ofs) {
        std::cerr << "\nERROR : failed to open output file " << ofs_file_name << std::endl;
        exit (EXIT_FAILURE);
    }
    for( std::vector<cv::DMatch>::iterator it=matches.begin() ;
            it!=matches.end(); ++it )
    {
        unsigned int id;
        cv::KeyPoint sk = source_kpt[ it->queryIdx ];
        cv::KeyPoint rk = reflection_kpt[ it->queryIdx ];
        id = get_id( sk );
        std::cout << frame_name << spc
             << id << spc
             << sk.pt.x << spc << sk.pt.y << spc
             << rk.pt.x << spc << rk.pt.y << std::endl;
    }
    ofs.close ();                                 /* close ofstream */
    return ;
}		/* -----  end of method ARC_Write::write_matches  ----- */


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
}		/* -----  end of method ARC_Write::get_id  ----- */

