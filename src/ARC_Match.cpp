/*
 * =====================================================================================
 *
 *       Filename:  ARC_Match.cpp
 *
 *    Description:  Matches an object to a scene using the specified feature
 *                  detector, matcher, and filters.
 *
 *                  Much of this code is based on
 *                  "OpenCV 2 Computer Vision Application Programming
 *                  Cookbook"
 *                  Chapter 9, section: "Matching images using random sample
 *                  consensus"
 *
 *                  URL:
 *                  http://proquest.safaribooksonline.com.proxy2.library.illinois.edu/
 *                  book/photo-and-graphic-manipulation/9781849513241/
 *                  estimating-projective-relations-in-images/ch09lvl1sec59
 *
 *        Version:  1.0
 *        Created:  06/09/2013 01:39:22 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), miller7@illinois.edu 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <ARC_Match.hpp>
#include <iostream>
#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#define ANGLE_HIGH_DEV 1.9 // Max dx/dy deviation
#define ANGLE_LOW_DEV 0.60 // Min dx/dy deviation
using namespace cv;
using std::vector;


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Match
 *      Method:  ARC_Match
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
ARC_Match::ARC_Match (void)
{
    detector = new SurfFeatureDetector();
    extractor = new SurfDescriptorExtractor();
    matcher = new FlannBasedMatcher();
    confidence=-1;
    ratio=-1;
    distance=-1;
    refineF=false;
}  /* -----  end of method ARC_Match::ARC_Match  (constructor)  ----- */

    void 
ARC_Match::angle_test(
    const vector<KeyPoint>& keypoints_scene,
    const vector<KeyPoint>& keypoints_object,
    vector<DMatch>& matches, 
    vector<DMatch>& good_matches )
{
    Point2f sums(0, 0);
    Point2f averages, delta;
    double upper_x, upper_y, lower_x, lower_y;

    for( size_t i = 0; i < matches.size(); i++)
    {
        Point2f point1 = keypoints_scene[matches[i].queryIdx].pt;
        Point2f point2 = keypoints_object[matches[i].trainIdx].pt;

        sums+=(point1-point2);
    }
    averages.x=sums.x/matches.size();
    averages.y=sums.y/matches.size();
    /*
    if( verbosity>=VERY_VERY_VERBOSE )
    {
        cout << "angle_filter: Avg x: " << averages.x << endl;
        cout << "angle_filter: Avg y: " << averages.y << endl;
    }
    */

    vector<DMatch>::iterator match_iterator=matches.begin() ;
    while( match_iterator!=matches.end() )
    {
        Point2f point1 = keypoints_scene[match_iterator->queryIdx].pt;
        Point2f point2 = keypoints_object[match_iterator->trainIdx].pt;
        delta=point1 - point2;
        /*
        if( verbosity>=VERY_VERBOSE )
        {
            cout << "angle_filter: delta.x: " << delta.x << endl;
            cout << "angle_filter: delta.y: " << delta.y << endl;
        }
        */
        upper_x = fmax(averages.x * ANGLE_HIGH_DEV, averages.x * ANGLE_LOW_DEV);
        lower_x = fmin(averages.x * ANGLE_HIGH_DEV, averages.x * ANGLE_LOW_DEV);

        upper_y = fmax(averages.y * ANGLE_HIGH_DEV, averages.y * ANGLE_LOW_DEV);
        lower_y = fmin(averages.y * ANGLE_HIGH_DEV, averages.y * ANGLE_LOW_DEV);

        if( delta.x < upper_x &&
            delta.x > lower_x  &&
            delta.y < upper_y &&
            delta.y > lower_y )
        {
            good_matches.push_back(*match_iterator);
            ++match_iterator;
            //current_segment->good_matches.push_back(current_segment->matches[i]); 
            //if( verbosity>=VERY_VERY_VERBOSE  ) cout << "angle_filter: push" << endl;
        }
        else
        {
            match_iterator++;
        }
    }
    return;
}

    Mat
ARC_Match::ransac_test (
        const vector<DMatch>& matches,
        const vector<KeyPoint>& keypoints_scene,
        const vector<KeyPoint>& keypoints_object,
        vector<DMatch>& out_matches )
{
    if( distance==-1 || confidence==-1 )
    {
        cerr << "Distance or confidence not set." << endl;
        return Mat();
    }
    // Convert keypoints into Point2f
    vector<Point2f> points_scene, points_object;
    for( vector<DMatch>::const_iterator it=matches.begin();
            it!=matches.end(); ++it )
    {
        points_scene.push_back( (Point2f) keypoints_scene[it->queryIdx].pt );
        points_object.push_back( (Point2f) keypoints_object[it->trainIdx].pt );
    }

    // Compute fundamental matrix using RANSAC
    vector<uchar> inliers( points_scene.size(), 0 );
    Mat fundamental = findFundamentalMat(
            Mat(points_scene),
            Mat(points_object),
            inliers,
            CV_FM_RANSAC,
            distance,
            confidence );
    
    // extract the remaining inliers
    vector<uchar>::const_iterator itIn=inliers.begin();
    vector<DMatch>::const_iterator itM=matches.begin();
    for( ; itIn!=inliers.end(); ++itIn, ++itM )
    {
        if( *itIn )
            out_matches.push_back(*itM);
    }

    if( refineF )
    {
        points_scene.clear();
        points_object.clear();

        for( vector<DMatch>::const_iterator it=out_matches.begin();
                it!=out_matches.end(); ++it )
        {
            points_scene.push_back( (Point2f) keypoints_scene[it->queryIdx].pt );
            points_object.push_back( (Point2f) keypoints_object[it->trainIdx].pt );
        }
        fundamental=findFundamentalMat(
                Mat(points_scene),
                Mat(points_object),
                CV_FM_8POINT);
    }
    return fundamental;
}		/* -----  end of method ARC_Match::ransac_test  ----- */

    void
ARC_Match::symmetry_test ( 
        const vector<vector<DMatch> >& matches_scene,
        const vector<vector<DMatch> >& matches_object,
        vector<DMatch>& sym_matches )
{
    // for all scene->object matches
    for( vector<vector<DMatch> >::const_iterator match_iterator_scene = matches_scene.begin() ;
            match_iterator_scene!=matches_scene.end() ; ++match_iterator_scene )
    {
        if( match_iterator_scene->size() < 2 ) // Ignores elements cleared in previous step.
            continue;
        // for all object->scene matches
        for( vector<vector<DMatch> >::const_iterator match_iterator_object=matches_object.begin() ;
                match_iterator_object!=matches_object.end() ; ++match_iterator_object )
        {
            if( match_iterator_object->size() < 2 ) // Ignores elements cleared in previous step.
                continue;
            // symmetry test
            if( (*match_iterator_scene)[0].queryIdx
                    ==(*match_iterator_object)[0].trainIdx
                    &&(*match_iterator_object)[0].queryIdx
                    ==(*match_iterator_scene)[0].trainIdx)
            {
                sym_matches.push_back(
                        DMatch( (*match_iterator_scene)[0].queryIdx,
                            (*match_iterator_scene)[0].trainIdx,
                            (*match_iterator_scene)[0].distance) );
                break; // found a symmtrical match, go to next scene->object.
            }
        }

    }
    return ;
}		/* -----  end of method ARC_Match::symmetry_test  ----- */

    int
ARC_Match::ratio_test ( vector<vector<DMatch> > &matches)
{
    if( ratio==-1 )
    {
        cerr << "Ratio not set" << endl;
        return -1;
    }
    int removed=0;

    for( vector<vector<DMatch> >::iterator match_iterator=matches.begin() ;
            match_iterator!=matches.end() ; ++match_iterator )
    {
        // Continue if 2 NN were found. Remove otherwise.
        if( match_iterator->size() > 1 )
        {
            // If distance of #1 match is not small enough compared to distance
            // of #2 we clear it so that size is 0.
            if( (*match_iterator)[0].distance > (*match_iterator)[1].distance * ratio )
            {
                match_iterator->clear() ;
                removed++;
            }
        }
        else
        {
            match_iterator->clear() ;
            removed++;
        }
    }
    return removed;
}		/* -----  end of method ARC_Match::ratio_test  ----- */

    bool
ARC_Match::match ( Mat& scene_img, Mat& object_img,
            Mat& scene_mask, Mat& object_mask,
            vector<DMatch>& matches,
            vector<KeyPoint>& keypoints_scene,
            vector<KeyPoint>& keypoints_object )
{

    matches.clear();
    Mat descriptors_scene, descriptors_object;
    // Only update object keypoints if we don't have any.
    if( keypoints_object.size()==0 )
    {
        keypoints_object.clear();
        detector->detect( object_img, keypoints_object );
    }
    //keypoints_object.clear();
    //detector->detect( object_img, keypoints_object );
    extractor->compute( object_img, keypoints_object, descriptors_object );

    // Detect features
    keypoints_scene.clear();
    detector->detect( scene_img, keypoints_scene );
    extractor->compute( scene_img, keypoints_scene, descriptors_scene );

    // Match descriptors
    // find 2 nearest-neighbors for each match point (the 2 most likely matches)

    // From scene->object
    vector<vector<DMatch> > matches_scene;
    matcher->knnMatch( descriptors_scene, descriptors_object, matches_scene, 2 );

    // From object->scene
    vector<vector<DMatch> > matches_object;
    matcher->knnMatch( descriptors_object, descriptors_scene, matches_object, 2 );

    // Perform ratio test. If the 2 nearest neighbors are of similar
    // distance(match quality), then the match is ambiguous, so we discard.
    // Only matches that are much better than the second best match are kept.
    ratio_test( matches_scene );
    ratio_test( matches_object );

    // Perform symmetry test. For a match pair to be accepted both points
    // must be the best matching feature of the other. This is to say we
    // accept a match pair of A chose B in matches_scene and B chose A in
    // matches_object.
    vector<DMatch> sym_matches;
    symmetry_test( matches_scene, matches_object, sym_matches );
    cout << "match: sym_matches: " << sym_matches.size() << endl;
    //matches=sym_matches;
    if( sym_matches.size()<min_points ) return false;

    ransac_test( sym_matches, keypoints_scene, keypoints_object, matches );
    /*
    // TODO: This is not calculating E[X] and Var(X) correctly.
    Moments mome = moments( points_scene, false );
    Point2f scm( mome.m10/mome.m00, mome.m01/mome.m00 );
    Point2f stddev( sqrt(abs(mome.mu20/mome.m00)), sqrt(abs(mome.mu02/mome.m00)) );
    if( a.debug == DEBUG )
        cout << "Normalized CM: " << mome.nu20 << ", " << mome.nu02<< endl;
        */
    /*
    // Create an ellipse shaped contour
    Mat ellipse_mask;
    ellipse_mask=Mat::zeros(cur_frame.size(), CV_8UC1);
    ellipse(ellipse_mask, scm, (Size) (3 * stddev), 0, 0, 360, 255, 1, 8);
    vector<vector<Point> > c;
    findContours( ellipse_mask, c, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    // Test if there are any contours (No contour if our ellipse was bad).
    if( c.size() < 1 )
        continue;
    // Remove points that are not in the contour.
    vector<Point2f>::iterator it=points_scene.begin() ;
    while( it!=points_scene.end() )
    {
        if( pointPolygonTest(c[0], *it, false)>=0 )
        {
            circle( cur_frame,*it,2,Scalar(255) );
            ++it;
        }
        else
        {
            it=points_scene.erase(it);
        }
    }
    */
    return true;
}		/* -----  end of method ARC_Match::match  ----- */

