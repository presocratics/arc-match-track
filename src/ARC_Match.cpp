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

#include "ARC_Match.hpp"
#define ANGLE_HIGH_DEV 1.9 // Max dx/dy deviation
#define ANGLE_LOW_DEV 0.60 // Min dx/dy deviation


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
    isRatio = true;
    isSym = true;
    isRansac = true;
    verbosity = NOT_VERBOSE;
}  /* -----  end of method ARC_Match::ARC_Match  (constructor)  ----- */



    float
ARC_Match::median ( float list[], int n )
{
    std::priority_queue<float> vals;
    for( int i=0; i<n; ++i )
    {
        vals.push( list[i] );
    }
    int k = n/2;
    while( k-->0 ) vals.pop();
    return vals.top();
}		// -----  end of method ARC_Match::median  ----- 

//--------------------------------------------------------------------------------------
//       Class:  ARC_Match
//      Method:  angle_test :: angle_test
// Description:  Filters matches that deviate too far from average dx, dy.
//               NOTE: This method does not currently work!
//--------------------------------------------------------------------------------------
    void 
ARC_Match::angle_test(
    const vector<KeyPoint>& keypoints_scene,
    const vector<KeyPoint>& keypoints_object,
    vector<DMatch>& matches, 
    vector<DMatch>& good_matches )
{
    // Get median x and y.
    float *Xs, *Ys;

    Xs	= (float*) calloc ( (size_t)( matches.size() ), sizeof(float) );
    if ( Xs==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    Ys	= (float*) calloc ( (size_t)( matches.size() ), sizeof(float) );
    if ( Ys==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    vector<Point2f> deltas;
    for( size_t i = 0; i < matches.size(); i++)
    {
        Point2f point1 = keypoints_scene[matches[i].queryIdx].pt;
        Point2f point2 = keypoints_object[matches[i].trainIdx].pt;

        Point2f diff=(point1-point2);
        Xs[i] = diff.x ;
        Ys[i] =  diff.y ;
        deltas.push_back( diff );
    }
    float medianX = median( Xs, matches.size() );
    float medianY = median( Ys, matches.size() );
    Xs	= NULL;
    free (Xs);
    Ys	= NULL;
    free (Ys);

        int k = matches.size()/2;
        cout << k << "th x: " << medianX << endl;
        cout << k << "th y: " << medianY << endl;

    // Remove outliers.
    float upper_x, lower_x, upper_y, lower_y;
    upper_x = fmax(medianX * ANGLE_HIGH_DEV, medianX * ANGLE_LOW_DEV);
    lower_x = fmin(medianX * ANGLE_HIGH_DEV, medianX * ANGLE_LOW_DEV);

    upper_y = fmax(medianY * ANGLE_HIGH_DEV, medianY * ANGLE_LOW_DEV);
    lower_y = fmin(medianY * ANGLE_HIGH_DEV, medianY * ANGLE_LOW_DEV);
    for( size_t i=0; i<matches.size(); ++i )
    {
        Point2f delta = deltas[i];
        if( delta.x < upper_x &&
            delta.x > lower_x  &&
            delta.y < upper_y &&
            delta.y > lower_y )
        {
            good_matches.push_back( matches[i] );
        }
    }
    return;
}

//--------------------------------------------------------------------------------------
//       Class:  ARC_Match
//      Method:  ARC_Match :: ransac_test
// Description:  Performs RANSAC test to remove outliers.
//--------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------
//       Class:  ARC_Match
//      Method:  ARC_Match :: symmetry_test
// Description:  Takes matches from knnMatch run in both directions,
// source->scene and scene->source. Keeps only those matches that have
// agreement in both sets and stores them in sym_matches.
//--------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------
//       Class:  ARC_Match
//      Method:  ARC_Match :: ratio_test
// Description:  Takes a set of matches from 2 neighbor knnMatch. Compares each
// pair of matches, and removes the pair if the distance between the two is too
// similar. "Too similar" is defined by ARC_Match::ratio.
//--------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------
//       Class:  ARC_Match
//      Method:  ARC_Match :: match
// Description:  Runs the matching algorithm between and object and a scene.
// Ratio, symmetry, and RANSAC tests are performed as specified by the user.
//--------------------------------------------------------------------------------------
    bool
ARC_Match::match ( Mat& scene_img, Mat& object_img,
            Mat& scene_mask, Mat& object_mask,
            vector<DMatch>& matches,
            vector<KeyPoint>& keypoints_scene,
            vector<KeyPoint>& keypoints_object )
{

    matches.clear();
    Mat descriptors_scene, descriptors_object;
    // Only update object keypoints if we have too few.
    if( keypoints_object.size()<3 )
    {
        keypoints_object.clear();
        detector->detect( object_img, keypoints_object );
    }
    extractor->compute( object_img, keypoints_object, descriptors_object );

    // Detect scene features
    keypoints_scene.clear();
    detector->detect( scene_img, keypoints_scene );
    extractor->compute( scene_img, keypoints_scene, descriptors_scene );

    if( keypoints_scene.size()<1 || keypoints_object.size()<1 )
        return false;
    // Match descriptors
    vector<vector<DMatch> > matches_scene;
    vector<vector<DMatch> > matches_object;
    try {
        // find 2 nearest-neighbors for each match point (the 2 most likely matches)
        // From scene->object
        matcher->knnMatch( descriptors_scene, descriptors_object, matches_scene, 2 );
        // From object->scene
        matcher->knnMatch( descriptors_object, descriptors_scene, matches_object, 2 );
    }
    // TODO: this is pretty bullshit catching.
    catch (...) {		// handle exception: unspecified 
        cerr << "Unknown knnMatch Error" << endl;
        return false;
    }

    if( isRatio )
    {
        // Perform ratio test. If the 2 nearest neighbors are of similar
        // distance(match quality), then the match is ambiguous, so we discard.
        // Only matches that are much better than the second best match are kept.
        ratio_test( matches_scene );
        ratio_test( matches_object );
    }

    // Perform symmetry test. For a match pair to be accepted both points
    // must be the best matching feature of the other. This is to say we
    // accept a match pair of A chose B in matches_scene and B chose A in
    // matches_object.
    vector<DMatch> sym_matches;
    if( isSym )                      
        symmetry_test( matches_scene, matches_object, sym_matches );
    else
        sym_matches = matches_scene[0];
    if( verbosity>=VERY_VERBOSE )
        cout << "match: sym_matches: " << sym_matches.size() << endl;
    if( isRansac )
    {
        if( sym_matches.size()<min_points ) return false;
        ransac_test( sym_matches, keypoints_scene, keypoints_object, matches );
    }
    else
    {
        if( sym_matches.size()>0 )
            matches = sym_matches;
        else 
            return false;
    }
    return true;
}		/* -----  end of method ARC_Match::match  ----- */

