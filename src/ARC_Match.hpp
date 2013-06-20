#ifndef  ARC_MATCH_INC
#define  ARC_Match_INC
#include <iostream>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "config.hpp"
using std::vector;
using std::cerr;
using std::cout;
using std::endl;
using namespace cv;
// =====================================================================================
//        Class:  ARC_Match
//  Description:  Match an object to a scene using specified detector,
//                matcher, and filters.
// =====================================================================================
class ARC_Match
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_Match (void); // constructor      
        //ARC_Match ( const ARC_Match &other );   // copy constructor 
        //~ARC_Match () {};                            // destructor       

        // ====================  ACCESSORS     ======================================= 

        // ====================  MUTATORS      ======================================= 
        void set_feature_detector ( Ptr<FeatureDetector>& detect )
        {
            detector = detect;
        }

        void set_descriptor_extractor ( Ptr<DescriptorExtractor>& desc )
        {
            extractor = desc;
        }

        void set_descriptor_matcher ( Ptr<DescriptorMatcher>& match )
        {
            matcher = match;
        }

        void set_confidence ( double conf )
        {
            confidence = conf;
        }

        void set_distance ( double dist )
        {
            distance = dist;
        }

        void set_ratio ( float rat )
        {
            ratio = rat;
        }

        void set_refineF ( bool is )
        {
            refineF = is;
        }

        void set_min_points ( unsigned int mp )
        {
            min_points = mp;
        }

        void set_ratio_test ( bool tf )
        {
            isRatio = tf;
        }

        void set_symmetry_test ( bool tf )
        {
            isSym = tf;
        }

        void set_ransac_test ( bool tf )
        {
            isRansac = tf;
        }

        void set_verbosity ( int v )
        {
            verbosity = v;
        }

        // ====================  METHODS       ======================================= 
        bool match ( Mat& scene_img, Mat& object_img,
                    Mat& scene_mask, Mat& object_mask,
                    vector<DMatch>& matches,
                    vector<KeyPoint>& keypoints_scene,
                    vector<KeyPoint>& keypoints_object );

        // ====================  OPERATORS     ======================================= 
        //ARC_Match& operator = ( const ARC_Match &other ); // assignment operator 

    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 
        int ratio_test( vector<vector<DMatch> > &matches );
        void symmetry_test ( 
            const vector<vector<DMatch> >& matches_scene,
            const vector<vector<DMatch> >& matches_object,
            vector<DMatch>& sym_matches );
        Mat ransac_test (
            const vector<DMatch>& matches,
            const vector<KeyPoint>& keypoints_scene,
            const vector<KeyPoint>& keypoints_object,
            vector<DMatch>& out_matches );
        void angle_test(
            const vector<KeyPoint>& keypoints_scene,
            const vector<KeyPoint>& keypoints_object,
            vector<DMatch>& matches,
            vector<DMatch>& good_matches );

        // ====================  DATA MEMBERS  ======================================= 
        Ptr<FeatureDetector> detector;
        Ptr<DescriptorExtractor> extractor;
        Ptr<DescriptorMatcher> matcher;
        float ratio;
        double confidence;
        double distance;
        bool refineF;
        unsigned int min_points;                         // Bails if fewer points after symmetry test.
        bool isRatio;
        bool isSym;
        bool isRansac;

        int verbosity;

}; // -----  end of class ARC_Match  ----- 

#endif   /* ----- #ifndef ARC_Match_INC  ----- */
