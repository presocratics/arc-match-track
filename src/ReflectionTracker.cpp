//Track the reflections of templates across water
//Simon Peter 
//speter3@illinois.edu

#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <iostream>
#include <string>
#include "ARC_Pair.hpp"

using namespace std;
using namespace cv;

class Cropper{
	public: 
		Mat original, cropped;
		int x1,y1,x2,y2;
		bool isCropping;
		bool displayRegions = true;
		bool displayWindows = false;
		Cropper(Mat& image);
		vector<Mat> templates[2];
		int verbosity = NOT_VERBOSE;
};

Cropper::Cropper(Mat& image){
	
	original = image;
	isCropping = false;
}

		
void getReflections(Mat frame, int patchSize, vector<ARC_Pair> outvector){
		
	int c;
	int argNum = 1;
	/*
	while((c=getopt(argc,argv,"r"))!=-1){
		switch(c){
			case 'r':
				displayRegions = true;
				argNum = 2;
				break;
			case '?':
				cout<<"Use -r to display regions instead of points.\n";
				return 0;
		}
	}*/

	Mat image = frame;
		if(!image.data) cout<<"Image couldn't be loaded\n";
	Cropper obj (image);
	namedWindow("Source", CV_WINDOW_AUTOSIZE);
//	setMouseCallback("Source", Cropper::onMouse, &obj);
//	obj.isCropping=true;

	Mat sourceCopy;
	obj.original.copyTo(sourceCopy);
	cvtColor(sourceCopy,sourceCopy,CV_RGB2GRAY,1);
		if(verbosity==VERBOSE) cout<<"Image loaded and converted to grayscale\n";	
	vector<Point2f> points[2]; vector<Mat> templates[2]; vector<Rect> reflections; vector<Rect> originalMatches;
	int maxNumFeaturesToTrack = (obj.original.cols-patchSize)*(obj.original.rows-patchSize);
	
	//FIND GOOD FEATURES TO TRACK BASED ON CERTAIN PARAMETERS AND STORE IN points[0]
	goodFeaturesToTrack(sourceCopy,points[0],20,.01,patchSize+10,Mat(),3,0,0.04);
		if(verbosity==VERBOSE){
			cout<<"Ran goodFeaturesToTrack\n";
			cout<<"Features To Track\n";
			for(int i=0;i<points[0].size();i++){
				cout<<"x: "<<points[0][i].x-20<<" y: "<<points[0][i].y-20<<endl;
			}
		}
	Mat sourceCopy2 = obj.original.clone();
	int pointsLength = points[0].size();

	//medianBlur(sourceCopy2,obj.original,3);
	//namedWindow("Median blur",CV_WINDOW_AUTOSIZE);
	//imshow("Median blur", obj.original);
	//waitKey(0);
	
	if(verbosity==VERY_VERBOSE) cout<<"Top left corners of templates\n";
	for(int i=0; i<pointsLength;i++){
		points[0][i].x-=patchSize/2; points[0][i].y-=patchSize/2;
			if(verbosity==VERY_VERBOSE) cout<<" x: "<<points[0][i].x<<" y: "<<points[0][i].y<<endl;
		if(points[0][i].x<0 || points[0][i].y<0 || sourceCopy.cols-points[0][i].x-patchSize < 0 || sourceCopy.rows-points[0][i].y-patchSize < 0){
			points[0].erase(points[0].begin()+i);
			if(verbosity==VERY_VERBOSE) cout<<"failed\n";
			i--;pointsLength--;
		}
		else{	
		Rect rect (points[0][i].x,points[0][i].y,patchSize,patchSize);
			if(verbosity==VERY_VERBOSE) cout<<"Created template rect\n";
		ARC_Pair temp;
		temp.roi.source = rect;
		outvector.push_back(temp);

		Scalar color (26,7,191);
		Mat temp = obj.original.clone();
		//	cout<<"Right side difference: "<<sourceCopy.cols-points[0][i].x-patchSize<<endl;
		//	cout<<"Bottom side difference: "<<sourceCopy.rows-points[0][i].y-patchSize<<endl;
		temp = temp(rect);
		templates[0].push_back(temp);
			if(verbosity==VERY_VERBOSE) cout<<"Added template rect to output vector\n";
		}
	}

	if(verbosity==VERBOSE){
		cout<<"Verified Points to Match:"<<endl;
		for(int i=0;i<points[0].size();i++){
			cout<<i<<" x: "<<points[0][i].x<<" y: "<<points[0][i].y<<endl;
		}
	}

	//FLIP TEMPLATE, RUN TEMPLATE THROUGH MATCHTEMPLATE, AND STORE RESULTS
	for(int i=0;i<points[0].size();i++){
		Mat tmplte = templates[0][i];
		flip(tmplte,tmplte,0);

		int result_cols = obj.original.cols-tmplte.cols+1;
		int result_rows = obj.original.rows-tmplte.rows+1;
		Mat result;
		result.create(result_cols,result_rows, CV_32FC1);

		//0:CV_TM_SQDIFF
		//1:CV_TM_SQDIFF_NORMED
		//2:CV_TM_CORR
		//3:CV_TM_CCOR_NORMED
		//4:CV_TM_CCOEFF
		//5:CV_TM_CCOEFF_NORMED <----Most succesful at finding reflections
		int match_method = 5;
		//Specifies vertical search region that extends 2.5 times template width on each side from initial template location
		Mat mask; Mat masked_scene;
		mask = Mat::zeros(obj.original.size(),CV_8UC1);
		//int leftSearchArea = points[0][i].x-(int)(.75*tmplte.cols);
				int leftSearchArea = points[0][i].x-30;
		Rect searchRegion (leftSearchArea,0,2*tmplte.cols,obj.original.rows);
		rectangle(mask,searchRegion,255,CV_FILLED);
		obj.original.copyTo(masked_scene,mask);
		//namedWindow("Mask",CV_WINDOW_AUTOSIZE);
		//imshow("Mask",masked_scene);
		//moveWindow("Mask",800,800);
		//waitKey(0);
		//namedWindow("flipped",CV_WINDOW_AUTOSIZE);
		//imshow("flipped",tmplte);
		//waitKey(0);	
		matchTemplate(masked_scene, tmplte, result, match_method);
		normalize(result,result,0,1,NORM_MINMAX,-1,Mat());
			if(verbosity==VERY_VERBOSE) cout<<"Ran matchTemplate and normalized\n";
		double minVal; double maxVal; 
		Point minLoc; Point maxLoc; Point matchLoc;
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		matchLoc = minLoc;
		 
		
		if(match_method == 0 || match_method == 1){
			matchLoc = minLoc;	
		}
		else {
			matchLoc = maxLoc;
		}
		if(verbosity==VERY_VERBOSE) cout<<"Max value from results matrix: "<<result.at<float>(matchLoc.x,matchLoc.y)<<endl;
		//if(result.at<float>(matchLoc.x,matchLoc.y)>.6){
		Scalar reflectionColor(0,0,0);

		Rect rect(matchLoc.x,matchLoc.y,patchSize,patchSize);
		
		ARC_Pair temp = outvector[i];
		temp.roi.reflection = rect;

		reflections.push_back(rect); 
		/*
		if(!displayRegions){
			matchLoc.x+=patchSize/2;
			matchLoc.y+=patchSize/2;
			points[0][i].x+=patchSize/2;
			points[0][i].y+=patchSize/2;
			circle(sourceCopy2,matchLoc,4,reflectionColor,-1,8,0);
			line(sourceCopy2,points[0][i],matchLoc,reflectionColor,1,8,0);
		}
		else{
			rectangle(sourceCopy2, matchLoc, Point(matchLoc.x+tmplte.cols,matchLoc.y+tmplte.rows),reflectionColor ,2,8,0);
			line(sourceCopy2,points[0][i],matchLoc,reflectionColor,1,8,0);
			circle(obj.original,matchLoc,4,reflectionColor,-1,8,0);
		}*/
	//	cout<<"Original x: "<<points[0][i].x<<" Original y: "<<points[0][i].y<<endl;
	//	cout<<"Reflection x: "<<matchLoc.x<<" Reflection.y: "<<matchLoc.y<<endl;
	//	cout<<"Done drawing match\n";
    //  cout<<"Max value from results matrix: "<<result.at<float>(matchLoc.x,matchLoc.y)<<endl;

       // imshow("Source",sourceCopy2);
       // waitKey(0);


		bool debugResults = false;
		if(debugResults) {
		int surroundingPixel = 5;
		cout<<"TL: "<<result.at<float>(matchLoc.x-surroundingPixel,matchLoc.y-surroundingPixel)<<" T: "<<result.at<float>(matchLoc.x,matchLoc.y-surroundingPixel)<<" TR: "<<result.at<float>(matchLoc.x+surroundingPixel,matchLoc.y-surroundingPixel)<<endl;
		cout<<"BL: "<<result.at<float>(matchLoc.x-surroundingPixel,matchLoc.y+surroundingPixel)<<" B: "<<result.at<float>(matchLoc.x,matchLoc.y+surroundingPixel)<<" BR: "<<result.at<float>(matchLoc.x+surroundingPixel,matchLoc.y+surroundingPixel)<<endl;
		rectangle(result, matchLoc, Point(matchLoc.x+tmplte.cols,matchLoc.y+tmplte.rows),reflectionColor ,2,8,0);
		imshow("Source",sourceCopy2);	
		namedWindow("Results",CV_WINDOW_AUTOSIZE);
		imshow("Results", result);
		moveWindow("Results", 700,0);
		waitKey(0);
		}
		//}
	}

	/////////SYMMETRY TEST///////////
	
	if(verbosity==VERBOSE) {cout<<"\n\n"<<endl; cout<<"Running symmetry test\n";
	for(int i=0;i<reflections.size();i++){
	
		Mat temp = obj.original.clone();	
		Mat refl = temp(reflections[i]);
		flip(refl,refl,0);

		int result_cols = obj.original.cols-refl.cols+1;
        int result_rows = obj.original.rows-refl.rows+1;
        Mat result;
        result.create(result_cols,result_rows, CV_32FC1);

        //0:CV_TM_SQDIFF
        //1:CV_TM_SQDIFF_NORMED
        //2:CV_TM_CORR
        //3:CV_TM_CCOR_NORMED
        //4:CV_TM_CCOEFF
        //5:CV_TM_CCOEFF_NORMED <----Most succesful at finding reflections
        int match_method = 5;
        //SPECIFIES VERTICAL SEARCH REGION THAT EXTENDS .75 TIMES TEMPLATE WIDTH ON EACH SIDE FROM INITIAL TEMPLATE LOCATION
        Mat mask; Mat masked_scene;
        mask = Mat::zeros(obj.original.size(),CV_8UC1);
        int leftSearchArea = reflections[i].x-40;
        Rect searchRegion (leftSearchArea,0,2*refl.cols,obj.original.rows);
        rectangle(mask,searchRegion,255,CV_FILLED);
        obj.original.copyTo(masked_scene,mask);

		matchTemplate(masked_scene, refl, result, match_method);
        normalize(result,result,0,1,NORM_MINMAX,-1,Mat());
        double minVal; double maxVal;
        Point minLoc; Point maxLoc; Point matchLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
        matchLoc = minLoc;

        if(match_method == 0 || match_method == 1){
            matchLoc = minLoc;
        }
        else {
            matchLoc = maxLoc;
        }

		Rect org_rfl(matchLoc.x,matchLoc.y,patchSize,patchSize);
		originalMatches.push_back(org_rfl);
	
	/*	Mat sourceCopy3 = obj.original.clone();
		Scalar reflecColor(0,0,255);//red
		Scalar origiColor(0,0,0);//black
		Scalar matchColor(255,255,255);//white
		Rect reflectionDrawing (reflections[i].x,reflections[i].y,patchSize,patchSize);
		Rect originalDrawing (points[0][i].x,points[0][i].y,patchSize,patchSize);
		Rect matchedDrawing (matchLoc.x,matchLoc.y,patchSize,patchSize);
		rectangle(sourceCopy3,reflectionDrawing,reflecColor,2,8,0);
		rectangle(sourceCopy3,originalDrawing,origiColor,2,8,0);
		rectangle(sourceCopy3,matchedDrawing,matchColor,2,8,0);	
		cout<<"Refl x: "<<reflections[i].x<<" Refl y: "<<reflections[i].y<<endl;
		cout<<"Original x: "<<points[0][i].x<<" Original y: "<<points[0][i].y<<endl;
		cout<<"Matched source x: "<<matchLoc.x<<" Matched source y: "<<matchLoc.y<<endl;

		namedWindow("Symmetry Test", CV_WINDOW_AUTOSIZE);
		imshow("Symmetry Test",sourceCopy3);
		moveWindow("Symmetry Test",700,500);*/

	}
	
	int symmetrySize = reflections.size();
	for(int i=0;i<symmetrySize;i++){
		int xdifference = abs(points[0][i].x-originalMatches[i].x);
		int ydifference = abs(points[0][i].y-originalMatches[i].y);
		int reflectionXdifference = abs(originalMatches[i].x-reflections[i].x);
		int reflectionYdifference = abs(originalMatches[i].y-reflections[i].y);
		//	cout<<"Original Reflection x: "<<reflectionXdifference<<" Original Reflection y: "<<reflectionYdifference<<endl;
		if(xdifference>patchSize || ydifference>patchSize ||  reflectionYdifference<patchSize){
			points[0].erase(points[0].begin()+i);
			reflections.erase(reflections.begin()+i);
			originalMatches.erase(originalMatches.begin()+i);
			outvector.erase(outvector.begin()+i);
			i--; symmetrySize--;
		}
		if(verbosity==VERY_VERBOSE) cout<<"Outvector array size after test: "<<reflections.size()<<endl;
	}
//CHECKS Y COORDINATES TO DISTINGUISH BETWEEN THE REAL OBJECTS AND THEIR REFLECTIONS
	for(int i=0;i<regions.size();i++){
        if(regions[i].roi.source.y>regions[i].roi.reflection.y){
            Rect temp = regions[i].roi.reflection;
            regions[i].roi.reflection = regions[i].roi.source;
            regions[i].roi.source = temp;
        }
    }

	if(displayWindows){
		cout<<reflections.size()<<endl;
		for(int i=0;i<reflections.size();i++){
			Scalar reflectionColor(0,0,0);
			Point reflectionTLCorner(reflections[i].x,reflections[i].y);
			cout<<"Drew one rectangle\n";
			Scalar pointColor (0,255,0);
 			if(!displayRegions){
				Point centerReflection(reflections[i].x+=patchSize/2,reflections[i].y+=patchSize/2);
       	   		Point centerSource(originalMatches[i].x+patchSize/2,originalMatches[i].y+patchSize/2);
       	     	circle(sourceCopy2,centerReflection,4,reflectionColor,-1,8,0);
				circle(sourceCopy2,centerSource,4,pointColor,-1,8,0);
       		    line(sourceCopy2,centerSource,centerReflection,reflectionColor,1,8,0);
  	     	 }
 	     	 else{
				Scalar originalColor(0,0,255);
				Point sourceTL (originalMatches[i].x,originalMatches[i].y);
				rectangle(sourceCopy2,originalMatches[i],originalColor,2,8,0);
				rectangle(sourceCopy2, reflections[i],reflectionColor ,2,8,0);
        	    line(sourceCopy2,sourceTL,reflectionTLCorner,reflectionColor,1,8,0);
    	    }
		}
		cout<<"showing final result\n";
		imshow("Source", sourceCopy2);
		moveWindow("Source",0,0);
		waitKey(0);
	}
}