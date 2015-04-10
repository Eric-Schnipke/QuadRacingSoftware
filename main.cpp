```C++
/***************************************
 Usage Instructions:
 1.) Hold object up to rectangle for picture.
 2.) Press 'b' key to take picture.
 5.) Adjust H,S,V sliders such that the peaks are bounded on the histogram.
 6.) Track!  Yaw = -1 indicates a neccesary yaw to left, Yaw = 0 is center, and Yaw = 1 is to the right.  Pitch behaves similarly.
 7.) Press 'q' key to quit.
 8.) Press '1' to pause tracking.
 9.) Press '2' to show HSV and Binary videofeeds.
 10.) Press '3' to show live histograms of BGR and HSV videofeeds.
 11.) Press '4' to start tracking a new object.
 
 ************************************/

// include the necessary libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////////////////////////////
//Credit given to Kyle Hounslow 2013 for basic shell of color tracking program.
//Credit given to OpenCV for library development.
//Histrogram functionality added by E. Schnipke - Feb. 2014
//SURF functionality added by E. Schnipke - Feb. 2014

//*** LEGAL STUFF ***
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.
//////////////////////////////////////////////////////////////////////////////////////////////////

//using namespace cv;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int HSVMinMaxThreshold = 3; //horizontal line that, if the histogram rise above, set the minimum and maximum threshold values.
bool lockHSVThreshold = false;//locks threshold values
//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const cv::string windowName = "Object Tracking: Press 'q' to quit.";
const cv::string windowName1 = "HSV Image";
const cv::string windowName2 = "Thresholded Image";
const cv::string windowName3 = "After Morphological Operations";
const cv::string trackbarWindowName = "Trackbars";

/// Variables to be publicly returned to flight control program
int yaw = 0;
int pitch = 0;
bool objectFound = false;

/// Variable to control camera input or file input
bool fromCamera = true;

void on_trackbar( int, void* ){//This function gets called whenever a trackbar position is changed
// Originally by Kyle Hounslow 2013
}
cv::string intToString(int number){
// Originally by Kyle Hounslow 2013
	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
// Originally by Kyle Hounslow 2013
	//create window for trackbars
    
    cv::namedWindow(trackbarWindowName,cv::WINDOW_AUTOSIZE);
	//create memory to store trackbar name on window
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
    cv::createTrackbar( "H_MIN (Blue)", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    cv::createTrackbar( "H_MAX (Blue)", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    cv::createTrackbar( "S_MIN (Green)", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    cv::createTrackbar( "S_MAX (Green)", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    cv::createTrackbar( "V_MIN (Red)", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    cv::createTrackbar( "V_MAX (Red)", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

void drawObject(int x, int y,cv::Mat &frame){
// Originally by Kyle Hounslow 2013
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
    
    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
    cv::Scalar color = cv::Scalar(0,255,0);
    
	circle(frame,cv::Point(x,y),20,color,2);
    if(y-25>0)
        line(frame,cv::Point(x,y),cv::Point(x,y-25),color,2);
    else line(frame,cv::Point(x,y),cv::Point(x,0),color,2);
    if(y+25<FRAME_HEIGHT)
        line(frame,cv::Point(x,y),cv::Point(x,y+25),color,2);
    else line(frame,cv::Point(x,y),cv::Point(x,FRAME_HEIGHT),color,2);
    if(x-25>0)
        line(frame,cv::Point(x,y),cv::Point(x-25,y),color,2);
    else line(frame,cv::Point(x,y),cv::Point(0,y),color,2);
    if(x+25<FRAME_WIDTH)
        line(frame,cv::Point(x,y),cv::Point(x+25,y),color,2);
    else line(frame,cv::Point(x,y),cv::Point(FRAME_WIDTH,y),color,2);
    
	putText(frame,intToString(x)+","+intToString(y),cv::Point(x,y+30),1,1,color,2);
    
}
void morphOps(cv::Mat &thresh){
// Originally by Kyle Hounslow 2013
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
    
    cv::Mat erodeElement = getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = getStructuringElement( cv::MORPH_RECT,cv::Size(8,8));
    
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
    
    
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}
void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed){
// Originally by Kyle Hounslow 2013
// Yaw and pitch notification by E. Schnipke - Feb. 5th, 2014
    cv::Mat temp;
    int yawPadding = 100; // how wide the center yaw area is in pixels
    int pitchPadding = 100; // how tall the center pitch area is in pixels
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                
                cv::Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
                
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;
                
                
			}
			//let user know you found an object
			if(objectFound ==true){
                putText(cameraFeed,"Tracking Object",cv::Point(0,50),2,1,cv::Scalar(0,255,0),2);
                // let user know whether object is to left of screen, center, or to right of screen
                if (x<(FRAME_WIDTH - yawPadding)/2) {
                    yaw = -1; //left
                }else if(x>(FRAME_WIDTH + yawPadding)/2){
                    yaw = 1; //right
                }else{
                    yaw = 0; //center
                }
                putText(cameraFeed, "Yaw = " + intToString(yaw), cv::Point(0,100),2,1,cv::Scalar(0,255,0),2);
                
                // let user know whether object is to the top, center, or bottom of screen
                if (y<(FRAME_HEIGHT - pitchPadding)/2) {
                    pitch = -1; //bottom
                }else if(y>(FRAME_HEIGHT + pitchPadding)/2){
                    pitch = 1; //top
                }else{
                    pitch = 0; //center
                }
                putText(cameraFeed, "Pitch = " + intToString(pitch), cv::Point(0,150),2,1,cv::Scalar(0,255,0),2);

				//draw object location on screen
				drawObject(x,y,cameraFeed);}
            
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",cv::Point(0,50),1,2,cv::Scalar(0,0,255),2);
	}
}
int drawHistogram(cv::String histogramWindowName, cv::Mat img, bool displayHSVThresholdLines){
// Originally by E. Schnipke Feb. 5th, 2014
    /// Separate the HSV image in 3 places ( Hue, Saturation, and Value )
    cv::vector<cv::Mat> hsv_planes;
    split( img, hsv_planes );
    
    /// Establish the number of bins
    int HistSize = 256;
    
    /// Set the ranges ( for H,S,V )
    float hRange[] = { 0, 256 } ;
    float sRange[] = { 0, 256 } ;
    float vRange[] = { 0, 256 } ;
    const float* hHistRange = { hRange };
    const float* sHistRange = { sRange };
    const float* vHistRange = { vRange };
    
    bool uniform = true; bool accumulate = false;
    
    cv::Mat h_hist, s_hist, v_hist;
    
    /// Compute the histograms:
    calcHist( &hsv_planes[0], 1, 0, cv::Mat(), h_hist, 1, &HistSize, &hHistRange, uniform, accumulate );
    calcHist( &hsv_planes[1], 1, 0, cv::Mat(), s_hist, 1, &HistSize, &sHistRange, uniform, accumulate );
    calcHist( &hsv_planes[2], 1, 0, cv::Mat(), v_hist, 1, &HistSize, &vHistRange, uniform, accumulate );
    
    /// Draw the histograms for H, S, and V
    int hist_w = 512; int hist_h = 400;
    int hbin_w = cvRound( (double) hist_w/HistSize );
    int sbin_w = cvRound( (double) hist_w/HistSize );
    int vbin_w = cvRound( (double) hist_w/HistSize );
    
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
    
    /// Normalize the result to [ 0, histImage.rows ]
    normalize(h_hist, h_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    normalize(s_hist, s_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    normalize(v_hist, v_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw histogram for each channel
    for(int i = 1; i < HistSize; i++){
        // Hue channel
        line( histImage, cv::Point( hbin_w*(i-1), hist_h - cvRound(h_hist.at<float>(i-1)) ) ,
             cv::Point( hbin_w*(i), hist_h - cvRound(h_hist.at<float>(i)) ),
             cv::Scalar( 255, 0, 0), 2, 8, 0  );
        // Saturation channel
        line( histImage, cv::Point( sbin_w*(i-1), hist_h - cvRound(s_hist.at<float>(i-1)) ) ,
             cv::Point( sbin_w*(i), hist_h - cvRound(s_hist.at<float>(i)) ),
             cv::Scalar( 0, 255, 0), 2, 8, 0  );
        // Value channel
        line( histImage, cv::Point( vbin_w*(i-1), hist_h - cvRound(v_hist.at<float>(i-1)) ) ,
             cv::Point( vbin_w*(i), hist_h - cvRound(v_hist.at<float>(i)) ),
             cv::Scalar( 0, 0, 255), 2, 8, 0  );
    }
    
    for (int i = 1; i < HistSize; i++){ // Analyze histograms from left to right and set max thresholds to last value above threshold value (+ buffer).
        if(h_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){H_MAX=i+10;}
        if(s_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){S_MAX=i+10;}
        if(v_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){V_MAX=i+10;}
    }
    
    for (int i = HistSize; i > 0; i--){ // Analyze histograms from right to left and set min thresholds to last value above threshold value (- buffer).
        if(h_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){H_MIN=i-10;}
        if(s_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){S_MIN=i-10;}
        if(v_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThreshold){V_MIN=i-10;}
    }
    
    /// Plot threshold lines on histogram
    if (displayHSVThresholdLines){
        // Plot lower Hue threshold line
        line( histImage,
             cv::Point(H_MIN*hbin_w, hist_h) ,
             cv::Point(H_MIN*hbin_w, 0),
             cv::Scalar( 255, 200, 200), 2, 8, 0  );
        // Plot upper Hue threshold line
        line( histImage,
             cv::Point(H_MAX*hbin_w, hist_h) ,
             cv::Point(H_MAX*hbin_w, 0),
             cv::Scalar( 255, 200, 200), 2, 8, 0  );
        // Plot lower Saturation threshold line
        line( histImage,
             cv::Point(S_MIN*sbin_w, hist_h) ,
             cv::Point(S_MIN*sbin_w, 0),
             cv::Scalar( 200, 255, 200), 2, 8, 0  );
        // Plot upper Saturation threshold line
        line( histImage,
             cv::Point(S_MAX*sbin_w, hist_h) ,
             cv::Point(S_MAX*sbin_w, 0),
             cv::Scalar( 200, 255, 200), 2, 8, 0  );
        // Plot lower Value threshold line
        line( histImage,
             cv::Point(V_MIN*vbin_w, hist_h) ,
             cv::Point(V_MIN*vbin_w, 0),
             cv::Scalar( 200, 200, 255), 2, 8, 0  );
        // Plot upper Value threshold line
        line( histImage,
             cv::Point(V_MAX*vbin_w, hist_h) ,
             cv::Point(V_MAX*vbin_w, 0),
             cv::Scalar( 200, 200, 255), 2, 8, 0  );
    }
    
    /// Display
    imshow(histogramWindowName, histImage );
    
    return 0;
}
int objectInitialization(cv::VideoCapture &vid, bool displayInitialization){
// Originally by E. Schnipke Feb. 6th, 2014.
    cv::Mat pict;// capture picture of initialized object
    char k;
    
    cv::destroyAllWindows();
    lockHSVThreshold = false;//unlock ability to set HSV thresholds.
    while(k != 'b'){
        vid.read(pict);
        //flip(pict, pict, 1);// flip image to act as mirror.
        putText(pict,"Press 'b' to take picture of object.",cv::Point(0,50),2,1,cv::Scalar(0,255,0),2);
        rectangle(pict, cv::Point(FRAME_WIDTH*0.45, FRAME_HEIGHT*0.45), cv::Point(FRAME_WIDTH*0.55,FRAME_HEIGHT*0.55), cv::Scalar(0,255,0));
        imshow("Object Initialization: Press 'b' to take picture", pict);
        k = cv::waitKey(30);
    }
    k = 0;
    cv::destroyWindow("Object Initialization: Press 'b' to take picture");

    // Crop the full image to the image contained by the rectangle
    cv::Mat croppedImage = pict(cv::Rect(FRAME_WIDTH*0.45, FRAME_HEIGHT*0.45, FRAME_WIDTH*0.1, FRAME_HEIGHT*0.1)); // Rect(x,y,w,h)
    if (displayInitialization){
        imshow("Region of Interest", croppedImage);
    }
    // Create the HSV image of the object
    cv::cvtColor(croppedImage,croppedImage,cv::COLOR_BGR2HSV);
    
    //create slider bars for HSV filtering
	if(displayInitialization){
        createTrackbars();
    }
    
    // lock in threshold values. refresh histogram to show updated position of threshold lines
    while(k != 'b'){
        drawHistogram("Object Histogram: Press 'b' to lock threshold values", croppedImage, true);
        k = cv::waitKey(30);
        if(!displayInitialization){k = 'b';}
    }
    cv::destroyAllWindows();
    lockHSVThreshold = true;
    
    return 0;
}
int colorRecognition(){
// Originally by Kyle Hounslow 2013.
// Heavy modifications by E. Schnipke - Feb. 5th, 2014.
	// program control character.
    char k;
    bool feedToggle = false;
    bool histToggle = false;
    
    //boolean variables to toggle object tracking and morphological operations.
    bool trackObjects = true;
    bool useMorphOps = true;
    
	//Matrix to store each frame of the webcam feed, HSV image, and binary threshold image
	cv::Mat cameraFeed, HSV, threshold;
    
	//x and y values for the location of the object
	int x=0, y=0;
    
	//video capture object to acquire webcam feed
	cv::VideoCapture capture;
    
	//open capture object at location zero (default location for webcam) or filename
    if(fromCamera){
        capture.open(0);
    }else{
        capture.open("/Users/Swanson/Downloads/Object Recognition%2C Flight 2.mp4");
    }

	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    
    // take picture of object
    objectInitialization(capture, false);
    
	//start an infinite loop where webcam feed is copied to cameraFeed matrix.  all of our operations will be performed within this loop
	while(k != 'q'){
        // toggle which frames to show
        switch (k) {
            case '1':// pause execution
                cv::waitKey();
                break;
            case '2':// show HSV and Binary videofeeds
                feedToggle = !feedToggle;
                break;
            case '3': // show BGR and HSV histograms
                histToggle = !histToggle;
                break;
            case '4': // initialize a new object
                objectInitialization(capture, true);
                break;
            default:
                break;
        }
        
        //store image to matrix
		capture.read(cameraFeed);
        //flip(cameraFeed, cameraFeed, 1);
		//convert frame from BGR to HSV colorspace
		cv::cvtColor(cameraFeed,HSV,cv::COLOR_BGR2HSV);
        
		//filter HSV image between values and store filtered image to threshold matrix
		cv::inRange(HSV,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),threshold);
        
		//perform morphological operations on thresholded image to eliminate noise and emphasize the filtered object(s)
		if(useMorphOps){morphOps(threshold);}
        
		//pass in thresholded frame to our object tracking function. This function will return the x and y coordinates of the filtered object
		if(trackObjects){trackFilteredObject(x,y,threshold,cameraFeed);}
        
		//Show videofeeds
		imshow(windowName,cameraFeed); // BGR videofeed.  This is always shown.
        if (feedToggle) {
            imshow(windowName2,threshold); // binary videofeed
            imshow(windowName1,HSV); // HSV videofeed
        }else if(!feedToggle){
            cv::destroyWindow(windowName2);
            cv::destroyWindow(windowName1);
        }
        //Write cameraFeed to 
        
        //Show histograms
        if (histToggle) {
            /// histogram refresh to display threshold values
            drawHistogram("BGR Feed Histogram", cameraFeed, false);
            drawHistogram("HSV Feed Histogram", HSV, true);
        }else if(!histToggle){
            cv::destroyWindow("BGR Feed Histogram");
            cv::destroyWindow("HSV Feed Histogram");
        }

		//delay 30ms so that screen can refresh. image will not appear without this waitKey() command
		k = cv::waitKey(30);
	}
    
    capture.release();
	return 0;
}

int main(int argc, char* argv[]){
// Originally by E. Schnipke - Feb 6th, 2014.
// This program provides flight directives to a target tracking UAV drone.  The program acquires a target and returns whether yaw or pitch are necessary in order to center the target in the frame.
    
    colorRecognition();
    return 0;
}
```
