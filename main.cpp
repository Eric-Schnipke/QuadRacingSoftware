// include the necessary libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <stdio.h>
#include <vector>

const int H_MIN = 0;
const int H_MAX = 256;
const int S_MIN = 0;
const int S_MAX = 256;
const int V_MIN = 0;
const int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

/// Variables to be publicly returned to flight control program
int yaw = 0;
int pitch = 0;
bool objectFound = false;

class targetObject
/* Class of objects that may be tracked using color-based object recognition.  In this case, a quadrotor UAV. */
{
public:
    /* -----------------[Name]------------------------*/
    /* return name of target object */
    cv::string getName()
    {return name;}
    
    /* -----------------[Search Status]--------------------- */
    
    /* set searching status of target object */
    void setFoundStatus(bool fIsFound)          
    {fObjectFound = fIsFound;}
    
    /* return searching status of target object */
    bool getFoundStatus()                       
    {return fObjectFound;}
    
    /* ------------------[Lap Counting]--------------------- */
    
    /* increment lap count since beginning of race for target object */
    void incrCurrentLap()
    {usCurrentLap += 1;}
    
    /* return number of laps since beginning of race for target object */
    unsigned short getCurrentLap()
    {return usCurrentLap;}
    
    /* ---------------[Checkpoint Counting]----------------- */
    
    /* increment checkpoint count since beginning of race for target object */
    void incrCurrentCheckpoint()
    {usCurrentCheckpoint += 1;}
    
    /* return number of checkpoints since beginning of race for target object */
    unsigned short getCurrentCheckpoint()
    {return usCurrentCheckpoint;}
    
    /* ------------------[HSV Thresholds]------------------- */
    
    /* set min/max hue, saturation, and value thresholds for target object.  this is the filtering range for the binary image. */
    void setHsvThresh(unsigned short usHMax,
                      unsigned short usSMax,
                      unsigned short usVMax,
                      unsigned short usHMin,
                      unsigned short usSMin,
                      unsigned short usVMin)
    {
        usHMaxThresh = usHMax;
        usSMaxThresh = usSMax;
        usVMaxThresh = usVMax;
        usHMinThresh = usHMin;
        usSMinThresh = usSMin;
        usVMinThresh = usVMin;
    }
    
    /* return min/max hue, saturation, and value thresholds for target object */
    unsigned short getHMaxThresh()
    {return usHMaxThresh;}
    
    unsigned short getSMaxThresh()
    {return usSMaxThresh;}
    
    unsigned short getVMaxThresh()
    {return usVMaxThresh;}
    
    unsigned short getHMinThresh()
    {return usHMinThresh;}
    
    unsigned short getSMinThresh()
    {return usSMinThresh;}
    
    unsigned short getVMinThresh()
    {return usVMinThresh;}
    
    /* -----------------[Initialization]-------------------- */
    
    /* initialize all values of target object */
    void initialize(cv::string sName)
    {
        setFoundStatus(false);
        usCurrentLap = 0;
        usCurrentCheckpoint = 0;
        setHsvThresh(H_MAX, S_MAX, V_MAX, H_MIN, S_MIN, V_MIN);
        name = sName;
    }
private:
    bool fObjectFound;                  /* found status of object */
    unsigned short usCurrentLap;        /* lap count of target object since beginning of race */
    unsigned short usCurrentCheckpoint; /* checkpoint count of target object since beginning of race */
    unsigned short usHMaxThresh;        /* Max hue threshold */
    unsigned short usSMaxThresh;        /* Max saturation threshold */
    unsigned short usVMaxThresh;        /* Max value threshold */
    unsigned short usHMinThresh;        /* Min hue threshold */
    unsigned short usSMinThresh;        /* Min saturation threshold */
    unsigned short usVMinThresh;        /* Min value threshold */
    cv::string name;                    /* name of target object */
};

class camera
{
public:
    void initialize(unsigned short usAddress, cv::string sName)
    {
        /********************************************************************************************************/
        /* Summary: Initializes capturing capabilities of this camera, including camera address and frame size. */
        /********************************************************************************************************/
        
        /* open capture object at address location (zero is default webcam location) */
        capture.open(usAddress);
        
        /* set height and width of capture frame */
        capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
        
        /* set name of camera object */
        name = sName;
    }
    
    void destroy()
    {
        /*****************************************************/
        /* Summary: Returns camera resources back to system. */
        /*****************************************************/
        capture.release();
    }
    
    cv::string getName()
    {
        /*******************************************/
        /* Summary: Returns string name of camera. */
        /*******************************************/
        return name;
    }
    
    void targetInitialize(targetObject &targ)
    {
        /********************************************************************************************************/
        /* Summary: Determines descriptive HSV values of target object and assigns them to the target object.   */
        /*          Adds target object to a list of target objects to be tracked by this camera.                */
        /********************************************************************************************************/

        cv::Mat frame; /* matrix for storing frames */
        char controlChar = 0; /* sequence control character for use by user */
        
        /* clean up any previously opened windows */
        cv::destroyAllWindows();
        
        while(controlChar != ' ')
        {
            capture.read(frame);
            putText(frame, "Press space bar to take picture of object.", cv::Point( 0, 50 ), 2, 1, cv::Scalar( 0, 255, 0 ), 2);
            rectangle(frame, cv::Point(FRAME_WIDTH*0.45, FRAME_HEIGHT*0.45), cv::Point(FRAME_WIDTH*0.55,FRAME_HEIGHT*0.55), cv::Scalar(0,255,0));
            imshow("Object Initialization", frame);
            controlChar = cv::waitKey(30);
        }
        controlChar = 0;
        cv::destroyAllWindows();
        
        /* crop frame to the image contained by the rectangle */
        cv::Mat croppedImage = frame(cv::Rect(FRAME_WIDTH*0.45, FRAME_HEIGHT*0.45, FRAME_WIDTH*0.1, FRAME_HEIGHT*0.1)); // Rect(x,y,w,h)
        
        /* create the HSV image of the object */
        cv::cvtColor(croppedImage,croppedImage,cv::COLOR_BGR2HSV);
        
        /* lock in threshold values. refresh histogram to show updated position of threshold lines */
        lockHSVThresh = false;
        while(controlChar != ' ')
        {
            getHSVThresholds(croppedImage);
            controlChar = cv::waitKey(30);
        }
        lockHSVThresh = true;
        cv::destroyAllWindows();
        
        /* assign HSV threshold values to 'targ' object and add 'targ' object to set of targets to be tracked by this camera */
        targ.setHsvThresh(usHMaxThresh, usSMaxThresh, usVMaxThresh, usHMinThresh, usSMinThresh, usVMinThresh);
        //targets.push_back(targ);
        std::cout << targ.getName() << " " << usHMaxThresh << " " << usSMaxThresh << " " << usVMaxThresh << " " << usHMinThresh << " " << usSMinThresh << " " << usVMinThresh << std::endl;
    }
    
    //void copyTargets(camera &thisCamera)
    //{
        /*************************************************************************************/
        /* Summary: Copies all target objects of passed camera object to this camera object. */
        /*************************************************************************************/
    //targets = thisCamera.getTargets();
    //}
    
    //std::vector<targetObject> getTargets()
    //{
        /******************************************************************/
        /* Summary: Returns all targets set to be tracked by this camera. */
        /******************************************************************/
    //return targets;
    //}

    void trackTarget(targetObject &thisTarget){
        /* boolean alerting whether target has been found */
        bool targFound = false;
        
        /* matrix to store each frame of the webcam feed, HSV image, and binary threshold image */
        cv::Mat cameraFeed, HSV, threshold;
        
        /* x and y location of object */
        int x = 0, y = 0;
        
        /* read one frame of video */
        capture.read(cameraFeed);
        
        /* convert frame from BGR to HSV colorspace */
        cv::cvtColor(cameraFeed, HSV, cv::COLOR_BGR2HSV);
        
        /* filter HSV image between values and store filtered image to threshold matrix */
		cv::inRange(HSV,
                    cv::Scalar(thisTarget.getHMinThresh(),
                               thisTarget.getSMinThresh(),
                               thisTarget.getVMinThresh()),
                    cv::Scalar(thisTarget.getHMaxThresh(),
                               thisTarget.getSMaxThresh(),
                               thisTarget.getVMaxThresh()),
                    threshold);
        /* perform morphological operations on thresholded image to eliminate noise and emphasize the filtered object(s) */
		morphOps(threshold);
        
        /* pass in thresholded frame to our object tracking function. This function will return the x and y coordinates of the filtered object */
        targFound = trackFilteredObject( x, y, threshold, cameraFeed);
        thisTarget.setFoundStatus(targFound);
        
        /* show videofeeds */
        imshow("Tracking " + thisTarget.getName(), cameraFeed);
        imshow("Thresholded image of " + thisTarget.getName(), threshold);
        //imshow("HSV image of " + thisTarget.getName(), HSV);
        
        std::cout << "Tracking: " << thisTarget.getName() << "\t | Found: " << thisTarget.getFoundStatus() << std::endl;
    }
private: 
    /* video capture object to acquire webcam feed */
    cv::VideoCapture capture;               
    
    /* locks HSV thresholds */
    bool lockHSVThresh;
    
    /* HSV threshold values used for filtering */
    unsigned short usHMaxThresh;
    unsigned short usSMaxThresh;
    unsigned short usVMaxThresh;
    unsigned short usHMinThresh;
    unsigned short usSMinThresh;
    unsigned short usVMinThresh;
    //std::vector<targetObject> targets;
    
    /* name of camera object */
    cv::string name;
    
    void getHSVThresholds(cv::Mat img){
        // Originally by E. Schnipke Feb. 5th, 2014
        /// Separate the HSV image in 3 places ( Hue, Saturation, and Value )
        cv::vector<cv::Mat> hsv_planes;
        split( img, hsv_planes );
        
        /// Establish the number of bins
        int HistSize = 256;
        
        /* horizontal line that, if the histogram rise above, set the minimum and maximum threshold values. */
        const int HSVMinMaxThreshold = 3;
        
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
            if(h_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usHMaxThresh=i+10;}
            if(s_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usSMaxThresh=i+10;}
            if(v_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usVMaxThresh=i+10;}
        }
        
        for (int i = HistSize; i > 0; i--){ // Analyze histograms from right to left and set min thresholds to last value above threshold value (- buffer).
            if(h_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usHMinThresh=i-10;}
            if(s_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usSMinThresh=i-10;}
            if(v_hist.at<float>(i)>HSVMinMaxThreshold && !lockHSVThresh){usVMinThresh=i-10;}
        }
        
        /* Plot threshold lines on histogram */
        /* Plot lower Hue threshold line */
        line(histImage,
             cv::Point(usHMinThresh*hbin_w, hist_h),
             cv::Point(usHMinThresh*hbin_w, 0),
             cv::Scalar( 255, 200, 200), 2, 8, 0  );
        /* Plot upper Hue threshold line */
        line(histImage,
             cv::Point(usHMaxThresh*hbin_w, hist_h),
             cv::Point(usHMaxThresh*hbin_w, 0),
             cv::Scalar( 255, 200, 200), 2, 8, 0  );
        /* Plot lower Saturation threshold line */
        line(histImage,
             cv::Point(usSMinThresh*sbin_w, hist_h),
             cv::Point(usSMinThresh*sbin_w, 0),
             cv::Scalar( 200, 255, 200), 2, 8, 0  );
        /* Plot upper Saturation threshold line */
        line(histImage,
             cv::Point(usSMaxThresh*sbin_w, hist_h),
             cv::Point(usSMaxThresh*sbin_w, 0),
             cv::Scalar( 200, 255, 200), 2, 8, 0  );
        /* Plot lower Value threshold line */
        line(histImage,
             cv::Point(usVMinThresh*vbin_w, hist_h),
             cv::Point(usVMinThresh*vbin_w, 0),
             cv::Scalar( 200, 200, 255), 2, 8, 0  );
        /* Plot upper Value threshold line */
        line(histImage,
             cv::Point(usVMaxThresh*vbin_w, hist_h),
             cv::Point(usVMaxThresh*vbin_w, 0),
             cv::Scalar( 200, 200, 255), 2, 8, 0  );
        
        /* Display */
        imshow("Object Histogram: Press [space] to lock threshold values", histImage );
    }
    
    /* ----------------------------------------------------- */
    cv::string intToString(int number){
        // Originally by Kyle Hounslow 2013
        std::stringstream ss;
        ss << number;
        return ss.str();
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
        
        cv::Mat erodeElement = getStructuringElement( cv::MORPH_ELLIPSE,cv::Size(10,10));
        //dilate with larger element so make sure object is nicely visible
        cv::Mat dilateElement = getStructuringElement( cv::MORPH_ELLIPSE,cv::Size(10,10));
        
        // morphological opening (remove small objects from foreground)
        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);
        
        // morphological closing (fill small holes in foreground)
        dilate(thresh,thresh,dilateElement);
        erode(thresh,thresh,erodeElement);
        
        // morphological opening (remove small objects from foreground)
        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);
        
        // morphological closing (fill small holes in foreground)
        dilate(thresh,thresh,dilateElement);
        erode(thresh,thresh,erodeElement);
    }
    
    bool trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed){
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
        findContours(temp,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE );
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
                if(objectFound == true){
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
        return objectFound;
    }

};

int main(int argc, char* argv[]){
// Originally by E. Schnipke - Apr 11th, 2015.
    
    /* set the number of targets to be tracked */
    unsigned short numTargs = 2;
    
    /* set the number of cameras to be used */
    unsigned short numCameras = 1;
    
    char controlChar = 0;   /* program control character */
    
    /* create target objects and put them into a collection of targets */
    std::vector<targetObject> targets;
    for( unsigned short int i = 1; i <= numTargs; i++ )
    {
        targetObject thisTarget;
        cv::string targName;
        
        std::cout << "Enter target object name: ";
        getline( std::cin, targName );
        
        /* initialize properties for target objects */
        thisTarget.initialize(targName);
        
        /* add target to collection of targets */
        targets.push_back(thisTarget);
        
        std::cout << "Target object '" << thisTarget.getName() << "' added." << std::endl;
    }
    
    /* create camera objects and put them into a collection of cameras */
    std::vector<camera> cameras;
    for( unsigned short int i = 0; i <= numCameras-1; i++ )
    {
        camera thisCamera;
        cv::string cameraName;
        
        std::cout << "Enter camera name: ";
        getline( std::cin, cameraName );
        
        /* initialize properties for target objects */
        thisCamera.initialize(i, cameraName);
        
        /* add target to collection of targets */
        cameras.push_back(thisCamera);
        
        std::cout << "Camera '" << thisCamera.getName() << "' added." << std::endl;
    }
    
    /* use camera 1 to determine calibration values for each of the target objects */
    for( unsigned short int i = 0; i < targets.size(); i++)
    {
            cameras.at(0).targetInitialize(targets.at(i));
    }
    
    /* if more that one camera, copy object calibration from camera 1 to other cameras so that each camera doesn't have to be calibrated */
    /*if(numCameras > 1)
    {
        for( unsigned short int i = 1; i < cameras.size(); i++)
        {
            cameras.at(i).copyTargets(cameras.at(0));
        }
    }*/
    
    /* enter tracking portion of program */
    while(controlChar != 'q')
    {
        /* have each of the cameras advance one frame, searching for each of their initialized targets */
        for (unsigned short int i = 0; i < cameras.size(); i++)
        {
            for (unsigned short int j = 0; j < targets.size(); j++)
            {
                cameras.at(i).trackTarget(targets.at(j));
            }
            
        }
        
        /* capture program control character */
        controlChar = cv::waitKey(10);
    }
    
    /* return camera resources back to system */
    for( unsigned short int i = 0; i < cameras.size(); i++ )
    {
        cameras.at(i).destroy();
    }
    
    return 0;
}

