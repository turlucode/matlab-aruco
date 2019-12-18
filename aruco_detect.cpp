/**
 * @file   aruco_detect.cpp
 * @author Athanasios Tasoglou (athanasios@tasoglou.net)
 * @date   December, 2019
 * @brief  ArUco detector MATLAB mex function
 * @see https://turlucode.com/aruco-opencv-matlab/
 *
 * TurluCode 2019
 * 
 * \license This project is released under the GNU GPLv3.
 */

#include <iostream>
#include <vector>

#include <opencvmex.hpp>
#include <opencv2/aruco.hpp>

/**
 * @brief Check input arguments
 */
void checkMexInputArguments(bool* estimatePose, int nrhs, const mxArray *prhs[]) {
  /* check for proper number of arguments */
  if(nrhs == 2) {
      *estimatePose = false;
  } else if (nrhs == 5) {
      *estimatePose = true;
  } else {
    mexErrMsgIdAndTxt(
            "MATLAB:aruco_detect:invalidNumInputs", 
            "Invalid number of arguments! Use:\n"
            "a) aruco_detect(image,dictionary) or\n" 
            "b) aruco_detect(image,dictionary, cameraMatrix, distCoeffs, markerLength)");
  }
    
  /* Input 1: image must be a uint8 array */
  if (mxGetM(prhs[0]) == 0 || mxGetN(prhs[0]) < 3 || !mxIsNumeric(prhs[0])) {
      mexErrMsgIdAndTxt(
              "MATLAB:aruco_detect:image", 
              "Input 'image' must be a uint8 array: h*w*3");
  }
  
  /* Input 2: dictionaryType  must be a string */
  if (nrhs >= 2) {
    if (mxIsChar(prhs[1]) != 1) {
      mexErrMsgIdAndTxt(
              "MATLAB:aruco_detect:dictionaryType", 
              "Input 'dictionaryType' must be a string.");
    }
  }
  
  /* Input 3: cameraMatrix must be a 3x3 real matrix */
  if (nrhs >= 3) {
    if (mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 3 || !mxIsNumeric(prhs[2])) {
      mexErrMsgIdAndTxt(
              "MATLAB:aruco_detect:cameraMatrix", 
              "Input 'cameraMatrix' must be a 3x3 real matrix.");
    }   
  }
  
  /* Input 4: distCoeffs must be a 1x5 real matrix */
  if (nrhs >= 4) {
    if (mxGetM(prhs[3]) != 1 || mxGetN(prhs[3]) != 5 || !mxIsNumeric(prhs[3])) {
      mexErrMsgIdAndTxt(
              "MATLAB:aruco_detect:distCoeffs", 
              "Input 'distCoeffs' must be a 1x5 real vector.");
    }
  }
  
  /* Input 4: markerLength must be a real scalar */
  if (nrhs >= 5) {
    if (mxGetM(prhs[4]) != 1 || mxGetN(prhs[4]) != 1 || !mxIsNumeric(prhs[4])) {
      mexErrMsgIdAndTxt(
              "MATLAB:aruco_detect:markerLength", 
              "Input 'markerLength' must be a a real scalar.");
    }
  }
}

/**
 * @brief Prepare output struct that contains all markers Rotation (R) and 
 * translation (t) matrices.
 */
void prepareMarkersStructOutput(int nlhs, mxArray *plhs[], int nMarkers) {
  if (nlhs >= 2) {
    int nRows = 1;
    int nColumns = nMarkers; 
    int nFields = 2;
    const char* fnames[2];

    fnames[0] = (char*) mxMalloc(sizeof("R"));
    fnames[1] = (char*) mxMalloc(sizeof("t"));
    memcpy((void *) fnames[0],"R", sizeof("R"));
    memcpy((void *) fnames[1],"t", sizeof("t"));

    plhs[1] = mxCreateStructMatrix(1, nColumns, nFields, fnames);
  }
}

/**
 * @brief Fill in the output struct-array
 */
void fillInMarkersStructOutput(
        int nlhs, mxArray *plhs[], 
        const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs, 
        int iMarker) {
  if (nlhs >= 2) {
    // rotation matrix
    cv::Mat rotationMatrix(3, 3, CV_64F);
    cv::Rodrigues(rvecs[iMarker], rotationMatrix);

    mxArray* R = mxCreateDoubleMatrix(3, 3, mxREAL);
    double* Rptr = mxGetPr(R);
    
    Rptr[0] = rotationMatrix.at<double>(0,0);
    Rptr[1] = rotationMatrix.at<double>(0,1);
    Rptr[2] = rotationMatrix.at<double>(0,2);
    Rptr[3] = rotationMatrix.at<double>(1,0);
    Rptr[4] = rotationMatrix.at<double>(1,1);
    Rptr[5] = rotationMatrix.at<double>(1,2);
    Rptr[6] = rotationMatrix.at<double>(2,0);
    Rptr[7] = rotationMatrix.at<double>(2,1);
    Rptr[8] = rotationMatrix.at<double>(2,2);

    mxSetFieldByNumber(plhs[1], iMarker, 0, R);

    // translation vector
    mxArray *t = mxCreateDoubleMatrix(1, 3, mxREAL);
    double* tptr = mxGetPr(t);

    tptr[0] = tvecs[iMarker][0];
    tptr[1] = tvecs[iMarker][1];
    tptr[2] = tvecs[iMarker][2];

    mxSetFieldByNumber(plhs[1], iMarker, 1, t);
  }
}

/**
 * @brief Parse the input and create the corresponding dictionary
 */
cv::Ptr<cv::aruco::Dictionary> getDictionary(const mxArray *prhs[]) {
  char* dictionary = mxArrayToString(prhs[1]);
    
  if (strcmp(dictionary, "DICT_ARUCO_ORIGINAL") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
  } else if (strcmp(dictionary, "DICT_4X4_50") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
  } else if (strcmp(dictionary, "DICT_4X4_100") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
  } else if (strcmp(dictionary, "DICT_4X4_250") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_250);
  } else if (strcmp(dictionary, "DICT_4X4_1000") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000);
  } else if (strcmp(dictionary, "DICT_5X5_50") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50);
  } else if (strcmp(dictionary, "DICT_5X5_100") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_100);
  } else if (strcmp(dictionary, "DICT_5X5_250") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_250);
  } else if (strcmp(dictionary, "DICT_5X5_1000") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000);
  } else if (strcmp(dictionary, "DICT_6X6_50") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
  } else if (strcmp(dictionary, "DICT_6X6_100") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_100);
  } else if (strcmp(dictionary, "DICT_6X6_250") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);
  } else if (strcmp(dictionary, "DICT_6X6_1000") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000);
  } else if (strcmp(dictionary, "DICT_7X7_50") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50);
  } else if (strcmp(dictionary, "DICT_7X7_100") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_100);
  } else if (strcmp(dictionary, "DICT_7X7_250") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_250);
  } else if (strcmp(dictionary, "DICT_7X7_1000") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000);
  } else if (strcmp(dictionary, "DICT_APRILTAG_16h5") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_16h5);
  } else if (strcmp(dictionary, "DICT_APRILTAG_25h9") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_25h9);
  } else if (strcmp(dictionary, "DICT_APRILTAG_36h10") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h10);
  } else if (strcmp(dictionary, "DICT_APRILTAG_36h11") == 0){
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11);
  } else {
    mexErrMsgIdAndTxt(
            "MATLAB:aruco_detect:dictionary", 
            "Unknown or unsupported dictionary: %s", dictionary);
  }
      
  return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
}

/*
 * @brief MEX main function
 *
 * Inputs:
 *  prhs[0]: image            uint8[]
 *  prhs[1]: cameraMatrix     real 3x3
 *  prhs[2]: distCoeffs       real 1x5
 *  prhs[3]: dictionaryType   string
 *  prhs[4]: markerLength     real scalar
 *
 * Outputs:
 *  plhs[0]: image with markers  uint8[]
 *  plhs[1]: struct array containing R,t
 *  
 * ref: http://cs-courses.mines.edu/csci507/schedule/24/SquareMarkersOpenCV.pdf
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  cv::Mat image;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  bool estimatePose = false;
  mxArray* cellArrayOut;
    
  /* Check inputs */
  checkMexInputArguments(&estimatePose, nrhs, prhs);

  /* Convert inputs */
  ocvMxArrayToImage_uint8(prhs[0], image);
  cv::Ptr<cv::aruco::Dictionary> dictionary = getDictionary(prhs);
  
  if (estimatePose) {
    double* K = mxGetPr(prhs[2]);
    cameraMatrix = (cv::Mat_<double>(3,3) << K[0], K[3], K[6], 
                                             K[1], K[4], K[7], 
                                             K[2], K[5], K[8]);
    double* r = mxGetPr(prhs[3]);
    distCoeffs = (cv::Mat_<double>(1,5) << r[0], r[1], r[2], r[3], r[4]);
  }
    
  /* Detect markers */
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create(); // TODO: config from MATLAB
  parameters->adaptiveThreshWinSizeMin = 2;
  parameters->adaptiveThreshWinSizeStep = 1;
  parameters->adaptiveThreshWinSizeMax = 28;
  // parameters->aprilTagMinWhiteBlackDiff = 200;
  std::vector<std::vector<cv::Point2f>> rejectedCandidates; // TODO: maybe return them to MATLAB?
  
  cv::aruco::detectMarkers(
          image, 
          dictionary, 
          corners, 
          ids,
          parameters,
          rejectedCandidates);

  if(ids.size() > 0) {
    // mexPrintf("MATLAB:aruco_detect: Found %d markers\n", ids.size());
    cv::aruco::drawDetectedMarkers(image, corners, ids);
  }
  
  /* Estimate pose */
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  if (ids.size() > 0 && estimatePose) {
    double markerLength = mxGetScalar(prhs[4]);
    cv::aruco::estimatePoseSingleMarkers(
            corners, 
            markerLength, 
            cameraMatrix, distCoeffs, 
            rvecs, tvecs);
    
    // markers: Prepare mex output
    prepareMarkersStructOutput(nlhs, plhs, ids.size());
       
    for(int iMarker = 0; iMarker < ids.size(); iMarker++) {
      // draw axis for each marker
      cv::aruco::drawAxis(
              image, 
              cameraMatrix, distCoeffs, 
              rvecs[iMarker], tvecs[iMarker], 
              0.1);
      
      // fill-in mex output
      fillInMarkersStructOutput(nlhs, plhs, rvecs, tvecs, iMarker);
    }
  }
  
  if (ids.size() == 0 && estimatePose) {
    // no marker detected, return empty
    plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
  }
  
  // processed image: Prepeare mex output
  if (nlhs >= 1) {
    plhs[0] = ocvMxArrayFromImage_uint8(image);
  }
}
