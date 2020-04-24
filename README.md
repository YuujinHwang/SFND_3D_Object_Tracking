

# Track an Object in 3D Space

## FP.1 Match 3D Objects

Object matching is performed by the method below,
first count keypoints in region of interest for all bounding boxes in  current and previous frame. Next, iterating over previous boxes, find best current bounding box with the counted number.
```
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    map<int, map<int,int>> matchCount; // <prevBBidx, <currBBidx, count>>
    cv::KeyPoint kptPrev;
    cv::KeyPoint kptCurr;
    int bbidxPrev;
    int bbidxCurr;

    for (cv::DMatch match : matches)
    {
        kptPrev = prevFrame.keypoints[match.queryIdx];
        kptCurr = currFrame.keypoints[match.trainIdx];

        bbidxPrev = -1;
        bbidxCurr = -1;

        for(BoundingBox bbPrev : prevFrame.boundingBoxes)
        {
            if(bbPrev.roi.contains(kptPrev.pt))
            {
                bbidxPrev = bbPrev.boxID;
                break;
            }
        }
        for(BoundingBox bbCurr : currFrame.boundingBoxes)
        {
            if(bbCurr.roi.contains(kptCurr.pt))
            {
                bbidxCurr = bbCurr.boxID;
                break;
            }
        }
        
        if(matchCount.count(bbidxPrev)){
            if(matchCount[bbidxPrev].count(bbidxCurr))
            {
                matchCount[bbidxPrev][bbidxCurr]++;
            }
            else
            {
                matchCount[bbidxPrev][bbidxCurr] = 1;
            }   
        }
        else
        {
            matchCount[bbidxPrev][bbidxCurr] = 1;
        }
    }

    for(auto it = matchCount.begin(); it!=matchCount.end();++it){
        int maxCount = 0;
        int maxId = 0;
        for(auto it2 = it->second.begin();it2!=it->second.end();++it2){
            if(it2->second > maxCount){
                maxId = it2->first;
                maxCount = it2->second;
            }
        }
        if(maxId!=-1)
        {
            bbBestMatches[it->first] = maxId;
        }
    }
    
}
```

## FP.2 Compute Lidar-based TTC

Lidar-based TTC estimation is simply implemented with difference between median x-axis distances. Using closest point might give better result regards to the definition of TTC; but it's hard to handle with outliers and sensing faults.

```
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
  vector<double> xPrev, xCurr;
  for (auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end();++it)
  {
      xPrev.push_back(it->x);
  }
  for (auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end();++it)
  {
      xCurr.push_back(it->x);
  }
  sort(xPrev.begin(),xPrev.end());
  sort(xCurr.begin(),xCurr.end());

  double mPrev, mCurr;
  int nPrev = xPrev.size();
  int nCurr = xCurr.size();

  if (xPrev.size() % 2 == 0){
      mPrev = (xPrev[nPrev/2] + xPrev[nPrev/2-1])/2;
  }
  else
  {
      mPrev = xPrev[nPrev/2];
  }
  if (xCurr.size() % 2 == 0){
      mCurr = (xCurr[nCurr/2] + xCurr[nCurr/2-1])/2;
  }
  else
  {
      mCurr = xCurr[nCurr/2];
  }

  TTC = mCurr / (frameRate * (mPrev - mCurr));    
}
```

## FP.3 Associate Keypoint Correspondence with Bounding Boxes

Keypoint matchc can be achieved by 2 following step.
After check keypoints are in current roi, if yes, drop far point using mean of eucledian distance.
```
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<double> distance;

    for (cv::DMatch match : kptMatches)
    {
        if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            boundingBox.kptMatches.push_back(match);
            distance.push_back(cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.trainIdx].pt));
        }
    }
    double mean = accumulate(distance.begin(), distance.end(),0) / distance.size();
    int idx = 0;
    for (auto it = boundingBox.kptMatches.begin(); it != boundingBox.kptMatches.end();)
    {
        if (distance[idx]>2*mean)
        {
            boundingBox.kptMatches.erase(it);
            distance.erase(distance.begin()+idx);
        {
            idx++;
        }
        
    }
}
```

## FP.4 Compute Camera-based TTC

Simillar way as lidar-based TTC, but when using cameras, distance calulation will be performed with differenct of median keypoints distance. Keypoints goes farther: obeject getting closer. Keypoints goes closer: object getting farther. With the relationship between focal length and distance to object projected in image and real world, regardless of scaling camera space, we can calculate TTC directly from image.

```
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;
    for (auto it = kptMatches.begin(); it != kptMatches.end() - 1; ++it) {
        auto it2 = next(it);
        cv::KeyPoint kptCurr1 = kptsCurr.at(it->trainIdx);  // kptsCurr is indexed by trainIdx, see NOTE in matchBoundinBoxes
        cv::KeyPoint kptPrev1 = kptsPrev.at(it->queryIdx);  // kptsPrev is indexed by queryIdx, see NOTE in matchBoundinBoxes
        cv::KeyPoint kptCurr2 = kptsCurr.at(it2->trainIdx);  // kptsCurr is indexed by trainIdx, see NOTE in matchBoundinBoxes
        cv::KeyPoint kptPrev2 = kptsPrev.at(it2->queryIdx); 
        double distCurr = cv::norm(kptCurr1.pt - kptCurr2.pt);
        double distPrev = cv::norm(kptPrev1.pt - kptPrev2.pt);

        double distMin = 100.0;
        if (distCurr >= distMin) {
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
    }

    if (distRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }
    double medianDistRatio;
    std::sort(distRatios.begin(), distRatios.end());
    if(distRatios.size() % 2 == 0){
        medianDistRatio = (distRatios[distRatios.size()/2] + distRatios[distRatios.size()/2+1])/2.0;
    }
    else
    {
        medianDistRatio = distRatios[distRatios.size() / 2];
    }
    TTC = (-1.0 / frameRate) / (1 - medianDistRatio);
}
```

## FP.5 Performance Evaluation 1

### Find lidar-based TTC detection fault

1. Lidar pointscloud classification

<img src = "lidar_3d.png" width = "800">


<img src = "lidar_detect.png" width = "800">

Only clustering lidar points cloud with ROI leads to outlier points into 
TTC calculation. The proper clustering algorithm is needed.

2. Lidar sensing fault

<img src = "lidar_fault.png" width = "800">


<img src = "lidar_fault_ttc.png" width = "800">

Filtered lidar points with ROI only contains outliers, although all the inlier points are disappeared.
That makes TTC very small value, eventhough the car is stopped in traffic.


## FP.6 Performance Evaluation 2

### Find combination of det/desc combination, and find camera-based TTC fault

1.FAST-BRISK


2.FAST-ORB


3.SHITOMASHI-BRIEF

<img src="spread.png" width = "800">


In some points, bounding box matching goes wrong and camera TTC significantly gets errors.
To handle this problem, more strict matching algorithm should be implemented.
<img src="bb_error_cam_fault.png" width = "800">