
#ifndef MULTIOBJECTTLD_H
#define MULTIOBJECTTLD_H

#include <iostream>
#include <fstream>
#include <vector>
#include "LKTracker.h"
#include "FernFilter.h"
#include "NNClassifier.h"
#include "Matrix.h"

#define COLOR_MODE_GRAY 0
#define COLOR_MODE_RGB 1


#define STATUS_LOST 0
#define STATUS_UNSURE 1
#define STATUS_OK 2


#define DEBUG_DRAW_DETECTIONS 0
#define DEBUG_DRAW_PATCHES 0
#define DEBUG_DRAW_CROSSES 0


#define ENABLE_CLUSTERING 1

/// Setting
struct MOTLDSettings
{

  int colorMode;
  /// patchSize 15
  int patchSize;
  /// number of ferns  8
  int numFerns;
  /// n tree depth: 10
  int featuresPerFern;
  /// sliding window = 1.2^n 
  int scaleMin, scaleMax; 
  /// sliding window size 24
  int bbMin;
  bool useColor;
  bool allowFastChange;
  bool enableFastRotation;

  /// setting
  MOTLDSettings(int cm = COLOR_MODE_RGB)
  {
    colorMode = cm;
    patchSize = 15;  
    numFerns = 8;
    featuresPerFern = 10;
    scaleMin = -10;
    scaleMax = 11;
    bbMin = 24;
    useColor = false;
    allowFastChange = false;
    enableFastRotation = false;
  }
};



class MultiObjectTLD
{
public:

  MultiObjectTLD(const int width, const int height, const MOTLDSettings& settings = MOTLDSettings())
       : ivWidth(width), ivHeight(height), 
         ivColorMode(settings.colorMode), ivPatchSize(settings.patchSize), ivBBmin(settings.bbMin), 
         ivUseColor(settings.useColor && ivColorMode == COLOR_MODE_RGB), 
         ivEnableFastRotation(settings.enableFastRotation), ivLKTracker(LKTracker(width, height)), 
         ivNNClassifier(NNClassifier(width, height, ivPatchSize, ivUseColor, settings.allowFastChange)), 
         ivFernFilter(FernFilter(width, height, settings.numFerns, settings.featuresPerFern)),
         ivNObjects(0), ivLearningEnabled(true), ivNLastDetections(0) { };

  void addObject(ObjectBox b);

  void addObjects(std::vector<ObjectBox> obs);

  void processFrame(unsigned char * img);

  void enableLearning(bool enable = true) { ivLearningEnabled = enable; };

  int getStatus(const int objId = 0) const;

  bool getValid() const { return ivNObjects > 0 ? ivValid[0] : false; };

  ObjectBox getObjectBox() const { return ivCurrentBoxes[0]; };

  std::vector<ObjectBox> getObjectBoxes() const { return ivCurrentBoxes; };

  void writeDebugImage(unsigned char * src, char * filename, int mode = 255) const;
  
  void getDebugImage(unsigned char * src, Matrix& rMat, Matrix& gMat, Matrix& bMat, int mode = 255) const;
   
  static MultiObjectTLD loadClassifier(const char * filename);
  void saveClassifier(const char * filename) const;
  
private:
  int ivWidth;
  int ivHeight;
  int ivColorMode;
  int ivPatchSize;
  int ivBBmin;
  bool ivUseColor;
  bool ivEnableFastRotation;
  LKTracker ivLKTracker;
  NNClassifier ivNNClassifier;
  FernFilter ivFernFilter;

  int ivNObjects;
  float ivAspectRatio;
  std::vector<ObjectBox> ivCurrentBoxes;
  std::vector<bool> ivDefined;
  std::vector<bool> ivValid;
  std::vector<NNPatch> ivCurrentPatches;
  bool ivLearningEnabled;
  
  Matrix ivCurImage;
  unsigned char * ivCurImagePtr;
  std::vector<FernDetection> ivLastDetections;
  std::vector<FernDetection> ivLastDetectionClusters;
  int ivNLastDetections;
  void clusterDetections(float threshold);
  
  MultiObjectTLD (int width, int height, int colorMode, int patchSize, int bbMin, bool useColor,
                  bool fastRotation, NNClassifier nnc, FernFilter ff, int nObjects, 
                  float aspectRatio, bool learningEnabled);
};



void MultiObjectTLD::addObject(ObjectBox b)
{
  std::vector<ObjectBox> obs;
  obs.push_back(b);
  addObjects(obs);
}

void MultiObjectTLD::addObjects(std::vector<ObjectBox> obs)
{
  int n = obs.size(); 
  if (n == 0)
    return;
  for (int i = 0; i < n; i++)
  {
    obs[i].objectId = ivNObjects + i;
    if (obs[i].objectId == 0)
      ivAspectRatio = obs[i].width / (float)obs[i].height;
    else
    { // bb size same as  first box
      float centerx = obs[i].x + 0.5 * obs[i].width,
            centery = obs[i].y + 0.5 * obs[i].height;
      obs[i].width = sqrt(ivAspectRatio * obs[i].width * obs[i].height);
      obs[i].height = obs[i].width / ivAspectRatio;
      obs[i].x = centerx - 0.5 * obs[i].width;
      obs[i].y = centery - 0.5 * obs[i].height;
    }
  }

  if (ivNObjects == 0)
  { 
    if (ivCurImage.size() == 0)
    { 
      std::cerr << "Please insert an image via processFrame() before adding objects!" << std::endl;
      return;
    }
    ivLKTracker.initFirstFrame(ivCurImage);
    std::vector<Matrix> initNegPatches = ivFernFilter.addObjects(ivCurImage, obs);
    for (size_t i = 0; i < initNegPatches.size(); i++)
    {
      NNPatch p(initNegPatches[i]);
      ivNNClassifier.trainNN(p, -1, false);
    }  
  }else 
    ivFernFilter.addObjects(ivCurImage, obs);
  
  for (int i = 0; i < n; i++)
  {
    ivCurrentBoxes.push_back(obs[i]);
    ivDefined.push_back(true);
    ivValid.push_back(true);
    NNPatch p(obs[i], ivCurImage, ivPatchSize, ivUseColor ? ivCurImagePtr : NULL, ivWidth, ivHeight);
    ivCurrentPatches.push_back(p);
    ivNNClassifier.addObject(p);
  }
  ivNObjects += n;
}

int MultiObjectTLD::getStatus(const int objId) const
{
  if (objId >= ivNObjects)
    return STATUS_LOST;
  if (ivValid[objId])
    return STATUS_OK;
  if (ivDefined[objId])
    return STATUS_UNSURE;
  return STATUS_LOST;
}
int getTime()
{

	clock_t t = clock();
	return t * 1000 / CLOCKS_PER_SEC;

}
void MultiObjectTLD::processFrame(unsigned char * img)
{
  ivCurImagePtr = img;
  ivCurImage = Matrix(ivWidth, ivHeight);

  if (ivColorMode == COLOR_MODE_RGB)
  {
    ivCurImage.fromRGB(img);
  }
  else //if(ivColorMode == COLOR_MODE_GRAY)
  {
    ivCurImage.copyFromCharArray(img);
  }
  if (ivNObjects <= 0)
    return;
  std::vector<NNPatch*> detectionPatches;
  #if TIMING
  int t_start = getTime(), t_end = 0, t_tracker = 0, t_detector = 0, t_nn = 0, t_learner = 0;
  #endif
  // TRACKER
  ivLKTracker.processFrame(ivCurImage, ivCurrentBoxes, ivDefined);
  #if TIMING
  t_end = getTime(); t_tracker = t_end - t_start;
  #endif

  std::vector<float> tConf;
  for (int o = 0; o < ivNObjects; o++)
  {
    if (ivDefined[o])
    {
      ivCurrentPatches[o] = NNPatch(ivCurrentBoxes[o], ivCurImage, ivPatchSize,
                                      ivUseColor ? img : NULL, ivWidth, ivHeight);
      tConf.push_back(ivNNClassifier.getConf(ivCurrentPatches[o], o, true));
      ivValid[o] = tConf[o] > 0.65;
    }else{
      tConf.push_back(0);
      ivValid[o] = false;
    }

  }
  
  // DETECTOR
  ivLastDetections = ivFernFilter.scanPatch(ivCurImage);

  ivNLastDetections = ivLastDetections.size();
  ivLastDetectionClusters.clear();
  if (ivNLastDetections > 0)
  {
    for (int i = 0; i < ivNLastDetections; ++i)
    {
      detectionPatches.push_back(new NNPatch(ivLastDetections[i].patch));
      ivLastDetections[i].confidence = 
          ivNNClassifier.getConf(*detectionPatches[i], ivLastDetections[i].box.objectId, false,
                              ivLastDetections[i].box, ivUseColor ? img : NULL, ivWidth, ivHeight);
    }
    #if ENABLE_CLUSTERING
    clusterDetections(0.5);
    #endif
    for (int o = 0; o < ivNObjects; o++)
    {
      double bestConf = -1;
      int bestId = -1;

      NNPatch bestCluster;      
      for (size_t i = 0; i < ivLastDetectionClusters.size(); ++i)
        if (ivLastDetectionClusters[i].box.objectId == o)
        {
          NNPatch curPatch(ivLastDetectionClusters[i].box, ivCurImage, ivPatchSize,
                           ivUseColor ? img : NULL, ivWidth, ivHeight);
          ivLastDetectionClusters[i].confidence = ivNNClassifier.getConf(curPatch, o, true);
          if (ivLastDetectionClusters[i].confidence > bestConf)
          {
            bestConf = ivLastDetectionClusters[i].confidence;
            bestId = i;
            bestCluster = curPatch;
          }
        }
      if (bestConf > tConf[o] && bestConf > 0.65 && (!ivDefined[o] 
              || rectangleOverlap(ivLastDetectionClusters[bestId].box, ivCurrentBoxes[o]) < 0.7))
      {
        //if there is a (better) cluster of detections away from tracker result / and tracker failed
        ivCurrentBoxes[o] = ivLastDetectionClusters[bestId].box;    
        ivCurrentPatches[o] = bestCluster;  

        ivDefined[o] = true;
        if (bestConf > 0.65)
          ivValid[o] = true;      
      }    
      else if (ivDefined[o]) // tracker defined
      {

        int tmpn = 10;
        float tmpx = 10*ivCurrentBoxes[o].x, tmpy = 10*ivCurrentBoxes[o].y, 
              tmpw = 10*ivCurrentBoxes[o].width, tmph = 10*ivCurrentBoxes[o].height;
        for (int i = 0; i < ivNLastDetections; ++i)
        {
          if (ivLastDetections[i].box.objectId == o)
          {
            float overlap = rectangleOverlap(ivCurrentBoxes[o], ivLastDetections[i].box);
            if (overlap > 0.7)
            {
              if (ivLastDetections[i].confidence > 0.6)
              {
                tmpn++;
                tmpx += ivLastDetections[i].box.x;
                tmpy += ivLastDetections[i].box.y;
                tmpw += ivLastDetections[i].box.width;
                tmph += ivLastDetections[i].box.height;
              }            
            }
          }
        }  
        if (tmpn > 10)
        {
          ivCurrentBoxes[o].x = tmpx / tmpn;
          ivCurrentBoxes[o].y = tmpy / tmpn;
          ivCurrentBoxes[o].width = tmpw / tmpn;
          ivCurrentBoxes[o].height = tmph / tmpn;
          ivCurrentPatches[o] = NNPatch(ivCurrentBoxes[o], ivCurImage, ivPatchSize,
                                  ivUseColor ? img : NULL, ivWidth, ivHeight);
          tConf[o] = ivNNClassifier.getConf(ivCurrentPatches[o], o, false);
          ivValid[o] = tConf[o] > 0.65;
        }    
        if (tConf[o] < 0.5)
          ivDefined[o] = false;
      }      
    }// end for
  }// end if

  
  // LEARNER
  if (ivEnableFastRotation)
    ivNNClassifier.removeWarps();
  std::vector<ObjectBox> learnBoxes;
  // train positive examples
  for (int o = 0; o < ivNObjects; o++)
  {
    ivCurrentBoxes[o].objectId = o;
    if (ivValid[o] && !(ivCurrentBoxes[o].x < 0 || ivCurrentBoxes[o].y < 0
                        || ivCurrentBoxes[o].x + ivCurrentBoxes[o].width >= ivWidth-1
                        || ivCurrentBoxes[o].y + ivCurrentBoxes[o].height >= ivHeight-1)
           && ivCurrentBoxes[o].width >= ivBBmin && ivCurrentBoxes[o].height >= ivBBmin)
    {
      learnBoxes.push_back(ivCurrentBoxes[o]);
      if (ivLearningEnabled)
        ivNNClassifier.trainNN(ivCurrentPatches[o], o, true);
    }
  }
  // train negative examples
  if (ivLearningEnabled)
  {
    for (int i = 0; i < ivNLastDetections; ++i)
    {
      bool learn = true;
      for (int o = 0; o < ivNObjects; o++)
        if(!ivDefined[ivLastDetections[i].box.objectId] 
            || (ivDefined[o] && rectangleOverlap(ivCurrentBoxes[o], ivLastDetections[i].box) > 0.3))
        {
          learn = false;
          break;
        }
      if (learn)
        ivNNClassifier.trainNN(*detectionPatches[i], ivLastDetections[i].box.objectId, false);
    }
  }

  // update fern filter
  std::vector<Matrix> warpedPatches = ivFernFilter.learn(ivCurImage, learnBoxes, !ivLearningEnabled);

  if (ivEnableFastRotation)
    for (size_t i = 0; i < learnBoxes.size(); ++i)
    {
      ivNNClassifier.trainNN(NNPatch(warpedPatches[2*i]), learnBoxes[i].objectId, true, true);
      ivNNClassifier.trainNN(NNPatch(warpedPatches[2*i+1]), learnBoxes[i].objectId, true, true);
    }
  
  // clean up
  for (int i = 0; i < ivNLastDetections; ++i)
    delete detectionPatches[i];
  detectionPatches.clear();

}

void MultiObjectTLD::clusterDetections(float threshold)
{
  if (ivNLastDetections == 0)
    return;
  if (ivNLastDetections == 1)
  {
    ivLastDetectionClusters = ivLastDetections;
    return;
  }
  //init
  int* clId = new int[ivNLastDetections];
  std::vector<ObjectBox> clBox;
  std::vector<int> clN;
  int nClusters = 1;
  clBox.push_back(ivLastDetections[0].box);
  clN.push_back(ivNLastDetections);
  for (int i = 0; i < ivNLastDetections; ++i)
    clId[i] = 0;
  bool terminated = false;
  int terminationcounter = 0;
  while (!terminated && terminationcounter < 100)
  {
    terminationcounter++;
    // compute new cluster means
    for (int i = 0; i < nClusters; ++i)
    {
      clBox[i].x = clBox[i].y = clBox[i].width = clBox[i].height = 0;
      clN[i] = 0;
    }
    for (int i = 0; i < ivNLastDetections; ++i)
    {
      clBox[clId[i]].x += ivLastDetections[i].box.x;
      clBox[clId[i]].y += ivLastDetections[i].box.y;
      clBox[clId[i]].width += ivLastDetections[i].box.width;
      clBox[clId[i]].height += ivLastDetections[i].box.height;
      clN[clId[i]]++;
    }
    for (int i = 0; i < nClusters; ++i)
    {
      if (!clN[i])
      {
        #if DEBUG
        std::cout << "there was an empty cluster" << std::endl;
        #endif
        continue; // should not happen, just to be sure
      }
      clBox[i].x /= clN[i]; 
      clBox[i].y /= clN[i];
      clBox[i].width /= clN[i]; 
      clBox[i].height /= clN[i];
    }
    
    // compute new assignments
    terminated = true;
    float minOverlap = 2.0f;
    int minOverlapId = -1;
    for (int i = 0; i < ivNLastDetections; ++i)
    {
      int maxOverlapId = -1;
      float maxOverlap = -2;
      // search for nearest cluster with same objectId
      for (int j = 0; j < nClusters; ++j){
        float overlap = -1;
        if (clBox[j].objectId == ivLastDetections[i].box.objectId)
          overlap = rectangleOverlap(clBox[j], ivLastDetections[i].box);
        if (overlap > maxOverlap){
          maxOverlap = overlap;
          maxOverlapId = j;
        }
      }
      if (clId[i] != maxOverlapId)
      {
        clId[i] = maxOverlapId;
        terminated = false;
      }
      if (maxOverlap < minOverlap)
      {
        minOverlap = maxOverlap;
        minOverlapId = i;
      }
    } 
    // no assignment changes
    if (terminated)
    { 
      if (minOverlap >= threshold)
      {
        terminated = true;
        break;
      }else{
        // add new cluster (the box with minimal overlap)
        clId[minOverlapId] = nClusters;
        clBox.push_back(ivLastDetections[minOverlapId].box);   
        clN.push_back(1);
        nClusters++;
        terminated = false;
      }
    }
  }
  for (int i = 0; i < nClusters; ++i)
  {
    FernDetection clDet = {clBox[i], Matrix(), 0, 0};
    ivLastDetectionClusters.push_back(clDet);
  }
  delete[] clId;
}

void MultiObjectTLD::writeDebugImage(unsigned char * src, char* filename, int mode) const
{ 
  Matrix rMat;
  Matrix gMat;
  Matrix bMat;
  getDebugImage(src, rMat, gMat, bMat, mode);
  writePPM(filename, rMat, gMat, bMat);  
}

void MultiObjectTLD::getDebugImage(unsigned char * src, Matrix& rMat, Matrix& gMat, Matrix& bMat, int mode) const
{ 
  int size = ivHeight * ivWidth;
  rMat.setSize(ivWidth, ivHeight); 
  rMat.copyFromCharArray(src);
  gMat.setSize(ivWidth, ivHeight); 
  gMat.copyFromCharArray(src + (ivColorMode == COLOR_MODE_RGB ? size : 0));
  bMat.setSize(ivWidth, ivHeight); 
  bMat.copyFromCharArray(src + (ivColorMode == COLOR_MODE_RGB ? 2*size : 0));
  
  if (mode & DEBUG_DRAW_DETECTIONS)
  {
    for (std::vector<FernDetection>::const_iterator it = ivLastDetections.begin(); 
          it < ivLastDetections.end(); ++it)
    {
      gMat.drawDashedBox(it->box, 255, 3, true);
      if (ivNObjects > 1)
        gMat.drawNumber(it->box.x, it->box.y, it->box.objectId);
    }
    
    for (std::vector<FernDetection>::const_iterator it = ivLastDetectionClusters.begin(); 
            it < ivLastDetectionClusters.end(); ++it)
    {
      bMat.drawDashedBox(it->box, 255);
      gMat.drawDashedBox(it->box, 0);
    }
  }
  
  for (int i = 0; i < ivNObjects; i++)
    if (ivDefined[i])
    {
      rMat.drawBox(ivCurrentBoxes[i], 255);
      gMat.drawBox(ivCurrentBoxes[i], (ivValid[i] ? 0 : 255));
      bMat.drawBox(ivCurrentBoxes[i], 0);
      if (ivNObjects > 1)
      {
        int numberposx = ivCurrentBoxes[i].x + ivCurrentBoxes[i].width - 1,
            numberposy = ivCurrentBoxes[i].y + ivCurrentBoxes[i].height - 8;
        rMat.drawNumber(numberposx, numberposy, i, 255);
        gMat.drawNumber(numberposx, numberposy, i, (ivValid[i] ? 0 : 255));
        bMat.drawNumber(numberposx, numberposy, i, 0);
      }
    }
  
  if (mode & DEBUG_DRAW_CROSSES)
  {
    const std::vector<int> * debugPoints = ivLKTracker.getDebugPoints();
    for (unsigned int i = 0; i < debugPoints->size(); i += 2)
    {
      rMat.drawCross((*debugPoints)[i], (*debugPoints)[i+1], 0);
      gMat.drawCross((*debugPoints)[i], (*debugPoints)[i+1], 0);
      bMat.drawCross((*debugPoints)[i], (*debugPoints)[i+1], 255);    
    }
  }
  
  const std::vector<std::vector<NNPatch> > * posPatches = ivNNClassifier.getPosPatches();
  const std::vector<NNPatch> * negPatches = ivNNClassifier.getNegPatches();
    
  unsigned int picspercol = ivHeight / ivPatchSize;  

  if (mode & DEBUG_DRAW_PATCHES)
    for (unsigned int i = 0; i < negPatches->size() && i < 3*picspercol; ++i)
    {
      int x = ivPatchSize * (i/picspercol), y = ivPatchSize * (i%picspercol);
      rMat.drawPatch((*negPatches)[i].patch, x, y, (*negPatches)[i].avg);
      gMat.drawPatch((*negPatches)[i].patch, x, y, (*negPatches)[i].avg);
      bMat.drawPatch((*negPatches)[i].patch, x, y, (*negPatches)[i].avg);
    }
  
  bool drawhistograms = posPatches->size() > 0 && (*posPatches)[0].size() > 0 
                            && (*posPatches)[0][0].histogram != NULL;
  int startx = ivWidth;
  for (unsigned int p = 0; p < posPatches->size(); p++)
  {
    for (unsigned int i = 0; i < (*posPatches)[p].size() && i < 3*picspercol; ++i)
    {
      if ((i%picspercol) == 0)
        startx -= ivPatchSize * (1 + drawhistograms);
      if (startx < 0)
        return;
      int y = ivPatchSize * (i%picspercol);
      if (drawhistograms && (*posPatches)[p][i].histogram != NULL)
      {
        rMat.drawHistogram((*posPatches)[p][i].histogram, startx+ivPatchSize, y, 255, 7, ivPatchSize);
        gMat.drawHistogram((*posPatches)[p][i].histogram, startx+ivPatchSize, y, 0, 7, ivPatchSize);
        bMat.drawHistogram((*posPatches)[p][i].histogram, startx+ivPatchSize, y, 0, 7, ivPatchSize);
      }
      rMat.drawPatch((*posPatches)[p][i].patch, startx, y, (*posPatches)[p][i].avg);
      gMat.drawPatch((*posPatches)[p][i].patch, startx, y, (*posPatches)[p][i].avg);
      bMat.drawPatch((*posPatches)[p][i].patch, startx, y, (*posPatches)[p][i].avg);
      if (!(mode & DEBUG_DRAW_PATCHES))
        break;
    }
  }
}

MultiObjectTLD::MultiObjectTLD(int width, int height, int colorMode, int patchSize, int bbMin, 
                                bool useColor, bool fastRotation, NNClassifier nnc, FernFilter ff, 
                                int nObjects, float aspectRatio, bool learningEnabled)
     : ivWidth(width), ivHeight(height), ivColorMode(colorMode), ivPatchSize(patchSize), 
       ivBBmin(bbMin), ivUseColor(useColor), ivEnableFastRotation(fastRotation),
       ivLKTracker(LKTracker(width, height)), ivNNClassifier(nnc), ivFernFilter(ff), 
       ivNObjects(nObjects), ivAspectRatio(aspectRatio), 
       ivLearningEnabled(learningEnabled), ivNLastDetections(0)
{

  ivCurrentBoxes = std::vector<ObjectBox>(nObjects);
  ivDefined = std::vector<bool>(nObjects, false);
  ivValid = std::vector<bool>(nObjects, false);
  ivCurrentPatches = std::vector<NNPatch>(nObjects);
}


void MultiObjectTLD::saveClassifier(const char* filename) const
{
  std::ofstream fileOutput(filename, std::ios::out | std::ios::binary);
  
  // 1. General motld Data
  fileOutput.write((char*)&ivWidth, sizeof(int));
  fileOutput.write((char*)&ivHeight, sizeof(int));
  fileOutput.write((char*)&ivColorMode, sizeof(int));
  fileOutput.write((char*)&ivPatchSize, sizeof(int));
  fileOutput.write((char*)&ivBBmin, sizeof(int));
  fileOutput.write((char*)&ivUseColor, sizeof(bool));
  fileOutput.write((char*)&ivEnableFastRotation, sizeof(bool));
  fileOutput.write((char*)&ivNObjects, sizeof(int));
  fileOutput.write((char*)&ivAspectRatio, sizeof(float));
  fileOutput.write((char*)&ivLearningEnabled, sizeof(bool));
  
  // 2. nnClassifier
  ivNNClassifier.saveToStream(fileOutput);
  
  // 3. FernFilter
  ivFernFilter.saveToStream(fileOutput);
  
  // eof string
  const char * endstring = "eNd!";
  fileOutput.write(endstring, 4*sizeof(char));
  fileOutput.close();
}

MultiObjectTLD MultiObjectTLD::loadClassifier(const char* filename)
{
  std::ifstream fileInput(filename, std::ios::in | std::ios::binary);
  
  // 1. General motld Data
  int width, height, colorMode, patchSize, bbMin, nObjects; 
  float aspectRatio; bool learningEnabled, useColor, fastRotation;
  fileInput.read((char*)&width, sizeof(int));
  fileInput.read((char*)&height, sizeof(int));
  fileInput.read((char*)&colorMode, sizeof(int));
  fileInput.read((char*)&patchSize, sizeof(int));
  fileInput.read((char*)&bbMin, sizeof(int));
  fileInput.read((char*)&useColor, sizeof(bool));
  fileInput.read((char*)&fastRotation, sizeof(bool));
  fileInput.read((char*)&nObjects, sizeof(int));
  fileInput.read((char*)&aspectRatio, sizeof(float));
  fileInput.read((char*)&learningEnabled, sizeof(bool));
  
  // 2. nnClassifier
  NNClassifier nnc(fileInput); 
  
  // 3. FernFilter
  FernFilter ff = FernFilter::loadFromStream(fileInput);
  
  // parse eof string
  char * endstring = new char[5]; endstring[4] = '\0';
  fileInput.read(endstring, 4*sizeof(char));
  if (endstring[0] == 'e' && endstring[1] == 'N' && 
      endstring[2] == 'd' && endstring[3] == '!') 
  {
    std::cout << "File sucessfully loaded!" << std::endl;
  }
  else
  {
    std::cerr << "Error on loading file!" << std::endl;
  }
  
  fileInput.close();
  
  return MultiObjectTLD(width, height, colorMode, patchSize, bbMin, useColor, fastRotation, nnc, ff, 
                  nObjects, aspectRatio, learningEnabled);
}


#endif // MULTIOBJECTTLD_H
