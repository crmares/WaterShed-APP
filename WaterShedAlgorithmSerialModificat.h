
/*
*  Vincent and Soille 分水岭浸没算法(1991)的实现
*/

#ifndef WATERSHEDALGORITHM_H
#define WATERSHEDALGORITHM_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <queue>
#include "WatershedStructure.h"

#include <omp.h>

class WatershedAlgorithm {
    static const int HMIN = 0;
    static const int HMAX = 256;

public:
    void run(IplImage* pSrc, const std::string& imgName) {
        std::string inTmp;

        IplImage* pGray = cvCreateImage(cvGetSize(pSrc), IPL_DEPTH_8U, 1);
        if (pSrc->nChannels == 3) {
            cvCvtColor(pSrc, pGray, CV_BGR2GRAY);
            inTmp = imgName + "_Gray.jpg"; cvSaveImage(inTmp.c_str(), pGray);
        }
        else if (pSrc->nChannels == 1)
            cvCopyImage(pSrc, pGray);

        IplImage* pBW = cvCreateImage(cvGetSize(pGray), IPL_DEPTH_8U, 1);
        cvAdaptiveThreshold(pGray, pBW, 255, 0, 0, 31);
        inTmp = imgName + "_BW.jpg"; cvSaveImage(inTmp.c_str(), pBW);

        char* pixels = pBW->imageData;
        int width = pBW->width;
        int height = pBW->height;

        //pixels reprezinta culoarea(grey -un singur canal) al pixelului
        //Fiind in format grey, reprezinta practic cat de luminos e pixelul
        WatershedStructure  watershedStructure(pixels, width, height);
	
	int wsSize = watershedStructure.size();
        std::queue<WatershedPixel*> pque;
        int curlab = 0;

        for (int h = HMIN; h < HMAX; ++h) {

            for (int pixelIndex = 0; pixelIndex < wsSize; ++pixelIndex) {
                WatershedPixel* p = watershedStructure.at(pixelIndex);

                if (p->getIntHeight() == h) { 

                	p->setLabelToMASK();

                	std::vector<WatershedPixel*> neighbours = p->getNeighbours();
                    for (unsigned i = 0 ; i < neighbours.size() ; ++i) {
                    	WatershedPixel* q =  neighbours.at(i);

                    	if (q->getLabel() >= 0) { 
				p->setDistance(1);
				pque.push(p);
				break; 
			}
                    }
		}
            } //pixel for

            int curdist = 1;
            pque.push(new WatershedPixel());

            while (true) {
                WatershedPixel* p = pque.front(); pque.pop();

                if (p->isFICTITIOUS())
                    if (pque.empty()) { delete p; p = NULL; break; }
                    else {
                        pque.push(new WatershedPixel());
                        curdist++;
                        delete p; p = pque.front(); pque.pop();
                    }

                std::vector<WatershedPixel*> neighbours = p->getNeighbours();
		int neighSize = neighbours.size();
                //parcurg vecinii
                for (unsigned i = 0 ; i < neighSize; ++i) {
                    WatershedPixel* q =  neighbours.at(i);

                    if ( (q->getDistance() <= curdist) &&  (q->getLabel() >= 0) ) {
                        if (q->getLabel() > 0) {
                            if ( p->isLabelMASK() )
                                p->setLabel(q->getLabel());
                            else if (p->getLabel() != q->getLabel())
                                p->setLabelToWSHED();
                        }
			else if (p->isLabelMASK())
				p->setLabelToWSHED();
                    } else if ( q->isLabelMASK() && (q->getDistance() == 0) ) {
                        q->setDistance( curdist + 1 );
                        pque.push(q);
                    }
                } // neighbours for
            } // while

		//aici pque e mereu empty
            for (int pixelIndex = 0; pixelIndex < wsSize; pixelIndex++) {
                WatershedPixel* p = watershedStructure.at(pixelIndex);

            	if (p->getIntHeight() == h) { 

                   p->setDistance(0);

                   if (p->isLabelMASK()) {
                     curlab++;
                     p->setLabel(curlab);
                     pque.push(p);

                     while (!pque.empty()) {
                        WatershedPixel* q = pque.front();
                        pque.pop();

                        std::vector<WatershedPixel*> neighbours = q->getNeighbours();

                        for (unsigned i = 0 ; i < neighbours.size() ; i++) { // 检查p2的邻域像素
                            WatershedPixel* r =  neighbours.at(i);

                            if ( r->isLabelMASK() ) {
				r->setLabel(curlab);
				pque.push(r);
			    }
                        }
               	     } // end while
                   } // end if
	       } //end ==h if
            } // end for
        }

        IplImage* pWS = cvCreateImage(cvGetSize(pBW), IPL_DEPTH_8U, 1);
        //cvCopyImage(pBW, pWS);
		cvZero(pBW);
        char* wsPixels = pWS->imageData;
        char* grayPixels = pGray->imageData;

        for (int pixelIndex = 0 ; pixelIndex < watershedStructure.size() ; pixelIndex++) {
            WatershedPixel* p = watershedStructure.at(pixelIndex);

            if (p->isLabelWSHED() && !p->allNeighboursAreWSHED()) {
                wsPixels[p->getX() + p->getY()*width] = (char)255; // 在黑色背景中绘制白色分水线
                grayPixels[p->getX() + p->getY()*width] = (char)255;	// 在灰度图中绘制白色分水线
            }
        }
        inTmp = imgName + "_WS.jpg"; cvSaveImage(inTmp.c_str(), pWS);
        inTmp = imgName + "_Gray_WS.jpg"; cvSaveImage(inTmp.c_str(), pGray);

		cvReleaseImage(&pGray);
		cvReleaseImage(&pBW);
    }
};

#endif
