
/*
*  Vincent and Soille ��ˮ���û�㷨(1991)��ʵ��
*/

#ifndef WATERSHEDALGORITHM_H
#define WATERSHEDALGORITHM_H

#include <cv.h>
#include <highgui.h>
#include <string>
#include <queue>
#include "WatershedStructure.h"

class WatershedAlgorithm {
    static const int HMIN = 0;	// ��С��
    static const int HMAX = 256;	// ����

public:
    void run(IplImage* pSrc, const std::string& imgName) { // ��ˮ������㷨
        std::string inTmp;

		/* ���ɻҶ�ͼ */
        IplImage* pGray = cvCreateImage(cvGetSize(pSrc), IPL_DEPTH_8U, 1);
        if (pSrc->nChannels == 3) {
            cvCvtColor(pSrc, pGray, CV_BGR2GRAY);
            inTmp = imgName + "_Gray.jpg"; cvSaveImage(inTmp.c_str(), pGray);
        }
        else if (pSrc->nChannels == 1)
            cvCopyImage(pSrc, pGray);

		/* ���ɶ�ֵͼ(�ڰ�ͼ) */
        IplImage* pBW = cvCreateImage(cvGetSize(pGray), IPL_DEPTH_8U, 1);
        cvAdaptiveThreshold(pGray, pBW, 255, 0, 0, 31); // ����Ӧ��ֵ������
        inTmp = imgName + "_BW.jpg"; cvSaveImage(inTmp.c_str(), pBW);

		/* ��ȡͼ����Ϣ */
        char* pixels = pBW->imageData;
        int width = pBW->width;
        int height = pBW->height;

        /* Vincent and Soille ��ˮ���㷨��1991����һ��: �����ص����ṹ�岢���� */
        WatershedStructure  watershedStructure(pixels, width, height);

		/* Vincent and Soille ��ˮ���㷨��1991���ڶ���: ���飨ģ���û�� */
        /************************ ���飨��û����ʼ ****************************/
        std::queue<WatershedPixel*> pque;	// �洢���ص���ʱ����
        int curlab = 0;
        int heightIndex1 = 0;
        int heightIndex2 = 0;

        for (int h = HMIN; h < HMAX; ++h) { // h-1 ��� Geodesic SKIZ

            for (int pixelIndex = heightIndex1 ; pixelIndex < watershedStructure.size() ; ++pixelIndex) {
                WatershedPixel* p = watershedStructure.at(pixelIndex);

				/* �����ص�λ�� h+1 �㣬�ݲ���������ѭ�� */
                if (p->getIntHeight() != h) { heightIndex1 = pixelIndex; break; }

                p->setLabelToMASK(); // ��Ǵ����ؽ�������

                std::vector<WatershedPixel*> neighbours = p->getNeighbours();
                for (unsigned i = 0 ; i < neighbours.size() ; ++i) {
                    WatershedPixel* q =  neighbours.at(i);

					 /* ��������ػ��ˮ���h����ڽ����ص���� */
                    if (q->getLabel() >= 0) { p->setDistance(1); pque.push(p); break; }
                }
            }

            int curdist = 1;
            pque.push(new WatershedPixel());

            while (true) { // ��չ��ˮ���
                WatershedPixel* p = pque.front(); pque.pop();

                if (p->isFICTITIOUS())
                    if (pque.empty()) { delete p; p = NULL; break; }
                    else {
                        pque.push(new WatershedPixel());
                        curdist++;
                        delete p; p = pque.front(); pque.pop();
                    }

                std::vector<WatershedPixel*> neighbours = p->getNeighbours();
                for (unsigned i = 0 ; i < neighbours.size() ; ++i) { // ͨ������ڽ���������� p
                    WatershedPixel* q =  neighbours.at(i);

					/* q����һ�����ڵ���ػ��ˮ�� */ 
                    if ( (q->getDistance() <= curdist) &&  (q->getLabel() >= 0) ) {             

                        if (q->getLabel() > 0) {
                            if ( p->isLabelMASK() )
                                p->setLabel(q->getLabel());
                            else if (p->getLabel() != q->getLabel())
                                p->setLabelToWSHED();
                        } else if (p->isLabelMASK()) 
							p->setLabelToWSHED();
                    } else if ( q->isLabelMASK() && (q->getDistance() == 0) ) {
                        q->setDistance( curdist + 1 );
                        pque.push(q);
                    }
                } // �����ڽ����ص�forѭ��
            } // ��չ��ص�whileѭ��

            /* ��Ѱ������h�����µ���Сֵ */
            for (int pixelIndex = heightIndex2 ; pixelIndex < watershedStructure.size() ; pixelIndex++) {
                WatershedPixel* p = watershedStructure.at(pixelIndex);

				/* �����ص�λ�� h+1 �㣬�ݲ���������ѭ�� */
                if (p->getIntHeight() != h) { heightIndex2 = pixelIndex; break; }

                p->setDistance(0); // ���þ���Ϊ0

                if (p->isLabelMASK()) { // ������λ������Сֵ����
                    curlab++;
                    p->setLabel(curlab);
                    pque.push(p);

                    while (!pque.empty()) {
                        WatershedPixel* q = pque.front();
                        pque.pop();

                        std::vector<WatershedPixel*> neighbours = q->getNeighbours();

                        for (unsigned i = 0 ; i < neighbours.size() ; i++) { // ���p2����������
                            WatershedPixel* r =  neighbours.at(i);

                            if ( r->isLabelMASK() ) { r->setLabel(curlab); pque.push(r); }
                        }
                    } // end while
                } // end if
            } // end for
        }
		/************************ ���飨��û������ ****************************/

		/* ���ɷ�ˮ��ͼ�� */
        IplImage* pWS = cvCreateImage(cvGetSize(pBW), IPL_DEPTH_8U, 1);
        //cvCopyImage(pBW, pWS);
		cvZero(pBW);
        char* wsPixels = pWS->imageData;
        char* grayPixels = pGray->imageData;
		/*
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                wsPixels[x + y * width] = (char)0;
		*/

        for (int pixelIndex = 0 ; pixelIndex < watershedStructure.size() ; pixelIndex++) {
            WatershedPixel* p = watershedStructure.at(pixelIndex);

            if (p->isLabelWSHED() && !p->allNeighboursAreWSHED()) {
                wsPixels[p->getX() + p->getY()*width] = (char)255; // �ں�ɫ�����л��ư�ɫ��ˮ��
                grayPixels[p->getX() + p->getY()*width] = (char)255;	// �ڻҶ�ͼ�л��ư�ɫ��ˮ��
            }
        }
        inTmp = imgName + "_WS.jpg"; cvSaveImage(inTmp.c_str(), pWS);
        inTmp = imgName + "_Gray_WS.jpg"; cvSaveImage(inTmp.c_str(), pGray);

		cvReleaseImage(&pGray);
		cvReleaseImage(&pBW);
    }
};

#endif