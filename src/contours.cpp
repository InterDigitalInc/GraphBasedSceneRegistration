#include "contours.hpp"

#include <opencv2/highgui.hpp> //imshow
#include <iostream>  //cout

using namespace cv;
using namespace std;

void drawAllContours(Mat image, Contours contours, Scalar color,
  int thickness, int lineType, int start) {
  for (int b = start; b < contours.size(); b++)
    if (contours[b].size())
      drawContours(image, contours, b, color, thickness, lineType);
}

void drawAllContours(Mat image, Contours contours,
  int thickness, int lineType, int start) {
  for (int b = start; b < contours.size(); b++)
    if (contours[b].size())
      drawContours(image, contours, b,
        Scalar(rand()&255,rand()&255,rand()&255), thickness, lineType);
}

void drawAllContours(Mat image, Contours contours, Mat lut,
  int thickness, int lineType, int start) {
  for (int b = start; b < contours.size(); b++)
    if (contours[b].size())
      drawContours(image, contours, b,
        lut.at<Vec3b>(b+1), thickness, lineType);
}

void colorize(Mat image, Mat& colIm, Mat lut) {
  Mat graIm;
  Mat temp[] = {image, image, image};
  merge(temp, 3, graIm);
  LUT(graIm, lut, colIm);
}

Mat colorize(Mat image, Mat& colIm) {
  Mat lut(1, 256, CV_8UC3);
  Vec3b* p = (Vec3b*)lut.ptr();
  p[0] = Vec3b(0,0,0);
  for (int i = 1; i < 256; i++)
    p[i] = Vec3b((rand()+1)&255,(rand()+1)&255,(rand()+1)&255);
  colorize(image, colIm, lut);
  return lut;
}

Mat colorize(Mat image) {
  Mat colIm;
  colorize(image, colIm);
  return colIm;
}

/*double greenArea(Contour exterior, Contours interiors = Contours()) {
	double area = exterior.GetArea();
  if (interiors.size()) {
    Contours::iterator itContour;
    itContour = interiors.begin();
    while (itContour != interiors.end()) {
      if(*itContour)
        area -= (*itContour)->GetArea();
      itContour++;
    }
  }
	return area;
}*/

double pixelArea(Contour exterior, Mat intMask) {
	double area=0;
	Mat image = Mat::zeros(intMask.size(), CV_8U);
  Contours contours;
  contours.push_back(exterior);
  drawContours(image, contours, 0, Scalar(255), FILLED, 8);
	area = countNonZero(image & intMask);
	return area;
}

int contourArea(Contour exterior, Mat intMask) {
	Mat image = Mat::zeros(intMask.size(), CV_8U);
  Contours contours;
  contours.push_back(exterior);
  drawContours(image, contours, 0, Scalar(255), FILLED, 8);
	return countNonZero(image & intMask);
}

Point2i computeBlobCenter(Contour exterior, Mat intMask) {
  RotatedRect ellipse;
  double area = pixelArea(exterior, intMask);
  cout << "area " << area << endl;
  //if (pixelArea(exterior, intMask) >= 5) {
  if (area >= 5) {
    ellipse = fitEllipse(exterior);
  } else {
    float r;
    minEnclosingCircle(exterior, ellipse.center, r);
    ellipse.size = Size(r, r);
  }

  // Make sure the center of the fitted ellipse/circle is contained in the
  // blob, as this pixel coordinate will be used to compute the 3D position
  // of the associated semantic entity.
  // pointPolygonTest returns positive (inside), negative (outside), or zero (on
  // an edge) value.
  if(pointPolygonTest(exterior, ellipse.center, true) < 0) {
    int closest_lab = -1;
    int min_distance_squared = numeric_limits<int>::max();
    const Point2i old_center(ellipse.center.x, ellipse.center.y);
    for(int i = 0; i < exterior.size(); ++i) {
      const Point2i& pixel = exterior[i];
      const Point2i diff = pixel - old_center;
      const int dist_squared = diff.dot(diff);
      if(dist_squared < min_distance_squared) {
        min_distance_squared = dist_squared;
        closest_lab = i;
      }
    }

    if (closest_lab == -1)
      return Point2i(-1,-1);
    // Assign the pixel associated to the closest lab to the pixel center.
    ellipse.center = exterior[closest_lab];
  }
  // Scale down the ellipse by a factor of two.
  ellipse.size.height *= 0.5;
  ellipse.size.width *= 0.5;
  return ellipse.center;
}

Point2i contourCenter(Contour exterior, Mat intMask) {
  RotatedRect ellipse = fitEllipse(exterior);
  // Make sure the center of the fitted ellipse/circle is contained in the
  // blob, as this pixel coordinate will be used to compute the 3D position
  // of the associated semantic entity.
  // pointPolygonTest returns positive (inside), negative (outside), or zero (on
  // an edge) value.
  if(pointPolygonTest(exterior, ellipse.center, true) < 0) {
    int closest_lab = -1;
    int min_distance_squared = numeric_limits<int>::max();
    const Point2i old_center(ellipse.center.x, ellipse.center.y);
    for(int i = 0; i < exterior.size(); ++i) {
      const Point2i& pixel = exterior[i];
      const Point2i diff = pixel - old_center;
      const int dist_squared = diff.dot(diff);
      if(dist_squared < min_distance_squared) {
        min_distance_squared = dist_squared;
        closest_lab = i;
      }
    }

    if (closest_lab == -1)
      return Point2i(-1,-1);
    // Assign the pixel associated to the closest lab to the pixel center.
    ellipse.center = exterior[closest_lab];
  }
  // Scale down the ellipse by a factor of two.
  ellipse.size.height *= 0.5;
  ellipse.size.width *= 0.5;
  return ellipse.center;
}

//------------------------------------------------------------------------------
const int freemanC[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
const int freemanR[8] = { 0,-1,-1,-1, 0, 1, 1, 1};

bool CCW(int &r, int &c, int &pos, uchar &dir, uchar* colors, uchar* labels,
  const int w, const int h, const uchar label = 255, const uchar color = 255) {
	dir = (dir+6)%8; //+90°
	for (int i = 0; i < 8; i++) {
		int tempR = r+freemanR[dir], tempC = c+freemanC[dir]; //next CCW move
 		if (!(tempR < 0 || tempR >= h || tempC < 0 || tempC >= w)) { //in frame?
			pos = tempR*w+tempC;
			if (colors[pos] == color) { //make the move if we are still in the blob
				r = tempR;
				c = tempC;
        return false;
			}
      if (labels)
				labels[pos] = label; //only mark as contour if its an actual edge
		}
    dir = (dir+1)%8; //-45°
	}
	pos = r*w+c; //cancel making the move
  return true;
}

void CW(int &r, int &c, int &pos, uchar &dir, uchar* colors, uchar* labels,
  const int w, const int h, const uchar label = 127, const uchar color = 255) {
	dir = (dir+2)%8; //-90°
	for (int i = 0; i < 8; i++) {
		int tempR = r+freemanR[dir], tempC = c+freemanC[dir];
		if (!(tempR < 0 || tempR >= h || tempC < 0 || tempC >= w)) {
			pos = tempR*w + tempC;
			if (colors[pos] == color) { //blob is defined by unique color
 				r = tempR;
 				c = tempC;
				return;
			}
      if (labels)
				labels[pos] = label;
		}
		dir = (dir+7)%8; //+45°
	}
	pos = r*w+c;
}

void EXT(int r, int c, int startPos, uchar* colors, const int w, const int h,
  uchar* labels, Contour& contour,
  const uchar label = 255, uchar color = 255) {
	int pos = startPos;//r*w+c;
	uchar dir = 6; //down==-90°
 	if (CCW(r, c, pos, dir, colors, labels, w, h, label, color)) {
    //could not find any neighbors: 1-pixel blob
		labels[pos] = label; //pos == startPos in theory
 		return;
 	}
	while (pos != startPos) { //go around the contours of the blob
    contour.emplace_back(c,r);
		labels[pos] = label;
		CCW(r, c, pos, dir, colors, labels, w, h, label, color);
	}
  contour.emplace_back(c,r);
	labels[pos] = label;
	//For blobs in which the starting point must be crossed many times
  //size 4: can the point only be reached from this limited number of angles?
	for (int i = 0; i < 3; i++) {
		CCW(r, c, pos, dir, colors, labels, w, h, label, color);
		if (!labels[pos]) { //check if we have not been here already
			while (pos != startPos) {
        contour.emplace_back(c,r);
        labels[pos] = label;
				CCW(r, c, pos, dir, colors, labels, w, h, label, color);
			}
      contour.emplace_back(c,r);
			labels[pos] = label;
		} else
			break;
	}
}

void INT(int r, int c, int startPos, uchar* colors, const int w, const int h,
  uchar* labels, Contour& contour,
  const uchar label = 127, const uchar color = 255) {
	int pos = startPos;//r*w+c;
	uchar dir = 5;
	CW(r, c, pos, dir, colors, labels, w, h, label, color);
  contour.emplace_back(c,r);
	labels[pos] = label;
	while (pos != startPos) {
		CW(r, c, pos, dir, colors, labels, w, h, label, color);
    contour.emplace_back(c,r);
    labels[pos] = label;
	}
	//If the starting point must be crossed many times:
	for (int i = 0; i < 3; i++) {
		CW(r, c, pos, dir, colors, labels, w, h, label, color);
    //pixels COULD be shared by internal contours but MUST NOT be
    if (!labels[pos]) {//if (labels[pos] != label) {
			while (pos != startPos) {
        contour.emplace_back(c,r);
        labels[pos] = label;
				CW(r, c, pos, dir, colors, labels, w, h, label, color);
			}
      contour.emplace_back(c,r);
			labels[pos] = label;
		} else
			break;
	}
}

// Mono-labeler
void labeling(Mat binIm, Exteriors &exteriors, Interiors &interiors) {
  Mat image = binIm.isContinuous() ? binIm : binIm.clone();
  Mat labIm = Mat::zeros(image.size(), CV_8U);
  uchar* labels = labIm.data;
  uchar* colors = image.data;
	const int h = image.size().height;
	const int w = image.size().width;

	uchar label = 0;
  //color
	for (int r = 0; r < h; r++) {
		//First col, c==0
		int pos = r*w;
    //color
		if (colors[pos]) {                        //we are in a blob
      if (labels[pos]) //we do not want to change the label for the interiors
        label = labels[pos];
			else { //if (!labels[pos])            //the pixel is not part of a contour
        exteriors.emplace_back();           //create a new exterior
        interiors.emplace_back();           //create a new interior group
        exteriors.back().emplace_back(0,r);
        label = exteriors.size();
        EXT(r, 0, pos, colors, w, h, labels, exteriors.back(), label);
			}
      //we are not going in a blob and it was not labeled (as an exterior)
			if (!labels[pos+1] && !colors[pos+1]) { //leaving the blob temporarily
        interiors[label-1].emplace_back();           //create a new interior
        interiors[label-1].back().emplace_back(0,r);
        //label = 255-interiors.size();
        INT(r, 0, pos, colors, w, h, labels, interiors[label-1].back(), label);
      }
		}
		//Other cols
    pos++;
		for (int c = 1; c < w-1; c++, pos++) {
      //color
			if (colors[pos]) {                        //we are in a blob
        //the pixel is not part of a contour and we were not in a blob before
        if (labels[pos]) //we do not want to change the label for the interiors
          label = labels[pos];
  			else if (!colors[pos-1]) { //!labels[pos] && //entering the blob
          exteriors.emplace_back();                //create a new exterior
          interiors.emplace_back();                //create a new interior group
          exteriors.back().emplace_back(c,r);
          label = exteriors.size();
          EXT(r, c, pos, colors, w, h, labels, exteriors.back(), label);
				}

 				if (!labels[pos+1] && !colors[pos+1]) { //leaving the blob temporarily
          interiors[label-1].emplace_back();           //create a new interior
          interiors[label-1].back().emplace_back(c,r);
          //label = 255-interiors.size();
          INT(r, c, pos, colors, w, h, labels, interiors[label-1].back(), label);
        }
			}
		}
    //Last col, c==w-1
    //color
		if (colors[pos]) {
      if (labels[pos]) //we do not want to change the label for the interiors
        label = labels[pos];

      else if (!colors[pos-1]) {   //entering the blob
        exteriors.emplace_back();           //create a new exterior
        interiors.emplace_back();           //create a new interior group
        exteriors.back().emplace_back(w-1,r);
        label = exteriors.size();
        EXT(r, w-1, pos, colors, w, h, labels, exteriors.back(), label);
			}
      //there is nowhere to leave to
		}
	}

  imshow("label",255*labIm);
  waitKey(0);
  exit(0);
}

// Multi-labeler
void multiLabeling(Mat mulIm, Exteriors &exteriors, Interiors &interiors) {
  Mat image = mulIm.isContinuous() ? mulIm : mulIm.clone();
  Mat labIm = Mat::zeros(image.size(), CV_8U);
  uchar* labels = labIm.data;
	uchar* colors = image.data;
	const int h = image.size().height;
	const int w = image.size().width;

  uchar label = 0; //contour index
  uchar color = 0; //object label
	for (int r = 0; r < h; r++) {
		//First col, c==0
		int pos = r*w;
    color = colors[pos];
		if (color) {                        //we are in a blob (non-zero)
      if (labels[pos]) //we do not want to change the label for the interiors
        label = labels[pos]; //TODO: not working because of "thick" countours
			else { //if (!labels[pos])            //the pixel is not part of a contour
        exteriors.emplace_back();           //create a new exterior
        interiors.emplace_back();           //create a new interior group
        exteriors.back().emplace_back(0,r);
        label = exteriors.size();//label++; //
        EXT(r, 0, pos, colors, w, h, labels, exteriors.back(), label, color);
			}
      //changing color: leaving the blob temporarily
			if (!labels[pos+1] && colors[pos+1] != color) {
        interiors[label-1].emplace_back();           //create a new interior
        interiors[label-1].back().emplace_back(0,r);
        //label = 255-interiors.size();
        INT(r, 0, pos, colors, w, h, labels, interiors[label-1].back(), label, color);
      }
		}
		//Other cols
    pos++;
		for (int c = 1; c < w-1; c++, pos++) {
      color = colors[pos];
			if (color) {                        //we are in a blob
        if (labels[pos])
          label = labels[pos]; //TODO: not working because of "thick" countours
        //changing color: entering a new blob
        else if (colors[pos-1] != color) { //!labels[pos] &&
          exteriors.emplace_back();           //create a new exterior
          interiors.emplace_back();           //create a new interior group
          exteriors.back().emplace_back(c,r); //add initial point
          label = exteriors.size();//label++; //
          EXT(r, c, pos, colors, w, h, labels, exteriors.back(), label, color);
				}
        //changing color: leaving the blob temporarily
 				if (!labels[pos+1] && colors[pos+1] != color) {
          interiors[label-1].emplace_back();           //create a new interior
          interiors[label-1].back().emplace_back(c,r); //add initial point
          //label = 255-interiors.size();
          INT(r, c, pos, colors, w, h, labels, interiors[label-1].back(), label, color);
        }
			}
		}
    //Last col, c==w-1
    color = colors[pos];
    if (color) {                        //we are in a blob
      if (labels[pos])
        label = labels[pos]; //TODO: not working because of "thick" countours
      //changing color: entering a new blob
      else if (colors[pos-1] != color) { //!labels[pos] &&
        exteriors.emplace_back();           //create a new exterior
        interiors.emplace_back();           //create a new interior group
        exteriors.back().emplace_back(w-1,r);
        label = exteriors.size();
        EXT(r, w-1, pos, colors, w, h, labels, exteriors.back(), label, color);
			}
      //there is nowhere to leave to
		}
	}

  imshow("label",16*labIm);
  waitKey(0);
  exit(0);
}

// Fast multi-labeler
void INT(int r, int c, int startPos, uchar* colors, uchar color,
  const int w, const int h, uchar* labels) {
	int pos = startPos;//sR*w+sC;
	uchar dir = 5;
	CW(r, c, pos, dir, colors, labels, w, h, 127, color);
	labels[pos] = 127;
	while (pos != startPos) {
		CW(r, c, pos, dir, colors, labels, w, h, 127, color);
    labels[pos] = 127;
	}
	//If the starting point must be crossed many times:
	for (int i = 0; i < 3; i++) {
		CW(r, c, pos, dir, colors, labels, w, h, 127, color);
    //pixels COULD be shared by internal contours but MUST NOT be
    if (!labels[pos]) {//if (labels[pos] != label) {
			while (pos != startPos) {
        labels[pos] = 127;
				CW(r, c, pos, dir, colors, labels, w, h, 127, color);
			}
			labels[pos] = 127;
		} else
			break;
	}
}

void multiLabeling(Mat mulIm, Contours &exteriors) {
  Mat image = mulIm.isContinuous() ? mulIm : mulIm.clone();
  Mat labIm = Mat::zeros(image.size(), CV_8U);
  uchar* labels = labIm.data;
	uchar* colors = image.data;
	const int h = image.size().height;
	const int w = image.size().width;

  uchar color = 0; //object label
	for (int r = 0; r < h; r++) {
		//First col, c==0
		int pos = r*w;
    color = colors[pos];
		if (color) {                        //we are in a blob (non-zero)
			if (!labels[pos]) {                  //the pixel is not part of a contour
        exteriors.emplace_back();           //create a new exterior
        exteriors.back().emplace_back(0,r);
        EXT(r, 0, pos, colors, w, h, labels, exteriors.back(), color);
			}
      //changing color: leaving the blob temporarily
			if (!labels[pos+1] && colors[pos+1] != color)
        INT(r, 0, pos, colors, color, w, h, labels);
		}
		//Other cols
    pos++;
		for (int c = 1; c < w-1; c++, pos++) {
      color = colors[pos];
			if (color) {                        //we are in a blob
        //changing color: entering a new blob
        if (!labels[pos] && colors[pos-1] != color) {
          exteriors.emplace_back();           //create a new exterior
          exteriors.back().emplace_back(c,r);
          EXT(r, c, pos, colors, w, h, labels, exteriors.back(), color);
				}
        //changing color: leaving the blob temporarily
 				if (!labels[pos+1] && colors[pos+1] != color)
          INT(r, c, pos, colors, color, w, h, labels);
			}
		}
    //Last col, c==w-1
    color = colors[pos];
    if (color) {                        //we are in a blob
      //changing color: entering a new blob
      if (!labels[pos] && colors[pos-1] != color) {
        exteriors.emplace_back();           //create a new exterior
        exteriors.back().emplace_back(w-1,r);
        EXT(r, w-1, pos, colors, w, h, labels, exteriors.back(), color);
			}
      //there is nowhere to leave to
		}
	}

  imshow("label",16*labIm);
  waitKey(0);
  exit(0);
}

// Thin multi-labeler
void EXT_thin(int r, int c, int startPos, uchar* colors, const int w, const int h,
  uchar* labels, Contour& contour,
  const uchar label = 255, uchar color = 255) {
	int pos = startPos;//r*w+c;
	uchar dir = 6; //down==-90°
 	if (CCW(r, c, pos, dir, colors, NULL, w, h, label, color)) {
    //could not find any neighbors: 1-pixel blob
		labels[pos] = label; //pos == startPos in theory
 		return;
 	}
	while (pos != startPos) { //go around the contours of the blob
    contour.emplace_back(c,r);
		labels[pos] = label;
		CCW(r, c, pos, dir, colors, NULL, w, h, label, color);
	}
  contour.emplace_back(c,r);
	labels[pos] = label;
	//For blobs in which the starting point must be crossed many times
  //size 4: can the point only be reached from this limited number of angles?
	for (int i = 0; i < 3; i++) {
		CCW(r, c, pos, dir, colors, NULL, w, h, label, color);
		if (!labels[pos]) { //check if we have not been here already
			while (pos != startPos) {
        contour.emplace_back(c,r);
        labels[pos] = label;
				CCW(r, c, pos, dir, colors, NULL, w, h, label, color);
			}
      contour.emplace_back(c,r);
			labels[pos] = label;
		} else
			break;
	}
}

void INT_thin(int r, int c, int startPos, uchar* colors, const int w, const int h,
  uchar* labels, Contour& contour,
  const uchar label = 127, const uchar color = 255) {
	int pos = startPos;//r*w+c;
	uchar dir = 5;
	CW(r, c, pos, dir, colors, NULL, w, h, label, color);
  contour.emplace_back(c,r);
	labels[pos] = label;
	while (pos != startPos) {
		CW(r, c, pos, dir, colors, NULL, w, h, label, color);
    contour.emplace_back(c,r);
    labels[pos] = label;
	}
	//If the starting point must be crossed many times:
	for (int i = 0; i < 3; i++) {
		CW(r, c, pos, dir, colors, NULL, w, h, label, color);
    //pixels COULD be shared by internal contours but MUST NOT be
    if (!labels[pos]) {//if (labels[pos] != label) {
			while (pos != startPos) {
        contour.emplace_back(c,r);
        labels[pos] = label;
				CW(r, c, pos, dir, colors, NULL, w, h, label, color);
			}
      contour.emplace_back(c,r);
			labels[pos] = label;
		} else
			break;
	}
}

void thinLabeling(Mat mulIm, Exteriors &exteriors, Interiors &interiors) {
  Mat image = mulIm.isContinuous() ? mulIm : mulIm.clone();
  Mat labIm = Mat::zeros(image.size(), CV_8U);
  uchar* labels = labIm.data;
	uchar* colors = image.data;
	const int h = image.size().height;
	const int w = image.size().width;

  uchar label = 0; //contour index
  uchar color = 0; //object label
	for (int r = 0; r < h; r++) {
		//First col, c==0
		int pos = r*w;
    color = colors[pos];
    if (labels[pos]) //we do not want to change the label for the interiors
      label = labels[pos]; //TODO: not working because of "thick" countours
    else { //if (!labels[pos])            //the pixel is not part of a contour
      exteriors.emplace_back();           //create a new exterior
      interiors.emplace_back();           //create a new interior group
      exteriors.back().emplace_back(0,r);
      label = exteriors.size();//label++; //
      EXT_thin(r, 0, pos, colors, w, h, labels, exteriors.back(), label, color);
    }
		//Other cols
    pos++;
		for (int c = 1; c < w-1; c++, pos++) {
      color = colors[pos];
      if (labels[pos])
        label = labels[pos]; //TODO: not working because of "thick" countours
      //changing color: entering a new blob
      else if (colors[pos-1] != color) { //!labels[pos] &&
        exteriors.emplace_back();           //create a new exterior
        interiors.emplace_back();           //create a new interior group
        exteriors.back().emplace_back(c,r); //add initial point
        label = exteriors.size();//label++; //
        EXT_thin(r, c, pos, colors, w, h, labels, exteriors.back(), label, color);
      }
		}
    //Last col, c==w-1
    color = colors[pos];
    if (labels[pos])
      label = labels[pos]; //TODO: not working because of "thick" countours
    //changing color: entering a new blob
    else if (colors[pos-1] != color) { //!labels[pos] &&
      exteriors.emplace_back();           //create a new exterior
      interiors.emplace_back();           //create a new interior group
      exteriors.back().emplace_back(w-1,r);
      label = exteriors.size();
      EXT_thin(r, w-1, pos, colors, w, h, labels, exteriors.back(), label, color);
    }
	}

  //imshow("label",255*labIm);
  //imwrite("label.png",labIm);

  string name = "t3";
  Mat colIm = colorize(mulIm);
  imshow(name+"-label",colIm);
  imwrite(name+"-label.png",colIm);
  cout << name+"-label" << endl;

  Mat lut = colorize(labIm, colIm);
  imshow(name+"-contours",colIm);
  imwrite(name+"-contours.png",colIm);
  cout << name+"-contours" << endl;

  colIm = Scalar(0,0,0);
  drawAllContours(colIm, exteriors, lut, 1, LINE_8);
  imshow(name+"-exteriors",colIm);
  imwrite(name+"-exteriors.png",colIm);
  cout << name+"-exteriors" << endl;

  waitKey(0);
  exit(0);
}
