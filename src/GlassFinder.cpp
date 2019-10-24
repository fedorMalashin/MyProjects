#include <opencv2\opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace cv;

//int AverageBrightness

Point FindFirstBrightestPoint(const Mat &aImage, const Vec4i &aLine)
{
  LineIterator _iterator = LineIterator(aImage, Point(aLine[0], aLine[1]), Point(aLine[2], aLine[3]));

  Point _ret_point = Point(0, 0);

  int _channels = aImage.channels();
  int _bright_shift = 2;

  for (int i = 0; i < _iterator.count; ++i, _iterator++)
  {
    int _brightness = aImage.at<Vec3b>(_iterator.pos())[0];

    if (_brightness >= 55)
    {
      _ret_point.x = _iterator.pos().x; 
      _ret_point.y = _iterator.pos().y;

      _iterator++;
      for (int j = i + 1; i < _iterator.count; ++j, _iterator++)
      {
        int _new_brightness = aImage.at<Vec3b>(_iterator.pos())[0];
        if (_new_brightness >= _brightness - _bright_shift)
        {
          _ret_point.x = _iterator.pos().x;
          _ret_point.y = _iterator.pos().y;
        }
        else
          break;
      }
      return _ret_point;
    }
  }
  return _ret_point;
}

Vec4i LineByAngle(const Point &aBegin, double aLength, double aAngle)
{
  return Vec4i(
    aBegin.x, 
    aBegin.y, 
    Point(static_cast<int>(aBegin.x + aLength * cos(aAngle)), static_cast<int>(aBegin.y + aLength * sin(aAngle))).x,
    Point(static_cast<int>(aBegin.x + aLength * cos(aAngle)), static_cast<int>(aBegin.y + aLength * sin(aAngle))).y
  );
}

std::vector<Rect> FindPointFromCenterRect(Mat &aImage, const Point &aCenter)
{
  std::vector<Rect> _rects;
  std::vector<Point> _found_points_vec;

  for (double i = 0; i <= 360.0; i += 0.05)
  {
    Vec4i _line_from_center = LineByAngle(aCenter, 600.0, M_PI * i / 180.0);
    Point _found_point = FindFirstBrightestPoint(aImage, _line_from_center);

    if (_found_point.x != 0.0 && _found_point.y != 0.0)
    {
      _rects.push_back(Rect(_found_point, Size(2, 2)));
      _found_points_vec.push_back(_found_point);
    }
  }

  if (aImage.channels() == 1)
    cvtColor(aImage, aImage, COLOR_GRAY2BGR);

  for (Rect element : _rects)
    rectangle(aImage, element, Scalar(7, 7, 247));

  return _rects;
}

std::vector<Point> FindPointFromCenterPoints(Mat &aImage, const Point &aCenter)
{
  std::vector<Rect> _rects;
  std::vector<Point> _found_points_vec;

  for (double i = 0; i <= 360.0; i += 0.05)
  {
    Vec4i _line_from_center = LineByAngle(aCenter, 600.0, M_PI * i / 180.0);
    Point _found_point = FindFirstBrightestPoint(aImage, _line_from_center);

    if (_found_point.x != 0.0 && _found_point.y != 0.0)
    {
      _rects.push_back(Rect(_found_point, Size(2, 2)));
      _found_points_vec.push_back(_found_point);
    }
  }

  if (aImage.channels() == 1)
    cvtColor(aImage, aImage, COLOR_GRAY2BGR);

  for (Rect element : _rects)
    rectangle(aImage, element, Scalar(7, 7, 247));

  return _found_points_vec;
}



Mat CreateVector(Mat &aSrc)
{
  Mat _dst;

  if (aSrc.channels() > 1)
    cvtColor(aSrc, _dst, COLOR_BGR2GRAY);
  else _dst = aSrc.clone();

  int _channels = aSrc.channels();

  double _top_left, _left, _top, _center, _top_right, _right, _bott_left, _bottom, _bott_right, _diffGreen, _diffBlue, _diffRed;
  _diffGreen = 0;

  auto _src_ptr = aSrc.data;
  auto _dst_ptr = _dst.data;
  auto _src_step_0 = aSrc.step[0];
  auto _src_step_1 = aSrc.step[1];
  auto _dst_step_0 = _dst.step[0];
  auto _dst_step_1 = _dst.step[1];


  for (int x = 1; x < _dst.rows - 1; ++x)
  {
    for (int y = 1; y < _dst.cols - 1; ++y)
    {
      if (_channels == 1)
      {
        _top_left = aSrc.at<unsigned char>(x - 1, y - 1);
        _left = aSrc.at<unsigned char>(x, y - 1);
        _top = aSrc.at<unsigned char>(x - 1, y);
        _center = aSrc.at<unsigned char>(x, y);

        _diffGreen = (abs(_top_left - _center) + abs(_left - _center) + abs(_top - _center)) / 3;
      }
      else if (_channels >= 3)
      {
        _top_left = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y - 1) + 0];
        _left = _src_ptr[_src_step_0 * (x) + _src_step_1 * (y - 1) + 0];
        _top = _src_ptr[_src_step_0 * (x - 1)+_src_step_1 * (y) + 0];
        _center = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y) + 0];
        _top_right = _src_ptr[_src_step_0 * (x - 1)+_src_step_1 * (y + 1) + 0];
        _right = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y - 1) + 0];
        _bott_left = _src_ptr[_src_step_0 * (x + 1)+_src_step_1 * (y - 1) + 0];
        _bottom = _src_ptr[_src_step_0 * (x + 1)+_src_step_1 * (y) + 0];
        _bott_right = _src_ptr[_src_step_0 * (x + 1)+_src_step_1 * (y + 1) + 0];

        _diffBlue = abs(_left - _center) + abs(_top - _center) + abs(_right - _center) + abs(_bottom - _center);

        _top_left = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y - 1) + 1];
        _left = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y - 1) + 1];
        _top = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y) + 1];
        _center = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y) + 1];
        _top_right = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y + 1) + 1];
        _right = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y - 1) + 1];
        _bott_left = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y - 1) + 1];
        _bottom = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y) + 1];
        _bott_right = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y + 1) + 1];

        _diffGreen = abs(_left - _center) + abs(_top - _center) + abs(_right - _center) + abs(_bottom - _center);

        _top_left = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y - 1) + 2];
        _left = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y - 1) + 2];
        _top = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y) + 2];
        _center = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y) + 2];
        _top_right = _src_ptr[_src_step_0 * (x - 1) + _src_step_1 * (y + 1) + 2];
        _right = _src_ptr[_src_step_0 * (x)+_src_step_1 * (y - 1) + 2];
        _bott_left = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y - 1) + 2];
        _bottom = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y) + 2];
        _bott_right = _src_ptr[_src_step_0 * (x + 1) + _src_step_1 * (y + 1) + 2];

        _diffRed = abs(_left - _center) + abs(_top - _center) + abs(_right - _center) + abs(_bottom - _center);

        _diffGreen = _diffBlue + _diffGreen + _diffRed;

        if (_diffGreen > 255) _diffGreen = 255;
        if (_diffGreen < 0) _diffGreen = 0;

        _dst_ptr[_dst_step_0 * (x) + (y)+0] = static_cast<unsigned char>(_diffGreen);
      }
    }
  }

  // убираем рамку
  // Наводим резкозть (убираем неяркие точки, повышаем контрастность)
  // Удаляем одиночные точки, а неодиночным добавляем яркости
  for (int x = 0; x < _dst.rows; ++x)
    for (int y = 0; y < _dst.cols; ++y)
    {
      // убираем рамку
      if (x < 2 || y < 2 || x > _dst.rows - 2 || y > _dst.cols - 2)
        _dst_ptr[_dst_step_0 * (x) + (y)+0] = 0;
    
      else // повышаем контрастность
      {
        _top_left = _dst_ptr[_dst_step_0 * (x) + (y)+0];
        if (_top_left > 255) _top_left = 255;
        //if (_top_left <= 30) _top_left = 0;

        _dst_ptr[_dst_step_0 * (x) + (y)+0] = static_cast<unsigned char>(_top_left);
      }
    }

  cvtColor(_dst, aSrc, COLOR_GRAY2BGR);
 
  return aSrc;
}


Mat DrawLinesPoints(Mat &aSrc, std::vector<Point> &aPoints)
{
  Mat _dst;

  _dst = aSrc;

  _dst.setTo(0);

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _dst);
  waitKey(0);

  int _channels = _dst.channels();

  cv::Mat _dis = Mat(_dst.rows, _dst.cols, CV_8UC1);

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _dis);
  waitKey(0);

  Canny(_dis, _dis, 50, 200, 3);

  std::vector<Point2f> _fpoints;

  for (auto el : aPoints)
    _fpoints.push_back(Point2f(static_cast<float>(el.x), static_cast<float>(el.y)));


  // Probabilistic Line Transform
  double rhoMin = 0.0f, rhoMax = 360.0f, rhoStep = 1;
  double thetaMin = 0.0f, thetaMax = CV_PI / 2.0f, thetaStep = CV_PI / 180.0f;

  std::vector<Vec3d> linesP; // will hold the results of the detection
  //HoughLinesP(_dis, linesP, 15, 0.5, 100, 50, 150); // runs the actual detection

  Mat lines;
  HoughLinesPointSet(_fpoints, lines, _fpoints.size(), 1, rhoMin, rhoMax, rhoStep,
    thetaMin, thetaMax, thetaStep);

  // Draw the lines
  for (size_t i = 0; i < linesP.size(); i++)
  {
    Vec3d l = linesP[i];
    line(aSrc, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
  }



  return aSrc;
}

Mat DrawLinesRect(Mat &aSrc, std::vector<Rect> &aRects)
{
  Mat _dst;

  _dst = aSrc;

  _dst.setTo(0);

  for (Rect element : aRects)
    rectangle(_dst, element, Scalar(7, 7, 247));

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _dst);
  waitKey(0);

  int _channels = _dst.channels();

  cv::Mat _dis = Mat(_dst.rows, _dst.cols, CV_8UC1);

  for (Rect element : aRects)
    rectangle(_dis, element, Scalar(7, 7, 247));

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _dis);
  waitKey(0);

  Canny(_dis, _dis, 50, 200, 3);

  //Mat _corners, _dilated_corners;
  //preCornerDetect(_dis, _corners, 3);
  //// dilation with 3x3 rectangular structuring element
  //dilate(_corners, _dilated_corners, Mat());

 /* namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _dilated_corners);
  waitKey(0);*/

  // Probabilistic Line Transform
  std::vector<Vec4i> linesP; // will hold the results of the detection
  HoughLinesP(_dis, linesP, 3, 0.01, 100, 100, 100); // runs the actual detection

  // Draw the lines
  for (size_t i = 0; i < linesP.size(); i++)
  {
    Vec4i l = linesP[i];
    line(aSrc, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, LINE_AA);
  }

  return aSrc;
}


int main()
{
  Mat _image = imread("d:\\Dev\\RoboLine\\CameraCalibrationVSSolution\\x64\\Debug\\001.bmp");
  Mat _vector_image = CreateVector(_image);

  medianBlur(_vector_image, _vector_image, 3);
  //GaussianBlur(_vector_image, _vector_image, Size(7,7), 2.0);

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _vector_image);
  waitKey(0);
  imwrite("vector_image.jpg", _vector_image);

  std::vector<Rect> _found_points = FindPointFromCenterRect(_vector_image, Point(1034, 873));

  //std::vector<Point> _found_points = FindPointFromCenterPoints(_vector_image, Point(1034, 873));

  _vector_image = DrawLinesRect(_vector_image, _found_points);

  //_vector_image = DrawLinesPoints(_vector_image, _found_points);

  namedWindow("Vector image", WINDOW_FREERATIO);
  imshow("Vector image", _vector_image);
  waitKey(0);

  imwrite("found_points.jpg", _vector_image);

  return 0;
}
