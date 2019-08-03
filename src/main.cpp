#include <opencv4/opencv2/opencv.hpp>

#include <iostream>

int main()
{
    cv::Mat image = cv::imread("/home/fedor/picture.jpg");

    if(image.empty())
    {
        std::cout << "Image is empty!" << std::endl;
        return -1;
    }

    std::cout << "Hello, Worlddd!" << std::endl;

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
