#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#define Points 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < Points) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_control_points;
    for(int i = 0; i < control_points.size() - 1; i++) {
        cv::Point2f prev_cp = control_points[i];
        cv::Point2f next_cp = control_points[i + 1];
        cv::Point2f new_cp = prev_cp * t + next_cp * (1 - t);
        new_control_points.push_back(new_cp);
    }
    return recursive_bezier(new_control_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float step = 0.0001f;
    std::vector<cv::Point2f> points;
    for(float t = 0; t <= 1; t += step) {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        points.push_back(point);
    }
    // convolution
    // 0.5 1.0 0.5
    // 1.0 4.0 1.0
    // 0.5 1.0 0.5
    for(auto &point: points) {
        int x = point.x;
        int y = point.y;
        int left = x - 1, right = x + 1;
        int top = y - 1, bottom = y + 1;
        for(int i = left; i <= right; i++) {
            for(int j = top; j <= bottom; j++) {
                if((int)window.at<cv::Vec3b>(j, i)[1] == 0) {
                    int left_box = i - 1, right_box = i + 1;
                    int top_box = j - 1, bottom_box = j + 1;
                    float green = 0;
                    for(int i_box = left_box; i_box <= right_box; i_box++) {
                        for(int j_box = top_box; j_box <= bottom_box; j_box++) {
                            if(i_box == i && j_box == j) {
                                green += (int)window.at<cv::Vec3b>(j_box, i_box)[1] * 4;
                            } else {
                               green += (int)window.at<cv::Vec3b>(j_box, i_box)[1] / (float)(std::abs(i_box - i) + std::abs(j_box - j));
                            }
                        }
                    }
                    window.at<cv::Vec3b>(j, i)[1] = int(green/10);
                }
            }
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == Points)
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
