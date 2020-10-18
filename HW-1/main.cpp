#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float radian = rotation_angle / 180.0 * MY_PI;
    model(0, 0) = cos(radian);
    model(0, 1) = -sin(radian);
    model(1, 0) = sin(radian);
    model(1, 1) = cos(radian);
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // eye_fov : field of view, mean the height of pic
    // aspect_ratio: the ratio of width / height

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f perspective = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthographic = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float n = -zNear;
    float f = -zFar;

    float top = std::abs(n) * std::tan(eye_fov / 180.0 * MY_PI);
    float bottom = -top;
    float right = aspect_ratio * top;
    float left = -right;


    perspective(0, 0) = n;
    perspective(1, 1) = n;
    perspective(2, 2) = n + f;
    perspective(2, 3) = - n * f;
    perspective(3, 2) = 1;
    perspective(3, 3) = 0;

    orthographic(0, 0) = 2 / (right - left);
    orthographic(1, 1) = 2 / (top - bottom);

    projection = orthographic * perspective;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {

    Eigen::Matrix4f rotation;

    // TODO: Implement the Rodrigues Rotation Formula

    float radian = angle / 180.0f * MY_PI;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N;
    Eigen::MatrixXf rm3f;
    rm3f.resize(3, 3);
    float nx = axis(0);
    float ny = axis(1);
    float nz = axis(2);
    N << 0, -nz, ny,
         nz, 0, -nx,
         -ny, nx, 0;
    rm3f = cos(radian) * I + (1 - cos(radian)) * axis * axis.transpose() + sin(radian) * N;
    rm3f.conservativeResize(4, 4);
    rm3f.col(3).setZero();
    rm3f.row(3).setZero();
    rm3f(3, 3) = 1;

    rotation = rm3f;

    return rotation;

}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    Eigen::Vector3f axis = {0, 0, 1};

    if (argc >= 3) {
        // command_line = true;
        // angle = std::stof(argv[2]); // -r by default
        // if (argc == 4) {
        //     filename = std::string(argv[3]);
        // }
        // else
        //     return 0;
        if(std::string(argv[1]) == "-r") {
            command_line = true;
            angle = std::stof(argv[2]); // -r by default
            if (argc == 4) {
                filename = std::string(argv[3]);
            }
        } else if (std::string(argv[1]) == "-axis") {
            std::string axis_name = std::string(argv[2]);
            if (axis_name == "x") {
                axis = {1, 0, 0};
            } else if(axis_name == "y") {
                axis = {0, 1, 0};
            } else if(axis_name == "z") {
                axis = {0, 0, 1};
            } else {
                float x, y, z;
                sscanf(axis_name.c_str(), "%f,%f,%f", &x, &y, &z);
                axis = {x, y, z};
            }
        } else {
            return 0;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        angle = ((int)angle + 4) % 360;
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'w') {
            eye_pos(2) -= 0.5;
        }
        else if (key == 's') {
            eye_pos(2) += 0.5;
        }
    }

    return 0;
}
