// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float cz[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        Vector3f op1 = *(_v+i), op2 = *(_v + (i + 1) % 3);
        Vector3f p1, p2, p3;
        p1 << op1(0), op1(1), 1;
        p2 << op2(0), op2(1), 1;
        p3 << x, y, 1;
        Vector3f v1, v2, cv;
        v1 = p2 - p1;
        v2 = p3 - p1;
        cv = v1.cross(v2);
        cz[i] = cv(2);
    }
    if(cz[0] * cz[1] >= 0 && cz[1] * cz[2] >= 0 && cz[2] * cz[3] >= 0) {
        return true;
    } else {
        return false;
    }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type, bool ssaa)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        if(ssaa) {
            rasterize_ssaa_triangle(t);
        } else {
            rasterize_triangle(t);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int top = (int)std::max(std::max(v[0].y(), v[1].y()), v[2].y()) + 1;
    int bottom = (int)std::min(std::min(v[0].y(), v[1].y()), v[2].y()) - 1;
    int left = (int)std::min(std::min(v[0].x(), v[1].x()), v[2].x()) - 1;
    int right = (int)std::max(std::max(v[0].x(), v[1].x()), v[2].x()) + 1;

    top = std::min(top, height);
    bottom = std::max(bottom, 0);
    right = std::min(right, width);
    left = std::max(left, 0);

    for(int x = left; x <= right; x++) {
        for(int y = bottom; y <= top; y++) {
            if(insideTriangle(x, y, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                float depth = -z_interpolated;
                int index = get_index(x, y);
                if(depth_buf[index] > depth) {
                    depth_buf[index] = depth;
                    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                    set_pixel(point, t.getColor());
                }
            }
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

//Screen space rasterization
void rst::rasterizer::rasterize_ssaa_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int top = (int)std::max(std::max(v[0].y(), v[1].y()), v[2].y()) + 1;
    int bottom = (int)std::min(std::min(v[0].y(), v[1].y()), v[2].y()) - 1;
    int left = (int)std::min(std::min(v[0].x(), v[1].x()), v[2].x()) - 1;
    int right = (int)std::max(std::max(v[0].x(), v[1].x()), v[2].x()) + 1;

    top = std::min(top, height);
    bottom = std::max(bottom, 0);
    right = std::min(right, width);
    left = std::max(left, 0);

    for(int x = 2 * left; x <= 2 * right; x++) {
        for(int y = 2 * bottom; y <= 2 * top; y++) {
            float real_x = x / 2.0f;
            float real_y = y / 2.0f;
            if(insideTriangle(real_x, real_y, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(real_x, real_y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                float depth = -z_interpolated;
                // int index = get_index(x, y);
                int index = get_ssaa_index(x, y);
                if(ssaa_depth_buf[index] > depth) {
                    ssaa_depth_buf[index] = depth;
                    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                    // set_pixel(point, t.getColor());
                    set_ssaa_pixel(point, t.getColor());
                }
            }
        }
    }

    for(int x = left; x <= right; x++) {
        for(int y = bottom; y <= top; y++) {
            int ssaa_bottom = y * 2;
            int ssaa_top = y * 2 + 1;
            int ssaa_left = x * 2;
            int ssaa_right = x * 2 + 1;
            Vector3f c0, c1, c2, c3;
            c0 = ssaa_frame_buf[get_ssaa_index(ssaa_left, ssaa_bottom)];
            c1 = ssaa_frame_buf[get_ssaa_index(ssaa_right, ssaa_bottom)];
            c2 = ssaa_frame_buf[get_ssaa_index(ssaa_left, ssaa_top)];
            c3 = ssaa_frame_buf[get_ssaa_index(ssaa_right, ssaa_top)];
            int r = (int)(std::round((c0(0) + c1(0) + c2(0) + c3(0)) / 4.0f));
            int g = (int)(std::round((c0(1) + c1(1) + c2(1) + c3(1)) / 4.0f));
            int b = (int)(std::round((c0(2) + c1(2) + c2(2) + c3(2)) / 4.0f));
            Vector3f ssaa_color;
            ssaa_color << r, g, b;
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, ssaa_color);
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(ssaa_frame_buf.begin(), ssaa_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(ssaa_depth_buf.begin(), ssaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    ssaa_depth_buf.resize(w * 2 * h * 2);
    ssaa_frame_buf.resize(w * 2 * h * 2);
}

int rst::rasterizer::get_ssaa_index(int x, int y)
{
    return (height * 2 - 1 - y) * width * 2 + x;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_ssaa_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height * 2 - 1 - point.y()) * width * 2 + point.x();
    ssaa_frame_buf[ind] = color;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on