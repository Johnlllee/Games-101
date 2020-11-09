#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);
    void SSAARender(const Scene& scene);
    int get_ssaa_index(int x, int y, int width);

private:
};