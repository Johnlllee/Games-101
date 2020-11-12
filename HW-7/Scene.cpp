//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // if (depth > this->maxDepth) {
    //     return Vector3f(0.0,0.0,0.0);
    // }
    // intersection
    Intersection inter = Scene::intersect(ray);

    Vector3f hitColor = Vector3f(0.0, 0.0, 0.0);

    if(inter.happened) {
        Material *m = inter.m;
        if(m->hasEmission()) {
            hitColor = Vector3f(1.0f);
        } else {
            // Object *hitObject = inter.obj;
            Vector3f hitPoint = inter.coords;
            Vector3f N = inter.normal;
            Vector3f wo = -ray.direction;
            float pdf;

            Vector3f f_r;
            Vector3f L_dir(0.0f), L_indir(0.0f);

            // sample all light at hit point
            // pdf_light will calc in func by light area
            // dont need to init before this sampling
            Intersection sample_point_of_light;
            float pdf_light;
            sampleLight(sample_point_of_light, pdf_light);

            // check if there is sth block the hit_point and the light
            Vector3f ws = normalize(sample_point_of_light.coords - hitPoint);
            // Ray hp_to_light(hitPoint, ws);
            Intersection check_inter = Scene::intersect(Ray(hitPoint, ws));
            Vector3f dist = check_inter.coords - sample_point_of_light.coords;
            if(dist.norm() < Scene::delta) {
                // f_r = m->eval(ws, wo, N);
                // Vector3f emit = sample_point_of_light.emit;
                // Vector3f NN = normalize(sample_point_of_light.normal);
                // float cos_theta = dotProduct(ws, N);
                // float cos_theta_x = dotProduct(-ws, NN);
                Vector3f path = sample_point_of_light.coords - hitPoint;
                // float falloff = dotProduct(path, path);
                // L_dir = emit * f_r * cos_theta * cos_theta_x / falloff / pdf_light;

                // simplify ver
                if(pdf_light > 0.00001) {
                    L_dir = sample_point_of_light.emit * m->eval(ws, wo, N)
                            * dotProduct(ws, N) * dotProduct(-ws, normalize(sample_point_of_light.normal))
                            / dotProduct(path, path) / pdf_light;
                }

            }

            float p_RR = get_random_float();

            if (p_RR < RussianRoulette) {
                // sample a ray at hit point
                auto wi = m->sample(wo, N);
                pdf = m->pdf(wi, wo, N);
                // f_r = m->eval(wi, wo, N);
                // Ray wi_ray(hitPoint, wi);
                // float cos_theta = dotProduct(wi, N);
                // if (pdf > Scene::delta) {
                //     // L_indir = Scene::castRay(wi_ray, depth + 1) * f_r * cos_theta / pdf / RussianRoulette;
                // }

                // simplify ver
                if (pdf > Scene::delta) {
                    L_indir = Scene::castRay(Ray(hitPoint, wi), depth) * m->eval(wi, wo, N)
                              * dotProduct(wi, N) / pdf / RussianRoulette;
                }
            }

            // L_indir = L_indir / RussianRoulette;

            hitColor = L_dir + L_indir;
        }
    }

    return hitColor;
}

// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
//     // TODO Implement Path Tracing Algorithm here
//     Vector3f color(0);

//     // check hit with scene
//     Intersection inter_ray = intersect(ray);

//     // hit light source
//     if (inter_ray.emit.norm() > 0)
//     {
//         color = Vector3f(1);
//         // std::clog << "hit light source " << color << std::endl;
//     }
//     // ray hit an object, perform path tracing on hit (shading) point
//     else if (inter_ray.happened)
//     {
//         // member for shading point
//         Vector3f p = inter_ray.coords;  // shading point
//         Vector3f wo = - ray.direction;  // observation dir.
//         Material* m = inter_ray.m;  // material at shading point
//         Vector3f N = normalize(inter_ray.normal); // normal of shading point

//         // sample light
//         Intersection inter_light;
//         float pdf_light;
//         sampleLight(inter_light, pdf_light);
//         Vector3f x = inter_light.coords;  // sampled position
//         Vector3f ws_unorm = x - p;  // shading point -> source
//         Vector3f ws = normalize(ws_unorm);  // shading point -> source
//         Vector3f NN = normalize(inter_light.normal); // normal of sampled position
        
//         // Direct Illumination
//         Vector3f L_dir(0);
//         // check block between shading point & sampled light position
//         Intersection inter_dir = intersect(Ray(p, ws));
//         // no block, add direct illumination
//         // if (inter_dir.obj == inter_light.obj)
//         float dist_dir = (inter_dir.coords - inter_light.coords).norm();
//         if ( dist_dir < 0.01 )
//         {
//             // apply rendering equation (dir. part)
//             L_dir = inter_light.emit * m->eval(ws, wo, N) 
//                     * dotProduct(ws, N) * dotProduct(-ws, NN)
//                     / dotProduct(ws_unorm, ws_unorm) / pdf_light;
//         }

//         // Indirect Illumination
//         Vector3f L_indir(0);
//         float p_RR = get_random_float();
//         if (p_RR < RussianRoulette)
//         {
//             // apply rendering equation (indir. part)
//             Vector3f wi = m->sample(wo, N);  // sample a direction for indierect illumination
//             Vector3f indir_shade_color = castRay(Ray(p, wi), depth);  // cast ray recursively
//             L_indir = indir_shade_color * m->eval(wi, wo, N) 
//                       * dotProduct(wi, N) / m->pdf(wi, wo, N) / RussianRoulette;
//         }
//         L_indir = L_indir * 1.0 / RussianRoulette;

//         color = L_dir + L_indir;
//     }

//     return color;
// }