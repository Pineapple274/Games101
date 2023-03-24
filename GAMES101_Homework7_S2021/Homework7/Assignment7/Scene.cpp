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
    auto L_dir = Vector3f(0, 0, 0), L_indir = Vector3f(0, 0, 0);
    auto obj_inter = intersect(ray);

    // No intersection
    if(!obj_inter.happened) return L_dir;

    // Is light oringin
    if(obj_inter.m->hasEmission()) return obj_inter.m->getEmission();

    // Is objects
    auto p = obj_inter.coords;
    auto n = obj_inter.normal.normalized();
    auto wo = ray.direction;
    
    // Sample light
    auto light_inter = Intersection();
    auto pdf_light = 0.0f;
    sampleLight(light_inter, pdf_light);
    
    // Adjust ray is blocked or not
    auto x  = light_inter.coords;
    auto emit = light_inter.emit;
    auto nn = light_inter.normal.normalized();
    auto ws = (x - p).normalized();
    auto shadow_ray = Ray(p, ws);
    auto shadow_inter = intersect(shadow_ray);
    auto light_distance = (x - p).norm();

    // Direct light
    if(shadow_inter.distance - light_distance > -EPSILON) {
        L_dir = emit * obj_inter.m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, nn)
                / pow(light_inter.distance, 2) / pdf_light;
    }

    // Indirect light
    if(RussianRoulette > get_random_float()){
        auto wi = obj_inter.m->sample(wo, n).normalized();
        auto new_ray = Ray(p, wi);
        auto hit = intersect(new_ray);
        if(hit.happened){
            L_indir = castRay(new_ray, depth + 1) * obj_inter.m->eval(wo, wi, n) * dotProduct(wi, n) 
            / obj_inter.m->pdf(wo, wi, n) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}