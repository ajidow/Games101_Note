//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
	for (uint32_t k = 0; k < objects.size(); ++k)
	{
		if (objects[k]->hasEmit())
		{
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k)
	{
		if (objects[k]->hasEmit())
		{
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum)
			{
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray &ray,
	const std::vector<Object *> &objects,
	float &tNear, uint32_t &index, Object **hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k)
	{
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
		{
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

	Vector3f L0(0.0f, 0.0f, 0.0f);
	Vector3f wo = ray.direction;

	Intersection inter = intersect(ray);

	if (!inter.happened)
		return Vector3f(0.0f, 0.0f, 0.0f);

	if (inter.m->hasEmission()) // Self Emission
		return inter.m->getEmission();

	float pdf = 0.0f;
	Intersection light_pos;
	sampleLight(light_pos, pdf);

	Vector3f p = inter.coords;
	Vector3f N = inter.normal.normalized();

	Vector3f x = light_pos.coords;
	Vector3f ws = (p - x).normalized();
	Vector3f NN = light_pos.normal.normalized();

	float dis = (p - x).norm();
	float dis2 = dotProduct((p-x),(p-x));

	Ray lignt_to_obj(x, ws);
	Intersection lilgnt_to_scene = intersect(lignt_to_obj);
	if (lilgnt_to_scene.distance - dis + EPSILON > 0 && lilgnt_to_scene.happened)
	{
		Vector3f emit = light_pos.emit;
		Vector3f eval = inter.m->eval(wo, -ws, N);

		L0 += emit * eval * dotProduct(-ws, N) * dotProduct(ws, NN) / dis2 / pdf;
	}

	float P_RR = get_random_float();
	if (P_RR < RussianRoulette)
	{
		Vector3f wi = inter.m->sample(wo, N).normalized();
		Ray r(p, wi);
		Intersection SecondRayInter = intersect(ray);
		if (SecondRayInter.happened && !SecondRayInter.m->hasEmission())
		{
			Vector3f q = SecondRayInter.coords;
			Vector3f eval = inter.m->eval(wo, wi, N);
			float pdf1 = inter.m->pdf(wo, wi, N);

			L0 += castRay(r, depth + 1) * eval * dotProduct(wi, N) / pdf1 / RussianRoulette;
		}
	}
	return L0;
}