#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& ray, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

	double a = 1;
	Vector3D origin_offset = ray.o - this->o;
	double b = 2 * dot(origin_offset, ray.d);
	double c = (origin_offset).norm2() - r2;
	double delta = b*b - 4*a*c;
	if(delta < 0)
		return false;
	else
	{
		double delta_sqrt = sqrt(delta);
		t1 = 0.5 * (-b - delta_sqrt);
		t2 = t1 + delta_sqrt;
		return true;
	}

}

bool Sphere::intersect(const Ray& ray) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	double t1{}, t2{};
	return test(ray, t1, t2) && ray.minmax_t[0] < t2 && ray.minmax_t[1] > t1 &&
	!(ray.minmax_t[0] > t1 && ray.minmax_t[1] < t2);

}

bool Sphere::intersect(const Ray& ray, Intersection *i) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	double t1{}, t2{};
	if(test(ray, t1, t2) && ray.minmax_t[0] < t2 && ray.minmax_t[1] > t1 &&
		 !(ray.minmax_t[0] > t1 && ray.minmax_t[1] < t2))
	{
		i->t = t1 >= ray.minmax_t[0] ? t1 : t2;
		i->n = normal(ray.at_time(i->t));
		i->primitive = this;
		i->bsdf = get_bsdf();
		
		if(ray.minmax_t[0] > t1 && ray.minmax_t[0] < t2)
		{
			i->is_back_hit = true;
//			i->n *= -1;
		}
		else
			i->is_back_hit = false;
		
		return true;
	}
	else return false;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462
