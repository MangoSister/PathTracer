#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

	double a = 1;
	Vector3D origin_offset = r.o - this->o;
	double b = 2 * dot(origin_offset, r.d);
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

bool Sphere::intersect(const Ray& r) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	double t1{}, t2{};
	return test(r, t1, t2) && r.min_t < t2 && r.max_t > t1 &&
	!(r.min_t > t1 && r.max_t < t2);

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	double large_t;
	if(test(r, i->t, large_t))
	{
		i->n = normal(r.at_time(i->t));
		i->primitive = this;
		i->bsdf = get_bsdf();
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
