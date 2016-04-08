#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	double t_x_min{}, t_x_max{}, t_y_min{}, t_y_max{}, t_z_min{}, t_z_max{};

	t_x_min = (bounds[r.sign[0]].x - r.o.x) * r.inv_d.x;
	t_x_max = (bounds[1 - r.sign[0]].x - r.o.x) * r.inv_d.x;

	t_y_min = (bounds[r.sign[1]].y - r.o.y) * r.inv_d.y;
	t_y_max = (bounds[1 - r.sign[1]].y - r.o.y) * r.inv_d.y;
	
	if(t_x_min > t_y_max || t_y_min > t_x_max)
		return false;
	if(t_x_min < t_y_min)
		t_x_min = t_y_min;
	if(t_x_max > t_y_max)
		t_x_max = t_y_max;
	

	t_z_min = (bounds[r.sign[2]].z - r.o.z) * r.inv_d.z;
	t_z_max = (bounds[1 - r.sign[2]].z - r.o.z) * r.inv_d.z;

	
	if(t_x_min > t_z_max || t_z_min > t_x_max)
		return false;
	if(t_x_min < t_z_min)
		t_x_min = t_z_min;
	if(t_x_max > t_z_max)
		t_x_max = t_z_max;
	
	if(t_x_min < r.minmax_t[1] && t_x_max > r.minmax_t[0])
	{
		t0 = std::max(t_x_min, r.minmax_t[0]);
		t1 = std::min(t_x_max, r.minmax_t[1]);
		return true;
	}
	else return false;
	
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(bounds[1].x, bounds[1].y, bounds[1].z);
  glVertex3d(bounds[1].x, bounds[1].y, bounds[0].z);
  glVertex3d(bounds[0].x, bounds[1].y, bounds[0].z);
  glVertex3d(bounds[0].x, bounds[1].y, bounds[1].z);
  glVertex3d(bounds[1].x, bounds[1].y, bounds[1].z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(bounds[0].x, bounds[0].y, bounds[0].z);
  glVertex3d(bounds[0].x, bounds[0].y, bounds[1].z);
  glVertex3d(bounds[1].x, bounds[0].y, bounds[1].z);
  glVertex3d(bounds[1].x, bounds[0].y, bounds[0].z);
  glVertex3d(bounds[0].x, bounds[0].y, bounds[0].z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(bounds[1].x, bounds[1].y, bounds[1].z);
  glVertex3d(bounds[1].x, bounds[0].y, bounds[1].z);
	glVertex3d(bounds[1].x, bounds[1].y, bounds[0].z);
  glVertex3d(bounds[1].x, bounds[0].y, bounds[0].z);
	glVertex3d(bounds[0].x, bounds[1].y, bounds[0].z);
  glVertex3d(bounds[0].x, bounds[0].y, bounds[0].z);
	glVertex3d(bounds[0].x, bounds[1].y, bounds[1].z);
  glVertex3d(bounds[0].x, bounds[0].y, bounds[1].z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.bounds[0] << ", " << b.bounds[1] << ")";
}

} // namespace CMU462
