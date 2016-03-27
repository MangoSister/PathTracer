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
	if(!r.sign[0])
	{
		t_x_min = (min.x - r.o.x) * r.inv_d.x;
		t_x_max = (max.x - r.o.x) * r.inv_d.x;
	}
	else
	{
		t_x_min = (max.x - r.o.x) * r.inv_d.x;
		t_x_max = (min.x - r.o.x) * r.inv_d.x;
	}
	
	if(!r.sign[1])
	{
		t_y_min = (min.y - r.o.y) * r.inv_d.y;
		t_y_max = (max.y - r.o.y) * r.inv_d.y;
	}
	else
	{
		t_y_min = (max.y - r.o.y) * r.inv_d.y;
		t_y_max = (min.y - r.o.y) * r.inv_d.y;
	}
	
	if(t_x_min > t_y_max || t_y_min > t_x_max)
		return false;
	if(t_x_min < t_y_min)
		t_x_min = t_y_min;
	if(t_x_max > t_y_max)
		t_x_max = t_y_max;
	
	if(!r.sign[2])
	{
		t_z_min = (min.z - r.o.z) * r.inv_d.z;
		t_z_max = (max.z - r.o.z) * r.inv_d.z;
	}
	else
	{
		t_z_min = (max.z - r.o.z) * r.inv_d.z;
		t_z_max = (min.z - r.o.z) * r.inv_d.z;
	}
	
	if(t_x_min > t_z_max || t_z_min > t_x_max)
		return false;
	if(t_x_min < t_z_min)
		t_x_min = t_z_min;
	if(t_x_max > t_z_max)
		t_x_max = t_z_max;
	
	return (t_x_min > r.min_t && t_x_max < r.max_t);
	
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CMU462
