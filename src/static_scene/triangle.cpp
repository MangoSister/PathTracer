#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3)
	{
		const Vector3D& vt1 = mesh->positions[v1];
		const Vector3D& vt2 = mesh->positions[v2];
		const Vector3D& vt3 = mesh->positions[v3];
		
		face_normal = cross(vt1 - vt2, vt1 - vt3).unit();
	}

BBox Triangle::get_bbox() const {
  
  // TODO: 
  // compute the bounding box of the triangle
	auto pairX = std::minmax( {mesh->positions[v1].x, mesh->positions[v2].x, mesh->positions[v3].x} );
	auto pairY = std::minmax( {mesh->positions[v1].y, mesh->positions[v2].y, mesh->positions[v3].y} );
	auto pairZ = std::minmax( {mesh->positions[v1].z, mesh->positions[v2].z, mesh->positions[v3].z} );
	
	return BBox({pairX.first, pairY.first, pairZ.first}, {pairX.second, pairY.second, pairZ.second} );
}

bool Triangle::intersect(const Ray& r) const {
  
  // TODO: implement ray-triangle intersection
	// TODO: extract common subexpressions!!!
	const Vector3D& p0 = mesh->positions[v1];
	Vector3D p1_p0 = mesh->positions[v2] - p0;
	Vector3D p2_p0 = mesh->positions[v3] - p0;
	Vector3D o_p0 = r.o - p0;
	//closed form of inv of A: [p1_p0, p2_p0, _d]
	//x = [u, v, t]^T
	//b = o_po
	//Ax = b => x = inv_A * b
	//a11: p1_p0.x	a12: p2_p0.x	a13: _d.x
	//a21: p1_p0.y	a22: p2_p0.y	a23: _d.y
	//a31: p1_p0.z	a32: p2_p0.z	a33: _d.z
	
	//det: a11(a33 * a22 - a32 * a23) -
	//		a21(a33 * a12 - a32 * a13) +
	//		a31(a23 * a12 - a22 * a13)
	
	Vector3D cross_d_e2 = cross(r.d, p2_p0);
	
	double inv_det = 1 / dot(p1_p0, cross_d_e2);
	if(std::isinf(inv_det))
		return false;
	
	Vector3D cross_s_e1 = cross(o_p0, p1_p0);
	
	//early exit #1
	double t = dot(cross_s_e1, p2_p0) * inv_det;
	if(t < r.min_t || t > r.max_t)
		return false;
	
	//early exit #2
	double u = dot(cross_d_e2, o_p0) * inv_det;
	if(u > 1 || u < 0)
		return false;
	
	//(exit #3)
	double v = dot(cross_s_e1, r.d) * inv_det;
	if(v < 0 || u + v > 1)
		return false;
	
	return true;
	
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO: 
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
	
	// TODO: extract common subexpressions!!!
	const Vector3D& p0 = mesh->positions[v1];
	Vector3D p1_p0 = mesh->positions[v2] - p0;
	Vector3D p2_p0 = mesh->positions[v3] - p0;
	Vector3D o_p0 = r.o - p0;
	//closed form of inv of A: [p1_p0, p2_p0, _d]
	//x = [u, v, t]^T
	//b = o_po
	//Ax = b => x = inv_A * b
	//a11: p1_p0.x	a12: p2_p0.x	a13: _d.x
	//a21: p1_p0.y	a22: p2_p0.y	a23: _d.y
	//a31: p1_p0.z	a32: p2_p0.z	a33: _d.z
	
	//det: a11(a33 * a22 - a32 * a23) -
	//		a21(a33 * a12 - a32 * a13) +
	//		a31(a23 * a12 - a22 * a13)
	
	Vector3D cross_d_e2 = cross(r.d, p2_p0);
	
	double inv_det = 1 / dot(p1_p0, cross_d_e2);
	if(std::isinf(inv_det))
		return false;
	
	Vector3D cross_s_e1 = cross(o_p0, p1_p0);

	//early exit #1
	double t = dot(cross_s_e1, p2_p0) * inv_det;
	if(t < r.min_t || t > r.max_t)
		return false;
	
	//early exit #2
	double u = dot(cross_d_e2, o_p0) * inv_det;
	if(u > 1 || u < 0)
		return false;
	
	//(exit #3)
	double v = dot(cross_s_e1, r.d) * inv_det;
	if(v < 0 || u + v > 1)
		return false;
	
	isect->t = t;
	
	isect->n =
	mesh->normals[v1] * (1 - u - v) +
	mesh->normals[v2] * u +
	mesh->normals[v3] * v;
	isect->n.normalize();
	if(dot(isect->n,face_normal) < 0)
		isect->n *= -1;

	//back face issue
	
	if(dot(-r.d * t, face_normal) < 0)
	{
//		isect->n *= -1;
		isect->is_back_hit = true;
	}
	else
		isect->is_back_hit = false;
	
	isect->primitive = this;
	isect->bsdf = mesh->get_bsdf();
	
	return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462
