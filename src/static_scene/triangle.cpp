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
	const Vector3D& _d = - r.d;
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
	double inv_det = 1 /
	(p1_p0.x * ( _d.z * p2_p0.y - p2_p0.z * _d.y ) -
	 p1_p0.y * ( _d.z * p2_p0.x - p2_p0.z * _d.x ) +
	 p1_p0.z * ( _d.y * p2_p0.x - p2_p0.y * _d.x ));
	if(std::isinf(inv_det))
		return false;
	//early exit #1
	Vector3D inv_A_3 =
	{ p2_p0.z * p1_p0.y - p1_p0.z * p2_p0.y,
		p1_p0.z * p2_p0.x - p1_p0.x * p2_p0.z,
		p2_p0.y * p1_p0.x - p1_p0.y * p2_p0.x };
	inv_A_3 *= inv_det;
	double t = dot(inv_A_3, o_p0);
	if(t < r.min_t || t > r.max_t)
		return false;
	
	//early exit #2
	Vector3D inv_A_1 =
	{ _d.z * p2_p0.y - p2_p0.z * _d.y,
		p2_p0.z * _d.x - _d.z * p2_p0.x,
		_d.y * p2_p0.x - p2_p0.y * _d.x };
	inv_A_1 *= inv_det;
	double u = dot(inv_A_1, o_p0);
	if(u > 1 || u < 0)
		return false;
	
	//(exit #3)
	Vector3D inv_A_2 =
	{ p1_p0.z * _d.y - _d.z * p1_p0.y,
		_d.z * p1_p0.x - p1_p0.z * _d.x,
		p1_p0.y * _d.x - _d.y * p1_p0.x };
	inv_A_2 *= inv_det;
	double v = dot(inv_A_2, o_p0);
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
	const Vector3D& _d = - r.d;
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
	double inv_det = 1 /
	(p1_p0.x * ( _d.z * p2_p0.y - p2_p0.z * _d.y ) -
	 p1_p0.y * ( _d.z * p2_p0.x - p2_p0.z * _d.x ) +
	 p1_p0.z * ( _d.y * p2_p0.x - p2_p0.y * _d.x ));
	if(std::isinf(inv_det))
		return false;
	//early exit #1
	Vector3D inv_A_3 =
	{ p2_p0.z * p1_p0.y - p1_p0.z * p2_p0.y,
		p1_p0.z * p2_p0.x - p1_p0.x * p2_p0.z,
		p2_p0.y * p1_p0.x - p1_p0.y * p2_p0.x };
	inv_A_3 *= inv_det;
	double t = dot(inv_A_3, o_p0);
	if(t < r.min_t || t > r.max_t)
		return false;
	
	//early exit #2
	Vector3D inv_A_1 =
	{ _d.z * p2_p0.y - p2_p0.z * _d.y,
		p2_p0.z * _d.x - _d.z * p2_p0.x,
		_d.y * p2_p0.x - p2_p0.y * _d.x };
	inv_A_1 *= inv_det;
	double u = dot(inv_A_1, o_p0);
	if(u > 1 || u < 0)
		return false;
	
	//(exit #3)
	Vector3D inv_A_2 =
	{ p1_p0.z * _d.y - _d.z * p1_p0.y,
		_d.z * p1_p0.x - p1_p0.z * _d.x,
		p1_p0.y * _d.x - _d.y * p1_p0.x };
	inv_A_2 *= inv_det;
	double v = dot(inv_A_2, o_p0);
	if(v < 0 || u + v > 1)
		return false;
	
	isect->t = t;
	
	isect->n =
	mesh->normals[v1] * (1 - u - v) +
	mesh->normals[v2] * u +
	mesh->normals[v3] * v;
	isect->n.normalize();
	//back face issue
	
	if(dot(-r.d * t, face_normal) < 0)
	{
		isect->n *= -1;
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
