#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI) * std::max(0.0, wi[2]);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf)
{
	*wi = sampler.get_sample(pdf);
  return albedo * (1.0 / PI) * std::max(0.0, (*wi)[2]);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi)
{
	if(dot(reflect(wi), wo) > (1 - 10e-5))
	{
		return Spectrum{1, 1, 1};
	}
	else
		return Spectrum{0, 0, 0};
//	return dot(reflect(wi), wo) > (1 - 10e-5) ?
//	Spectrum{1, 1, 1} : Spectrum{0, 0, 0};
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf)
{

  // TODO:
  // Implement MirrorBSDF
	reflect(wo, wi);
	*pdf = 1.0f;
	return Spectrum{1,1,1};
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO:
  // Implement RefractionBSDF

  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi)
{
	double ior_modified{};
	Vector3D wo_modified_z{wo};
	if(wo[2] < 0)
	{
		wo_modified_z[2] *= -1;
		ior_modified = 1 / ior;
	}
	else
	{
		ior_modified = ior;
	}
	
	if(dot(reflect(wi), wo) > (1.0 - 10e-5))
	{
		//compute Fr
		double fr{};
		fr = compute_fr(wo_modified_z, ior_modified);
		return reflectance * fr;
	}
  else
	{
		Vector3D refract_wi{};
		double fr{};
		if(refract(wo_modified_z, &refract_wi, ior_modified, fr))
		{
			if(wo[2] < 0)
				refract_wi[2] *= -1;
			
			if(dot(refract_wi, wi) > (1.0 - 10e-5))
				return (1 / (ior_modified * ior_modified)) * (1 - fr) /*/ wo_modified_z[2]*/ * transmittance;
			else return Spectrum{0.0, 0.0, 0.0};
		}
		else return Spectrum{0.0, 0.0, 0.0};
	}
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf)
{

  // TODO:
	// Compute Fresnel coefficient and eitherrr reflect or refract based on it.
	double ior_modified{};
	Vector3D wo_modified_z{wo};
	if(wo[2] < 0)
	{
		wo_modified_z[2] *= -1;
		ior_modified = 1 / ior;
	}
	else
	{
		ior_modified = ior;
	}
	
	double fr = compute_fr(wo_modified_z, ior_modified);
	if( ((double)std::rand() / (double)RAND_MAX) < fr)
	{
		reflect(wo, wi);
		*pdf = fr;
		return reflectance * fr;
	}
	else
	{
		refract(wo_modified_z, wi, ior_modified);
		if(wo[2] < 0)
			(*wi)[2] *= -1;
		*pdf = 1.0 - fr;
		return (1 / (ior_modified * ior_modified)) * (1 - fr) /*/ wo_modified_z[2]*/ * transmittance;
	}
}

inline void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
	// r = d − 2*dot(d, n)*n
	*wi = {-wo.x, -wo.y, wo.z};
}

inline Vector3D BSDF::reflect(const Vector3D& wi)
{
	return Vector3D{-wi.x, -wi.y, wi.z};
}
	
bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
	double cos_wo = std::max(0.0, wo[2]);
	double sq_cos_wt = (cos_wo * cos_wo - 1) * ior * ior + 1;
		//ior : in/out ratio??
	if(sq_cos_wt  < 0)
	{
		return false;
	}
	else
	{
		double sin_wo = sqrt(1 - cos_wo * cos_wo);
		double sin_wt = ior * sin_wo;
		double cos_wt = sqrt(sq_cos_wt);
		double inv_sin_wo_mul_sin_wt = 1 / sin_wo * sin_wt;
		(*wi).x = - wo.x * inv_sin_wo_mul_sin_wt;
		(*wi).y = - wo.y * inv_sin_wo_mul_sin_wt;
		(*wi).z = - cos_wt;
		return true;
	}
}
	
bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior, double& fr)
{
	
	// TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
	double cos_wo = std::max(0.0, wo[2]);
	double sq_cos_wt = (cos_wo * cos_wo - 1) * ior * ior + 1;
	//ior : in/out ratio??
	if(sq_cos_wt  < 0)
	{
		fr = 1;
		return false;
	}
	else
	{
		double sin_wo = sqrt(1 - cos_wo * cos_wo);
		double sin_wt = ior * sin_wo;
		double cos_wt = sqrt(sq_cos_wt);
		double inv_sin_wo_mul_sin_wt = 1 / sin_wo * sin_wt;
		(*wi).x = - wo.x * inv_sin_wo_mul_sin_wt;
		(*wi).y = - wo.y * inv_sin_wo_mul_sin_wt;
		(*wi).z = - cos_wt;
		
		//fr = 0.5 * (r_parallel^2 + r_perpendicular^2)
		double r_par = (cos_wo - ior * cos_wt) / (cos_wo + ior * cos_wt);
		double r_per = (ior * cos_wo - cos_wt) / (ior * cos_wo + cos_wt);
		fr = 0.5 * (r_par * r_par + r_per * r_per);
		return true;
	}
}

double BSDF::compute_fr(const Vector3D& wo, float ior)
{
	double cos_wo = std::max(0.0, wo[2]);
	double sq_cos_wt = (cos_wo * cos_wo - 1) * ior * ior + 1;
	//ior : in/out ratio??
	if(sq_cos_wt  < 0)
	{
		return 1;
	}
	else
	{
		double cos_wt = sqrt(sq_cos_wt);
		//fr = 0.5 * (r_parallel^2 + r_perpendicular^2)
		double r_par = (cos_wo - ior * cos_wt) / (cos_wo + ior * cos_wt);
		double r_per = (ior * cos_wo - cos_wt) / (ior * cos_wo + cos_wt);
		return  0.5 * (r_par * r_par + r_per * r_per);
	}
}
	
// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CMU462
