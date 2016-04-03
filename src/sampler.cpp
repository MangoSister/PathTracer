#include "sampler.h"

namespace CMU462 {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {

  // TODO:
  // Implement uniform 2D grid sampler
	
  return Vector2D((double)(std::rand()) / RAND_MAX,
									(double)(std::rand()) / RAND_MAX);

}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {

  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);

}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const
{
	// You may implement this, but don't have to.
	double Xi1 = (double)(std::rand()) / RAND_MAX;
	double Xi2 = (double)(std::rand()) / RAND_MAX;

	double theta = 0.5 * acos(1 - 2 * Xi1);
	double phi = 2 * PI * Xi2;

	double cos_theta = cos(theta);
	double sin_theta = sin(theta);

	
	*pdf = cos_theta * INV_PI;
	
	return Vector3D{sin_theta * cos(phi), sin_theta * sin(phi), cos_theta};
	
//	*pdf = 1 / (2 * PI);
//	double Xi1 = (double)(std::rand()) / RAND_MAX;
//	double Xi2 = (double)(std::rand()) / RAND_MAX;
//	
//	double theta = acos(Xi1);
//	double phi = 2.0 * PI * Xi2;
//	
//	double xs = sinf(theta) * cosf(phi);
//	double ys = sinf(theta) * sinf(phi);
//	double zs = cosf(theta);
//	
//	return Vector3D(xs, ys, zs);

}


} // namespace CMU462
