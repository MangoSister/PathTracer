#ifndef CMU462_BBOX_H
#define CMU462_BBOX_H

#include <utility>
#include <algorithm>

#include "CMU462/CMU462.h"

#include "ray.h"

namespace CMU462 {

/**
 * Axis-aligned bounding box.
 * An AABB is given by two positions in space, the min and the max. An addition
 * component, the extent of the bounding box is stored as it is useful in a lot
 * of the operations on bounding boxes.
 */
struct BBox {

	Vector3D bounds[2];
//	Vector3D min;	    ///< max corner of the bounding box
//  Vector3D max;	    ///< min corner of the bounding box
  Vector3D extent;  ///< extent of the bounding box (min -> max)

  /**
   * Constructor.
   * The default constructor creates a new bounding box which contains no
   * points.
   */
  BBox() {
    bounds[1] = Vector3D(-INF_D, -INF_D, -INF_D);
    bounds[0] = Vector3D( INF_D,  INF_D,  INF_D);
    extent = bounds[1] - bounds[0];
  }

  /**
   * Constructor.
   * Creates a bounding box that includes a single point.
   */
  BBox(const Vector3D& p)
	{
		bounds[0] = bounds[1] = p;
		extent = bounds[1] - bounds[0];
	}

  /**
   * Constructor.
   * Creates a bounding box with given bounds.
   * \param min the min corner
   * \param max the max corner
   */
  BBox(const Vector3D& min, const Vector3D& max)
	{
		bounds[0] = min;
		bounds[1] = max;
		extent = max - min;
	}

  /**
   * Constructor.
   * Creates a bounding box with given bounds (component wise).
   */
  BBox(const double minX, const double minY, const double minZ,
       const double maxX, const double maxY, const double maxZ)
	{
    bounds[0] = Vector3D(minX, minY, minZ);
    bounds[1] = Vector3D(maxX, maxY, maxZ);
		extent = bounds[1] - bounds[0];
  }

  /**
   * Expand the bounding box to include another (union).
   * If the given bounding box is contained within *this*, nothing happens.
   * Otherwise *this* is expanded to the minimum volume that contains the
   * given input.
   * \param bbox the bounding box to be included
   */
  void expand(const BBox& bbox) {
    bounds[0].x = std::min(bounds[0].x, bbox.bounds[0].x);
    bounds[0].y = std::min(bounds[0].y, bbox.bounds[0].y);
    bounds[0].z = std::min(bounds[0].z, bbox.bounds[0].z);
    bounds[1].x = std::max(bounds[1].x, bbox.bounds[1].x);
    bounds[1].y = std::max(bounds[1].y, bbox.bounds[1].y);
    bounds[1].z = std::max(bounds[1].z, bbox.bounds[1].z);
    extent = bounds[1] - bounds[0];
  }

  /**
   * Expand the bounding box to include a new point in space.
   * If the given point is already inside *this*, nothing happens.
   * Otherwise *this* is expanded to a minimum volume that contains the given
   * point.
   * \param p the point to be included
   */
  void expand(const Vector3D& p) {
    bounds[0].x = std::min(bounds[0].x, p.x);
    bounds[0].y = std::min(bounds[0].y, p.y);
    bounds[0].z = std::min(bounds[0].z, p.z);
    bounds[1].x = std::max(bounds[1].x, p.x);
    bounds[1].y = std::max(bounds[1].y, p.y);
    bounds[1].z = std::max(bounds[1].z, p.z);
    extent = bounds[1] - bounds[0];
  }

  Vector3D centroid() const {
    return (bounds[0] + bounds[1]) / 2;
  }

  /**
   * Compute the surface area of the bounding box.
   * \return surface area of the bounding box.
   */
  double surface_area() const {
    if (empty()) return 0.0;
    return 2 * (extent.x * extent.z +
                extent.x * extent.y +
                extent.y * extent.z);
  }

  /**
   * Check if bounding box is empty.
   * Bounding box that has no size is considered empty. Note that since
   * bounding box are used for objects with positive volumes, a bounding
   * box of zero size (empty, or contains a single vertex) are considered
   * empty.
   */
  bool empty() const {
    return bounds[0].x > bounds[1].x || bounds[0].y > bounds[1].y || bounds[0].z > bounds[1].z;
  }

  /**
   * Ray - bbox intersection.
   * Intersects ray with bounding box, does not store shading information.
   * \param r the ray to intersect with
   * \param t0 lower bound of intersection time
   * \param t1 upper bound of intersection time
   */
  bool intersect(const Ray& r, double& t0, double& t1) const;

  /**
   * Draw box wireframe with OpenGL.
   * \param c color of the wireframe
   */
  void draw(Color c) const;
};

std::ostream& operator<<(std::ostream& os, const BBox& b);

} // namespace CMU462

#endif // CMU462_BBOX_H
