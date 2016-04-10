#include "pathtracer.h"
#include "bsdf.h"
#include "ray.h"

#include <stack>
#include <random>
#include <algorithm>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/lodepng.h"

#include "GL/glew.h"

#include "static_scene/sphere.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"

using namespace CMU462::StaticScene;

using std::min;
using std::max;

namespace CMU462 {

//#define ENABLE_RAY_LOGGING 1

PathTracer::PathTracer(size_t ns_aa,
                       size_t max_ray_depth, size_t ns_area_light,
                       size_t ns_diff, size_t ns_glsy, size_t ns_refr,
                       size_t num_threads, bool halton, HDRImageBuffer* envmap) {
  state = INIT,
  this->ns_aa = ns_aa;
  this->max_ray_depth = max_ray_depth;
  this->ns_area_light = ns_area_light;
  this->ns_diff = ns_diff;
  this->ns_glsy = ns_diff;
  this->ns_refr = ns_refr;
	this->halton_grid_sampling = halton;
	
  if (envmap) {
		this->envLight = new EnvironmentLight(envmap);
  } else {
    this->envLight = NULL;
  }

  bvh = NULL;
  scene = NULL;
  camera = NULL;

//  gridSampler = new UniformGridSampler2D();	
//	gridSampler = new HaltonSampler2D();
//  hemisphereSampler = new UniformHemisphereSampler3D();

  show_rays = true;

  imageTileSize = 32;
  numWorkerThreads = num_threads;
  workerThreads.resize(numWorkerThreads);

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;

}

PathTracer::~PathTracer() {

  delete bvh;
//  delete gridSampler;
//  delete hemisphereSampler;

}

void PathTracer::set_scene(Scene *scene) {

  if (state != INIT) {
    return;
  }

  if (this->scene != nullptr) {
    delete scene;
    delete bvh;
    selectionHistory.pop();
  }

  if (this->envLight != nullptr) {
    scene->lights.push_back(this->envLight);
  }

  this->scene = scene;
  build_accel();

  if (has_valid_configuration()) {
    state = READY;
  }
}

void PathTracer::set_camera(Camera *camera) {

  if (state != INIT) {
    return;
  }

  this->camera = camera;
  if (has_valid_configuration()) {
    state = READY;
  }

}

void PathTracer::set_frame_size(size_t width, size_t height) {
  if (state != INIT && state != READY) {
    stop();
  }
  sampleBuffer.resize(width, height);
  frameBuffer.resize(width, height);
  if (has_valid_configuration()) {
    state = READY;
  }
}

bool PathTracer::has_valid_configuration() {
  return scene && camera /*&& gridSampler && hemisphereSampler*/ &&
         (!sampleBuffer.is_empty());
}

void PathTracer::update_screen() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
    case DONE:
        //sampleBuffer.tonemap(frameBuffer, tm_gamma, tm_level, tm_key, tm_wht);
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
  }
}

void PathTracer::stop() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
            workerThreads[i]->join();
            delete workerThreads[i];
        }
      state = READY;
      break;
  }
}

void PathTracer::clear() {
  if (state != READY) return;
  delete bvh;
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  selectionHistory.pop();
  sampleBuffer.resize(0, 0);
  frameBuffer.resize(0, 0);
  state = INIT;
}

void PathTracer::start_visualizing() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE;
}

void PathTracer::start_raytracing() {
  if (state != READY) return;

  rayLog.clear();
  workQueue.clear();

  state = RENDERING;
  continueRaytracing = true;
  workerDoneCount = 0;

  sampleBuffer.clear();
  frameBuffer.clear();
  num_tiles_w = sampleBuffer.w / imageTileSize + 1;
  num_tiles_h = sampleBuffer.h / imageTileSize + 1;
  tile_samples.resize(num_tiles_w * num_tiles_h);
  memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

  // populate the tile work queue
  for (size_t y = 0; y < sampleBuffer.h; y += imageTileSize) {
      for (size_t x = 0; x < sampleBuffer.w; x += imageTileSize) {
          workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize));
      }
  }

  // launch threads
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&PathTracer::worker_thread, this);
  }
}


void PathTracer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH... "); fflush(stdout);
  timer.start();
  bvh = new BVHAccel(primitives);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // initial visualization //
  selectionHistory.push(bvh->get_root());
}

void PathTracer::log_ray_miss(const Ray& r) {
    rayLog.push_back(LoggedRay(r, -1.0));
}

void PathTracer::log_ray_hit(const Ray& r, double hit_t) {
    rayLog.push_back(LoggedRay(r, hit_t));
}

void PathTracer::visualize_accel() const {

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color cnode = Color(.5, .5, .5, .25);
  Color cnode_hl = Color(1., .25, .0, .6);
  Color cnode_hl_child = Color(1., 1., 1., .6);

  Color cprim_hl_left = Color(.6, .6, 1., 1);
  Color cprim_hl_right = Color(.8, .8, 1., 1);
  Color cprim_hl_edges = Color(0., 0., 0., 0.5);

  BVHNode *selected = selectionHistory.top();

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  if (selected->isLeaf()) {
    for (size_t i = 0; i < selected->range; ++i) {
       bvh->primitives[selected->start + i]->draw(cprim_hl_left);
    }
  } else {
      if (selected->l) {
          BVHNode* child = selected->l;
          for (size_t i = 0; i < child->range; ++i) {
              bvh->primitives[child->start + i]->draw(cprim_hl_left);
          }
      }
      if (selected->r) {
          BVHNode* child = selected->r;
          for (size_t i = 0; i < child->range; ++i) {
              bvh->primitives[child->start + i]->draw(cprim_hl_right);
          }
      }
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  for (size_t i = 0; i < selected->range; ++i) {
      bvh->primitives[selected->start + i]->drawOutline(cprim_hl_edges);
  }

  // keep depth buffer check enabled so that mesh occluded bboxes, but
  // disable depth write so that bboxes don't occlude each other.
  glDepthMask(GL_FALSE);

  // create traversal stack
  stack<BVHNode *> tstack;

  // push initial traversal data
  tstack.push(bvh->get_root());

  // draw all BVH bboxes with non-highlighted color
  while (!tstack.empty()) {

    BVHNode *current = tstack.top();
    tstack.pop();

    current->bb.draw(cnode);
    if (current->l) tstack.push(current->l);
    if (current->r) tstack.push(current->r);
  }

  // draw selected node bbox and primitives
  if (selected->l) selected->l->bb.draw(cnode_hl_child);
  if (selected->r) selected->r->bb.draw(cnode_hl_child);

  glLineWidth(3.f);
  selected->bb.draw(cnode_hl);

  // now perform visualization of the rays
  if (show_rays) {
      glLineWidth(1.f);
      glBegin(GL_LINES);

      for (size_t i=0; i<rayLog.size(); i+=500) {

          const static double VERY_LONG = 10e4;
          double ray_t = VERY_LONG;

          // color rays that are hits yellow
          // and rays this miss all geometry red
          if (rayLog[i].hit_t >= 0.0) {
              ray_t = rayLog[i].hit_t;
              glColor4f(1.f, 1.f, 0.f, 0.1f);
          } else {
              glColor4f(1.f, 0.f, 0.f, 0.1f);
          }

          Vector3D end = rayLog[i].o + ray_t * rayLog[i].d;

          glVertex3f(rayLog[i].o[0], rayLog[i].o[1], rayLog[i].o[2]);
          glVertex3f(end[0], end[1], end[2]);
      }
      glEnd();
  }

  glDepthMask(GL_TRUE);
  glPopAttrib();
}

void PathTracer::key_press(int key) {

  BVHNode *current = selectionHistory.top();
  switch (key) {
  case ']':
      ns_aa *=2;
      printf("Samples per pixel changed to %lu\n", ns_aa);
      //tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
      break;
  case '[':
      //tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
      ns_aa /=2;
      if (ns_aa < 1) ns_aa = 1;
      printf("Samples per pixel changed to %lu\n", ns_aa);
      break;
  case KEYBOARD_UP:
      if (current != bvh->get_root()) {
          selectionHistory.pop();
      }
      break;
  case KEYBOARD_LEFT:
      if (current->l) {
          selectionHistory.push(current->l);
      }
      break;
  case KEYBOARD_RIGHT:
      if (current->l) {
          selectionHistory.push(current->r);
      }
      break;
  case 'a':
  case 'A':
      show_rays = !show_rays;
  default:
      return;
  }
}

Spectrum PathTracer::trace_ray(const Ray &r)
{
	Ray curr_ray{r};

	Spectrum transmit_factor{1,1,1};
	Spectrum L_out{};

	while(curr_ray.depth > 0)
	{
		Spectrum curr_L_out{};
		Intersection isect;
		if (!bvh->intersect(curr_ray, &isect))
		{
			// log ray miss
#ifdef ENABLE_RAY_LOGGING
			log_ray_miss(r);
#endif
			
			// TODO:
			// If you have an environment map, return the Spectrum this ray
			// samples from the environment map. If you don't return black.
			if(envLight)
				L_out += envLight->sample_dir(curr_ray) * transmit_factor;
			break;
		}
		else
		{
			// log ray hit
#ifdef ENABLE_RAY_LOGGING
			log_ray_hit(r, isect.t);
#endif
			
			if(isect.is_back_hit && !isect.bsdf->consider_back_face())
			{
				break;
			}
			else
			{
				Spectrum curr_L_out = isect.bsdf->get_emission(); // Le
				//FIX: saturate the Le component
				curr_L_out.saturate();
				// TODO :
				// Instead of initializing this value to a constant color, use the direct,
				// indirect lighting components calculated in the code below. The starter
				// code overwrites L_out by (.5,.5,.5) so that you can test your geometry
				// queries before you implement path tracing.
				//  L_out = Spectrum(0.5f, 0.5f, 0.5f);
				
				Vector3D hit_p = curr_ray.o + curr_ray.d * isect.t;
				Vector3D hit_n = isect.n;
				
				// make a coordinate system for a hit point
				// with N aligned with the Z direction.
				Matrix3x3 o2w;
				make_coord_space(o2w, isect.n);
				Matrix3x3 w2o = o2w.T();
				
				// w_out points towards the source of the ray (e.g.,
				// toward the camera if this is a primary ray)
				Vector3D w_out = w2o * (curr_ray.o - hit_p);
				w_out.normalize();
				
				// TODO:
				// Extend the below code to compute the direct lighting for all the lights
				// in the scene, instead of just the dummy light we provided in part 1.
				for(auto light : scene->lights)
				{
					Vector3D dir_to_light{};
					float dist_to_light{};
					float pdf{};
					
					// no need to take multiple samples from a directional source
					int num_light_samples = light->is_delta_light() ? 1 : ns_area_light;
					
					// integrate light over the hemisphere about the normal
					Spectrum L_out_perlight{};
					double scale = 1.0 / num_light_samples;
					for (int i=0; i<num_light_samples; i++)
					{
						// returns a vector 'dir_to_light' that is a direction from
						// point hit_p to the point on the light source.  It also returns
						// the distance from point x to this point on the light source.
						// (pdf is the probability of randomly selecting the random
						// sample point on the light source -- more on this in part 2)
						Spectrum light_L = light->sample_L(hit_p, &dir_to_light, &dist_to_light, &pdf);

						// convert direction into coordinate space of the surface, where
						// the surface normal is [0 0 1]
						Vector3D w_in = w2o * dir_to_light;
						
						// evaluate surface bsdf
						Spectrum f = isect.bsdf->f(w_out, w_in);
						
						// TODO:
						// Construct a shadow ray and compute whether the intersected surface is
						// in shadow and accumulate reflected radiance
						Ray shadow_ray{hit_p /*+ 0 * hit_n*/, dir_to_light};
						shadow_ray.minmax_t[0] = 10e-5;
						shadow_ray.minmax_t[1] = dist_to_light - 10e-5;
						if(!bvh->intersect(shadow_ray))
						{
							L_out_perlight += (f * light_L) * (1 / pdf);
						}
					}
					L_out_perlight *= scale;
					curr_L_out += L_out_perlight;
				}
			
				L_out += (transmit_factor * curr_L_out);

				// TODO:
				// Compute an indirect lighting estimate using pathtracing with Monte Carlo.
				// Note that Ray objects have a depth field now; you should use this to avoid
				// traveling down one path forever.
				
				Vector3D indirect_w_in{};
				float dir_pdf{};
				Spectrum f = isect.bsdf->sample_f(w_out, &indirect_w_in, &dir_pdf);

//				float termination = f.illum() > 0.25f ? 0.0f : 0.5f;
				float termination = 1 - f.illum();
				termination = clamp(termination, 0, 1);

				//Russian Roulette
				if(((double)(std::rand()) / RAND_MAX) < termination)
					break;
				
				curr_ray = Ray{hit_p, o2w * indirect_w_in, curr_ray.depth};
				curr_ray.minmax_t[0] = 10e-5;
				curr_ray.depth--;
				transmit_factor *= f * (1 / (dir_pdf * (1 - termination)));
			}
		
		}
		
	}
	
	return L_out;
}

Spectrum PathTracer::raytrace_pixel(size_t x, size_t y, const Sampler2D* grid_sampler) {

  // TODO:
  // Sample the pixel with coordinate (x,y) and return the result spectrum.
  // The sample rate is given by the number of camera rays per pixel.
	
	Vector2D p = Vector2D
	{static_cast<double>(x), static_cast<double>(y)};
	double inv_w = 1 / static_cast<double>(sampleBuffer.w);
	double inv_h = 1 / static_cast<double>(sampleBuffer.h);
	
  size_t num_samples = ns_aa;
	Spectrum output{};
	for(size_t i = 0; i < num_samples; ++i)
	{
		Vector2D sample = p + grid_sampler->get_sample();
		sample.x *= inv_w;
		sample.y *= inv_h;
		
		Ray cam_ray = camera->generate_ray(sample.x, sample.y);
		cam_ray.depth = max_ray_depth;
		output += trace_ray(cam_ray);
	}
	
	return output * (1 / (double)num_samples);
}

void PathTracer::raytrace_tile(int tile_x, int tile_y,
                               int tile_w, int tile_h, const Sampler2D* grid_sampler) {

  size_t w = sampleBuffer.w;
  size_t h = sampleBuffer.h;

  size_t tile_start_x = tile_x;
  size_t tile_start_y = tile_y;

  size_t tile_end_x = std::min(tile_start_x + tile_w, w);
  size_t tile_end_y = std::min(tile_start_y + tile_h, h);

  size_t tile_idx_x = tile_x / imageTileSize;
  size_t tile_idx_y = tile_y / imageTileSize;
  size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    if (!continueRaytracing) return;
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
        Spectrum s = raytrace_pixel(x, y, grid_sampler);
        sampleBuffer.update_pixel(s, x, y);
    }
  }

  tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;
  sampleBuffer.toColor(frameBuffer, tile_start_x, tile_start_y, tile_end_x, tile_end_y);
}

void PathTracer::worker_thread() {

	Sampler2D* grid_sampler;
	if(halton_grid_sampling)
		grid_sampler = new HaltonSampler2D();
	else grid_sampler = new UniformGridSampler2D();

  Timer timer;
  timer.start();
	
  WorkItem work;
  while (continueRaytracing && workQueue.try_get_work(&work)) {
    raytrace_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h, grid_sampler);
  }

  workerDoneCount++;
  if (!continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "Canceled!\n");
    state = READY;
  }

  if (continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "Done! (%.4fs)\n", timer.duration());
    state = DONE;
  }
	
	delete grid_sampler;	
}

void PathTracer::increase_area_light_sample_count() {
  ns_area_light *= 2;
  fprintf(stdout, "[PathTracer] Area light sample count increased to %zu!\n", ns_area_light);
}

void PathTracer::decrease_area_light_sample_count() {
  if (ns_area_light > 1) ns_area_light /= 2;
  fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu!\n", ns_area_light);
}

void PathTracer::save_image() {

  if (state != DONE) return;

  time_t rawtime;
  time (&rawtime);

  string filename = "Screen Shot ";
  filename += string(ctime(&rawtime));
  filename.erase(filename.end() - 1);
  filename += string(".png");

  uint32_t* frame = &frameBuffer.data[0];
  size_t w = frameBuffer.w;
  size_t h = frameBuffer.h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
}

}  // namespace CMU462
