#include <algorithm>
#include <functional>
#include <vector>
#include <iostream>
#include "environment_light.h"

namespace CMU462
{
	namespace StaticScene
	{
		EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
		: envMap(envMap)
		{
			// TODO: initialize things here as needed
			size_t pixel_num = envMap->w * envMap->h;
			cond_cmf.reserve(pixel_num);
			cond_cmf.resize(pixel_num);
			
			assoc_pmf.reserve(pixel_num);
			assoc_pmf.resize(pixel_num);
			
			double inv_sum_weight{};
			for(size_t x = 0; x < envMap->w; ++x)
			{
				for(size_t y = 0; y < envMap->h; ++y)
				{
					double illum = envMap->data[x + y * envMap->w].illum();
					double sin_theta = std::sin( ((double)y + 0.5) / (double)envMap->h * PI);
					//flux ~ L * sin(theta)
					double weight = illum * sin_theta;
					cond_cmf[x + y * envMap->w] = weight;
					inv_sum_weight += weight;
				}
			}
			
			inv_sum_weight = 1 / inv_sum_weight;
			
			for(size_t x = 0; x < envMap->w; ++x)
			{
				for(size_t y = 0; y < envMap->h; ++y)
				{
					cond_cmf[x + y * envMap->w] *= inv_sum_weight;
					assoc_pmf[x + y * envMap->w] = cond_cmf[x + y * envMap->w];
				}
			}
			
			
			
			margin_cmf.reserve(envMap->h);
			margin_cmf.resize(envMap->h);
			
			for(size_t y = 0; y < envMap->h; ++y)
			{
				margin_cmf[y] = 0.0f;
				for(size_t x = 0; x < envMap->w; ++x)
				{
//					float z = cond_cmf[x + y * envMap->w];
					margin_cmf[y] += cond_cmf[x + y * envMap->w];
				}
				
				double inv_margin_pmf = 1 / margin_cmf[y];
				
				for(size_t x = 0; x < envMap->w; ++x)
				{
					cond_cmf[x + y * envMap->w] *= inv_margin_pmf;
					if(x > 0)
						cond_cmf[x + y * envMap->w] += cond_cmf[x - 1 + y * envMap->w];
//					std::cout<<cond_cmf[x + y * envMap->w]<<std::endl;
				}
				
				if(y > 0)
					margin_cmf[y] += margin_cmf[y - 1];
//				std::cout<<margin_cmf[y]<<std::endl;
			}
		
		}
		
		Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
																				float* distToLight,
																				float* pdf) const
		{
			// TODO: Implement
			
			// Uniform sphere sampling
//			double cos_theta = (((double)std::rand() / (double)RAND_MAX) - 0.5) * 2; //theta: 0 - pi
//			double theta = acos(cos_theta);
//			double phi = ((double)std::rand() / (double)RAND_MAX) * 2 * PI; //phi: 0 - 2*pi
//			double sin_theta = sqrt(1 - cos_theta * cos_theta);
//			(*wi).x = sin_theta * std::sin(phi);
//			(*wi).z = sin_theta * std::cos(phi);
//			(*wi).y = cos_theta;
//			*pdf = 0.25 * INV_PI;
//			*distToLight = INF_F;
		
			// Importance image-based sampling
			size_t row{}, col{};
			double theta{}, phi{};

			//1. inversion method to get theta
			double xi1 = (double)std::rand() / (double)RAND_MAX;
			auto it = std::lower_bound(margin_cmf.cbegin(), margin_cmf.cend(), xi1, std::less<float>());
			row = std::distance(margin_cmf.begin(), it);
			theta =  ((double)(row) + 0.5) / double(envMap->h) * PI;
			
			//2. inversion method to get phi
			double xi2 = (double)std::rand() / (double)RAND_MAX;
			it = std::lower_bound(cond_cmf.cbegin() + row * envMap->w,
														cond_cmf.cbegin() + (row + 1) * envMap->w,
														xi2, std::less<float>());
			col = std::distance(cond_cmf.cbegin() + row * envMap->w, it);
			phi = ((double)(row) + 0.5) / double(envMap->h) * 2 * PI;
			
			double sin_theta = std::sin(theta);
			(*wi).x = sin_theta * std::sin(phi);
			(*wi).z = sin_theta * std::cos(phi);
			(*wi).y = std::cos(theta);
			*pdf = assoc_pmf[col + row * envMap->w] * ((double)envMap->h * INV_PI) * ((double)envMap->w * 0.5 * INV_PI);
			*distToLight = INF_F;

			return sample_dir(theta, phi);
		}
		
		Spectrum EnvironmentLight::sample_dir(double theta, double phi) const
		{
			int x0{}, x1{}, y0{}, y1{};
			float t0{}, t1{};
			x0 = static_cast<int>(std::floor(phi - 0.5));
			t0 = phi - x0;
			x0 = x0 % envMap->w;
			x1 = (x0 + 1) % envMap->w;
			
			y0 = static_cast<int>(std::floor(theta - 0.5));
			t1 = theta - y0;
			y0 = y0 % envMap->h;
			y1 = (y0 + 1) % envMap->h;
			
			Spectrum s00 = envMap->data[x0 + y0 * envMap->w];
			Spectrum s01 = envMap->data[x0 + y1 * envMap->w];
			Spectrum s10 = envMap->data[x1 + y0 * envMap->w];
			Spectrum s11 = envMap->data[x1 + y1 * envMap->w];
			
			Spectrum result = (s00 * (1 - t0) + s10 * t0) * (1 - t1) +
			(s01 * (1 - t0) + s11 * t0) * t1;
			
			return result;

		}
		
		Spectrum EnvironmentLight::sample_dir(const Ray& r) const
		{
			// TODO: Implement
			double theta = acos(r.d.y); // from 0 - pi
			double sin_theta = sqrt(1 - r.d.y * r.d.y); //Edge case: sin_theta equals to 0
			double phi = acos(r.d.z / sin_theta); // from 0 - 2*pi
			if(r.d.x < 0)
				phi = 2 * PI - phi;
			
			theta *= INV_PI * static_cast<double>(envMap->h);
			phi *= INV_PI * 0.5 * static_cast<double>(envMap->w);
			int x0{}, x1{}, y0{}, y1{};
			float t0{}, t1{};
			x0 = static_cast<int>(std::floor(phi - 0.5));
			t0 = phi - 0.5 - x0;
			if(x0 < 0)
				x0 += static_cast<int>(envMap->w);
			x1 = (x0 + 1) % static_cast<int>(envMap->w);
			
			y0 = static_cast<int>(std::floor(theta - 0.5));
			t1 = theta - 0.5 - y0;
			if(y0 < 0)
				y0 += static_cast<int>(envMap->h);
			y1 = (y0 + 1) % static_cast<int>(envMap->h);
			
			Spectrum s00 = envMap->data[x0 + y0 * envMap->w];
			Spectrum s01 = envMap->data[x0 + y1 * envMap->w];
			Spectrum s10 = envMap->data[x1 + y0 * envMap->w];
			Spectrum s11 = envMap->data[x1 + y1 * envMap->w];
			
			Spectrum result = (s00 * (1 - t0) + s10 * t0) * (1 - t1) +
												(s01 * (1 - t0) + s11 * t0) * t1;
	
			return result;
		}

	} // namespace StaticScene
} // namespace CMU462
