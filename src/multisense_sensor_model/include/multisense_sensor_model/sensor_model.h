#ifndef MULTISENSE_SENSOR_MODEL__SENSOR_MODEL_H_
#define MULTISENSE_SENSOR_MODEL__SENSOR_MODEL_H_

#include <boost/math/distributions/exponential.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>
#include <cmath>
#include <string>

namespace multisense_sensor_model
{

	typedef struct IntrinsicParams_ {
		IntrinsicParams_()
		{
				zhit = zshort = zmax = zrand = .25;
				sigma_hit = 0.01;
				lambda_short = .75;
				converged = false;
		}

		double measurementProbability(const double sensor_value,
									  const double map_value)
			{
				return zhit*pHit(sensor_value, map_value) +
					zshort*pShort(sensor_value, map_value) +
					zmax*pMax(sensor_value) +
					zrand*pRand(sensor_value);
			}

		double pHit(const double sensor_value, const double map_value)
			{
				boost::math::normal_distribution<> dist(map_value, sigma_hit);

				return boost::math::pdf(dist, sensor_value);
			}

		double pShort(const double sensor_value, const double map_value)
			{
				boost::math::exponential_distribution<> dist(lambda_short);

				if(sensor_value < 0 && sensor_value > map_value)
					return 0;

				double eta = 1.0/(1.0 - exp(-lambda_short*map_value));

				return eta*boost::math::pdf(dist, sensor_value);
			}

		double pMax(const double sensor_value, const double epsilon=.01)
			{
				if(fabs(sensor_value - 30.0) > epsilon)
					return 0.0;
				return 1.0;
			}

		double pRand(const double sensor_value)
			{
				boost::math::uniform_distribution<> dist(0.0, 30.0);

				return boost::math::pdf(dist, sensor_value);
			}

		double zhit, zshort, zmax, zrand;
		double sigma_hit;
		double lambda_short;
		bool converged;
	} IntrinsicParams;

	void writeIntrinsicParamsToFile(const std::string& filename, const IntrinsicParams& params);
	
	void readIntrinsicParamsFromFile(const std::string& filename, IntrinsicParams* params);	
}
#endif
