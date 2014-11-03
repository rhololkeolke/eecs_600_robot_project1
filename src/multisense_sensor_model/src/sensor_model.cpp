#include <multisense_sensor_model/sensor_model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace multisense_sensor_model
{
	void writeIntrinsicParamsToFile(const std::string& filename, const IntrinsicParams& params)
	{
		YAML::Emitter emitter;

		emitter << YAML::BeginMap;
		emitter << YAML::Key << "zhit";
		emitter << YAML::Value << params.zhit;
		emitter << YAML::Key << "zshort";
		emitter << YAML::Value << params.zshort;
		emitter << YAML::Key << "zmax";
		emitter << YAML::Value << params.zmax;
		emitter << YAML::Key << "zrand";
		emitter << YAML::Value << params.zrand;
		emitter << YAML::Key << "sigma_hit";
		emitter << YAML::Value << params.sigma_hit;
		emitter << YAML::Key << "lambda_short";
		emitter << YAML::Key << params.lambda_short;
		emitter << YAML::EndMap;

		std::ofstream yaml_file(filename.c_str());
		yaml_file << emitter.c_str();
	}

	void readIntrinsicParamsFromFile(const std::string& filename, IntrinsicParams* params)
	{
		if(params == NULL)
			return;
		
		YAML::Node yaml_file = YAML::LoadFile(filename.c_str());
		params->zhit = yaml_file["zhit"].as<double>();
		params->zshort = yaml_file["zshort"].as<double>();
		params->zmax = yaml_file["zmax"].as<double>();
		params->zrand = yaml_file["zrand"].as<double>();
		params->sigma_hit = yaml_file["sigma_hit"].as<double>();
		params->lambda_short = yaml_file["lambda_short"].as<double>();
	}
}
