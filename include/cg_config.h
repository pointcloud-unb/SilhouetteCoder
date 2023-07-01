#ifndef CG_CONFIG_H
#define CG_CONFIG_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "cg_config_parser.h"

namespace gpcc
{
	class EncoderConfigParams
	{
	public:
		EncoderConfigParams(int argc, char *args[]);
		void showConfigs();
		std::string getConfigFilePath();
		std::string getInputFilePath();
		std::string getOutputFilePath();
		std::string getReconstructedFilePath();
		int getMode();
		int getAction();
		int getAxis();
		int8_t getNParallelism();
		int8_t getNThreads();
		int8_t getAlgorithmChoice();
		void writeConfigFile(std::string filepath);
		enum configs
		{
			CONFIG_FILE,
			INPUT_FILE,
			OUTPUT_FILE,
			RECONSTRUCTED_FILE,
			MODE,
			ENCODE,
			DECODE,
			AXIS,
			PARALLELISM,
			THREAD,
			ALGORITHM
		};

		enum axisOpt
		{
			EMPTY_AXIS,
			X,
			Y,
			Z,
			ALL_AXIS
		};
		enum actions
		{
			EMPTY_ACTION,
			ENCODE_ACTION,
			DECODE_ACTION
		};
		enum algorithm
		{
			DD_SM,
			BLOCK_MODE,
			INVERTED_MODE,
			DD_SM_THREAD,
			DD_SM_PYRAMID_THREAD,
		};

	private:
		std::map<std::string, configs> configsMap =
				{{"-c", CONFIG_FILE}, {"-i", INPUT_FILE}, {"-o", OUTPUT_FILE}, {"-r", RECONSTRUCTED_FILE}, {"-m", MODE}, {"-enc", ENCODE}, {"-dec", DECODE}, {"-axis", AXIS}, {"-nparallelism", PARALLELISM}, {"-nthread", THREAD}, {"-a", ALGORITHM}};

		std::string configFilePath;
		std::string inputFilePath;
		std::string outputFilePath;
		std::string reconstructedFilePath;
		actions action;
		axisOpt axis;
		// 0 - Normal | >= 1 - SingleMode
		int mode;
		int8_t nparallelism;
		int8_t nthreads;
		int8_t algorithmChoice;

		void useParser(char *configFilePath);
		void useCmd(char *args[]);

		void setInitialValues();
		void setConfigFilePath(std::string configFilePath);
		void setInputFilePath(std::string inputFilePath);
		void setOutputFilePath(std::string outputFilePath);
		void setReconstructedFilePath(std::string reconstructedFilePath);
		void setMode(int mode);
		void setAction(actions action);
		void setAxis(axisOpt axis);
		void setNParallelism(int8_t nparallelism);
		void setNThreads(int8_t nthread);
		void setAlgorithmChoice(int8_t algorithmChoice);
	};

} // namespace gpcc

#endif