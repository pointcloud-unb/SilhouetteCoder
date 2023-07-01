#include "cg_config.h"

void gpcc::EncoderConfigParams::setConfigFilePath(std::string configFilePath)
{
    this->configFilePath = configFilePath;
}

void gpcc::EncoderConfigParams::setInputFilePath(std::string inputFilePath)
{
    this->inputFilePath = inputFilePath;
}

void gpcc::EncoderConfigParams::setOutputFilePath(std::string outputFilePath)
{
    this->outputFilePath = outputFilePath;
}

void gpcc::EncoderConfigParams::setReconstructedFilePath(std::string reconstructedFilePath)
{
    this->reconstructedFilePath = reconstructedFilePath;
}

void gpcc::EncoderConfigParams::setAction(actions action)
{
    this->action = action;
}

void gpcc::EncoderConfigParams::setAxis(axisOpt axis)
{
    this->axis = axis;
}

void gpcc::EncoderConfigParams::setMode(int mode)
{
    this->mode = mode;
}

void gpcc::EncoderConfigParams::setNParallelism(int8_t nparallelism)
{
    this->nparallelism = nparallelism;
}

void gpcc::EncoderConfigParams::setNThreads(int8_t nthreads)
{
    this->nthreads = nthreads;
}

void gpcc::EncoderConfigParams::setAlgorithmChoice(int8_t algorithmChoice)
{
    this->algorithmChoice = algorithmChoice;
}

std::string gpcc::EncoderConfigParams::getConfigFilePath()
{
    return this->configFilePath;
}

std::string gpcc::EncoderConfigParams::getInputFilePath()
{
    return this->inputFilePath;
}

std::string gpcc::EncoderConfigParams::getOutputFilePath()
{
    return this->outputFilePath;
}

std::string gpcc::EncoderConfigParams::getReconstructedFilePath()
{
    return this->reconstructedFilePath;
}

int gpcc::EncoderConfigParams::getMode()
{
    return this->mode;
}

int gpcc::EncoderConfigParams::getAction()
{
    return this->action;
}

int gpcc::EncoderConfigParams::getAxis()
{
    return this->axis;
}

int8_t gpcc::EncoderConfigParams::getNParallelism()
{
    return this->nparallelism;
}

int8_t gpcc::EncoderConfigParams::getNThreads()
{
    return this->nthreads;
}

int8_t gpcc::EncoderConfigParams::getAlgorithmChoice()
{
    return this->algorithmChoice;
}

void gpcc::EncoderConfigParams::useParser(char *configFilePath)
{
    ConfigParser p;
    p.parseFile(std::string(configFilePath));
    std::map<std::string, std::string> dic = p.getDic();

    if (getInputFilePath() == "")
    {
        setInputFilePath(dic["InputFile"]);
    }

    if (getOutputFilePath() == "")
    {
        setOutputFilePath(dic["OutputFile"]);
    }

    if (getReconstructedFilePath() == "")
    {
        setReconstructedFilePath(dic["ReconstructedFile"]);
    }

    if (getMode() == -1)
    {
        if (dic["SingleMode"] != "")
        {
            setMode(stoi(dic["SingleMode"]));
        }
    }

    if (getAction() == EMPTY_ACTION)
    {
        if (dic["Action"] != "")
        {
            setAction(static_cast<actions>(stoi(dic["Action"])));
        }
    }

    if (getAxis() == EMPTY_AXIS)
    {
        if (dic["Axis"] != "")
        {
            setAxis(static_cast<axisOpt>(stoi(dic["Axis"])));
        }
    }

    if (getNParallelism() == -1)
    {
        if (dic["NParallelism"] != "")
        {
            setNParallelism(stoi(dic["NParallelism"]));
        }
    }

    if (getNThreads() == -1)
    {
        if (dic["NThreads"] != "")
        {
            setNThreads(stoi(dic["NThreads"]));
        }
    }

    if (getAlgorithmChoice() == -1)
    {
        if (dic["AlgorithmChoice"] != "")
        {
            setAlgorithmChoice(stoi(dic["AlgorithmChoice"]));
        }
    }
}

void gpcc::EncoderConfigParams::setInitialValues()
{
    setConfigFilePath("");
    setInputFilePath("");
    setOutputFilePath("");
    setReconstructedFilePath("");
    setMode(-1);
    setAction(EMPTY_ACTION);
    setAxis(EMPTY_AXIS);
    setNParallelism(-1);
    setNThreads(-1);
    setAlgorithmChoice(-1);
}

gpcc::EncoderConfigParams::EncoderConfigParams(int argc, char *args[])
{
    if (argc == 1)
    {
        std::cout << "ERROR! A configuration file and/or arguments are needed!" << std::endl;
        return;
    }
    else
    {
        setInitialValues();
        for (int i = 1; i < argc; i = i + 2)
        {
            switch (configsMap[std::string(args[i])])
            {
            case CONFIG_FILE:
                setConfigFilePath(args[i + 1]);
                useParser(args[i + 1]);
                break;
            case INPUT_FILE:
                setInputFilePath(std::string(args[i + 1]));
                break;
            case OUTPUT_FILE:
                setOutputFilePath(std::string(args[i + 1]));
                break;
            case RECONSTRUCTED_FILE:
                setReconstructedFilePath(std::string(args[i + 1]));
                break;
            case MODE:
                setMode(stoi(std::string(args[i + 1])));
                break;
            case ENCODE:
                setAction(ENCODE_ACTION);
                i--;
                break;
            case DECODE:
                setAction(DECODE_ACTION);
                i--;
                break;
            case AXIS:
                setAxis(static_cast<axisOpt>(stoi(std::string(args[i + 1]))));
                break;
            case PARALLELISM:
                setNParallelism(stoi(std::string(args[i + 1])));
                break;
            case THREAD:
                setNThreads(stoi(std::string(args[i + 1])));
                break;
            case ALGORITHM:
                setNThreads(stoi(std::string(args[i + 1])));
                break;
            default:
                std::cout << "ERROR! Unknown operation in command line." << std::endl;
                break;
            }
        }
    }
}

void gpcc::EncoderConfigParams::showConfigs()
{
    std::cout << "Config File Path: " + this->configFilePath << std::endl;
    std::cout << "Input File Path: " + this->inputFilePath << std::endl;
    std::cout << "Output File Path: " + this->outputFilePath << std::endl;
    std::cout << "Reconstructed File Path: " + this->reconstructedFilePath << std::endl;
    std::cout << "Mode: " + std::to_string(this->mode) << std::endl;
    std::cout << "Action: " + std::to_string(this->action) << std::endl;
    std::cout << "Axis: " + std::to_string(this->axis) << std::endl;
    std::cout << "Number of Parallelism " + std::to_string(this->nparallelism) << std::endl;
    std::cout << "Number of Threads " + std::to_string(this->nthreads) << std::endl;
    std::cout << "Algorithm Choice " + std::to_string(this->algorithmChoice) << std::endl;
}

void gpcc::EncoderConfigParams::writeConfigFile(std::string filepath)
{
    std::ofstream file;
    file.open(filepath);

    file << "InputFile = ";
    file << this->inputFilePath << "\n";

    file << "OutputFile = ";
    file << this->outputFilePath << "\n";

    file << "ReconstructedFile = ";
    file << this->reconstructedFilePath << "\n";

    file << "SingleMode = ";
    file << this->mode;
    file << "\n";

    file << "Action = ";
    file << (this->action == ENCODE_ACTION ? ENCODE_ACTION : DECODE_ACTION);
    file << "\n";

    file << "Axis = ";
    if (this->axis == X)
    {
        file << X;
    }
    else if (this->axis == Y)
    {
        file << Y;
    }
    else
    {
        file << Z;
    }
    file << "\n";

    file << "NParallelism = ";
    file << this->nparallelism << "\n";

    file << "NThread = ";
    file << this->nthreads << "\n";

    file << "AlgorithmChoice = ";
    file << this->nthreads << "\n";

    file.close();
}