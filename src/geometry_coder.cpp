#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>
#include <string.h>
#include <tuple>
#include <chrono>
#include <algorithm>
#include <string>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "ds_silhouette_decomposition.h"
#include "ds_point_cloud.h"
#include "cg_config.h"
#include "en_encoder.h"
#include "en_decoder.h"
#include "en_codec_parameter_bitstream.h"
#include "ac_bitstream.h"
#include "ac_bac_encoder.h"
#include "ac_bac_decoder.h"
#include "ac_context.h"
#include "en_cabac.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#include "th_thread_manager_pyramid.h"
#include "th_thread_manager.h"

void testImageTraversingOrder();
void testSilhouetteDecomposition();
// void testCabacSingleImage();
void testCodecParameterBitstream();
void testArithmeticCoderEngine();
void testBitstream();

void headerMessage();
void helpMessage();
bool checkDecoding(gpcc::EncoderConfigParams *params, gpcc::PointCloud *ppc);
inline bool fileExists(const std::string &name);
typedef struct results
{
	double rate_x;
	double rate_y;
	double rate_z;
	double best_axis_rate;
	int nc2D;
	std::string filePath;
} results;
typedef struct headerData
{
	uint16_t nBits;
	uint16_t iAxis;
	uint16_t algorithmChoice;
} headerData;
std::vector<results> getRateValues(std::string mode, std::vector<std::string> files_path, int n_frame, int k, int nc2D);
std::vector<results> getStandardRateValues(std::vector<std::string> files_path, int n_frame, int nc2D);
std::vector<results> getInvertedModeRateValues(std::vector<std::string> files_path, int n_frame, int k, int nc2D);
std::vector<results> getBlockModeRateValues(std::vector<std::string> files_path, int n_frame, int k, int nc2D);
std::vector<std::string> getFilesFromDirectory(std::string directory_path);
void exportResultsToCSV(std::string pc_name, std::vector<results> bpovResults, int k);
void getResults(std::string mode, std::string directory, int n_frame, std::string pcName, int k, int nc2D);
headerData parseHeaderData(Bitstream &bstream);

void writeResultsToTxtFileEnc(std::string path, double axis_time[], double total_time, double axis_rate_bpov[],double best_rate);
void writeResultsToTxtFileDec(std::string path, bool success);


int main(int argc, char *argv[])
{
	clock_t tStart, tEnd;
	gpcc::EncoderConfigParams *pParams;
	if (argc != 1)
	{
		pParams = new gpcc::EncoderConfigParams(argc, argv);
	}
	else
	{
		pParams = nullptr;
	}

	headerMessage();

	if (pParams == nullptr)
	{
		helpMessage();
		return 0;
	}

	// Time's table
	double main_time, load_time, code_time, flush_time;

	std::string inputPly, binFile, outputPly;

	// Is encoding or decoding?
	if (pParams->getAction() == gpcc::EncoderConfigParams::ENCODE_ACTION)
	{
		inputPly = pParams->getInputFilePath();
		binFile = pParams->getOutputFilePath();

		// Loads the point cloud.
		std::cout << "Loading point cloud... " << std::endl;

		auto start_main_time = std::chrono::high_resolution_clock::now();
		auto start_aux_time = std::chrono::high_resolution_clock::now();

		gpcc::PointCloud *ppc = new gpcc::PointCloud();
		bool pcLoaded = ppc->Load(inputPly);

		auto end_aux_time = std::chrono::high_resolution_clock::now();

		if (pcLoaded)
		{
			load_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;
		}
		else
		{
			std::cout << "FAILED. " << std::endl;
			std::cout << "Terminating program..." << std::endl;

			if (ppc != NULL)
				delete ppc;
			if (pParams != NULL)
				delete pParams;
			return 1;
		}

		// Point cloud is loaded. I should test the axis.
		Bitstream *axis_bitstream[3] = {nullptr, nullptr, nullptr};
		uint32_t axis_nBits[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
		Bitstream *finalBitstream = nullptr;
		double axis_time[3] = {0, 0, 0};
		double axis_rate_bpov[3] = {0, 0, 0};
		bool axis_tested[3] = {false, false, false};
		uint16_t currAxis;

		for (currAxis = gpcc::EncoderConfigParams::X;
				 currAxis <= gpcc::EncoderConfigParams::Z; currAxis++)
		{
			if ((currAxis == pParams->getAxis()) || pParams->getAxis() == 4)
			{
				gpcc::Axis axis = ((currAxis == gpcc::EncoderConfigParams::X) ? gpcc::Axis::X : ((currAxis == gpcc::EncoderConfigParams::Y) ? gpcc::Axis::Y : gpcc::Axis::Z));

				start_aux_time = std::chrono::high_resolution_clock::now();

				std::cout << "Encoding point cloud using axis " << ((currAxis == 1) ? "X" : ((currAxis == 2) ? "Y" : "Z")) << "..." << std::endl;

				// TODO: Get these from the config.
				uint64_t m = 16;
				uint64_t nc2D = 6;
				uint64_t nc3D = 9;

				switch (pParams->getAlgorithmChoice())
				{
					case (gpcc::EncoderConfigParams::algorithm::DD_SM):
					{
						std::cout << "ENCODING WITH single thread S3D..." << std::endl;
						gpcc::Encoder* enc = new gpcc::Encoder(ppc, pParams, axis);
												
						// Initialize cabac
						Cabac* cabac = new Cabac();
						cabac->initEncoder(m, nc2D, nc3D);

						gpcc::encodeReturnData encodingData = enc->encodePointCloud2(0, cabac);

						cabac = encodingData.cabacOut;

						axis_bitstream[currAxis - 1] = enc->closeBitstream(cabac);

						break;
					}

					case (gpcc::EncoderConfigParams::algorithm::BLOCK_MODE):
					{
						/* code */
						break;
					}
					case (gpcc::EncoderConfigParams::algorithm::INVERTED_MODE):
					{
						/* code */
						break;
					}

					case (gpcc::EncoderConfigParams::algorithm::DD_SM_THREAD):
					{
						std::cout << "ENCODING WITH multi thread S3D using " << (1 << pParams->getNThreads()) << " threads" << std::endl;
						gpcc::ThreadManager enc(pParams);
						axis_bitstream[currAxis - 1] = enc.Encode(ppc, axis);
						break;
					}

					case (gpcc::EncoderConfigParams::algorithm::DD_SM_PYRAMID_THREAD):
					{
						std::cout << "ENCODING WITH multi thread S3D-Illuminati using " << (1 << pParams->getNThreads()) << " threads" << std::endl;
						gpcc::ThreadManagerPyramid enc(pParams);
						axis_bitstream[currAxis - 1] = enc.Encode(ppc, axis);
						break;
					}

					default:
						std::cout << "ALGORITHM CHOICE INVALID." << std::endl;
						std::cout << "Terminating..." << std::endl;
						return -1;
						break;
				}							

				end_aux_time = std::chrono::high_resolution_clock::now();
				axis_time[currAxis - 1] = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;

				axis_nBits[currAxis - 1] = axis_bitstream[currAxis - 1]->totalSize() + 8;

				axis_rate_bpov[currAxis - 1] = (double(axis_nBits[currAxis - 1])) / ppc->NumberOccupiedVoxels();

				axis_tested[currAxis - 1] = true;
			}
		}

		// Decides for the best axis.
		int bestAxis = (((axis_nBits[0] < axis_nBits[1]) && (axis_nBits[0] < axis_nBits[2])) ? 0 : ((axis_nBits[1] < axis_nBits[2]) ? 1 : 2));
		finalBitstream = axis_bitstream[bestAxis];

		start_aux_time = std::chrono::high_resolution_clock::now();
		finalBitstream->flushesToFile(binFile);
		end_aux_time = std::chrono::high_resolution_clock::now();
		flush_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;

		auto end_main_time = std::chrono::high_resolution_clock::now();
		main_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_main_time - start_main_time).count()) / 1000;

		std::cout << "\n...... Time Table ......\n"
							<< "Load PC time ... "
							<< load_time << " seconds.\n"
							<< "Flush BITSTREAM to file time ... "
							<< flush_time << " seconds.\n"
							<< "Encode time ... \n";

		if (axis_tested[0])
		{
			std::cout << "X Axis: " << axis_time[0] << " seconds - Rate: " << axis_rate_bpov[0] << " bpov. \n";
		}
		if (axis_tested[1])
		{
			std::cout << "Y Axis: " << axis_time[1] << " seconds - Rate: " << axis_rate_bpov[1] << " bpov. \n";
		}
		if (axis_tested[2])
		{
			std::cout << "Z Axis: " << axis_time[2] << " seconds - Rate: " << axis_rate_bpov[2] << " bpov. \n";
		}

		std::cout << "FULL time ... "
							<< main_time << " seconds."
							<< std::endl;

		std::cout << "Final Rate = " << axis_rate_bpov[bestAxis] << " bits per occupied voxel."
						  << std::endl;

		writeResultsToTxtFileEnc(binFile, axis_time, main_time, axis_rate_bpov, axis_rate_bpov[bestAxis]);

		if (axis_bitstream[0] != nullptr)
			delete axis_bitstream[0];
		if (axis_bitstream[1] != nullptr)
			delete axis_bitstream[1];
		if (axis_bitstream[2] != nullptr)
			delete axis_bitstream[2];
		if (ppc != NULL)
			delete ppc;
	}
	else if (pParams->getAction() == gpcc::EncoderConfigParams::DECODE_ACTION)
	{

		binFile = pParams->getOutputFilePath();
		outputPly = pParams->getReconstructedFilePath();

		gpcc::PointCloud *ppc;

		// Loads the bitstream.
		std::cout << "Loading bitstream... " << std::endl;

		auto start_main_time = std::chrono::high_resolution_clock::now();

		auto start_aux_time = std::chrono::high_resolution_clock::now();
		Bitstream bstream = Bitstream(binFile);
		Bitstream bstreamHeader = Bitstream(binFile);
		auto end_aux_time = std::chrono::high_resolution_clock::now();

		load_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;

		start_aux_time = std::chrono::high_resolution_clock::now();

		std::cout << "Decoding point cloud... " << std::endl;
		headerData header = parseHeaderData(bstreamHeader);
		gpcc::Axis axis = ((header.iAxis == gpcc::EncoderConfigParams::X) ? gpcc::Axis::X : ((header.iAxis == gpcc::EncoderConfigParams::Y) ? gpcc::Axis::Y : gpcc::Axis::Z));

		switch (header.algorithmChoice)
		{
		case (gpcc::EncoderConfigParams::algorithm::DD_SM):
		{
			std::cout << "DECODING WITH single thread S3D..." << std::endl;
			gpcc::Decoder dec(bstream, axis, pParams);
			ppc = dec.decodePointCloud2();
			break;
		}

		case (gpcc::EncoderConfigParams::algorithm::BLOCK_MODE):
		{
			/* code */
			break;
		}
		case (gpcc::EncoderConfigParams::algorithm::INVERTED_MODE):
		{
			/* code */
			break;
		}

		case (gpcc::EncoderConfigParams::algorithm::DD_SM_THREAD):
		{
			std::cout << "DECODING WITH multi thread S3D..." << std::endl;
			gpcc::ThreadManager dec(pParams);
			ppc = dec.Decode(bstream);
			break;
		}
		case (gpcc::EncoderConfigParams::algorithm::DD_SM_PYRAMID_THREAD):
		{
			std::cout << "DECODING WITH multi thread S3D-Illuminati..." << std::endl;
			gpcc::ThreadManagerPyramid dec(pParams);
			ppc = dec.Decode(bstream);
			break;
		}

		default:
			break;
		}

		end_aux_time = std::chrono::high_resolution_clock::now();

		code_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;

		// TODO
		// Get the point cloud and dumps it into the output file.
		start_aux_time = std::chrono::high_resolution_clock::now();
		ppc->Flush(outputPly);
		end_aux_time = std::chrono::high_resolution_clock::now();

		flush_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_aux_time - start_aux_time).count()) / 1000;

		auto end_main_time = std::chrono::high_resolution_clock::now();
		main_time = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(end_main_time - start_main_time).count()) / 1000;

		if (checkDecoding(pParams, ppc))
		{
			std::cout << "DECODING SUCCESSFUL." << std::endl;
			std::cout << "\n...... Time Table ......\n"
								<< "Load BITSTREAM time ... "
								<< load_time << " seconds.\n"
								<< "Flush PC to file time ... "
								<< flush_time << " seconds.\n"
								<< "Decode time ... "
								<< code_time << " seconds.\n"
								<< "FULL time ... "
								<< main_time << " seconds."
								<< std::endl;

			writeResultsToTxtFileDec(outputPly, true);
		}
		else
		{
			std::cout << "DECODING FAILED." << std::endl;
			writeResultsToTxtFileDec(outputPly, false);
		}

		delete ppc;
	}
	else
	{
		std::cout << "ERROR in the Parameters: A valid action should be provided." << std::endl;
		helpMessage();
		return 1;
	}

	if (pParams != NULL)
		delete pParams;

	//std::cout << std::endl	<< std::endl;
	//std::cout << "Press ENTER to finish." << std::endl;
	//std::string name;
	//getline(std::cin, name);

	return 0;
}

void writeResultsToTxtFileEnc(std::string path,double axis_time[], double total_time, double axis_rate_bpov[], double best_rate)
{
	std::string filepath;
	std::ofstream myfile;

	int x = path.find_last_of('\\');

	if (x != -1)
	{
		filepath = path.substr(0, x + 1);
		filepath.append("results.txt");
	}
	else
	{
		filepath = std::string("results.txt");
	}

	myfile.open(filepath, std::ios::out | std::ios::app);

	myfile << std::fixed << std::setprecision(6);
	myfile << axis_time[0] << " " << axis_time[1] << " " << axis_time[2] << " " << total_time << " " << axis_rate_bpov[0] << " " << axis_rate_bpov[1] << " " << axis_rate_bpov[2] << " " << best_rate << " ";

	myfile.close();

}

void writeResultsToTxtFileDec(std::string path, bool success)
{
	std::string filepath;
	std::ofstream myfile;

	int x = path.find_last_of('\\');

	if (x != -1)
	{
		filepath = path.substr(0,x+1);
		filepath.append("results.txt");
	}
	else
	{
		filepath = std::string("results.txt");
	}

	myfile.open(filepath, std::ios::out | std::ios::app);

	myfile << std::fixed << std::setprecision(6);
	myfile << (success ? 1 : 0) << "\n";

	myfile.close();

}

void headerMessage()
{
	std::cout << "Running SILHOUETTE 3D - Point Cloud Intra Geometry Coder" << std::endl;
	std::cout << "Author: Eduardo Peixoto     - eduardopeixoto@ieee.org" << std::endl;
	std::cout << "Author: Edil Medeiros       - j.edil@ene.unb.br" << std::endl;
	std::cout << "Author: Eduardo Lemos       - " << std::endl;
	std::cout << "Author: Estevam Albuquerque - " << std::endl;
	std::cout << "Author: Otho Komatsu        - otho.tk@hotmail.com" << std::endl;
	std::cout << "Author: Rodrigo Borba       - " << std::endl
						<< std::endl;
}

void helpMessage()
{
	std::cout << std::endl
						<< "Usage:" << std::endl;
	std::cout << "SilhouetteCoder -enc/-dec -axis <enc:axis> -c <config_file> -i <input file - enc:pointcloud> -o <enc/dec:binary file> -r <dec:reconstructed file>" << std::endl;
}

void testImageTraversingOrder()
{
	std::string str = std::string("C:\\eduardo\\workspace\\workPointClouds\\small.bmp");

	gpcc::ImageRaster *im1 = new gpcc::ImageRaster(str);

	int nrows, ncols;
	std::tie(nrows, ncols) = im1->Size();
	std::cout << std::endl;

	int y, x;
	uint8_t currpixel;
	std::cout << "Image Raster:" << std::endl;
	for (y = 0; y < nrows; y++)
	{
		for (x = 0; x < ncols; x++)
		{
			currpixel = im1->PixelPresent(y, x);
			std::cout << " " << int(currpixel) << " ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;
	gpcc::ImageSparse *im2 = new gpcc::ImageSparse(im1);
	std::cout << "Image Sparse from Image Raster:" << std::endl;
	for (y = 0; y < nrows; y++)
	{
		for (x = 0; x < ncols; x++)
		{
			currpixel = im2->PixelPresent(y, x);
			std::cout << " " << int(currpixel) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	gpcc::Pixel<int> pixel;
	std::cout << "Traversing ImageSparse with the Iterator:" << std::endl;
	int nPixels = im2->NumberOccupiedPixels();

	for (int n = 0; n < nPixels; n++)
	{
		pixel = im2->CurrentPixel();
		std::cout << "( " << pixel[0] << " , " << pixel[1] << " )" << std::endl;
		im2->NextPixel();
	}

	delete im1;
	delete im2;
}

void testSilhouetteDecomposition()
{
	gpcc::SilhouetteDecomposition decomposition(6);
	decomposition.createDyadicDecomposition();
	decomposition.printList();
}

/*
void testCabacSingleImage()
{
 //gpcc::IImage image()
 std::string str = "C:\\eduardo\\workspace\\workPointClouds\\ricardo.bmp";
 std::cout << "str = " << str << std::endl;
 gpcc::IImage *image = new gpcc::ImageRaster(str);
 int s1,s2;
 std::tie(s1,s2) = image->Size();

 //Initialize CABAC
 Cabac cabacEncoder;
 cabacEncoder.initEncoder(16, 8);
 Bitstream bstream;

 clock_t tStart, tEnd;
 tStart = clock();

 cabacEncoder.encodeImage(*image, bstream);

 tEnd = clock();

 cabacEncoder.closeEncoder(bstream);

 std::cout << "Written in the bitstream: " << int(bstream.totalSize()) << " bits." << std::endl;
 std::cout << "Encoder Elapsed time    : " << (double(tEnd - tStart)) / CLOCKS_PER_SEC << " seconds." << std::endl;

 //Initialize CABAC Decoder
 bstream.changeModeToRead();
 Cabac cabacDecoder;
 cabacDecoder.initDecoder(16, 8, bstream);

 tStart = clock();

 gpcc::IImage* imageDec = cabacDecoder.decodeImage(s1, s2);

 tEnd = clock();

 std::cout << "Decoder Elapsed time    : " << (double(tEnd - tStart)) / CLOCKS_PER_SEC << " seconds." << std::endl;

 //Compares both images
 uint64_t N = 0;
 int y, x;
 bool pixel1, pixel2;
 for (y = 0; y < s1; y++)
 {
	 for (x = 0; x < s2; x++)
	 {
		 pixel1 = image->PixelPresent(y, x);
		 pixel2 = imageDec->PixelPresent(y, x);

		 if (pixel1 != pixel2) N++;
	 }
 }
 std::cout << "Number of different pixels = " << N << " ." << std::endl;

 std::string str2 = "C:\\eduardo\\workspace\\workPointClouds\\ricardo_out.bmp";
 imageDec->WriteBMP(str2);

 delete image;
 delete imageDec;
}
*/

void testCodecParameterBitstream()
{
	double p1 = 0.9;
	int nSymbols = 10000;
	int m = 16;
	int numberOfContextBits = 5;

	std::cout << "testing CodecParameterBitstream" << std::endl;
	std::cout << "Symbol probability          : [" << (1 - p1) << " , " << p1 << "]" << std::endl;
	std::cout << "Number of Symbols           : " << nSymbols << std::endl;
	std::cout << "Arithmetic Coder word size  : " << m << std::endl;
	std::cout << "Number of Context Bits Used : " << numberOfContextBits << std::endl
						<< std::endl;

	double H = -p1 * log2(p1) - (1 - p1) * log2(1 - p1);

	std::vector<bool> msg(nSymbols);

	// Creates the message.
	int n1s = 0;
	int n0s = 0;
	for (int i = 0; i < nSymbols; i++)
	{
		bool bit = (((float)rand() / RAND_MAX) < p1);
		msg[i] = bit;

		if (bit)
		{
			n1s++;
		}
		else
		{
			n0s++;
		}
	}

	// Creates the container
	CodecParameterBitstream cpBitstream;
	for (int i = 0; i < nSymbols; i++)
	{
		cpBitstream.putBit(msg[i]);
	}

	std::cout << "Message Profile:" << std::endl;
	std::cout << "Number of 1s = " << n1s << " ." << std::endl;
	std::cout << "Number of 0s = " << n0s << " ." << std::endl;
	std::cout << "Entropy = " << H << " bits / symbol" << std::endl;

	Bitstream bStream = cpBitstream.encode(m, numberOfContextBits);

	std::cout << "bitstream size = " << bStream.totalSize() << std::endl;

	double rate = ((double)bStream.totalSize()) / nSymbols;
	std::cout << "Rate = " << rate << " bits / symbol" << std::endl;

	bStream.changeModeToRead();

	CodecParameterBitstream cpBitstream2(bStream, m, numberOfContextBits);

	bool is_equal = true;
	for (int i = 0; i < nSymbols; i++)
	{
		bool currBit = cpBitstream2.readBit();
		is_equal = is_equal && (currBit == msg[i]);
	}

	std::cout << "The two vectors are " << (is_equal ? "EQUAL" : "NOT EQUAL") << " ." << std::endl;
}

void testArithmeticCoderEngine()
{
	int m = 16;
	Bitstream bstream;
	ContextInfo c;
	c.mps = true;
	c.countMPS = 116;
	c.totalCount = 128;

	double p1 = 0.9;
	double H = -p1 * log2(p1) - (1 - p1) * log2(1 - p1);

	int nSymbols = 2000000;
	std::vector<bool> msg(nSymbols);
	std::vector<bool> dec_msg(nSymbols);

	// Creates the message.
	int n1s = 0;
	int n0s = 0;
	for (int i = 0; i < nSymbols; i++)
	{
		msg[i] = (((float)rand() / RAND_MAX) < p1);
		if (msg[i])
		{
			n1s++;
		}
		else
		{
			n0s++;
		}
	}

	std::cout << "Message Profile:" << std::endl;
	std::cout << "Number of 1s = " << n1s << " ." << std::endl;
	std::cout << "Number of 0s = " << n0s << " ." << std::endl;
	std::cout << "Entropy = " << H << " bits / symbol" << std::endl;

	clock_t tStart, tEnd;
	tStart = clock();

	BACEncoder bac(m);

	for (int i = 0; i < nSymbols; i++)
	{
		bac.encodeOneSymbol(msg[i], c, bstream);
	}
	bac.closeBitstream(bstream);
	tEnd = clock();

	std::cout << std::dec;
	std::cout << "bitstream size = " << bstream.totalSize() << std::endl;
	std::cout << "Number of symbls encoded = " << nSymbols << std::endl;
	double rate = ((double)bstream.totalSize()) / nSymbols;
	std::cout << "Rate = " << rate << " bits / symbol" << std::endl;
	std::cout << "Encoding time = " << (double(tEnd - tStart)) / CLOCKS_PER_SEC << " seconds." << std::endl;
	bac.showStatus();

	bstream.changeModeToRead();

	/*
	std::cout << "Bitstream: [ ";
	while (bstream.hasBits())
	{
		bool bit = bstream.readBit();
		std::cout << (bit ? "1" : "0") << " ";
	}
	std::cout << "]" << std::endl;
	*/

	std::cout << "bitstream size = " << bstream.totalSize() << std::endl;

	tStart = clock();

	BACDecoder dec(&bstream, m);

	std::cout << std::dec;
	dec.init();
	for (int i = 0; i < nSymbols; i++)
	{
		bool symbol = dec.decodeOneSymbol(c);
		dec_msg[i] = symbol;
		// std::cout << "Bit = " << (symbol ? "1" : "0") << std::endl;
	}
	tEnd = clock();

	dec.showStatus();
	std::cout << "Decoding time = " << (double(tEnd - tStart)) / CLOCKS_PER_SEC << " seconds." << std::endl;

	// Compare both vectors
	bool are_same = true;
	for (int i = 0; i < 20; i++)
	{
		are_same = (msg[i] == dec_msg[i]);
		if (!are_same)
		{
			break;
		}
	}

	std::cout << "The two vectors are " << (are_same ? "EQUAL" : "NOT EQUAL") << " ." << std::endl;

	std::cout << "Leftover Bits:" << std::endl;
	std::cout << "[ ";
	while (bstream.hasBits())
	{
		bool bit = bstream.readBit();
		std::cout << (bit ? "1" : "0") << " ";
	}
	std::cout << "]" << std::endl;
}

void testBitstream()
{
	/*
	Bitstream inStream("test.bin");
	std::cout << std::dec;
	std::cout << "Total Size = " << inStream.totalSize() << " ." << std::endl;

	Bitstream inStream2(inStream, 10);
	std::cout << "inStream  = " << inStream.numberOfRemainingBits() << " ." << std::endl;
	std::cout << "inStream2 = " << inStream2.numberOfRemainingBits() << " ." << std::endl;

	int i = 0;
	std::cout << "inStream2" << std::endl;
	while (inStream2.hasBits())
	{
		bool bit = inStream2.readBit();
		std::cout << "Bit [" << i << "] = " << (bit ? "1" : "0") << std::endl;
		//std::cout << "Remaining " << inStream.numberOfRemainingBits() << " of a total of " << inStream.totalSize() << std::endl;
		i++;
	}

	std::cout << "inStream" << std::endl;
	i = 0;
	while (inStream.hasBits())
	{
		bool bit = inStream.readBit();
		std::cout << "Bit [" << i << "] = " << (bit ? "1" : "0") << std::endl;
		i++;
	}
	*/

	/*
	std::cout << "This is my geometry coder" << std::endl;

	Bitstream bstream;

	bstream.writeBit(1);
	bstream.writeBit(1);
	bstream.writeBit(1);
	bstream.writeBit(1);
	bstream.writeBit(0);
	bstream.writeBit(0);
	bstream.writeBit(0);
	bstream.writeBit(0);

	bstream.writeBit(0);
	bstream.writeBit(0);
	bstream.writeBit(0);
	bstream.writeBit(0);
	bstream.writeBit(1);
	bstream.writeBit(1);
	bstream.writeBit(1);
	bstream.writeBit(1);

	bstream.writeBit(1);
	bstream.writeBit(0);
	bstream.writeBit(1);

	bstream.flushesToFile("test.bin");

	Bitstream bstream2;

	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(0);

	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);

	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);
	bstream2.writeBit(0);

	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);
	bstream2.writeBit(1);

	bstream2.flushesToFile("test2.bin");


	bstream.merge(bstream2);
	bstream.flushesToFile("test3.bin");


	std::cout << "test.bin" << std::endl;
	Bitstream inStream("test.bin");

	int i = 0;
	std::cout << std::dec;
	while (inStream.hasBits())
	{
		bool bit = inStream.readBit();
		std::cout << "Bit [" << i << "] = " << (bit ? "1" : "0") << std::endl;
		//std::cout << "Remaining " << inStream.numberOfRemainingBits() << " of a total of " << inStream.totalSize() << std::endl;
		i++;
	}

	std::cout << "test2.bin" << std::endl;
	Bitstream inStream2("test2.bin");

	i = 0;
	std::cout << std::dec;
	while (inStream2.hasBits())
	{
		bool bit = inStream2.readBit();
		std::cout << "Bit [" << i << "] = " << (bit ? "1" : "0") << std::endl;
		i++;
	}

	std::cout << "test3.bin" << std::endl;
	Bitstream inStream3("test3.bin");

	i = 0;
	std::cout << std::dec;
	while (inStream3.hasBits())
	{
		bool bit = inStream3.readBit();
		std::cout << "Bit [" << i << "] = " << (bit ? "1" : "0") << std::endl;
		i++;
	}
	*/
}

bool checkDecoding(gpcc::EncoderConfigParams *params, gpcc::PointCloud *ppc)
{
	gpcc::PointCloud *ppc2 = new gpcc::PointCloud();
	ppc2->Load(params->getInputFilePath());

	auto vl1 = ppc->getVoxelList();
	auto vl2 = ppc2->getVoxelList();

	sort(vl1.begin(), vl1.end());
	sort(vl2.begin(), vl2.end());

	if (vl1.size() != vl2.size())
	{
		std::cout << "Decoded pc size: " << vl1.size() << " - "
							<< "Input pc size: " << vl2.size() << std::endl;
		return false;
	}

	for (int i = 0; i < vl1.size(); i++)
	{
		if (vl1[i] != vl2[i])
		{

			std::cout << "Different Voxels: "
								<< "("
								<< vl1[i][0]
								<< ","
								<< vl1[i][1]
								<< ","
								<< vl1[i][2]
								<< ")\t"
								<< "("
								<< vl2[i][0]
								<< ","
								<< vl2[i][1]
								<< ","
								<< vl2[i][2]
								<< ")\n";

			return false;
		}
	}

	return true;
}

// std::vector<results> getStandardRateValues(std::vector<std::string> files_path, int n_frame, int nc2D)
// {
// 	gpcc::EncoderConfigParams *pParams = nullptr;
// 	std::vector<results> resultsVector;

// 	// Point cloud is loaded. I should test the axis.
// 	Bitstream *axis_bitstream[3] = {nullptr, nullptr, nullptr};
// 	uint32_t axis_nBits[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
// 	Bitstream *finalBitstream = nullptr;
// 	double axis_time[3] = {0, 0, 0};
// 	double axis_rate_bpov[3] = {0, 0, 0};
// 	bool axis_tested[3] = {false, false, false};
// 	uint16_t currAxis;

// 	std::string name = files_path[n_frame];
// 	gpcc::PointCloud *ppc = new gpcc::PointCloud();
// 	bool pcLoaded = ppc->Load(name);

// 	for (currAxis = gpcc::EncoderConfigParams::X;
// 			 currAxis <= gpcc::EncoderConfigParams::Z; currAxis++)
// 	{

// 		gpcc::Axis axis = ((currAxis == gpcc::EncoderConfigParams::X) ? gpcc::Axis::X : ((currAxis == gpcc::EncoderConfigParams::Y) ? gpcc::Axis::Y : gpcc::Axis::Z));

// 		gpcc::Encoder *enc = new gpcc::Encoder(ppc, pParams, axis);

// 		// TODO: Get these from the config.
// 		uint64_t m = 16;
// 		uint64_t nc3D = 9;

// 		// Initialize cabac
// 		Cabac *cabac = new Cabac();
// 		cabac->initEncoder(m, nc2D, nc3D);
// 		gpcc::encodeReturnData encodingData;
// 		encodingData =
// 				enc->encodePointCloud2(0, cabac);

// 		axis_bitstream[currAxis - 1] =
// 				enc->closeBitstream(encodingData.cabacOut);

// 		axis_nBits[currAxis - 1] = axis_bitstream[currAxis - 1]->totalSize() + 8;

// 		axis_rate_bpov[currAxis - 1] = (double(axis_nBits[currAxis - 1])) / ppc->NumberOccupiedVoxels();

// 		axis_tested[currAxis - 1] = true;

// 		delete enc;
// 	}
// 	// Decides for the best axis.
// 	int bestAxis = (((axis_nBits[0] < axis_nBits[1]) && (axis_nBits[0] < axis_nBits[2])) ? 0 : ((axis_nBits[1] < axis_nBits[2]) ? 1 : 2));

// 	results finalResults = {
// 			axis_rate_bpov[0],
// 			axis_rate_bpov[1],
// 			axis_rate_bpov[2],
// 			axis_rate_bpov[bestAxis],
// 			nc2D,
// 			name};

// 	std::cout << name << "\n";

// 	resultsVector.push_back(finalResults);

// 	delete ppc;

// 	return resultsVector;
// }

// std::vector<results> getBlockModeRateValues(std::vector<std::string> files_path, int n_frame, int k, int nc2D)
// {
// 	gpcc::EncoderConfigParams *pParams = nullptr;
// 	std::vector<results> resultsVector;

// 	// Point cloud is loaded. I should test the axis.
// 	Bitstream *axis_bitstream[3] = {nullptr, nullptr, nullptr};
// 	uint32_t axis_nBits[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
// 	Bitstream *finalBitstream = nullptr;
// 	double axis_time[3] = {0, 0, 0};
// 	double axis_rate_bpov[3] = {0, 0, 0};
// 	bool axis_tested[3] = {false, false, false};
// 	uint16_t currAxis;

// 	std::string name = files_path[n_frame];
// 	gpcc::PointCloud *ppc = new gpcc::PointCloud();
// 	bool pcLoaded = ppc->Load(name);

// 	for (currAxis = gpcc::EncoderConfigParams::X;
// 			 currAxis <= gpcc::EncoderConfigParams::Z; currAxis++)
// 	{

// 		gpcc::Axis axis = ((currAxis == gpcc::EncoderConfigParams::X) ? gpcc::Axis::X : ((currAxis == gpcc::EncoderConfigParams::Y) ? gpcc::Axis::Y : gpcc::Axis::Z));

// 		gpcc::Encoder *enc = new gpcc::Encoder(ppc, pParams, axis, k);

// 		// TODO: Get these from the config.
// 		uint64_t m = 16;
// 		uint64_t nc3D = 9;

// 		// Initialize cabac
// 		Cabac *cabac = new Cabac();
// 		cabac->initEncoder(m, nc2D, nc3D);
// 		gpcc::encodeReturnData encodingData;
// 		encodingData =
// 				enc->encodePointCloud3(0, cabac);

// 		axis_bitstream[currAxis - 1] =
// 				enc->closeBitstream(encodingData.cabacOut);

// 		axis_nBits[currAxis - 1] = axis_bitstream[currAxis - 1]->totalSize() + 8;

// 		axis_rate_bpov[currAxis - 1] = (double(axis_nBits[currAxis - 1])) / ppc->NumberOccupiedVoxels();

// 		axis_tested[currAxis - 1] = true;

// 		delete enc;
// 	}
// 	// Decides for the best axis.
// 	int bestAxis = (((axis_nBits[0] < axis_nBits[1]) && (axis_nBits[0] < axis_nBits[2])) ? 0 : ((axis_nBits[1] < axis_nBits[2]) ? 1 : 2));

// 	results finalResults = {
// 			axis_rate_bpov[0],
// 			axis_rate_bpov[1],
// 			axis_rate_bpov[2],
// 			axis_rate_bpov[bestAxis],
// 			nc2D,
// 			name};

// 	std::cout << name << "\n";

// 	resultsVector.push_back(finalResults);

// 	delete ppc;

// 	return resultsVector;
// }

// std::vector<results> getInvertedModeRateValues(std::vector<std::string> files_path, int n_frame, int k, int nc2D)
// {
// 	gpcc::EncoderConfigParams *pParams = nullptr;
// 	std::vector<results> resultsVector;

// 	// Point cloud is loaded. I should test the axis.
// 	Bitstream *axis_bitstream[3] = {nullptr, nullptr, nullptr};
// 	uint32_t axis_nBits[3] = {UINT32_MAX, UINT32_MAX, UINT32_MAX};
// 	Bitstream *finalBitstream = nullptr;
// 	double axis_time[3] = {0, 0, 0};
// 	double axis_rate_bpov[3] = {0, 0, 0};
// 	bool axis_tested[3] = {false, false, false};
// 	uint16_t currAxis;

// 	std::string name = files_path[n_frame];
// 	gpcc::PointCloud *ppc = new gpcc::PointCloud();
// 	bool pcLoaded = ppc->Load(name);
// 	for (currAxis = gpcc::EncoderConfigParams::X;
// 			 currAxis <= gpcc::EncoderConfigParams::Z; currAxis++)
// 	{

// 		gpcc::Axis axis = ((currAxis == gpcc::EncoderConfigParams::X) ? gpcc::Axis::X : ((currAxis == gpcc::EncoderConfigParams::Y) ? gpcc::Axis::Y : gpcc::Axis::Z));

// 		gpcc::Encoder *enc = new gpcc::Encoder(ppc, pParams, axis, k);

// 		// TODO: Get these from the config.
// 		uint64_t m = 16;
// 		uint64_t nc3D = 9;

// 		// Initialize cabac
// 		Cabac *cabac = new Cabac();
// 		cabac->initEncoder(m, nc2D, nc3D);
// 		gpcc::encodeReturnData encodingData;

// 		axis_bitstream[currAxis - 1] =
// 				enc->encodePointCloud4(0, cabac);

// 		axis_nBits[currAxis - 1] = axis_bitstream[currAxis - 1]->totalSize() + 8;

// 		axis_rate_bpov[currAxis - 1] = (double(axis_nBits[currAxis - 1])) / ppc->NumberOccupiedVoxels();

// 		axis_tested[currAxis - 1] = true;

// 		delete enc;
// 	}
// 	// Decides for the best axis.
// 	int bestAxis = (((axis_nBits[0] < axis_nBits[1]) && (axis_nBits[0] < axis_nBits[2])) ? 0 : ((axis_nBits[1] < axis_nBits[2]) ? 1 : 2));

// 	results finalResults = {
// 			axis_rate_bpov[0],
// 			axis_rate_bpov[1],
// 			axis_rate_bpov[2],
// 			axis_rate_bpov[bestAxis],
// 			nc2D,
// 			name};

// 	resultsVector.push_back(finalResults);

// 	delete ppc;

// 	return resultsVector;
// }

// std::vector<results> getRateValues(std::string mode, std::vector<std::string> files_path, int n_frame, int k, int nc2D)
// {
// 	std::vector<results> resultsVector;
// 	if (mode == "--standard")
// 	{
// 		resultsVector = getStandardRateValues(files_path, n_frame, nc2D);
// 	}
// 	else if (mode == "--block")
// 	{
// 		resultsVector = getBlockModeRateValues(files_path, n_frame, k, nc2D);
// 	}
// 	else if (mode == "--inverted")
// 	{
// 		resultsVector = getInvertedModeRateValues(files_path, n_frame, k, nc2D);
// 	}
// 	else
// 	{
// 		throw std::invalid_argument("Mode argumnt invalid");
// 	}

// 	return resultsVector;
// }

// std::vector<std::string> getFilesFromDirectory(std::string directory_path)
// {
// 	std::vector<std::string> fileNames;
// 	for (const auto &entry : std::filesystem::directory_iterator(directory_path))
// 		fileNames.push_back(entry.path().string());

// 	std::sort(fileNames.begin(), fileNames.end());

// 	return fileNames;
// }

// inline bool fileExists(const std::string &name)
// {
// 	std::ifstream f(name.c_str());
// 	return f.good();
// }

// void exportResultsToCSV(std::string pc_name, std::vector<results> bpovResults, int k)
// {
// 	std::ofstream csvFile;

// 	if (fileExists(pc_name + "_output.csv") == 0)
// 	{
// 		csvFile.open(pc_name + "_output.csv");
// 		csvFile << "x;y;z;best axis;k;nc2D;path\n";
// 	}
// 	else
// 	{
// 		csvFile.open(pc_name + "_output.csv", std::ios_base::app);
// 	}

// 	for (auto r : bpovResults)
// 	{
// 		csvFile << r.rate_x
// 						<< ";"
// 						<< r.rate_y
// 						<< ";"
// 						<< r.rate_z
// 						<< ";"
// 						<< r.best_axis_rate
// 						<< ";"
// 						<< k
// 						<< ";"
// 						<< r.nc2D
// 						<< ";"
// 						<< r.filePath
// 						<< "\n";
// 	}

// 	csvFile.close();
// }

// void getResults(std::string mode, std::string directory, int n_frame, std::string pcName, int k, int nc2D)
// {
// 	std::cout << mode << " " << n_frame << " " << pcName << " " << k << " " << nc2D << std::endl;
// 	std::vector<std::string> filesNames = getFilesFromDirectory(
// 			directory + "/" + pcName +
// 			"/ply");
// 	std::vector<results> bpovResults = getRateValues(mode, filesNames, n_frame, k, nc2D);
// 	exportResultsToCSV(pcName, bpovResults, k);
// }

headerData parseHeaderData(Bitstream &bstream)
{
	headerData returnData;

	returnData.nBits = (uint16_t)bstream.readNumber(6);
	returnData.iAxis = (uint16_t)bstream.readNumber(2);
	returnData.algorithmChoice = (uint16_t)bstream.readNumber(4);

	return returnData;
}