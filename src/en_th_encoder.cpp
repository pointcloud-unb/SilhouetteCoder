#include <vector>
#include <cmath>

#include "en_th_encoder.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#include "cg_config.h"

// #define ENCODER_VERBOSE 0
#define DISABLE_SINGLE_MODE 1
#define SINGLE_MODE_LEVEL 5 // If single mode is enabled, it will start from this level.

//----------------------------------------
// Constructors
gpcc::EncoderThread::EncoderThread(PointCloud *ppc, EncoderConfigParams *params, gpcc::Axis axis, uint32_t nParallelism) : acbstream(nParallelism, nullptr)
{
	this->pPC = ppc;
	this->pParams = params;
	this->nBitsAC = 0;

	// Set the axis for the current encoder.
	this->axis = axis;

	// Derives the Point Cloud information.
	this->nbits = pPC->getResolution();
	this->imSize = 1 << nbits;

	this->nParallelism = nParallelism;

	// Creates the internal bitstreams.
	for (auto &bitstream : acbstream)
		bitstream = new Bitstream();

	for (int i = 0; i < nParallelism; i++)
	{
		uint32_t blockSize = (1 << this->nbits) / this->nParallelism;
		uint32_t iStart = i * blockSize;
		this->leftBounds.push_back(iStart);
		// std::cout << "iStart:" << iStart << std::endl;
	}

	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc2D = 6;
	uint64_t nc3D = 9;

	// Initialize cabac
	cabac.initEncoder(m, nc2D, nc3D);

	// Initialize the parameter bitstream
	paramBitstream = CodecParameterBitstream();
}

gpcc::EncoderThread::~EncoderThread()
{
	for (auto &bitstream : acbstream)
		if (bitstream != nullptr)
			delete bitstream;

	delete this->pParams;
}

//----------------------------------------
// Public Functions
gpcc::encodeThreadReturnData gpcc::EncoderThread::encodePointCloud(uint32_t npointcloud, uint32_t level, Cabac *cabacIn, uint32_t iStart,
																																	 uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	// Some parameters (TODO: later change to defines or read from the parameter file)
	bool testDyadicDecompositionMode = true;
	bool testSingleProjectionMode = true;

	// Keep track of the bits spent.
	uint32_t nBitsDyadicMode = UINT32_MAX;
	uint32_t nBitsSingleMode = UINT32_MAX;
	gpcc::encodeThreadReturnData encodingMetaData;
	encodingMetaData.cabacOut = new Cabac();

	uint64_t b1, b2;

	gpcc::ImageRaster *currImage = nullptr;
	gpcc::ImageSparse *maskImage = nullptr;

	Cabac *cabacDyadic = new Cabac();
	Cabac *cabacSingleMode = new Cabac();

	// The first time this function is called there is no silhouette, no parent. It needs to encode the top-level silhouette.
	if (level == log2(this->nParallelism))
	{
		fatherSilhouette = this->pPC->SilhouetteFromPointCloud(iStart, iEnd, this->axis);
		size_t nPixels = fatherSilhouette->NumberOccupiedPixels();

		if (nPixels != 0)
		{
			// Writes a 1 in the parameter bitstream to indicate that this image will be encoded.
			cabacIn->paramBitstream->putBit(true);

			// Creates an imageRaster from the ImageSparse
			currImage = new ImageRaster((ImageSparse *)fatherSilhouette, this->imSize, this->imSize, 0);

			// Encodes the silhuette

			b1 = cabacIn->acbstream->totalSize();
			cabacIn->encodeImage(*currImage);
			b2 = cabacIn->acbstream->totalSize();
			// Deletes the imageRaster object (it will not be needed anymore.
			delete currImage;
			currImage = nullptr;

			// std::cout << "Silhouette(" << iStart << "," << iEnd << ") = " << (b2 - b1) << " bits." << std::endl;
#if ENCODER_VERBOSE
			std::cout << "Silhouette(" << iStart << "," << iEnd << ") = " << (b2 - b1) << " bits." << std::endl;
#endif
		}
	}

	if (testDyadicDecompositionMode)
	{

		cabacDyadic->copyCabac(cabacIn);
		nBitsDyadicMode = 0;

		// Writes that it is using the dyadic mode.
		cabacDyadic->paramBitstream->putBit(false); // Dyadic mode: 0 - Single mode: 1
		nBitsDyadicMode++;

		uint32_t lStart, lEnd, rStart, rEnd, nInterval;
		lStart = iStart;
		lEnd = iStart + ((iEnd - iStart) >> 1);

		rStart = lEnd + 1;
		rEnd = iEnd;

		nInterval = lEnd - lStart + 1;

		// Creates both silhouettes.
		gpcc::ImageSparse *leftChild = this->pPC->SilhouetteFromPointCloud(lStart, lEnd, this->axis);
		gpcc::ImageSparse *rightChild = this->pPC->SilhouetteFromPointCloud(rStart, rEnd, this->axis);

		// Counts the number of pixels
		size_t nPixelsLeft = leftChild->NumberOccupiedPixels();
		size_t nPixelsRight = rightChild->NumberOccupiedPixels();

		bool encodeLeft = (nPixelsLeft > 0);
		bool encodeRight = (nPixelsRight > 0);

		// Writes this info in the param bitstream:
		cabacDyadic->paramBitstream->putBit(encodeLeft);
		cabacDyadic->paramBitstream->putBit(encodeRight);
		nBitsDyadicMode += 2;

		// std::cout << "encodeLeft  = " << encodeLeft  << " | Silhouette(" << lStart << "," << lEnd << ") = | nPixelsLeft:  " << nPixelsLeft << " ." << std::endl;
		// std::cout << "encodeRight = " << encodeRight << " | Silhouette(" << rStart << "," << rEnd << ") = | nPixelsRight: " << nPixelsRight << " ." << std::endl;

		// These silhouettes will only be encoded if BOTH have pixels (otherwise, I know how to decode it.
		if (encodeLeft && encodeRight)
		{
			gpcc::ImageRaster *leftChildRaster = new ImageRaster(leftChild, this->imSize, this->imSize, 0);
			if (encodeLeft)
			{
				// if (npointcloud == 1)
				// {
				// 	std::cout << "Encode Left - "
				// 						<< "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// 	cabacDyadic->printStatusEncoder();
				// }
				// Creates an imageRaster from the ImageSparse
				currImage = leftChildRaster;

				// Does this image have a left brother?
				// std::cout << "lStart: " << lStart << " leftBound: " << this->leftBounds[npointcloud] << std::endl;
				if (lStart == this->leftBounds[npointcloud])
				{

					// std::cout << "Without left mask" << std::endl;
					//  If it does NOT have a left brother, it uses the father as mask and encodes the image using just the mask (2D contexts only)
					maskImage = fatherSilhouette;

					b1 = cabacDyadic->acbstream->totalSize();
					cabacDyadic->encodeImage(*currImage, *maskImage);
					b2 = cabacDyadic->acbstream->totalSize();
				}
				else
				{
					// std::cout << "With left mask" << std::endl;
					//  If it does have a left brother:
					maskImage = fatherSilhouette;

					// Create the silhouette for the leftBrother
					gpcc::ImageSparse *leftBrother = this->pPC->SilhouetteFromPointCloud(lStart - nInterval, lEnd - nInterval, this->axis);
					gpcc::ImageRaster *leftBrotherImage = new ImageRaster(leftBrother, this->imSize, this->imSize, 0);
					b1 = cabacDyadic->acbstream->totalSize();
					cabacDyadic->encodeImage(*currImage, *maskImage, *leftBrotherImage);
					b2 = cabacDyadic->acbstream->totalSize();

					delete leftBrotherImage;
					delete leftBrother;
				}

				nBitsDyadicMode += b2 - b1;

				currImage = nullptr;
				maskImage = nullptr;

#if ENCODER_VERBOSE
				std::cout << "Silhouette(" << lStart << "," << lEnd << ") = " << (b2 - b1) << " bits." << std::endl;
#endif

				// if (npointcloud == 1)
				// {

				// 	cabacDyadic->printStatusEncoder();
				// }
			}

			if (encodeRight)
			{

				// if (npointcloud == 1)
				// {
				// 	std::cout << "Encode Right - "
				// 						<< "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// 	cabacDyadic->printStatusEncoder();
				// }

				// Creates an imageRaster from the ImageSparse
				currImage = new ImageRaster(rightChild, this->imSize, this->imSize, 0);

				// The right image for sure has a rightBrother.
				// if it IS a rightBrother, then the mask image is the left brother.
				maskImage = leftChild;

				b1 = cabacDyadic->acbstream->totalSize();
				cabacDyadic->encodeImage(*currImage, *maskImage, *leftChildRaster);
				b2 = cabacDyadic->acbstream->totalSize();
				delete currImage;

				currImage = nullptr;
				maskImage = nullptr;

#if ENCODER_VERBOSE
				std::cout << "Silhouette(" << rStart << "," << rEnd << ") = " << (b2 - b1) << " bits." << std::endl;
#endif
				nBitsDyadicMode += b2 - b1;

				// if (npointcloud == 1)
				// {

				// 	cabacDyadic->printStatusEncoder();
				// }
			}
			delete leftChildRaster;
		}

		// Checks the left branch.
		// This can be inferred at the decoder, no need to signal this in the bitstream.
		// std::cout << "level = " << level << " - nbits = " << this->nbits << " ." << std::endl;
		// std::cout << "nInterval = " << nInterval << " ." << std::endl;
		if (encodeLeft && (level < (this->nbits - 1)))
		{
			// std::cout << "lStart: " << lStart << " lEnd: " << lEnd << std::endl;

			encodeThreadReturnData leftData = this->encodePointCloud(npointcloud, level + 1, cabacDyadic, lStart, lEnd, leftChild);
			cabacDyadic->copyCabac(leftData.cabacOut);
			leftData.cabacOut->~Cabac();
			nBitsDyadicMode += leftData.encodingbitsSize;
		}

		// Checks the rights branch.
		// This can be inferred at the decoder, no need to signal this in the bitstream.
		if (encodeRight && (level < (this->nbits - 1)))
		{
			// std::cout << "rStart: " << rStart << " rEnd: " << rEnd << std::endl;

			encodeThreadReturnData rightData = this->encodePointCloud(npointcloud, level + 1, cabacDyadic, rStart, rEnd, rightChild);
			cabacDyadic->copyCabac(rightData.cabacOut);
			rightData.cabacOut->~Cabac();
			nBitsDyadicMode += rightData.encodingbitsSize;
		}

		delete leftChild;
		delete rightChild;
		leftChild = nullptr;
		rightChild = nullptr;
	}

	if (testSingleProjectionMode)
	{
		cabacSingleMode->copyCabac(cabacIn);
		nBitsSingleMode = 0;

		// Writes that it is using the single mode.
		cabacSingleMode->paramBitstream->putBit(true); // Dyadic mode: 0 - Single mode: 1
		nBitsSingleMode++;

		encodeThreadReturnData singleModeData = this->encodeIntervalSingleMode(cabacSingleMode, iStart, iEnd, fatherSilhouette, npointcloud);
		cabacSingleMode->copyCabac(singleModeData.cabacOut);
		singleModeData.cabacOut->~Cabac();
		nBitsSingleMode += singleModeData.encodingbitsSize;
	}

	if (level == log2(this->nParallelism))
	{
		delete fatherSilhouette;
		fatherSilhouette = nullptr;
	}

	if (nBitsDyadicMode <= nBitsSingleMode)
	{
		encodingMetaData.cabacOut->copyCabac(cabacDyadic);
		encodingMetaData.encodingbitsSize = nBitsDyadicMode;
	}
	else
	{
		encodingMetaData.cabacOut->copyCabac(cabacSingleMode);
		encodingMetaData.encodingbitsSize = nBitsSingleMode;
	}

	cabacSingleMode->~Cabac();
	cabacDyadic->~Cabac();
	if (level == log2(this->nParallelism))
	{
		// TODO: Get these from the config.
		// TODO: Get these from the config.
		uint64_t m = 16;
		uint64_t nc2D = 6;
		uint64_t nc3D = 9;
		uint64_t nc1D = 4;
		uint32_t length_param = encodingMetaData.cabacOut->paramBitstream->totalSize();
		Bitstream paramBitstream = encodingMetaData.cabacOut->paramBitstream->encode(m, nc1D);
		uint32_t length_encoded_param = paramBitstream.totalSize();

		// The bitstream for this part:
		//  - 16 bits for the lenght of the encoded parambitstream.
		//  - 24 bits for the AC payload
		this->acbstream[npointcloud]->writeNumber(length_encoded_param, 16);

		// Closes the AC Payload bitstream and merge in final bitstream
		encodingMetaData.cabacOut->closeEncoder(*encodingMetaData.cabacOut->acbstream);

		uint32_t length_acbstream = encodingMetaData.cabacOut->acbstream->totalSize();
		this->acbstream[npointcloud]->writeNumber(length_acbstream, 24);

		// Merge the paramBitstream with the final bitstream.
		this->acbstream[npointcloud]->merge(paramBitstream);
		this->acbstream[npointcloud]->merge(*encodingMetaData.cabacOut->acbstream);

		// Write data to compute the total rate
		this->nBitsHeader += 40;
		this->nBitsParamBitstream += length_encoded_param;
		this->nBitsAC += length_acbstream;

		// Compute number of voxels
		uint32_t nVoxels_in_this_part = pPC->countSliceVoxels(iStart, iEnd, this->axis);

		// DEBUG
		// std::cout << "Part: " << iStart << "..." << iEnd << " : " << nVoxels_in_this_part << " voxels." << std::endl;
		// std::cout << "LUMA_E(" << npointcloud << "): " << length_encoded_param << " (" << length_param << ") : " << length_acbstream << std::endl;
	}

	return encodingMetaData;
}

/* HEADER
 *  6 bits -> nBits of the point cloud.
 *  2 bits -> axis (00 X axis, 01 Y axis, 10 Z axis)
 * 3 bits -> nParallelism
 * 16 bits -> length of the bitstream parameters.
 */
void gpcc::EncoderThread::writeHeader(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint16_t nParallelism, Bitstream &bstream)
{
	bstream.writeNumber(nBits, 6);
	bstream.writeNumber(axis, 2);
	bstream.writeNumber(algorithmChoice, 4);
	bstream.writeNumber(nParallelism, 4);
}

Bitstream *gpcc::EncoderThread::closeBitstream()
{
	Bitstream *finalBitstream = new Bitstream();

	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc1D = 4;
	Bitstream paramBitstream = this->paramBitstream.encode(m, nc1D);

	uint16_t nBits = (uint16_t)this->nbits;
	uint16_t axis = ((this->axis == gpcc::Axis::X) ? 0 : ((this->axis == gpcc::Axis::Y) ? 1 : 2));

	this->writeHeader(nBits, axis, EncoderConfigParams::DD_SM_THREAD, log2((uint16_t)this->nParallelism), *finalBitstream);
	this->nBitsHeader += finalBitstream->totalSize();

	for (auto &bitstream : acbstream)
	{
		finalBitstream->merge(*bitstream);
		this->nBitsAC += bitstream->totalSize();
	}
	/*
	std::cout << "Final Status" << std::endl;
	std::cout << "Number of Bits: " << std::endl;
	std::cout << "  - HEADER     : " << nBitsHeader << std::endl;
	std::cout << "  - PARAMS     : " << nBitsParamBitstream << std::endl;
	std::cout << "  - AC Payload : " << nBitsAC << std::endl;
	std::cout << "         TOTAL : " << finalBitstream->totalSize() << std::endl;
	*/

	// double rate_bpov = (double(finalBitstream->totalSize() + 8)) / pPC->NumberOccupiedVoxels();
	// std::cout << "   RATE " << ((this->axis == gpcc::Axis::X) ? "X" : ((this->axis == gpcc::Axis::Y) ? "Y" : "Z")) << " Axis : " << rate_bpov << " bits per occupied voxel." << std::endl;
	return finalBitstream;
}

gpcc::encodeThreadReturnData gpcc::EncoderThread::encodeIntervalSingleMode(Cabac *cabacIn,
																																					 uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette, uint32_t npointcloud)
{
	// TODO:
	//  Test if I should add a new table for 3Dcontexts (it seems this is not the case)
	uint32_t encodingbitsSize = 0;
	encodeThreadReturnData encodingMetaData;
	gpcc::ImageSparse *leftImageSparse = nullptr;
	gpcc::ImageRaster *leftImageRaster = nullptr;
	gpcc::ImageSparse *currImageSparse = nullptr;
	gpcc::ImageRaster *currImageRaster = nullptr;

	// Creates an empty last mask.
	gpcc::ImageSparse *lastMask = new gpcc::ImageSparse();

	if (iStart == this->leftBounds[npointcloud])
	{
		leftImageSparse = new gpcc::ImageSparse();
	}
	else
	{
		leftImageSparse = this->pPC->SilhouetteFromPointCloud(iStart - 1, iStart - 1, this->axis);
	}

	// Make a raster object of this image.
	leftImageRaster = new ImageRaster(leftImageSparse, this->imSize, this->imSize, 0);

	// delete the leftImageSparse, as it will not be used anymore.
	delete leftImageSparse;
	leftImageSparse = nullptr;

	uint32_t k;
	uint64_t b1, b2;

	for (k = iStart; k <= iEnd; k++)
	{
		// Creaters the current silhouette.
		currImageSparse = this->pPC->SilhouetteFromPointCloud(k, k, this->axis);
		currImageRaster = new ImageRaster(currImageSparse, this->imSize, this->imSize, 0);

		size_t nPixels = currImageSparse->NumberOccupiedPixels();

		// Adds the pixels to the last mask.
		if (k != iEnd)
		{
			lastMask->addPixels(currImageSparse->getLocation());
		}

		// Deletes the current imageSparse as it will not be used anymore.
		delete currImageSparse;
		currImageSparse = nullptr;

		bool encodeThisSilhouette = (nPixels > 0);
		cabacIn->paramBitstream->putBit(encodeThisSilhouette);
		encodingbitsSize++;

		uint32_t nVoxelsInSilhouette = this->pPC->countSliceVoxels(k, k, this->axis);

		if (encodeThisSilhouette)
		{
			// Encodes the current silhouette.
			b1 = cabacIn->acbstream->totalSize();
			// std::cout << k << ":"
			// 					<< "\n";
			// cabacIn->printStatusEncoder();
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusEncoder();
			if (k != iEnd)
			{
				cabacIn->encodeImage(*currImageRaster, *fatherSilhouette, *leftImageRaster);
			}
			else
			{
				cabacIn->encodeImage(*currImageRaster, *lastMask, *leftImageRaster);
			}
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusEncoder();
			// cabacIn->printStatusEncoder();
			b2 = cabacIn->acbstream->totalSize();
			encodingbitsSize += b2 - b1;

			// std::cout << "        Leaf Silhouette(" << k << ") - " << (nPixels) << " pixels - " << (nVoxelsInSilhouette) << " voxels - " << (b2 - b1) << " bits." << std::endl;

#if ENCODER_VERBOSE
			std::cout << "Silhouette(" << k << "," << k << ") = " << (b2 - b1) << " bits." << std::endl;
#endif
		}
		else
		{
			b2 = 1;
			b1 = 0;
			// std::cout << "        Leaf Silhouette(" << k << ") - " << (nPixels) << " pixels - " << (nVoxelsInSilhouette) << " voxels - " << (b2 - b1) << " bits." << std::endl;
#if ENCODER_VERBOSE
			std::cout << "Silhouette(" << k << "," << k << ") = " << 0 << " bits." << std::endl;
#endif
		}
		// Deletes the old leftImageRaster
		delete leftImageRaster;
		// Updates the leftImageRaster
		leftImageRaster = currImageRaster;
		currImageRaster = nullptr;
	}

	// Deletes the leftImageRaster.
	delete leftImageRaster;
	delete lastMask;

	encodingMetaData.cabacOut = new Cabac();
	encodingMetaData.cabacOut->copyCabac(cabacIn);
	encodingMetaData.encodingbitsSize = encodingbitsSize;

	return encodingMetaData;
}

uint32_t gpcc::EncoderThread::getResolution()
{
	return this->nbits;
}