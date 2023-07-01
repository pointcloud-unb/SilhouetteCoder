#include <vector>

#include "en_encoder.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#define ENCODER_VERBOSE 0
#define DISABLE_SINGLE_MODE 1
#define SINGLE_MODE_LEVEL 5 // If single mode is enabled, it will start from this level.

//----------------------------------------
// Constructors
gpcc::Encoder::Encoder(PointCloud *ppc, EncoderConfigParams *params, gpcc::Axis axis, int k)
{
	this->pPC = ppc;
	this->pParams = params;
	this->k = k;
	this->algorithmChoice = this->pParams->getAlgorithmChoice();

	// Set the axis for the current encoder.
	this->axis = axis;

	// Derives the Point Cloud information.
	this->nbits = pPC->getResolution();
	this->imSize = 1 << nbits;
}

gpcc::Encoder::~Encoder()
{
	delete this->pParams;
}

//----------------------------------------
// Public Functions
// void gpcc::Encoder::encodePointCloud()
// {

// 	gpcc::SilhouetteDecomposition silhouette_list;

// 	// Perform the silhouette decomposition list.
// 	int nbits = pPC->getResolution();
// 	uint32_t imSize = 1 << nbits;
// 	silhouette_list = SilhouetteDecomposition(nbits);
// 	silhouette_list.createDyadicDecomposition();

// 	// Prepares an array of images.
// 	int numberOfSilhouettes = silhouette_list.getNumberOfDecompositions();

// #if ENCODER_VERBOSE
// 	std::cout << "Created " << numberOfSilhouettes << " silhouettes." << std::endl;
// #endif
// 	std::vector<gpcc::IImage *> image_list;
// 	image_list.resize(numberOfSilhouettes, nullptr);

// 	gpcc::Silhouette s;
// 	int i;
// 	size_t nPixels;
// 	// For all silhouettes
// 	for (i = 0; i < numberOfSilhouettes; i++)
// 	{
// 		// Get silhouette details
// 		s = silhouette_list.getSilhouette(i);
// 		int32_t currID = s.myID;

// 		// Creates the Silhouette
// 		image_list[currID] = pPC->SilhouetteFromPointCloud(s.iStart, s.iEnd, this->axis);
// 		nPixels = image_list[currID]->NumberOccupiedPixels();

// 		if (nPixels != 0)
// 		{
// 			this->paramBitstream.putBit(true);
// 			uint64_t b1, b2;
// #if ENCODER_VERBOSE

// 			std::cout << "Encoding image " << i << " ... ";
// #endif
// 			// Creates an imageRaster from the ImageSparse
// 			gpcc::ImageRaster *currImage = new ImageRaster((ImageSparse *)image_list[currID], imSize, imSize, 0);

// 			// Does this image have a father?
// 			if (s.fatherID == -1)
// 			{
// 				// Encodes the silhuette
// 				b1 = acbstream->totalSize();
// 				cabac->encodeImage(*currImage, *acbstream);
// 				b2 = acbstream->totalSize();
// 			}
// 			else
// 			{
// 				// Does this image have a left brother?
// 				if (s.leftBrotherID == -1)
// 				{
// 					// Creates a sparse representation of the father.
// 					// If it does NOT have a left brother, it uses the father as mask and encodes the image using just the mask (2D contexts only)
// 					int32_t fatherID = s.fatherID;
// 					gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[fatherID];

// 					b1 = acbstream->totalSize();
// 					cabac->encodeImage(*currImage, *maskImage, *acbstream);
// 					b2 = acbstream->totalSize();
// 				}
// 				else
// 				{
// 					// Is this image a rightBrother?
// 					if (s.isRightBrother == false)
// 					{
// 						// If it is not rightBrother, then the mask image is the father.
// 						int32_t fatherID = s.fatherID;
// 						gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[fatherID];
// 						int32_t leftID = s.leftBrotherID;
// 						gpcc::ImageRaster *leftBrotherImage = new ImageRaster((ImageSparse *)image_list[leftID], imSize, imSize, 0);

// 						b1 = acbstream->totalSize();
// 						cabac->encodeImage(*currImage, *maskImage, *leftBrotherImage, *acbstream);
// 						b2 = acbstream->totalSize();

// 						delete leftBrotherImage;
// 					}
// 					else
// 					{
// 						// if it IS a rightBrother, then the mask image is the left brother.
// 						int32_t leftID = s.leftBrotherID;
// 						gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[leftID];
// 						gpcc::ImageRaster *leftBrotherImage = new ImageRaster((ImageSparse *)image_list[leftID], imSize, imSize, 0);

// 						b1 = acbstream->totalSize();
// 						cabac->encodeImage(*currImage, *maskImage, *leftBrotherImage, *acbstream);
// 						b2 = acbstream->totalSize();

// 						delete leftBrotherImage;
// 					}
// 				}
// 			}

// 			delete currImage;
// #if ENCODER_VERBOSE
// 			std::cout << " : " << (b2 - b1) << " bits." << std::endl;
// #endif
// 		}
// 		else
// 		{
// 			this->paramBitstream.putBit(false);
// #if ENCODER_VERBOSE
// 			std::cout << "Bypassing image " << i << std::endl;
// #endif
// 		}
// 	}

// 	// Delete all images in the list.
// 	for (i = 0; i < numberOfSilhouettes; i++)
// 	{
// 		if (image_list[i] != nullptr)
// 		{
// 			delete image_list[i];
// 		}
// 	}
// }

gpcc::encodeReturnData gpcc::Encoder::encodePointCloud2(uint32_t level, Cabac *cabacIn, uint32_t iStart,
																												uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	// Some parameters (TODO: later change to defines or read from the parameter file)
	bool testDyadicDecompositionMode = true;
	bool testSingleProjectionMode = true;

	// Keep track of the bits spent.
	uint32_t nBitsDyadicMode = UINT32_MAX;
	uint32_t nBitsSingleMode = UINT32_MAX;
	gpcc::encodeReturnData encodingMetaData;
	encodingMetaData.cabacOut = new Cabac();

	uint64_t b1, b2;

	gpcc::ImageRaster *currImage = nullptr;
	gpcc::ImageSparse *maskImage = nullptr;

	Cabac *cabacDyadic = new Cabac();
	Cabac *cabacSingleMode = new Cabac();

#if DISABLE_SINGLE_MODE
#else
	if (level < SINGLE_MODE_LEVEL)
	{
		testDyadicDecompositionMode = true;
		testSingleProjectionMode = false;
	}
	else
	{
		testDyadicDecompositionMode = false;
		testSingleProjectionMode = true;
	}
#endif

	// The first time this function is called there is no silhouette, no parent. It needs to encode the top-level silhouette.
	if (level == 0)
	{
		// Check the size
		iStart = 0;
		iEnd = (1 << this->nbits) - 1;

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
				// Creates an imageRaster from the ImageSparse
				currImage = leftChildRaster;

				// Does this image have a left brother?
				if (lStart == 0)
				{
					// If it does NOT have a left brother, it uses the father as mask and encodes the image using just the mask (2D contexts only)
					maskImage = fatherSilhouette;

					b1 = cabacDyadic->acbstream->totalSize();
					cabacDyadic->encodeImage(*currImage, *maskImage);
					b2 = cabacDyadic->acbstream->totalSize();
				}
				else
				{
					// If it does have a left brother:
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
			}

			if (encodeRight)
			{
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
			}
			delete leftChildRaster;
		}

		// Checks the left branch.
		// This can be inferred at the decoder, no need to signal this in the bitstream.
		// std::cout << "level = " << level << " - nbits = " << this->nbits << " ." << std::endl;
		// std::cout << "nInterval = " << nInterval << " ." << std::endl;
		if (encodeLeft && (level < (this->nbits - 1)))
		{
			encodeReturnData leftData = encodePointCloud2(level + 1, cabacDyadic, lStart, lEnd, leftChild);
			cabacDyadic->copyCabac(leftData.cabacOut);
			nBitsDyadicMode += leftData.encodingbitsSize;
			leftData.cabacOut->~Cabac();
		}

		// Checks the rights branch.
		// This can be inferred at the decoder, no need to signal this in the bitstream.
		if (encodeRight && (level < (this->nbits - 1)))
		{
			encodeReturnData rightData = encodePointCloud2(level + 1, cabacDyadic, rStart, rEnd, rightChild);
			cabacDyadic->copyCabac(rightData.cabacOut);
			nBitsDyadicMode += rightData.encodingbitsSize;
			rightData.cabacOut->~Cabac();
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

		encodeReturnData singleModeData = this->encodeIntervalSingleMode(cabacSingleMode, iStart, iEnd, fatherSilhouette);
		cabacSingleMode->copyCabac(singleModeData.cabacOut);
		singleModeData.cabacOut->~Cabac();
		nBitsSingleMode += singleModeData.encodingbitsSize;
	}

	if (level == 0)
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

	return encodingMetaData;
}

/* HEADER
 *  6 bits -> nBits of the point cloud.
 *  2 bits -> axis (00 X axis, 01 Y axis, 10 Z axis)
 * 16 bits -> length of the bitstream parameters.
 */
void gpcc::Encoder::writeHeader(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint64_t lengthBitstreamParam, Bitstream &bstream)
{
	bstream.writeNumber(nBits, 6);
	bstream.writeNumber(axis, 2);
	bstream.writeNumber(algorithmChoice, 4);
	bstream.writeNumber(lengthBitstreamParam, 16);
}

void gpcc::Encoder::writeHeader2(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint64_t lengthBitstreamParam, uint64_t k, Bitstream &bstream)
{
	bstream.writeNumber(nBits, 6);
	bstream.writeNumber(axis, 2);
	bstream.writeNumber(algorithmChoice, 4);
	bstream.writeNumber(lengthBitstreamParam, 16);
	bstream.writeNumber(k, 8);
}

Bitstream *gpcc::Encoder::closeBitstream(Cabac *cabac)
{
	Bitstream *finalBitstream = new Bitstream();

	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc1D = 4;
	Bitstream paramBitstream = cabac->paramBitstream->encode(m, nc1D);

	uint16_t nBits = (uint16_t)this->nbits;
	uint32_t k = this->k;
	uint16_t axis = ((this->axis == gpcc::Axis::X) ? 0 : ((this->axis == gpcc::Axis::Y) ? 1 : 2));

	uint64_t nBitsParamBitstream = paramBitstream.totalSize();
	// this->writeHeader(nBits, axis, nBitsParamBitstream, *finalBitstream);
	this->writeHeader2(nBits, axis, this->algorithmChoice, nBitsParamBitstream, k, *finalBitstream);
	uint64_t nBitsHeader = finalBitstream->totalSize();

	// Merge the paramBitstream with the final bitstream.
	finalBitstream->merge(paramBitstream);

	// Closes the AC Payload bitstream and merges to the final bitstream.
	cabac->closeEncoder(*(cabac->acbstream));
	finalBitstream->merge(*(cabac->acbstream));

	uint64_t nBitsAC = cabac->acbstream->totalSize();

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

Bitstream *gpcc::Encoder::closeBitstream2(Cabac *cabac1, Cabac *cabac2)
{
	Bitstream *finalBitstream = new Bitstream();

	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc1D = 4;
	Bitstream paramBitstream1 = cabac1->paramBitstream->encode(m, nc1D);
	Bitstream paramBitstream2 = cabac2->paramBitstream->encode(m, nc1D);

	uint16_t nBits = (uint16_t)this->nbits;
	uint32_t k = this->k;
	uint16_t axis = ((this->axis == gpcc::Axis::X) ? 0 : ((this->axis == gpcc::Axis::Y) ? 1 : 2));

	uint64_t nBitsParamBitstream = paramBitstream1.totalSize();
	// this->writeHeader(nBits, axis, nBitsParamBitstream, *finalBitstream);
	this->writeHeader2(nBits, axis, this->algorithmChoice, nBitsParamBitstream, k, *finalBitstream);
	uint64_t nBitsHeader = finalBitstream->totalSize();

	// Merge the paramBitstream with the final bitstream.
	finalBitstream->merge(paramBitstream2);
	cabac2->closeEncoder(*(cabac2->acbstream));
	finalBitstream->merge(*(cabac2->acbstream));

	// Closes the AC Payload bitstream and merges to the final bitstream.
	finalBitstream->merge(paramBitstream1);
	cabac1->closeEncoder(*(cabac1->acbstream));
	finalBitstream->merge(*(cabac1->acbstream));

	uint64_t nBitsAC = cabac1->acbstream->totalSize() + cabac2->acbstream->totalSize();

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

gpcc::encodeReturnData gpcc::Encoder::encodeIntervalSingleMode(Cabac *cabacIn,
																															 uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	// TODO:
	//  Test if I should add a new table for 3Dcontexts (it seems this is not the case)
	uint32_t encodingbitsSize = 0;
	encodeReturnData encodingMetaData;
	gpcc::ImageSparse *leftImageSparse = nullptr;
	gpcc::ImageRaster *leftImageRaster = nullptr;
	gpcc::ImageSparse *currImageSparse = nullptr;
	gpcc::ImageRaster *currImageRaster = nullptr;

	// Creates an empty last mask.
	gpcc::ImageSparse *lastMask = new gpcc::ImageSparse();

	if (iStart == 0)
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
			if (k != iEnd)
			{
				// cabacIn->printStatusEncoder();
				cabacIn->encodeImage(*currImageRaster, *fatherSilhouette, *leftImageRaster);
				// cabacIn->printStatusEncoder();
			}
			else
			{
				// cabacIn->printStatusEncoder();
				cabacIn->encodeImage(*currImageRaster, *lastMask, *leftImageRaster);
				// cabacIn->printStatusEncoder();
			}
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
