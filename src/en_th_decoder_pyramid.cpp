#include <vector>

#include "en_th_decoder_pyramid.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#define DecoderThreadPyramid_VERBOSE 0

//----------------------------------------
// Constructors
gpcc::DecoderThreadPyramid::DecoderThreadPyramid(Bitstream &pbstream, EncoderConfigParams *params) : nBits(0),
																																																		 axis(gpcc::Axis::X),
																																																		 acbstream()
{
	this->pParams = params;

	// Parses the header.
	this->parseHeader(pbstream);

	// Root silhouette info
	this->pPC.push_back(new gpcc::PointCloud());
	this->leftBounds.push_back(0);

	// Root Silhouette doesn't need father mask
	this->fatherSilhouettes.push_back(nullptr);

	// Subtrees codes
	for (int i = 0; i < this->nParallelism; i++)
	{
		this->pPC.push_back(new gpcc::PointCloud());
		uint32_t blockSize = (1 << this->nBits) / this->nParallelism;
		uint32_t iStart = i * blockSize;
		this->leftBounds.push_back(iStart);
		//std::cout << "iStart:" << iStart << std::endl;
	}
}

gpcc::DecoderThreadPyramid::~DecoderThreadPyramid()
{
	for (auto &bitstream : this->acbstream)
		if (bitstream != nullptr)
			delete bitstream;
}

//----------------------------------------
// Private Functions
// This function reads the header, and splits
void gpcc::DecoderThreadPyramid::parseHeader(Bitstream &bstream)
{
	this->nBits = (uint16_t)bstream.readNumber(6);
	uint16_t iAxis = (uint16_t)bstream.readNumber(2);
	uint16_t algorithmChoice = (uint16_t)bstream.readNumber(4);
	this->nParallelism = (uint16_t)bstream.readNumber(4);
	this->nParallelism = 1 << this->nParallelism;
	this->axis = ((iAxis == 0) ? gpcc::Axis::X : ((iAxis == 1) ? gpcc::Axis::Y : gpcc::Axis::Z));

	uint16_t nPointClouds = this->nParallelism + 1;

	this->isEncodedSubTree.reserve(nPointClouds);
	this->paramBitstream.reserve(nPointClouds);
	this->acbstream.reserve(nPointClouds);

	for (uint16_t i = 0; i < nPointClouds; i++)
	{

		bool isEncodedThisSubtree;
		if (i == 0)
		{
			isEncodedThisSubtree = true;
		}
		else
		{		
			isEncodedThisSubtree = bstream.readBit();
		}
		this->isEncodedSubTree.push_back(isEncodedThisSubtree);

		if (isEncodedThisSubtree)
		{

			uint64_t length_encoded_param = bstream.readNumber(16);
			uint64_t length_acbstream = bstream.readNumber(24);

			// Parse the codecparameter bitstream.
			// The first N bits are consumed and a new bitstream is created.
			Bitstream paramBitstream = Bitstream(bstream, length_encoded_param);

			// Initialize the parameter bitstream
			// TODO: Get these from the config.
			uint64_t m = 16;
			uint64_t nc1D = 4;
			this->paramBitstream.push_back(CodecParameterBitstream(paramBitstream, m, nc1D));
			uint32_t length_param = this->paramBitstream[i].totalSize();

			// Assign the rest of the bitstream.
			this->acbstream.push_back(new Bitstream(bstream, length_acbstream));

			//std::cout << "LUMA_D(" << i << "): " << length_encoded_param << " (" << length_param << ") : " << length_acbstream << std::endl;
#if DecoderThreadPyramid_VERBOSE
			std::cout << "Inside DecoderThreadPyramid" << std::endl;
			std::cout << "nBits = " << this->nBits << std::endl;
			std::cout << "AlgorithmChoice  = " << algorithmChoice << std::endl;
			std::cout << "axis  = " << this->axis << std::endl;
			std::cout << "Symbols in the CodecParameterBitstream = " << this->paramBitstream.totalSize() << std::endl;
			std::cout << "AC Payload = " << acbstream->numberOfRemainingBits() << std::endl;
#endif
		}
		else
		{

			this->paramBitstream.push_back(CodecParameterBitstream());
			this->acbstream.push_back(nullptr);
		}
	}
}

//----------------------------------------
// Public Functions

/*
 * LUMA
 */
uint32_t gpcc::DecoderThreadPyramid::getNumberOfPointClouds()
{
	return this->nParallelism;
}

uint16_t gpcc::DecoderThreadPyramid::getPCResolution()
{
	return this->nBits;
}

/*
 * This is the recursive version.
 * It should work with both dyadic and single modes.
 * It is faster and has a better compression performance compared to the non-recursive version.
 */
void gpcc::DecoderThreadPyramid::decodePointCloudRoot2()
{
	Cabac *decoderThreadPyramidCabac = new Cabac();
	uint64_t m = 16;
	uint64_t nc2D = 6;
	uint64_t nc3D = 9;
		
	// std::cout << "LUMA(" << npointcloud << "): " << this->acbstream[npointcloud]->numberOfRemainingBits() << std::endl;
	decoderThreadPyramidCabac->initDecoder(m, nc2D, nc3D, *(this->acbstream[0]));

	this->imSize = 1 << this->nBits;

	// Calls the recursive decoding function
	this->decodeRootSilhouette(decoderThreadPyramidCabac, 0);
	decoderThreadPyramidCabac->~Cabac();

	// std::cout << "Decoded " << this->pPC[npointcloud]->NumberOccupiedVoxels() << " voxels." << std::endl;

	return;
}

void gpcc::DecoderThreadPyramid::decodePointCloud2(uint32_t npointcloud,
																									 uint32_t iStart, uint32_t iEnd)
{
	Cabac *decoderThreadPyramidCabac = new Cabac();
	uint64_t m = 16;
	uint64_t nc2D = 6;
	uint64_t nc3D = 9;
	//std::cout << "LUMA(" << npointcloud << "): " << this->acbstream[npointcloud]->numberOfRemainingBits() << std::endl;
	
	//If the acbitstream is too small then it is probably made only of zeros, so I just skip it
	//Probably not the best option, though.
	//if (this->acbstream[npointcloud]->numberOfRemainingBits() >= m)
	if (this->isEncodedSubTree[npointcloud] == true)
	{
		decoderThreadPyramidCabac->initDecoder(m, nc2D, nc3D, *(this->acbstream[npointcloud]));

		this->imSize = 1 << this->nBits;

		// Calls the recursive decoding function
		this->xDecodePointCloud(npointcloud, decoderThreadPyramidCabac, log2(this->nParallelism), iStart, iEnd);
	}
	decoderThreadPyramidCabac->~Cabac();

	// std::cout << "Decoded " << this->pPC[npointcloud]->NumberOccupiedVoxels() << " voxels." << std::endl;

	return;
}

//----------------------------------------
// Private Functions
/*
 * This is the real recursive function.
 */
void gpcc::DecoderThreadPyramid::xDecodePointCloud(
		uint32_t npointcloud, Cabac *cabacIn, uint32_t level,
		uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	gpcc::ImageRaster *currImage = nullptr;
	gpcc::ImageSparse *maskImage = nullptr;

	bool currBit;

	// The first time this function is called there is no silhouette, no parent.
	// It needs to encode the top-level silhouette.
	if (level == log2(this->nParallelism))
	{
		// Create a sparse representation for this image and adds it to the silhouette list.
		fatherSilhouette = this->fatherSilhouettes[npointcloud];
	}

	// Reads another bit from the parameter that signals the dyadic / single mode
	currBit = this->paramBitstream[npointcloud].readBit();
	// if it is dyadic mode
	if (currBit == false)
	{
		// Decodes the dyadic decomposition.
		uint32_t lStart, lEnd, rStart, rEnd, nInterval;
		lStart = iStart;
		lEnd = iStart + ((iEnd - iStart) >> 1);

		rStart = lEnd + 1;
		rEnd = iEnd;

		// std::cout << "GENERATED:\n";
		// std::cout << "level:" << level << "\n";
		// std::cout << "lStart: " << lStart << " lEnd: " << lEnd << std::endl;
		// std::cout << "rStart: " << rStart << " rEnd: " << rEnd << std::endl;

		nInterval = lEnd - lStart + 1;

		// Creates both silhouettes.
		gpcc::ImageSparse *leftChild = nullptr;
		gpcc::ImageSparse *rightChild = nullptr;

		// Reads two bits from the parameter bitstream.
		bool encodeLeft = this->paramBitstream[npointcloud].readBit();
		bool encodeRight = this->paramBitstream[npointcloud].readBit();
		// std::cout << "encodeLeft: " << encodeLeft << " encodeRight: " << encodeRight << std::endl;

		// This level was encoded only if it had pixels in both images.
		if (encodeLeft && encodeRight)
		{
			gpcc::ImageRaster *leftChildRaster = nullptr;

			// Creates an empty image for the left brother
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// Does this image have a left brother?

			if (lStart == this->leftBounds[npointcloud])
			{
				// std::cout << "Without left mask" << std::endl;

				// if (npointcloud == 1)
				// {
				// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// cabacIn->printStatusDecoder();
				// }

				// If it does NOT have a left brother, it uses the father as mask and decodes the image using just the mask (2D contexts only)
				maskImage = fatherSilhouette;

				cabacIn->decodeImage(*currImage, *maskImage);

				// if (npointcloud == 1)
				// {
				// 	cabacIn->printStatusDecoder();
				// }
			}
			else
			{
				// std::cout << "With left mask" << std::endl;
				// if (npointcloud == 1)
				// {
				// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// 	cabacIn->printStatusDecoder();
				// }
				// If it does have a left brother:
				maskImage = fatherSilhouette;

				// Create the silhouette for the leftBrother
				gpcc::ImageSparse *leftBrother = this->pPC[npointcloud]->SilhouetteFromPointCloud(lStart - nInterval, lEnd - nInterval, this->axis);
				gpcc::ImageRaster *leftBrotherImage = new ImageRaster(leftBrother, this->imSize, this->imSize, 0);

				// Decodes the image

				cabacIn->decodeImage(*currImage, *maskImage, *leftBrotherImage);
				// if (npointcloud == 1)
				// {
				// 	cabacIn->printStatusDecoder();
				// }

				delete leftBrotherImage;
				delete leftBrother;
			}

			leftChild = new gpcc::ImageSparse(currImage);
			leftChildRaster = currImage;
			currImage = nullptr;
			maskImage = nullptr;

			// Creates an empty image for the right brother
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// The right image for sure has a rightBrother.
			// if it IS a rightBrother, then the mask image is the left brother.
			maskImage = leftChild;

			// Deduces the known pixels in the current image.
			deducePixels(*currImage, *fatherSilhouette, *leftChildRaster);

			// Decodes the image

			// if (npointcloud == 1)
			// {
			// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
			// 	cabacIn->printStatusDecoder();
			// }

			cabacIn->decodeImage(*currImage, *maskImage, *leftChildRaster);

			// if (npointcloud == 1)
			// {
			// 	cabacIn->printStatusDecoder();
			// }

			// Creates a sparse representation for the right child
			rightChild = new gpcc::ImageSparse(currImage);

			delete currImage;
			delete leftChildRaster;

			currImage = nullptr;
			maskImage = nullptr;
		}
		else
		{
			// If only one side had pixels, I have to do something else.
			if (encodeLeft && !encodeRight)
			{
				// This means only the left image had pixels.
				// Thus:
				// I need to copy the leftChild so I can manage the memory.
				leftChild = new gpcc::ImageSparse(fatherSilhouette->getLocation());
				rightChild = new gpcc::ImageSparse();
			}
			else if (!encodeLeft && encodeRight)
			{
				// This means only the right image had pixels.
				// Thus:
				// I need to copy the rightChild so I can manage the memory.
				leftChild = new gpcc::ImageSparse();
				rightChild = new gpcc::ImageSparse(fatherSilhouette->getLocation());
			}
			else
			{
				// This means that neither image had pixels, and thus this
				// function should NOT have been called.
				leftChild = new gpcc::ImageSparse();
				rightChild = new gpcc::ImageSparse();
			}
		}

		// std::cout << "encodeLeft  = " << encodeLeft  << " | Silhouette(" << lStart << "," << lEnd << ") = | nPixelsLeft:  " << leftChild->NumberOccupiedPixels() << " ." << std::endl;
		// std::cout << "encodeRight = " << encodeRight << " | Silhouette(" << rStart << "," << rEnd << ") = | nPixelsRight: " << rightChild->NumberOccupiedPixels() << " ." << std::endl;

		// If I have reached the leaf nodes
		if (nInterval == 1)
		{
			// If this is at the leaf nodes, add these silhouettes to the point cloud.
			this->pPC[npointcloud]->AddSilhouette(this->axis, lStart, leftChild);
			this->pPC[npointcloud]->AddSilhouette(this->axis, rStart, rightChild);
		}
		else
		{

			if (encodeLeft && (level < (this->nBits - 1)))
			{
				// std::cout << "lStart: " << lStart << " lEnd: " << lEnd << std::endl;
				xDecodePointCloud(
						npointcloud, cabacIn, level + 1,
						lStart, lEnd, leftChild);
			}
			if (encodeRight && (level < (this->nBits - 1)))
			{
				// std::cout << "rStart: " << rStart << " rEnd: " << rEnd << std::endl;
				xDecodePointCloud(
						npointcloud, cabacIn, level + 1,
						rStart, rEnd, rightChild);
			}
		}

		delete leftChild;
		delete rightChild;
	}
	else
	{
		// if it is the single mode
		decodeIntervalSingleMode(npointcloud, cabacIn, iStart, iEnd, fatherSilhouette);
	}

	if (level == log2(this->nParallelism))
	{
		// std::cout << "Thread " << npointcloud << " - FINISHED !" << std::endl;
		delete fatherSilhouette;
	}

	return;
}

//----------------------------------------
// Private Functions
/*
 * This is the real recursive function.
 */
void gpcc::DecoderThreadPyramid::decodeRootSilhouette(Cabac *cabacIn, uint32_t level,
																											uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	gpcc::ImageRaster *currImage = nullptr;
	gpcc::ImageSparse *maskImage = nullptr;

	bool currBit;

	// The first time this function is called there is no silhouette, no parent. It needs to encode the top-level silhouette.
	if (level == 0)
	{

		// Check the size
		iStart = 0;
		iEnd = (1 << this->nBits) - 1;

		// Reads the bitstreamParam
		currBit = this->paramBitstream[0].readBit();

		// If it is true, then an image should be decoded.
		if (currBit == true)
		{
			// Creates an empty image
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// Decodes this image.

			cabacIn->decodeImage(*currImage);

			// Create a sparse representation for this image and adds it to the silhouette list.
			fatherSilhouette = new gpcc::ImageSparse(currImage);

			// Deletes the raster image and erases the pointer.
			delete currImage;
			currImage = nullptr;
		}
	}

	// Reads another bit from the parameter that signals the dyadic / single mode
	currBit = this->paramBitstream[0].readBit();
	// if it is dyadic mode
	if (currBit == false)
	{
		// Decodes the dyadic decomposition.
		uint32_t lStart, lEnd, rStart, rEnd, nInterval;
		lStart = iStart;
		lEnd = iStart + ((iEnd - iStart) >> 1);

		rStart = lEnd + 1;
		rEnd = iEnd;

		// std::cout << "GENERATED:\n";
		// std::cout << "level:" << level << "\n";
		// std::cout << "lStart: " << lStart << " lEnd: " << lEnd << std::endl;
		// std::cout << "rStart: " << rStart << " rEnd: " << rEnd << std::endl;

		nInterval = lEnd - lStart + 1;

		// Creates both silhouettes.
		gpcc::ImageSparse *leftChild = nullptr;
		gpcc::ImageSparse *rightChild = nullptr;

		// Reads two bits from the parameter bitstream.
		bool encodeLeft = this->paramBitstream[0].readBit();
		bool encodeRight = this->paramBitstream[0].readBit();
		// std::cout << "encodeLeft: " << encodeLeft << " encodeRight: " << encodeRight << std::endl;

		// This level was encoded only if it had pixels in both images.
		if (encodeLeft && encodeRight)
		{
			gpcc::ImageRaster *leftChildRaster = nullptr;

			// Creates an empty image for the left brother
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// Does this image have a left brother?

			if (lStart == this->leftBounds[0])
			{
				// std::cout << "Without left mask" << std::endl;

				// if (npointcloud == 1)
				// {
				// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// cabacIn->printStatusDecoder();
				// }

				// If it does NOT have a left brother, it uses the father as mask and decodes the image using just the mask (2D contexts only)
				maskImage = fatherSilhouette;

				cabacIn->decodeImage(*currImage, *maskImage);

				// if (npointcloud == 1)
				// {
				// cabacIn->printStatusDecoder();
				// }
			}
			else
			{
				// std::cout << "With left mask" << std::endl;
				// if (npointcloud == 1)
				// {
				// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
				// cabacIn->printStatusDecoder();
				// }
				// If it does have a left brother:
				maskImage = fatherSilhouette;

				// Create the silhouette for the leftBrother
				gpcc::ImageSparse *leftBrother = this->pPC[0]->SilhouetteFromPointCloud(lStart - nInterval, lEnd - nInterval, this->axis);
				gpcc::ImageRaster *leftBrotherImage = new ImageRaster(leftBrother, this->imSize, this->imSize, 0);

				// Decodes the image

				cabacIn->decodeImage(*currImage, *maskImage, *leftBrotherImage);
				// if (npointcloud == 1)
				// {
				// cabacIn->printStatusDecoder();
				// }

				delete leftBrotherImage;
				delete leftBrother;
			}

			leftChild = new gpcc::ImageSparse(currImage);
			leftChildRaster = currImage;
			currImage = nullptr;
			maskImage = nullptr;

			// Creates an empty image for the right brother
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// The right image for sure has a rightBrother.
			// if it IS a rightBrother, then the mask image is the left brother.
			maskImage = leftChild;

			// Deduces the known pixels in the current image.
			deducePixels(*currImage, *fatherSilhouette, *leftChildRaster);

			// Decodes the image

			// if (npointcloud == 1)
			// {
			// 	std::cout << "iStart: " << iStart << " iEnd: " << iEnd << std::endl;
			// cabacIn->printStatusDecoder();
			// }

			cabacIn->decodeImage(*currImage, *maskImage, *leftChildRaster);

			// if (npointcloud == 1)
			// {
			// cabacIn->printStatusDecoder();
			// }

			// Creates a sparse representation for the right child
			rightChild = new gpcc::ImageSparse(currImage);

			delete currImage;
			delete leftChildRaster;

			currImage = nullptr;
			maskImage = nullptr;
		}
		else
		{
			// If only one side had pixels, I have to do something else.
			if (encodeLeft && !encodeRight)
			{
				// This means only the left image had pixels.
				// Thus:
				// I need to copy the leftChild so I can manage the memory.
				leftChild = new gpcc::ImageSparse(fatherSilhouette->getLocation());
				rightChild = new gpcc::ImageSparse();
			}
			else if (!encodeLeft && encodeRight)
			{
				// This means only the right image had pixels.
				// Thus:
				// I need to copy the rightChild so I can manage the memory.
				leftChild = new gpcc::ImageSparse();
				rightChild = new gpcc::ImageSparse(fatherSilhouette->getLocation());
			}
			else
			{
				// This means that neither image had pixels, and thus this
				// function should NOT have been called.
				leftChild = new gpcc::ImageSparse();
				rightChild = new gpcc::ImageSparse();
			}
		}

		// std::cout << "encodeLeft  = " << encodeLeft  << " | Silhouette(" << lStart << "," << lEnd << ") = | nPixelsLeft:  " << leftChild->NumberOccupiedPixels() << " ." << std::endl;
		// std::cout << "encodeRight = " << encodeRight << " | Silhouette(" << rStart << "," << rEnd << ") = | nPixelsRight: " << rightChild->NumberOccupiedPixels() << " ." << std::endl;

		// Got the fatherSilhouettes
		// The preorder traversal guarantees the fatherSilhouettes
		// correct order.
		if (level == (log2(this->nParallelism) - 1))
		{
			this->pPC[0]->AddSilhouette(this->axis, lStart, leftChild);
			this->pPC[0]->AddSilhouette(this->axis, rStart, rightChild);
		}
		else
		{

			if (encodeLeft && (level < log2(this->nParallelism) - 1))
			{
				// std::cout << "lStart: " << lStart << " lEnd: " << lEnd << std::endl;
				decodeRootSilhouette(
						cabacIn, level + 1,
						lStart, lEnd, leftChild);
			}
			if (encodeRight && (level < log2(this->nParallelism) - 1))
			{
				// std::cout << "rStart: " << rStart << " rEnd: " << rEnd << std::endl;
				decodeRootSilhouette(
						cabacIn, level + 1,
						rStart, rEnd, rightChild);
			}
		}
		if (level < (log2(this->nParallelism) - 1))
		{
			delete leftChild;
			delete rightChild;
		}
	}
	else
	{
		// if it is the single mode
		decodeIntervalSingleModeTopPyramid(0, cabacIn, iStart, iEnd, fatherSilhouette);
	}

	// Adding Subtrees's Father Silhouettes 
	if (level == 0)
	{
		for (int i = 0; i < this->nParallelism; i++)
		{
			uint32_t blockSize = (1 << this->nBits) / this->nParallelism;
			uint32_t iStart = (i)*blockSize;
			uint32_t iEnd = (i + 1) * blockSize - 1;

			ImageSparse *slice = this->pPC[0]->SilhouetteFromPointCloud(iStart, iEnd, this->axis);
			this->fatherSilhouettes.push_back(slice);
		}

		// std::cout << "Thread " << npointcloud << " - FINISHED !" << std::endl;
		delete fatherSilhouette;
	}

	return;
}

/*
 *
 */
void gpcc::DecoderThreadPyramid::decodeIntervalSingleMode(uint32_t npointcloud, Cabac *cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
{
	// TODO:
	//  Test if I should add a new table for 3Dcontexts (it seems this is not the case)

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
		leftImageSparse = this->pPC[npointcloud]->SilhouetteFromPointCloud(iStart - 1, iStart - 1, this->axis);
	}

	// Make a raster object of this image.
	leftImageRaster = new ImageRaster(leftImageSparse, this->imSize, this->imSize, 0);

	// delete the leftImageSparse, as it will not be used anymore.
	delete leftImageSparse;
	leftImageSparse = nullptr;

	uint32_t k;

	for (k = iStart; k <= iEnd; k++)
	{
		// Was this silhouette encoded?
		bool encodeThisSilhouette = this->paramBitstream[npointcloud].readBit();

		// Creates an empty image
		currImageRaster = new gpcc::ImageRaster(this->imSize, this->imSize);

		if (encodeThisSilhouette)
		{
			// If it was encoded, I have to read and decode it.
			// Decodes the image
			// std::cout << k << ":"
			// 					<< "\n";
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusDecoder();
			if (k != iEnd)
			{

				cabacIn->decodeImage(*currImageRaster, *fatherSilhouette, *leftImageRaster);
			}
			else
			{

				// For the last mask
				// Deduce pixels.
				gpcc::ImageRaster *lastMaskRaster = new ImageRaster(lastMask, this->imSize, this->imSize, 0);
				deducePixels(*currImageRaster, *fatherSilhouette, *lastMaskRaster);
				delete lastMaskRaster;
				cabacIn->decodeImage(*currImageRaster, *lastMask, *leftImageRaster);
			}
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusDecoder();
		}

		// Either way, I have to add it to the point cloud.
		// Create a sparse representation of the current image
		currImageSparse = new gpcc::ImageSparse(currImageRaster);

		// Adds the pixels to the last mask.
		if (k != iEnd)
		{
			lastMask->addPixels(currImageSparse->getLocation());
		}

		// Add it to the point cloud
		this->pPC[npointcloud]->AddSilhouette(this->axis, k, currImageSparse);

		// Deletes the current imageSparse as it will not be used anymore.
		delete currImageSparse;
		currImageSparse = nullptr;

		// Deletes the old leftImageRaster
		delete leftImageRaster;
		// Updates the leftImageRaster
		leftImageRaster = currImageRaster;
		currImageRaster = nullptr;
	}

	// Deletes the leftImageRaster.
	delete leftImageRaster;
	delete lastMask;
}

/*
 *
 */
void gpcc::DecoderThreadPyramid::decodeIntervalSingleModeTopPyramid(uint32_t npointcloud, Cabac* cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse* fatherSilhouette)
{
	// TODO:
	//  Test if I should add a new table for 3Dcontexts (it seems this is not the case)

	gpcc::ImageSparse* leftImageSparse = nullptr;
	gpcc::ImageRaster* leftImageRaster = nullptr;
	gpcc::ImageSparse* currImageSparse = nullptr;
	gpcc::ImageRaster* currImageRaster = nullptr;

	uint32_t numberOfSilhouettesInBlock = (1 << this->nBits) / this->nParallelism;
	
	
	// Make a raster object of this image.
	leftImageRaster = new gpcc::ImageRaster(this->imSize, this->imSize);
		
	uint32_t k;

	for (k = iStart; k <= iEnd; k += numberOfSilhouettesInBlock)
	{
		// Was this silhouette encoded?
		bool encodeThisSilhouette = this->paramBitstream[npointcloud].readBit();

		// Creates an empty image
		currImageRaster = new gpcc::ImageRaster(this->imSize, this->imSize);

		if (encodeThisSilhouette)
		{
			// If it was encoded, I have to read and decode it.
			// Decodes the image
			// std::cout << k << ":"
			// 					<< "\n";
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusDecoder();
			cabacIn->decodeImage(*currImageRaster, *fatherSilhouette, *leftImageRaster);
			
			// std::cout << "subtreeLeftBound: " << this->leftBounds[npointcloud] << std::endl;
			// cabacIn->printStatusDecoder();
		}

		// Either way, I have to add it to the point cloud.
		// Create a sparse representation of the current image
		currImageSparse = new gpcc::ImageSparse(currImageRaster);

		//Keeps the silhouette decoded.
		this->pPC[0]->AddSilhouette(this->axis, k, currImageSparse);
		// Add it to the point cloud
		// In the top pyramid this should NOT be added to the point cloud.
		//this->pPC[npointcloud]->AddSilhouette(this->axis, k, currImageSparse);

		// Deletes the current imageSparse as it will not be used anymore.
		delete currImageSparse;
		currImageSparse = nullptr;

		// Deletes the old leftImageRaster
		delete leftImageRaster;
		// Updates the leftImageRaster
		leftImageRaster = currImageRaster;
		currImageRaster = nullptr;
	}

	// Deletes the leftImageRaster.
	delete leftImageRaster;
}

/*
		A	A -> fatherImage
	 B C  B -> leftImage
				C -> image

	 By construction:
	 A = B + C

	 When encoding C, we already know A and B.
	 So, we know if A is 1 and B is 0, then C MUST BE ONE in that position.
	 C is only unknown if B is 1. Therefore, we only encode the pixels in C where B is one (we use B as a mask).

	 This function discovers the "known" pixels in C, i.e., those where A is 1 and B is 0.
 */
void gpcc::DecoderThreadPyramid::deducePixels(gpcc::ImageRaster &image, gpcc::ImageSparse &fatherImage, gpcc::ImageRaster &leftImage)
{
	uint64_t nSymbolsInFather = fatherImage.NumberOccupiedPixels();
	gpcc::Pixel<int> currPixelLocation;

	uint64_t n;

	// Rewinds the mask.
	fatherImage.Rewind();

	for (n = 0; n < nSymbolsInFather; n++)
	{
		// Gets the current pixel location.
		currPixelLocation = fatherImage.CurrentPixel();

		// Is it one or zero in the leftImage?
		if (leftImage.PixelPresent(currPixelLocation) == false)
		{
			// Then it MUST BE one in the current image.
			image.addPixel(currPixelLocation);
		}

		// Move the iterator to the next pixel.
		fatherImage.NextPixel();
	}
}

gpcc::PointCloud *gpcc::DecoderThreadPyramid::getPointCloud(uint32_t npointcloud)
{
	return this->pPC[npointcloud];
}

gpcc::Axis gpcc::DecoderThreadPyramid::getAxis()
{
	return this->axis;
}
