#include <vector>

#include "en_decoder.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#define DECODER_VERBOSE 0

//----------------------------------------
// Constructors
gpcc::Decoder::Decoder(Bitstream &pbstream, Axis axis, EncoderConfigParams *params) : nBits(0),
																																											axis(gpcc::Axis::X),
																																											acbstream(nullptr)
{
	this->pParams = params;
	this->axis = axis;
	// Parses the header.
	this->parseHeader(pbstream);

	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc2D = 6;
	uint64_t nc3D = 9;

	// Initialize cabac
	if (acbstream != NULL)
	{
		cabac.initDecoder(m, nc2D, nc3D, *acbstream);
	}
}

gpcc::Decoder::~Decoder()
{
}

//----------------------------------------
// Private Functions
// This function reads the header, and splits
void gpcc::Decoder::parseHeader(Bitstream &bstream)
{
	this->nBits = (uint16_t)bstream.readNumber(6);
	uint16_t iAxis = (uint16_t)bstream.readNumber(2);
	uint16_t algorithmChoice = (uint16_t)bstream.readNumber(4);

	this->axis = ((iAxis == 0) ? gpcc::Axis::X : ((iAxis == 1) ? gpcc::Axis::Y : gpcc::Axis::Z));

	uint64_t lengthBitstreamParam = bstream.readNumber(16);
	this->k = (uint32_t)bstream.readNumber(8);

	// Parse the codecparameter bitstream.
	// The first N bits are consumed and a new bitstream is created.
	Bitstream paramBitstream = Bitstream(bstream, lengthBitstreamParam);

	// Initialize the parameter bitstream
	// TODO: Get these from the config.
	uint64_t m = 16;
	uint64_t nc1D = 4;
	this->paramBitstream = CodecParameterBitstream(paramBitstream, m, nc1D);

	// Assign the rest of the bitstream.
	this->acbstream = &bstream;
#if DECODER_VERBOSE
	std::cout << "Inside Decoder" << std::endl;
	std::cout << "nBits = " << this->nBits << std::endl;
	std::cout << "axis  = " << this->axis << std::endl;
	std::cout << "Symbols in the CodecParameterBitstream = " << this->paramBitstream.totalSize() << std::endl;
	std::cout << "AC Payload = " << acbstream->numberOfRemainingBits() << std::endl;
#endif
}

//----------------------------------------
// Public Functions
/*
 * This is the non-recursive version.
 * It performs only the dyadic decomposition.
 */
gpcc::PointCloud *gpcc::Decoder::decodePointCloud()
{

	gpcc::PointCloud *ppc = new PointCloud();

	gpcc::SilhouetteDecomposition silhouette_list;

	// Perform the silhouette decomposition list.
	uint32_t imSize = 1 << this->nBits;
	silhouette_list = SilhouetteDecomposition(this->nBits);
	silhouette_list.createDyadicDecomposition();

	// Prepares an array of images.
	int numberOfSilhouettes = silhouette_list.getNumberOfDecompositions();
#if DECODER_VERBOSE
	std::cout << "Created " << numberOfSilhouettes << " silhouettes." << std::endl;
#endif
	std::vector<gpcc::ImageSparse *> image_list;
	image_list.resize(numberOfSilhouettes, nullptr);

	gpcc::ImageRaster *currImage = nullptr;

	gpcc::Silhouette s;
	int i;
	int index_pc = 0;
	// size_t nPixels;
	// For all silhouettes
	for (i = 0; i < numberOfSilhouettes; i++)
	{
		// Get silhouette details
		s = silhouette_list.getSilhouette(i);
		int32_t currID = s.myID;

		// Reads the bitstreamParam
		bool currBit = paramBitstream.readBit();

		// If it is true, then an image should be decoded.
		if (currBit == true)
		{
			// Creates an empty image
			currImage = new gpcc::ImageRaster(imSize, imSize);

			// Decodes one silhouette.
			if (s.fatherID == -1)
			{
				// Decodes this image.
				cabac.decodeImage(*currImage);
			}
			else
			{
				// Does this image have a left brother?
				if (s.leftBrotherID == -1)
				{
					// Creates a sparse representation of the father.
					// If it does NOT have a left brother, it uses the father as mask and decodes the image using just the mask (2D contexts only)
					int32_t fatherID = s.fatherID;
					gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[fatherID];

					// Decodes the image
					cabac.decodeImage(*currImage, *maskImage);
				}
				else
				{
					// Is this image a rightBrother?
					if (s.isRightBrother == false)
					{
						// If it is not rightBrother, then the mask image is the father.
						int32_t fatherID = s.fatherID;
						gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[fatherID];
						int32_t leftID = s.leftBrotherID;
						gpcc::ImageRaster *leftBrotherImage = new ImageRaster((ImageSparse *)image_list[leftID], imSize, imSize, 0);

						// Decodes the image
						cabac.decodeImage(*currImage, *maskImage, *leftBrotherImage);

						delete leftBrotherImage;
					}
					else
					{
						// if it IS a rightBrother, then the mask image is the left brother.
						int32_t leftID = s.leftBrotherID;
						gpcc::ImageSparse *maskImage = (ImageSparse *)image_list[leftID];
						gpcc::ImageRaster *leftBrotherImage = new ImageRaster((ImageSparse *)image_list[leftID], imSize, imSize, 0);

						// I need to use the father and the mask to extract some known pixels.
						int32_t fatherID = s.fatherID;
						gpcc::ImageSparse *fatherImage = (ImageSparse *)image_list[fatherID];

						// Deduces the known pixels in the current image.
						deducePixels(*currImage, *fatherImage, *leftBrotherImage);

						// Decodes the image
						cabac.decodeImage(*currImage, *maskImage, *leftBrotherImage);

						delete leftBrotherImage;
					}
				}
			}

			// Create a sparse representation for this image and adds it to the silhouette list.
			image_list[currID] = new gpcc::ImageSparse(currImage);

			// Deletes the raster image and erases the pointer.
			delete currImage;
			currImage = nullptr;
		}
		else
		{
			// In this case, Just create an empty image and add it to the silhouette list.
			image_list[currID] = new gpcc::ImageSparse();
		}

		// If this is a leaf silhouette
		if (s.iEnd == s.iStart)
		{
			// This is where I should add this image to the silhouette
			gpcc::ImageSparse *ptr_img_sparse;
			ptr_img_sparse = image_list[currID];

			if (ptr_img_sparse->NumberOccupiedPixels() > 0)
			{
				ppc->AddSilhouette(this->axis, s.iStart, ptr_img_sparse);
			}

			// Thinking of this counter as a silhouette sorting in order to link them together at the end
			//  in the correct positions
			index_pc++;
		}

		uint32_t nPixels = image_list[currID]->NumberOccupiedPixels();
#if DECODER_VERBOSE
		std::cout << "Decoded image " << i << " -> number of occupied pixels: " << nPixels << std::endl;
#endif
	}

	// Delete all images in the list.
	for (i = 0; i < numberOfSilhouettes; i++)
	{
		if (image_list[i] != nullptr)
		{
			delete image_list[i];
		}
	}

	if (currImage != nullptr)
		delete currImage;

	return ppc;
}

/*
 * This is the recursive version.
 * It should work with both dyadic and single modes.
 * It is faster and has a better compression performance compared to the non-recursive version.
 */
gpcc::PointCloud *gpcc::Decoder::decodePointCloud2()
{
	// Creates an empty point cloud.
	gpcc::PointCloud *ppc = new PointCloud();

	this->pPC = ppc;
	this->imSize = 1 << this->nBits;

	// Calls the recursive decoding function
	this->xDecodePointCloud(0);

	return ppc;
}

//----------------------------------------
// Private Functions
/*
 * This is the real recursive function.
 */
void gpcc::Decoder::xDecodePointCloud(uint32_t level, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette)
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
		currBit = paramBitstream.readBit();

		// If it is true, then an image should be decoded.
		if (currBit == true)
		{
			// Creates an empty image
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// Decodes this image.
			cabac.decodeImage(*currImage);

			// Create a sparse representation for this image and adds it to the silhouette list.
			fatherSilhouette = new gpcc::ImageSparse(currImage);

			// Deletes the raster image and erases the pointer.
			delete currImage;
			currImage = nullptr;
		}
	}

	// Reads another bit from the parameter that signals the dyadic / single mode
	currBit = paramBitstream.readBit();
	// if it is dyadic mode
	if (currBit == false)
	{
		// Decodes the dyadic decomposition.
		uint32_t lStart, lEnd, rStart, rEnd, nInterval;
		lStart = iStart;
		lEnd = iStart + ((iEnd - iStart) >> 1);

		rStart = lEnd + 1;
		rEnd = iEnd;

		nInterval = lEnd - lStart + 1;

		// Creates both silhouettes.
		gpcc::ImageSparse *leftChild = nullptr;
		gpcc::ImageSparse *rightChild = nullptr;

		// Reads two bits from the parameter bitstream.
		bool encodeLeft = paramBitstream.readBit();
		bool encodeRight = paramBitstream.readBit();
		// std::cout << "encodeLeft  = " << encodeLeft << " ." << std::endl;
		// std::cout << "encodeRight = " << encodeRight << " ." << std::endl;

		// This level was encoded only if it had pixels in both images.
		if (encodeLeft && encodeRight)
		{
			gpcc::ImageRaster *leftChildRaster = nullptr;

			// Creates an empty image for the left brother
			currImage = new gpcc::ImageRaster(this->imSize, this->imSize);

			// Does this image have a left brother?
			if (lStart == 0)
			{
				// If it does NOT have a left brother, it uses the father as mask and decodes the image using just the mask (2D contexts only)
				maskImage = fatherSilhouette;

				// Decodes the image
				cabac.decodeImage(*currImage, *maskImage);
			}
			else
			{
				// If it does have a left brother:
				maskImage = fatherSilhouette;

				// Create the silhouette for the leftBrother
				gpcc::ImageSparse *leftBrother = this->pPC->SilhouetteFromPointCloud(lStart - nInterval, lEnd - nInterval, this->axis);
				gpcc::ImageRaster *leftBrotherImage = new ImageRaster(leftBrother, this->imSize, this->imSize, 0);

				// Decodes the image
				cabac.decodeImage(*currImage, *maskImage, *leftBrotherImage);

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
			cabac.decodeImage(*currImage, *maskImage, *leftChildRaster);

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
			this->pPC->AddSilhouette(this->axis, lStart, leftChild);
			this->pPC->AddSilhouette(this->axis, rStart, rightChild);
		}
		else
		{
			if (encodeLeft && (level < (this->nBits - 1)))
			{
				this->xDecodePointCloud(level + 1, lStart, lEnd, leftChild);
			}
			if (encodeRight && (level < (this->nBits - 1)))
			{
				this->xDecodePointCloud(level + 1, rStart, rEnd, rightChild);
			}
		}

		delete leftChild;
		delete rightChild;
	}
	else
	{
		// if it is the single mode
		decodeIntervalSingleMode(iStart, iEnd, fatherSilhouette, &this->cabac);
	}

	if (level == 0)
	{
		delete fatherSilhouette;
	}
}

/*
 *
 */
void gpcc::Decoder::decodeIntervalSingleMode(uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette, Cabac *cabac)
{
	// TODO:
	//  Test if I should add a new table for 3Dcontexts (it seems this is not the case)
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

	for (k = iStart; k <= iEnd; k++)
	{
		// Was this silhouette encoded?
		bool encodeThisSilhouette = this->paramBitstream.readBit();

		// Creates an empty image
		currImageRaster = new gpcc::ImageRaster(this->imSize, this->imSize);

		if (encodeThisSilhouette)
		{

			// std::cout << k << ":"
			// 					<< "\n";
			// cabac->printStatusDecoder();

			if (k != iEnd)
			{
				cabac->decodeImage(*currImageRaster, *fatherSilhouette, *leftImageRaster);
			}
			else
			{
				// For the last mask
				// Deduce pixels.
				gpcc::ImageRaster *lastMaskRaster = new ImageRaster(lastMask, this->imSize, this->imSize, 0);
				deducePixels(*currImageRaster, *fatherSilhouette, *lastMaskRaster);
				delete lastMaskRaster;

				cabac->decodeImage(*currImageRaster, *lastMask, *leftImageRaster);
			}
			// cabac->printStatusDecoder();
		}

		// Either way, I have to add it to the point cloud.
		// Create a sparse representation of the current image
		currImageSparse = new gpcc::ImageSparse(currImageRaster);

		if (k != iEnd)
		{
			// Adds the pixels to the last mask.
			lastMask->addPixels(currImageSparse->getLocation());
		}

		// Add it to the point cloud
		this->pPC->AddSilhouette(this->axis, k, currImageSparse);

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
void gpcc::Decoder::deducePixels(gpcc::ImageRaster &image, gpcc::ImageSparse &fatherImage, gpcc::ImageRaster &leftImage)
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
