#include "en_cabac.h"
#include "ds_image_raster.h"
#include "ds_pixel.h"
#include "en_encoder.h"

#define DEBUG_CABAC 0

//----------------------------------------
// Constructors
Cabac::Cabac() : m(0),
								 numberOfContexts2D(0),
								 enc(NULL),
								 dec(NULL)
{
	this->mode = CabacMode::Undefined;
	this->initEncoder(16, 6, 9);
}

// Destructor
// I have to take care of the encoder and decoder objects.
Cabac::~Cabac()
{
	if (this->mode == CabacMode::Encoder)
	{
		if (enc != NULL)
			delete enc;
	}
	else if (this->mode == CabacMode::Decoder)
	{
		if (dec != NULL)
			delete dec;
	}
}

// Initializers
void Cabac::initEncoder(uint64_t m, uint64_t nc2D, uint64_t nc3D)
{
	this->mode = CabacMode::Encoder;

	this->m = m;
	this->numberOfContexts2D = nc2D;
	this->numberOfContexts3D = nc3D;

	// Initialize contexts
	this->initContexts2D();
	this->initContexts3D();

	// Initialize BACEngine
	enc = new BACEncoder(m);

	// Initialize the parameter bitstream
	this->paramBitstream = new CodecParameterBitstream();

	// Creates the internal bitstreams.
	this->acbstream = new Bitstream();
}

void Cabac::initDecoder(uint64_t m, uint64_t nc2D, uint64_t nc3D, Bitstream &bstream)
{
	this->mode = CabacMode::Decoder;

	this->m = m;
	this->numberOfContexts2D = nc2D;
	this->numberOfContexts3D = nc3D;

	// Initialize contexts
	this->initContexts2D();
	this->initContexts3D();

	// Initialize BACEngine
	dec = new BACDecoder(&bstream, m);
	dec->init();
}

//----------------------------------------
// Public Functions used at the encoder side
void Cabac::encodeImage(gpcc::ImageRaster &image)
{
	ContextInfo c;
	uint32_t currContext;
	bool currSymbol;

	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	// loops through all the symbols in the image, encoding each one.
	uint32_t j, i;
	for (j = 0; j < nrows; j++)
	{
		for (i = 0; i < ncols; i++)
		{
			// Gets the current symbol.
			currSymbol = image.PixelPresent(j, i);

			// Get the current context.
			currContext = getContext2D(image, j, i, ncols, this->numberOfContexts2D);

			// Fetches the current context table.
			if (contexts2D[currContext][0] > contexts2D[currContext][1])
			{
				c.mps = false;
				c.countMPS = contexts2D[currContext][0];
				c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
			}
			else
			{
				c.mps = true;
				c.countMPS = contexts2D[currContext][1];
				c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
			}

			// Encodes the current symbol using this context.
			enc->encodeOneSymbol(currSymbol, c, *(this->acbstream));

			// Updates the context table.
			if (currSymbol == false)
			{
				contexts2D[currContext][0]++;
			}
			else
			{
				contexts2D[currContext][1]++;
			}
		}
	}
}

void Cabac::encodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask)
{
	ContextInfo c;
	uint32_t currContext;
	bool currSymbol;

	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	uint64_t nSymbolsToEncode = imageMask.NumberOccupiedPixels();
	gpcc::Pixel<int> currPixelLocation;

	uint64_t n;
	uint32_t j, i;

	// std::cout << " nSymbolsToEncode = " << nSymbolsToEncode << std::endl;

	// Rewinds the mask.
	imageMask.Rewind();

	for (n = 0; n < nSymbolsToEncode; n++)
	{
		// Gets the current pixel location.
		currPixelLocation = imageMask.CurrentPixel();
		j = currPixelLocation[0];
		i = currPixelLocation[1];

		// if (n == 0)
		//	std::cout << "Pixel " << n << " = (" << j << "," << i << ")" << std::endl;

		// Gets the current symbol.
		currSymbol = image.PixelPresent(currPixelLocation);

		// Get the current context.
		currContext = getContext2D(image, j, i, ncols, this->numberOfContexts2D);

		// Fetches the current context table.
		if (contexts2D[currContext][0] > contexts2D[currContext][1])
		{
			c.mps = false;
			c.countMPS = contexts2D[currContext][0];
			c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = contexts2D[currContext][1];
			c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
		}

		// Encodes the current symbol using this context.
		enc->encodeOneSymbol(currSymbol, c, *(this->acbstream));

		// Updates the context table.
		if (currSymbol == false)
		{
			contexts2D[currContext][0]++;
		}
		else
		{
			contexts2D[currContext][1]++;
		}

		// Move the iterator to the next pixel.
		imageMask.NextPixel();
	}
}

void Cabac::encodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask, gpcc::ImageRaster &image3D)
{
	ContextInfo c;
	uint32_t currContext3D, currContext2D;
	bool currSymbol;

	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	uint64_t nSymbolsToEncode = imageMask.NumberOccupiedPixels();
	gpcc::Pixel<int> currPixelLocation;

	uint64_t n;
	uint32_t j, i;

	// std::cout << " nSymbolsToEncode = " << nSymbolsToEncode << std::endl;

	// Rewinds the mask.
	imageMask.Rewind();

	for (n = 0; n < nSymbolsToEncode; n++)
	{
		// Gets the current pixel location.
		currPixelLocation = imageMask.CurrentPixel();
		j = currPixelLocation[0];
		i = currPixelLocation[1];

		// if (n == 0)
		//	std::cout << "Pixel " << n << " = (" << j << "," << i << ")" << std::endl;

		// Gets the current symbol.
		currSymbol = image.PixelPresent(currPixelLocation);

		// Get the current context.
		currContext2D = getContext2D(image, j, i, ncols, this->numberOfContexts2D);
		currContext3D = getContext3D(image3D, j, i, nrows, ncols, this->numberOfContexts3D);

		// Fetches the current context table.
		if (contexts3D[currContext3D][currContext2D][0] > contexts3D[currContext3D][currContext2D][1])
		{
			c.mps = false;
			c.countMPS = contexts3D[currContext3D][currContext2D][0];
			c.totalCount = contexts3D[currContext3D][currContext2D][0] + contexts3D[currContext3D][currContext2D][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = contexts3D[currContext3D][currContext2D][1];
			c.totalCount = contexts3D[currContext3D][currContext2D][0] + contexts3D[currContext3D][currContext2D][1];
		}

		// Encodes the current symbol using this context.
		enc->encodeOneSymbol(currSymbol, c, *(this->acbstream));

		// Updates the context table.
		if (currSymbol == false)
		{
			contexts3D[currContext3D][currContext2D][0]++;
		}
		else
		{
			contexts3D[currContext3D][currContext2D][1]++;
		}

		// Move the iterator to the next pixel.
		imageMask.NextPixel();
	}
}

void Cabac::closeEncoder(Bitstream &bstream)
{
	// Closes the bitstream.
	enc->closeBitstream(bstream);
}

//----------------------------------------
// Public Functions used at the decoder side
void Cabac::decodeImage(gpcc::ImageRaster &image)
{
	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	ContextInfo c;
	uint32_t currContext;
	bool currSymbol;

	// loops through all the symbols in the image, decoding each one.
	uint32_t j, i;
	for (j = 0; j < nrows; j++)
	{
		for (i = 0; i < ncols; i++)
		{
			// Get the current context.
			currContext = getContext2D(image, j, i, ncols, this->numberOfContexts2D);

			// Fetches the current context table.
			if (contexts2D[currContext][0] > contexts2D[currContext][1])
			{
				c.mps = false;
				c.countMPS = contexts2D[currContext][0];
				c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
			}
			else
			{
				c.mps = true;
				c.countMPS = contexts2D[currContext][1];
				c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
			}

			// Decodes the current symbol.
			currSymbol = dec->decodeOneSymbol(c);

			// Inserts it into the image.
			if (currSymbol == true)
			{
				image.addPixel(j, i);
			}

			// Updates the context table.
			if (currSymbol == false)
			{
				contexts2D[currContext][0]++;
			}
			else
			{
				contexts2D[currContext][1]++;
			}
		}
	}
}

void Cabac::decodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask)
{
	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	ContextInfo c;
	uint32_t currContext;
	bool currSymbol;

	uint64_t nSymbolsToDecode = imageMask.NumberOccupiedPixels();
	gpcc::Pixel<int> currPixelLocation;

	uint64_t n;
	uint32_t j, i;

	// std::cout << " nSymbolsToDecode = " << nSymbolsToDecode << std::endl;

	// Rewinds the mask.
	imageMask.Rewind();

	for (n = 0; n < nSymbolsToDecode; n++)
	{
		// Gets the current pixel location.
		currPixelLocation = imageMask.CurrentPixel();
		j = currPixelLocation[0];
		i = currPixelLocation[1];

		// std::cout << "Pixel " << n << " = (" << j << "," << i << ")" << std::endl;

		// Get the current context.
		currContext = getContext2D(image, j, i, ncols, this->numberOfContexts2D);

		// Fetches the current context table.
		if (contexts2D[currContext][0] > contexts2D[currContext][1])
		{
			c.mps = false;
			c.countMPS = contexts2D[currContext][0];
			c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = contexts2D[currContext][1];
			c.totalCount = contexts2D[currContext][0] + contexts2D[currContext][1];
		}

		// Decodes the current symbol.
		currSymbol = dec->decodeOneSymbol(c);

		// Inserts it into the image.
		if (currSymbol == true)
		{
			image.addPixel(j, i);
		}

		// Updates the context table.
		if (currSymbol == false)
		{
			contexts2D[currContext][0]++;
		}
		else
		{
			contexts2D[currContext][1]++;
		}

		// Move the iterator to the next pixel.
		imageMask.NextPixel();
	}
}

void Cabac::decodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask, gpcc::ImageRaster &image3D)
{
	// Get the image size.
	int nrows, ncols;
	std::tie(nrows, ncols) = image.Size();

	ContextInfo c;
	uint32_t currContext3D, currContext2D;
	bool currSymbol;

	uint64_t nSymbolsToDecode = imageMask.NumberOccupiedPixels();
	gpcc::Pixel<int> currPixelLocation;

	uint64_t n;
	uint32_t j, i;

	// std::cout << " nSymbolsToDecode = " << nSymbolsToDecode << std::endl;

	// Rewinds the mask.
	imageMask.Rewind();

	for (n = 0; n < nSymbolsToDecode; n++)
	{
		// Gets the current pixel location.
		currPixelLocation = imageMask.CurrentPixel();
		j = currPixelLocation[0];
		i = currPixelLocation[1];

		// std::cout << "Pixel " << n << " = (" << j << "," << i << ")" << std::endl;

		// Get the current context.
		currContext2D = getContext2D(image, j, i, ncols, this->numberOfContexts2D);
		currContext3D = getContext3D(image3D, j, i, nrows, ncols, this->numberOfContexts3D);

		// Fetches the current context table.
		if (contexts3D[currContext3D][currContext2D][0] > contexts3D[currContext3D][currContext2D][1])
		{
			c.mps = false;
			c.countMPS = contexts3D[currContext3D][currContext2D][0];
			c.totalCount = contexts3D[currContext3D][currContext2D][0] + contexts3D[currContext3D][currContext2D][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = contexts3D[currContext3D][currContext2D][1];
			c.totalCount = contexts3D[currContext3D][currContext2D][0] + contexts3D[currContext3D][currContext2D][1];
		}

		// Decodes the current symbol.
		currSymbol = dec->decodeOneSymbol(c);

		// Inserts it into the image.
		if (currSymbol == true)
		{
			image.addPixel(j, i);
		}

		// Updates the context table.
		if (currSymbol == false)
		{
			contexts3D[currContext3D][currContext2D][0]++;
		}
		else
		{
			contexts3D[currContext3D][currContext2D][1]++;
		}

		// Move the iterator to the next pixel.
		imageMask.NextPixel();
	}
}

//----------------------------------------
// Private utilitary functions
// Initialize 2D Contexts.
void Cabac::initContexts2D()
{
	uint32_t nContexts = 1 << this->numberOfContexts2D;

	contexts2D = std::vector<std::vector<uint32_t>>(nContexts, std::vector<uint32_t>(2, 1));
}

// Initialize 3D Contexts.
void Cabac::initContexts3D()
{
	uint32_t nC2D = 1 << this->numberOfContexts2D;
	uint32_t nC3D = 1 << this->numberOfContexts3D;

	// std::cout << "numberOfContexts3D = " << numberOfContexts3D << " -> nc3D = " << nC3D << std::endl;
	// std::cout << "numberOfContexts2D = " << numberOfContexts2D << " -> nc2D = " << nC2D << std::endl;

	contexts3D = std::vector<std::vector<std::vector<uint32_t>>>(nC3D, std::vector<std::vector<uint32_t>>(nC2D, std::vector<uint32_t>(2, 1)));
}

// Get Context2D Number from an image.
// At the moment we are considering at most 10 contexts.
/* Contexts2D
				 14
	 11  9  6 10 12
15  7  4  2  3  8 16
13  5  1  ?
*/
uint32_t Cabac::getContext2D(gpcc::ImageRaster &image, uint32_t y, uint32_t x, uint32_t ncols, uint32_t nc2D)
{
	bool pixels[10] = {false, false, false, false, false, false, false, false, false, false};

	if ((y > 2) && (x > 2) && (x < (ncols - 2)))
	{
		pixels[0] = image.PixelPresent(y, x - 1);
		pixels[1] = image.PixelPresent(y - 1, x);
		pixels[2] = image.PixelPresent(y - 1, x + 1);
		pixels[3] = image.PixelPresent(y - 1, x - 1);
		pixels[4] = image.PixelPresent(y, x - 2);
		pixels[5] = image.PixelPresent(y - 2, x);
		pixels[6] = image.PixelPresent(y - 1, x - 2);
		pixels[7] = image.PixelPresent(y - 1, x + 2);
		pixels[8] = image.PixelPresent(y - 2, x - 1);
		pixels[9] = image.PixelPresent(y - 2, x + 1);
	}
	else
	{
		pixels[0] = ((x > 1) ? image.PixelPresent(y, x - 1) : false);
		pixels[1] = ((y > 1) ? image.PixelPresent(y - 1, x) : false);
		pixels[2] = (((y > 1) && (x < (ncols - 1))) ? image.PixelPresent(y - 1, x + 1) : false);
		pixels[3] = (((y > 1) && (x > 1)) ? image.PixelPresent(y - 1, x - 1) : false);
		pixels[4] = ((x > 2) ? image.PixelPresent(y, x - 2) : false);
		pixels[5] = ((y > 2) ? image.PixelPresent(y - 2, x) : false);
		pixels[6] = (((y > 1) && (x > 2)) ? image.PixelPresent(y - 1, x - 2) : false);
		pixels[7] = (((y > 1) && (x < (ncols - 2))) ? image.PixelPresent(y - 1, x + 2) : false);
		pixels[8] = (((y > 2) && (x > 1)) ? image.PixelPresent(y - 2, x - 1) : false);
		pixels[9] = (((y > 2) && (x < (ncols - 1))) ? image.PixelPresent(y - 2, x + 1) : false);
	}

	uint32_t currContext = 0;
	uint32_t i;
	for (i = 0; i < nc2D; i++)
	{
		if (pixels[i])
			currContext += p2[i];
	}

	return currContext;
}

// Get Context3D Number from an image.
// At the moment we are considering at most 9 contexts.
/* Contexts3D
		 1  2  3
		 4  ?  6
		 7  8  9
*/
uint32_t Cabac::getContext3D(gpcc::ImageRaster &image, uint32_t y, uint32_t x, uint32_t nrows, uint32_t ncols, uint32_t nc3D)
{
	/*
	bool pixels[9] = {false, false, false, false, false, false, false, false, false};

	if ((y > 1) && (x > 1) && (y < (nrows - 1)) && (x < (ncols - 1)))
	{
		pixels[0] = image.PixelPresent(y - 1, x - 1);
		pixels[1] = image.PixelPresent(y - 1, x);
		pixels[2] = image.PixelPresent(y - 1, x + 1);
		pixels[3] = image.PixelPresent(y, x - 1);
		pixels[4] = image.PixelPresent(y, x);
		pixels[5] = image.PixelPresent(y, x + 1);
		pixels[6] = image.PixelPresent(y + 1, x - 1);
		pixels[7] = image.PixelPresent(y + 1, x);
		pixels[8] = image.PixelPresent(y + 1, x + 1);
	}
	else
	{
		pixels[0] = (((y > 1) & (x > 1)) ? image.PixelPresent(y - 1, x - 1) : false);
		pixels[1] = ((y > 1) ? image.PixelPresent(y - 1, x) : false);
		pixels[2] = (((y > 1) && (x < (ncols - 1))) ? image.PixelPresent(y - 1, x + 1) : false);
		pixels[3] = ((x > 1) ? image.PixelPresent(y, x - 1) : false);
		pixels[4] = image.PixelPresent(y, x);
		pixels[5] = ((x < (ncols - 1)) ? image.PixelPresent(y, x + 1) : false);
		pixels[6] = (((y < (nrows - 1)) && (x > 1)) ? image.PixelPresent(y + 1, x - 1) : false);
		pixels[7] = ((y < (nrows - 1)) ? image.PixelPresent(y + 1, x) : false);
		pixels[8] = (((y < (nrows - 1)) && (x < (ncols - 1))) ? image.PixelPresent(y + 1, x + 1) : false);
	}
	*/

	bool pixels[13] = { false, false, false, false, false, false, false, false, false, false, false, false, false };

	if ((y > 1) && (x > 1) && (y < (nrows - 1)) && (x < (ncols - 1)))
	{
		pixels[0] = image.PixelPresent(y - 1, x - 1);
		pixels[1] = image.PixelPresent(y - 1, x);
		pixels[2] = image.PixelPresent(y - 1, x + 1);
		pixels[3] = image.PixelPresent(y, x - 1);
		pixels[4] = image.PixelPresent(y, x);
		pixels[5] = image.PixelPresent(y, x + 1);
		pixels[6] = image.PixelPresent(y + 1, x - 1);
		pixels[7] = image.PixelPresent(y + 1, x);
		pixels[8] = image.PixelPresent(y + 1, x + 1);
	}
	else
	{
		pixels[0] = (((y > 1) & (x > 1)) ? image.PixelPresent(y - 1, x - 1) : false);
		pixels[1] = ((y > 1) ? image.PixelPresent(y - 1, x) : false);
		pixels[2] = (((y > 1) && (x < (ncols - 1))) ? image.PixelPresent(y - 1, x + 1) : false);
		pixels[3] = ((x > 1) ? image.PixelPresent(y, x - 1) : false);
		pixels[4] = image.PixelPresent(y, x);
		pixels[5] = ((x < (ncols - 1)) ? image.PixelPresent(y, x + 1) : false);
		pixels[6] = (((y < (nrows - 1)) && (x > 1)) ? image.PixelPresent(y + 1, x - 1) : false);
		pixels[7] = ((y < (nrows - 1)) ? image.PixelPresent(y + 1, x) : false);
		pixels[8] = (((y < (nrows - 1)) && (x < (ncols - 1))) ? image.PixelPresent(y + 1, x + 1) : false);
	}

	if ((y > 2) && (x > 2) && (y < (nrows - 2)) && (x < (ncols - 2)))
	{
		pixels[9]  = image.PixelPresent(y - 2, x    );
		pixels[10] = image.PixelPresent(y    , x - 2);
		pixels[11] = image.PixelPresent(y    , x + 2);
		pixels[12] = image.PixelPresent(y + 2, x    );		
	}
	else
	{
		pixels[9] = ((y > 2) ? image.PixelPresent(y - 2, x) : false);
		pixels[10] = ((x > 2) ? image.PixelPresent(y, x - 2) : false);
		pixels[11] = ((x < (ncols - 2)) ? image.PixelPresent(y, x + 2) : false);
		pixels[12] = ((y < (nrows - 2)) ? image.PixelPresent(y + 2, x) : false);		
	}

	uint32_t currContext = 0;
	uint32_t i;
	for (i = 0; i < nc3D; i++)
	{
		if (pixels[i])
			currContext += p2[i];
	}
	
	return currContext;
}

void Cabac::copyCabac(Cabac *cabacCopy)
{
	this->mode = cabacCopy->mode;

	this->m = cabacCopy->m;
	this->numberOfContexts2D = cabacCopy->numberOfContexts2D;
	this->numberOfContexts3D = cabacCopy->numberOfContexts3D;

	this->enc = new BACEncoder();
	this->enc->copyBACEncoder(cabacCopy->enc);

	this->contexts2D = cabacCopy->contexts2D;
	this->contexts3D = cabacCopy->contexts3D;

	this->paramBitstream->copyBitstream(cabacCopy->paramBitstream);
	this->acbstream->copyBitstream(cabacCopy->acbstream);
}

void Cabac::printStatusEncoder()
{
	enc->showStatus();
}

void Cabac::printStatusDecoder()
{
	dec->showStatus();
}

Bitstream *Cabac::getDecoderBitstream()
{
	return this->dec->getBitstream();
}
