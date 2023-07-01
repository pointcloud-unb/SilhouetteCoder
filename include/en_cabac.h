#ifndef CABAC_H
#define CABAC_H

#include <stdint.h>
#include <vector>
#include <tuple>
#include "ac_bac_encoder.h"
#include "ac_bac_decoder.h"
#include "ac_bitstream.h"
#include "ds_iimage.h"
#include "ds_image_sparse.h"
#include "en_codec_parameter_bitstream.h"

class Cabac
{

	enum class CabacMode
	{
		Encoder,
		Decoder,
		Undefined
	};

private:
	CabacMode mode;

	uint64_t m;
	uint64_t numberOfContexts2D;
	uint64_t numberOfContexts3D;
	BACEncoder *enc;
	BACDecoder *dec;

	std::vector<std::vector<uint32_t>> contexts2D;
	std::vector<std::vector<std::vector<uint32_t>>> contexts3D;

	const uint32_t p2[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

public:
	// Main constructor
	Cabac();

	~Cabac();

	// Encoding Constructor
	void initEncoder(uint64_t m, uint64_t nc2D, uint64_t nc3D);

	// Main Encoding Functions
	void encodeImage(gpcc::ImageRaster &image);
	void encodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask);
	void encodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask, gpcc::ImageRaster &image3D);

	void closeEncoder(Bitstream &bstream);

	// Decoding Constructor
	void initDecoder(uint64_t m, uint64_t nc2D, uint64_t nc3D, Bitstream &bstream);

	// Main Decoding Functions
	void decodeImage(gpcc::ImageRaster &image);
	void decodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask);
	void decodeImage(gpcc::ImageRaster &image, gpcc::ImageSparse &imageMask, gpcc::ImageRaster &image3D);

	void copyCabac(Cabac *cabacCopy);

	void printStatusEncoder();
	void printStatusDecoder();
	Bitstream* getDecoderBitstream();

	CodecParameterBitstream *paramBitstream;
	Bitstream *acbstream;
		
private:
	void initContexts2D();
	void initContexts3D();

	uint32_t getContext2D(gpcc::ImageRaster &image, uint32_t y, uint32_t x, uint32_t ncols, uint32_t nc2D);
	uint32_t getContext3D(gpcc::ImageRaster &image, uint32_t y, uint32_t x, uint32_t nrows, uint32_t ncols, uint32_t nc3D);
};

#endif // CABAC_H
