#ifndef BACDECODER_H
#define BACDECODER_H

#include <stdint.h>
#include "ac_context.h"
#include "ac_bitstream.h"

class BACDecoder
{
private:
	uint64_t m;
	uint64_t MSBMask;
	uint64_t validBitsMask;
	uint64_t ln;
	uint64_t un;
	Bitstream *pBitstream;
	uint64_t currTag;
	uint64_t nDecoded;

public:
	BACDecoder(Bitstream *bs);
	BACDecoder(Bitstream *bs, uint64_t nbits);
	~BACDecoder();

	void init();
	bool decodeOneSymbol(ContextInfo c);

	void showStatus();
	Bitstream *getBitstream();
};

#endif // BACDECODER_H