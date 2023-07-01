#ifndef CODECPARAMETERBITSTREAM_H
#define CODECPARAMETERBITSTREAM_H

#include <stdint.h>
#include <vector>
#include "ac_bitstream.h"

// This class encapsulates a 1D array of binary symbols used as flags in the codec.
// At the encoder it can:
//  - add a symbol to the stream
//  - encode these symbols with a given arithmetic coder
// At the decoder it can
//  - parse the arithmetic coded bitstream
//  - read a symbol from the stream
class CodecParameterBitstream
{
private:
	std::vector<bool> message;
	uint64_t p;
	uint64_t length;

public:
	CodecParameterBitstream();
	CodecParameterBitstream(Bitstream &bstream, uint64_t m, uint32_t numberOfContextBits);

	void putBit(bool bit);
	Bitstream encode(uint64_t m, uint32_t numberOfContextBits);

	bool readBit();
	uint64_t totalSize() { return length; }

	void copyBitstream(CodecParameterBitstream* bitstreamCopy);

private:
	std::vector<std::vector<uint64_t>> initContexts(uint32_t numberOfContextBits);
	void printCountTable(std::vector<std::vector<uint64_t>> &countsTable);
};

#endif // CODECPARAMETERBITSTREAM_H
