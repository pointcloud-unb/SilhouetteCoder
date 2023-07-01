#include <iostream>

#include "en_codec_parameter_bitstream.h"
#include "ac_bac_encoder.h"
#include "ac_bac_decoder.h"
#include "ac_context.h"

#define DEBUG_CODECPARAMETERBITSTREAM 0
//----------------------------------------
// Constructors
CodecParameterBitstream::CodecParameterBitstream() : p(0),
																										 length(0)
{
	message.reserve(1024);
}

CodecParameterBitstream::CodecParameterBitstream(Bitstream &bstream, uint64_t m, uint32_t numberOfContextBits) : p(0),
																																																								 length(0)
{
	// Reads the length (i.e., the first 16 bits).
	uint64_t message_length = 0;

	// The first 16 bits of the bitstream are the number of symbols.
	uint64_t numberValidMask = (1 << 16) - 1;
	uint64_t numberMSBMask = 1 << 15;

	// Debug
#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "Bits = "
						<< " = ";
#endif
	for (int k = 0; k < 16; k++)
	{
		bool bit = bstream.readBit();

		message_length <<= 1;
		message_length += uint64_t(bit);
		message_length &= numberValidMask;

#if DEBUG_CODECPARAMETERBITSTREAM
		std::cout << (bit ? 1 : 0);
#endif
	}

#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << " -> " << message_length << " ." << std::endl;
#endif
	this->message.reserve(message_length);

	// Initialize the contextValue
	uint64_t currContext = 0;
	uint64_t contextBitMask = (uint64_t(1) << numberOfContextBits) - 1;

	// Initialize the counts Table.
	std::vector<std::vector<uint64_t>> countTable = initContexts(numberOfContextBits);

#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "Start:" << std::endl;
	printCountTable(countTable);
#endif

	// Initialize the BACDecoder and the ContextInfo
	BACDecoder dec(&bstream, m);
	ContextInfo c;

	// Initialize decoder
	dec.init();

	// loops through all the bits in the message encoding them.
	uint32_t i;
	for (i = 0; i < message_length; i++)
	{
		// Gets the currentContext.
		currContext <<= 1;
		currContext += ((i > 0) ? uint64_t(message[i - 1]) : 0);
		currContext &= contextBitMask;

#if DEBUG_CODECPARAMETERBITSTREAM
		std::cout << "Last K bits = ";
		for (int k = numberOfContextBits; k > 0; k--)
		{
			std::cout << ((i >= k) ? (message[i - k] ? 1 : 0) : 0) << " ";
		}
		std::cout << " -> "
							<< "currContext = " << currContext << std::endl;
#endif

		// Fetches the current context table.
		if (countTable[currContext][0] > countTable[currContext][1])
		{
			c.mps = false;
			c.countMPS = countTable[currContext][0];
			c.totalCount = countTable[currContext][0] + countTable[currContext][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = countTable[currContext][1];
			c.totalCount = countTable[currContext][0] + countTable[currContext][1];
		}

		// Decodes the current symbol using this context.
		bool currSymbol = dec.decodeOneSymbol(c);
		this->putBit(currSymbol);

		// Updates the context table.
		if (currSymbol == false)
		{
			countTable[currContext][0]++;
		}
		else
		{
			countTable[currContext][1]++;
		}
	}

	// Debug
#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "End:" << std::endl;
	printCountTable(countTable);
#endif

	// Validates and return this bitstream.
	this->p = 0;
	this->length = message_length;
}

//----------------------------------------
// Functions used at the encoder side
void CodecParameterBitstream::putBit(bool bit)
{
	message.push_back(bit);
	p++;
	length++;
}

Bitstream CodecParameterBitstream::encode(uint64_t m, uint32_t numberOfContextBits)
{
	// Initialize the contextValue
	uint64_t currContext = 0;
	uint64_t contextBitMask = (uint64_t(1) << numberOfContextBits) - 1;

	// Initialize the counts Table.
	std::vector<std::vector<uint64_t>> countTable = initContexts(numberOfContextBits);

	// Debug
#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "Start:" << std::endl;
	printCountTable(countTable);
#endif

	// Initialize the BACEncoder, the ContextInfo and the bitstream
	BACEncoder bac(m);
	ContextInfo c;
	Bitstream bstream;

	// The first 16 bits of the bitstream are the number of symbols.
	uint64_t numberValidMask = (1 << 16) - 1;
	uint64_t numberMSBMask = 1 << 15;
	uint64_t message_length = this->length;

#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "Length = " << message_length << " = ";
#endif
	for (int k = 0; k < 16; k++)
	{
		bool bit = ((message_length & numberMSBMask) != 0);
		bstream.writeBit(bit);

		message_length <<= 1;
		message_length &= numberValidMask;

#if DEBUG_CODECPARAMETERBITSTREAM
		std::cout << (bit ? 1 : 0);
#endif
	}
#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << std::endl;
#endif

	// loops through all the bits in the message encoding them.
	uint32_t i;
	for (i = 0; i < this->length; i++)
	{
		// Gets the currentContext.
		currContext <<= 1;
		currContext += ((i > 0) ? uint64_t(message[i - 1]) : 0);
		currContext &= contextBitMask;

#if DEBUG_CODECPARAMETERBITSTREAM
		std::cout << "Last K bits = ";
		for (int k = numberOfContextBits; k > 0; k--)
		{
			std::cout << ((i >= k) ? (message[i - k] ? 1 : 0) : 0) << " ";
		}
		std::cout << " -> "
							<< "currContext = " << currContext << std::endl;
#endif

		// Fetches the current context table.
		if (countTable[currContext][0] > countTable[currContext][1])
		{
			c.mps = false;
			c.countMPS = countTable[currContext][0];
			c.totalCount = countTable[currContext][0] + countTable[currContext][1];
		}
		else
		{
			c.mps = true;
			c.countMPS = countTable[currContext][1];
			c.totalCount = countTable[currContext][0] + countTable[currContext][1];
		}

		// Encodes the current symbol using this context.
		bac.encodeOneSymbol(message[i], c, bstream);

		// Updates the context table.
		if (message[i] == false)
		{
			countTable[currContext][0]++;
		}
		else
		{
			countTable[currContext][1]++;
		}
	}

	// Closes the bitstream.
	bac.closeBitstream(bstream);

	// Debug
#if DEBUG_CODECPARAMETERBITSTREAM
	std::cout << "End:" << std::endl;
	printCountTable(countTable);
#endif

	return bstream;
}

//----------------------------------------
// Functions used at the decoder side
bool CodecParameterBitstream::readBit()
{
	// Should I check for validity??
	return message[p++];
}

std::vector<std::vector<uint64_t>> CodecParameterBitstream::initContexts(uint32_t numberOfContextBits)
{
	uint64_t numberOfContexts = uint64_t(1) << numberOfContextBits;
	std::vector<std::vector<uint64_t>> countsTable(numberOfContexts, std::vector<uint64_t>(2, 1));

	return countsTable;
}

void CodecParameterBitstream::printCountTable(std::vector<std::vector<uint64_t>> &countsTable)
{
	size_t numberOfContexts = countsTable.size();

	std::cout << "Count Table: " << std::endl;
	for (int k = 0; k < numberOfContexts; k++)
	{
		std::cout << "Context Number " << k << " (" << countsTable[k][0] << "," << countsTable[k][1] << ")" << std::endl;
	}
}

void CodecParameterBitstream::copyBitstream(CodecParameterBitstream *bitstreamCopy)
{
	this->message = bitstreamCopy->message;
	this->length = bitstreamCopy->length;
	this->p = bitstreamCopy->p;
}
