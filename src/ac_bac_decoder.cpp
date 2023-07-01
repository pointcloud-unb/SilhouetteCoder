#include <iostream>

#include "ac_bac_decoder.h"

//----------------------------------------
// Constructors
BACDecoder::BACDecoder(Bitstream *bs) : m(0),
																				MSBMask(0),
																				validBitsMask(0),
																				ln(0),
																				un(0),
																				pBitstream(NULL),
																				currTag(0),
																				nDecoded(0)
{
	m = 12;
	MSBMask = (uint64_t(1) << (m - 1));
	validBitsMask = (uint64_t(1) << m) - 1;
	ln = 0;
	un = (uint64_t(1) << m) - 1;
	pBitstream = bs;
	currTag = 0;
	nDecoded = 0;

	// std::cout << "m             = " << m << std::endl;
	// std::cout << "MSBMask       = " << MSBMask << std::endl;
	// std::cout << "validBitsMask = " << validBitsMask << std::endl;
	// std::cout << "ln            = " << ln << std::endl;
	// std::cout << "un            = " << un << std::endl;
}

BACDecoder::BACDecoder(Bitstream *bs, uint64_t nbits) : m(0),
																												MSBMask(0),
																												validBitsMask(0),
																												ln(0),
																												un(0),
																												pBitstream(NULL),
																												currTag(0),
																												nDecoded(0)
{
	m = nbits;
	MSBMask = (uint64_t(1) << (m - 1));
	validBitsMask = (uint64_t(1) << m) - 1;
	ln = 0;
	un = (uint64_t(1) << m) - 1;
	pBitstream = bs;
	currTag = 0;
	nDecoded = 0;

	// std::cout << "m             = " << m << std::endl;
	// std::cout << "MSBMask       = " << MSBMask << std::endl;
	// std::cout << "validBitsMask = " << validBitsMask << std::endl;
	// std::cout << "ln            = " << ln << std::endl;
	// std::cout << "un            = " << un << std::endl;
}

// Destructors
BACDecoder::~BACDecoder()
{
	// Don't have to do anything really.
}

//----------------------------------------
// Public Functions
// Consumes the first m bits in the bitstream, and initializes the currTag.
void BACDecoder::init()
{
	if (m <= pBitstream->numberOfRemainingBits())
	{
		bool bit;
		uint64_t i = m;

		while (i > 0)
		{
			bit = pBitstream->readBit();
			currTag <<= 1;
			currTag = (bit ? (currTag + 1) : currTag);
			i--;
		}

		// std::cout << "currTag            = " << currTag << std::endl;
	}
	else
	{
		// BIG BUG! EXCEPTION! DESTROY!
		std::cout << "BACDecoder::Tried to read more bits than I have." << std::endl;
	}
}

// Decodes one symbol using the contextInfo c.
// Consumes bits from the internal bitstream.
bool BACDecoder::decodeOneSymbol(ContextInfo c)
{
	bool mps = c.mps;
	bool lps = !(mps);
	bool symbol;

	uint64_t l0 = this->ln;
	uint64_t u0 = this->un;
	uint64_t l1, u1;
	uint64_t tagStar, temp;

	// Computes the tagStar
	tagStar = ((currTag - l0 + 1) * c.totalCount - 1) / (u0 - l0 + 1);

	// Decodes the symbol.
	if (tagStar < c.countMPS)
	{
		symbol = mps;

		l1 = l0;

		temp = ((u0 - l0 + 1) * c.countMPS) / c.totalCount;
		u1 = l0 + temp - 1;
	}
	else
	{
		symbol = lps;

		temp = ((u0 - l0 + 1) * c.countMPS) / c.totalCount;
		l1 = l0 + temp;

		u1 = u0;
	}

	// Checks if I have to flush bits.
	bool ok = true;
	bool msb_l, msb_u;
	bool bit;

	while (ok)
	{
		msb_l = (l1 & this->MSBMask) != 0;
		msb_u = (u1 & this->MSBMask) != 0;

		if (msb_l == msb_u)
		{
			// If they are the same, I have to read a Bit from the bitstream.
			bit = pBitstream->readBit();

			// Then I have to flush it out of l1and u1.
			l1 = ((l1 << 1) & validBitsMask);
			u1 = ((u1 << 1) & validBitsMask) + 1;

			// Updates the tag.
			currTag = ((currTag << 1) & validBitsMask); // Flushes one bit out
			currTag += (bit ? 1 : 0);										// Inserts the bit read into the tag.
		}
		else
		{
			ok = 0;
		}
	}

	// std::cout << "Symbol " << nDecoded << " = " << (symbol ? "1" : "0") << std::endl;
	// std::cout << "currTag       = " << currTag << std::endl;
	// std::cout << "ln            = " << l1 << std::endl;
	// std::cout << "un            = " << u1 << std::endl;
	// std::cout << std::endl;

	// Updates the decoder.
	this->ln = l1;
	this->un = u1;
	this->nDecoded++;

	return symbol;
}

// Closes the bitstream.
void BACDecoder::showStatus()
{
	std::cout << "DECODER STATUS: " << std::endl;
	std::cout << "ln            = " << this->ln << std::endl;
	std::cout << "un            = " << this->un << std::endl;
	std::cout << "Number of Decoded Symbols = " << this->nDecoded << std::endl;
	std::cout << "Number of bits in bitstream = " << this->pBitstream->numberOfRemainingBits() << std::endl;
	std::cout << std::endl;
}

Bitstream *BACDecoder::getBitstream()
{
	return this->pBitstream;
}
