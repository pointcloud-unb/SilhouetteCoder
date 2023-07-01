#include <iostream>

#include "ac_bac_encoder.h"

//----------------------------------------
// Constructors
BACEncoder::BACEncoder() : m(0),
													 MSBMask(0),
													 validBitsMask(0),
													 ln(0),
													 un(0)
{
	m = 12;
	MSBMask = (uint64_t(1) << (m - 1));
	validBitsMask = (uint64_t(1) << m) - 1;
	ln = 0;
	un = (uint64_t(1) << m) - 1;

	// std::cout << "m             = " << m << std::endl;
	// std::cout << "MSBMask       = " << MSBMask << std::endl;
	// std::cout << "validBitsMask = " << validBitsMask << std::endl;
	// std::cout << "ln            = " << ln << std::endl;
	// std::cout << "un            = " << un << std::endl;
}

BACEncoder::BACEncoder(uint64_t nbits) : m(0),
																				 MSBMask(0),
																				 validBitsMask(0),
																				 ln(0),
																				 un(0)
{
	m = nbits;
	MSBMask = (uint64_t(1) << (m - 1));
	validBitsMask = (uint64_t(1) << m) - 1;
	ln = 0;
	un = (uint64_t(1) << m) - 1;

	// std::cout << "m             = " << m << std::endl;
	// std::cout << "MSBMask       = " << MSBMask << std::endl;
	// std::cout << "validBitsMask = " << validBitsMask << std::endl;
	// std::cout << "ln            = " << ln << std::endl;
	// std::cout << "un            = " << un << std::endl;
}

// Destructors
BACEncoder::~BACEncoder()
{
	// Don't have to do anything really.
}

//----------------------------------------
// Public Functions

// Returns the current state of the encoder (ln,un).
// It does NOT change the bitstream or takes care of the contexts!
BACState BACEncoder::getState()
{
	BACState state;
	state.ln = this->ln;
	state.ln = this->un;

	return state;
}

// This function overrides the current state.
// HANDLE WITH CARE!
void BACEncoder::setState(BACState bacState)
{
	this->ln = bacState.ln;
	this->un = bacState.un;
}

// Encodes one symbol with a particular probability table and write the bits
// in a bitstream pointer.
void BACEncoder::encodeOneSymbol(bool symbol, ContextInfo c, Bitstream &bitstream)
{
	bool isMPS = (symbol == c.mps);
	uint64_t l0 = this->ln;
	uint64_t u0 = this->un;
	uint64_t l1, u1;
	uint64_t temp;

	// Updates the intervals
	if (isMPS)
	{
		l1 = l0;

		temp = ((u0 - l0 + 1) * c.countMPS) / c.totalCount;
		u1 = l0 + temp - 1;
	}
	else
	{
		temp = ((u0 - l0 + 1) * c.countMPS) / c.totalCount;
		l1 = l0 + temp;

		u1 = u0;
	}

	// Checks if I have to flush bits.
	bool ok = true;
	bool msb_l, msb_u;

	// std::cout << "Bits = [ ";
	while (ok)
	{
		msb_l = (l1 & this->MSBMask) != 0;
		msb_u = (u1 & this->MSBMask) != 0;

		if (msb_l == msb_u)
		{
			// std::cout << (msb_l ? "1" : "0") << " ";
			// If they are the same, I have to flush this into the bitstream.
			bitstream.writeBit(msb_l);

			// Then I have to flush it out of l1and u1.
			l1 = ((l1 << 1) & validBitsMask);
			u1 = ((u1 << 1) & validBitsMask) + 1;
		}
		else
		{
			ok = false;
		}
	}

	// std::cout << "]" << std::endl;
	// std::cout << "ln            = " << l1 << std::endl;
	// std::cout << "un            = " << u1 << std::endl;
	// std::cout << std::endl;

	// Updates the encoder.
	this->ln = l1;
	this->un = u1;
}

// Closes the bitstream.
void BACEncoder::closeBitstream(Bitstream &bitstream)
{
	uint64_t i = this->m;
	uint64_t l0 = this->ln;
	bool bit;

	while (i > 0)
	{
		bit = ((l0 & this->MSBMask) != 0);
		l0 = ((l0 << 1) & this->validBitsMask);
		bitstream.writeBit(bit);
		i--;
	}
}

// Closes the bitstream.
void BACEncoder::showStatus()
{
	std::cout << "ENCODER STATUS: " << std::endl;
	std::cout << "ln            = " << this->ln << std::endl;
	std::cout << "un            = " << this->un << std::endl;
	std::cout << std::endl;
}

void BACEncoder::copyBACEncoder(BACEncoder *BACEndoderCopy)
{
	this->m = BACEndoderCopy->m;
	this->MSBMask = BACEndoderCopy->MSBMask;
	this->validBitsMask = BACEndoderCopy->validBitsMask;
	this->ln = BACEndoderCopy->ln;
	this->un = BACEndoderCopy->un;
}
