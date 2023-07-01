#ifndef BACENCODER_H
#define BACENCODER_H

#include <stdint.h>
#include "ac_context.h"
#include "ac_bitstream.h"

typedef struct BACState
{
	uint64_t ln;
	uint64_t un;
} BACSTATE;

class BACEncoder {
	private:
		uint64_t m;
		uint64_t MSBMask;
		uint64_t validBitsMask;
		uint64_t ln;
		uint64_t un;

	public:
		BACEncoder();
		BACEncoder(uint64_t nbits);
		~BACEncoder();

		BACState getState();
		void setState(BACState bacState);

		void encodeOneSymbol(bool symbol, ContextInfo c, Bitstream& bitstream);

		void closeBitstream(Bitstream& bitstream);

		void showStatus();

		void copyBACEncoder(BACEncoder* BACEndoderCopy);
};


#endif // BACENCODER_H