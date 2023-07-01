#ifndef BITSTREAM_H
#define BITSTREAM_H

#include <vector>
#include <stdint.h>
#include <string>

enum Bitstream_Mode : uint8_t
{
	WRITE = 0,
	READ = 1,
	NUMBER_OF_MODES = 2
};

enum Bitstream_Reading_Status : uint8_t
{
	NOT_STARTED = 0,
	READING = 1,
	FINISHED = 2,
	NUMBER_OF_READING_STATUS = 2
};

/*
	Bitstream class that accumulates and reads bits.
	It has all functions to handle writing and loading from file.
	It has built-in functions to merge between two different bitstreams.
	The bitstream is either WRITE ONLY or READ ONLY.
*/
class Bitstream
{
	std::vector<uint8_t> data;

	uint8_t num_buf8; /// number of bits not flushed to data.
	uint8_t buf8;			/// the bits held and not flushed to data.

	uint64_t bitstream_pointer;

	Bitstream_Mode mode;
	Bitstream_Reading_Status reading_status;
	uint8_t reading_num_valid_bits_last_byte;

public:
	Bitstream();
	Bitstream(std::string filename);
	Bitstream(Bitstream &bs2, uint64_t nbits);
	~Bitstream();

	void writeBit(bool bit);

	void merge(Bitstream bs);

	void changeModeToRead();

	uint64_t totalSize();
	uint64_t numberOfRemainingBits();
	bool hasBits() { return (reading_status == READING); };

	void flushesToFile(std::string filename);

	bool readBit();
	void rewind();

	void writeNumber(uint64_t num, uint64_t nBits);
	uint64_t readNumber(uint64_t nBits);

	void copyBitstream(Bitstream *bitstreamCopy);
};

#endif // BITSTREAM_H
