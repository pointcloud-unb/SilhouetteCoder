#include <iostream>
#include <fstream>

#include "ac_bitstream.h"

//----------------------------------------
// Constructors
// Creates an empty, writeable bitstream.
// The bitstream created is returned in WRITE mode.
Bitstream::Bitstream() : data(),
												 num_buf8(0),
												 buf8(0),
												 bitstream_pointer(0),
												 reading_num_valid_bits_last_byte(0),
												 mode(WRITE),
												 reading_status(NOT_STARTED)
{
	data.reserve(1024);
}

// Creates a new bitstream object by reading nbits from an input bitstream
// These bits are consumed from the bitstream bs2, which should be in READ mode.
// The bitstream created is returned in READ mode.
Bitstream::Bitstream(Bitstream &bs2, uint64_t nbits) : data(),
																											 num_buf8(0),
																											 buf8(0),
																											 bitstream_pointer(0),
																											 reading_num_valid_bits_last_byte(0),
																											 mode(WRITE),
																											 reading_status(NOT_STARTED)
{
	// assert (bs2.mode == READ)

	if (nbits <= bs2.numberOfRemainingBits())
	{
		data.reserve(nbits * 8);

		bool bit;
		for (uint64_t i = 0; i < nbits; i++)
		{
			bit = bs2.readBit();
			this->writeBit(bit);
		}

		// After this, changes the mode of bs2.
		this->changeModeToRead();
	}
	else
	{
		// BIG BUG! EXCEPTION! DESTROY!
		std::cout << "Bitstream::Tried to cut more bits than I have." << std::endl;
	}
}

// Creates a new bitstream from a file.
// The whole file is consumed within the constructor - all bits are stored in memory.
// The file is closed before the constructor returns.
// The bitstream created is returned in READ mode.
Bitstream::Bitstream(std::string filename) : data(),
																						 num_buf8(0),
																						 buf8(0),
																						 bitstream_pointer(0),
																						 reading_num_valid_bits_last_byte(0),
																						 mode(READ),
																						 reading_status(NOT_STARTED)
{

	std::ifstream file;
	unsigned int nbytes;

	// ios::in : read in input mode
	// ios::binary : read in binary mode
	// ios::ate : set the initial position at the end of the file.
	file.open(filename, std::ios::in | std::ios::binary | std::ios::ate);

	if (file.is_open())
	{
		// Reads the number of bytes in the file.
		nbytes = (unsigned int)file.tellg();

		// std::cout << "Number of bytes = " << nbytes << std::endl;

		// Rewinds the file pointer.
		file.seekg(0, std::ios::beg);

		// Reads the first byte.
		uint8_t first_byte;
		file.read(reinterpret_cast<char *>(&first_byte), 1 * sizeof(first_byte));

		// std::cout << "First byte: " << std::hex << static_cast<int> (first_byte) << std::endl;

		// Tests the first nibble.
		if ((first_byte & 0xF0) == 0xE0)
		{
			if (nbytes == 1)
			{
				// This is not an error, but...
				std::cout << "The bitstream has only one byte (the header)." << std::endl;
			}
			else
			{
				reading_num_valid_bits_last_byte = first_byte & 0x0F;

				// Reserves the number of bytes.
				data.resize(nbytes - 1);
				// Reads all bytes to the data vector.
				file.read(reinterpret_cast<char *>(&data[0]), nbytes * sizeof(data[0]));

				// int i = 0;
				// for (auto& elem : data)
				//{
				//	std::cout << "Vec[" << i << "] = " << std::hex << static_cast<int> (elem) << std::endl;
				//	i++;
				// }

				// If all went well
				buf8 = data[0];						// Gets the first byte.
				num_buf8 = 8;							// Number of unread bits in this buffer.
				bitstream_pointer = 1;		// points to the next byte to be read.
				reading_status = READING; // status: reading!
			}
		}
		else
		{
			// BIG BAD ERROR
			std::cout << "The input binary file is not conforming to the Geometry Coder." << std::endl;
		}

		file.close();
	}
	else
	{
		// DO SOMETHING
		// Something is not right...
		// BIG BAD BUG
		std::cout << "NAO CONSEGUIU ABRIR O ARQUIVO." << std::endl;
	}

	data.reserve(1024);
}

// Destructors
Bitstream::~Bitstream()
{
	// Don't have to do anything really.
}

//----------------------------------------
// Public Functions
void Bitstream::writeBit(bool bit)
{
	buf8 <<= 1;
	buf8 |= uint8_t(bit);

	num_buf8++;

	if (num_buf8 == 8)
	{
		data.push_back(buf8);

		buf8 = 0;
		num_buf8 = 0;
		bitstream_pointer++;
	}
}

// This function merges the received bitstream bs to the current bitstream.
void Bitstream::merge(Bitstream bs)
{
	// Pushes all bits from bs to the current bitstream.
	uint8_t currByte;
	bool currBit;

	// First, I need to push what is in the byte buffer (data)
	//  uint64_t N = bs.data.size();

	for (auto &element : bs.data)
	{
		currByte = uint8_t(element);
		for (int i = 0; i < 8; i++)
		{
			currBit = ((currByte & 0x80) == 0x80);
			currByte <<= 1;

			this->writeBit(currBit);
		}
	}

	// Then, I need to do the same to the bits in the buffer.
	if (bs.num_buf8 > 0)
	{
		// Copies the buffer
		currByte = bs.buf8;

		// Pushes bits to byte align.
		currByte <<= (8 - bs.num_buf8);

		// Gets the correct number of bits
		for (int i = 0; i < bs.num_buf8; i++)
		{
			currBit = ((currByte & 0x80) == 0x80);
			currByte <<= 1;

			this->writeBit(currBit);
		}
	}
}

// Changes the bitstreamMode from WRITE to READ.
void Bitstream::changeModeToRead()
{
	// assert(mode == WRITE)

	// If there is anything in the temporary buffer, complete it and flushes to the data.
	if (num_buf8 != 0)
	{
		uint8_t temp = buf8;
		temp <<= (8 - num_buf8);
		data.push_back(temp);
	}

	// The number of valid bits in the last byte is whatever was in the buffer at this point.
	reading_num_valid_bits_last_byte = num_buf8;

	// Sets the bitstream in READ mode
	buf8 = data[0];						// Gets the first byte.
	num_buf8 = 8;							// Number of unread bits in this buffer.
	bitstream_pointer = 1;		// points to the next byte to be read.
	reading_status = READING; // status: reading!
	mode = READ;
}

// The size acconts for all bits already flushed to the data, plus the bits held in the small internal buffer,
// The size of the bitstream depends on if it is being written or read.
uint64_t Bitstream::totalSize()
{
	if (mode == WRITE)
	{
		return 8 * bitstream_pointer + num_buf8;
	}
	else
	{
		return 8 * (data.size() - 1) + reading_num_valid_bits_last_byte;
	}
}

// This only makes sense in the reading mode, otherwise it returns 0.
// It returns the number of bits left to read in the bitstream.
uint64_t Bitstream::numberOfRemainingBits()
{
	if (mode == WRITE)
	{
		return 0;
	}
	else
	{
		uint64_t total = 8 * (data.size() - 1) + reading_num_valid_bits_last_byte;
		uint64_t read = (bitstream_pointer < data.size()) ? (8 * (bitstream_pointer - 1) + 8 - num_buf8) : (8 * (bitstream_pointer - 1) + reading_num_valid_bits_last_byte - num_buf8);
		return (total - read);
	}
}

// The first byte written is output as:
//   - 1110  - Hexadecimal 'E'
//   - bbbb  - where bbb is the number of valid bits in the last byte.
void Bitstream::flushesToFile(std::string filename)
{
	std::ofstream file;

	file.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

	if (file.is_open())
	{
		// Computes and writes the first byte
		uint8_t num_valid_bits_in_last_byte = ((num_buf8 == 0) ? 8 : num_buf8);
		uint8_t first_byte = uint8_t(0xE0) | uint8_t(num_valid_bits_in_last_byte);

		file.write(reinterpret_cast<char *>(&first_byte), sizeof(first_byte));

		// Writes the data that is already packed to 8 bits.
		file.write(reinterpret_cast<char *>(&data[0]), data.size() * sizeof(data[0]));

		if (num_buf8 > 0)
		{
			// Computes and writes the last byte.
			uint8_t last_byte = buf8;
			uint8_t i = 8 - num_buf8;
			while (i != 0)
			{
				last_byte <<= 1;
				i--;
			}

			file.write(reinterpret_cast<char *>(&last_byte), sizeof(last_byte));
		}

		file.close();
	}
	else
	{
		// BIG BUG! EXCEPTION! DESTROY!
		std::cout << "DEU RUIM NO ARQUIVO." << std::endl;
	}
}

bool Bitstream::readBit()
{
	// assert(mode == READ); ?

	bool bit = false;

	if (reading_status == READING)
	{
		bit = ((buf8 & 0x80) == 0x80);

		buf8 <<= 1;

		num_buf8--;

		if (num_buf8 == 0)
		{
			// Is there a next byte?
			if (bitstream_pointer < (data.size() - 1))
			{
				// Just grabs the next byte.
				buf8 = data[bitstream_pointer++];
				num_buf8 = 8;
			}
			else if (bitstream_pointer < (data.size()))
			{
				// Gets the next byte.
				buf8 = data[bitstream_pointer++];
				// Adjusts the number of usable bits.
				num_buf8 = reading_num_valid_bits_last_byte;
			}
			else
			{
				// This was the last bit.
				reading_status = FINISHED;
			}
		}
	}
	else
	{
		// USAGE ERROR
		std::cout << "Error - Attempting to read a " << (reading_status == NOT_STARTED ? "NOT STARTED" : "FINISHED") << "bitstream." << std::endl;
	}

	return bit;
}

void Bitstream::rewind()
{
	bitstream_pointer = 0;
}

void Bitstream::writeNumber(uint64_t num, uint64_t nBits)
{
	uint64_t currMask = uint64_t(1) << (nBits - uint64_t(1));
	// uint64_t validMask = (uint64_t(1) << nBits) - uint64_t(1);

	// std::cout << "Writing number " << num << " using " << nBits << " bits." << std::endl;
	// std::cout << "validMask = " << std::hex << validMask << std::dec << std::endl;
	// std::cout << "mask      = " << std::hex << currMask << std::dec << std::endl;
	// std::cout << "Binary = ";

	while (nBits != 0)
	{
		bool currBit = num & currMask;
		// std::cout << (currBit ? "1 " : "0 ");
		this->writeBit(currBit);

		num <<= 1;
		nBits--;
	}
	// std::cout << std::endl;
}

uint64_t Bitstream::readNumber(uint64_t nBits)
{
	uint64_t num = 0;
	uint64_t validMask = (uint64_t(1) << nBits) - uint64_t(1);

	while (nBits != 0)
	{
		bool currBit = this->readBit();

		num <<= 1;
		num += uint64_t(currBit);
		nBits--;
	}

	num &= validMask;
	return num;
}

void Bitstream::copyBitstream(Bitstream *bitstreamCopy)
{
	this->data = bitstreamCopy->data;
	this->num_buf8 = bitstreamCopy->num_buf8;
	this->buf8 = bitstreamCopy->buf8;
	this->bitstream_pointer = bitstreamCopy->bitstream_pointer;
	this->mode = bitstreamCopy->mode;
	this->reading_status = bitstreamCopy->reading_status;
	this->reading_num_valid_bits_last_byte = bitstreamCopy->reading_num_valid_bits_last_byte;
}
