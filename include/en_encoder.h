#ifndef ENCODER_H
#define ENCODER_H

#include <string>

#include "ds_point_cloud.h"
#include "cg_config.h"
#include "en_codec_parameter_bitstream.h"
#include "en_cabac.h"
#include "ds_silhouette_decomposition.h"
#include "ac_bitstream.h"

namespace gpcc
{

    typedef struct
    {
        Cabac *cabacOut;
        uint32_t encodingbitsSize;
    } encodeReturnData;

    class Encoder
    {
    private:
        PointCloud *pPC;
        EncoderConfigParams *pParams;

        uint32_t nbits;
        uint32_t imSize;
        gpcc::Axis axis;
        int8_t algorithmChoice;
        uint32_t k = 2;

        encodeReturnData encodeIntervalSingleMode(Cabac *cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette);
        encodeReturnData encodeBlockMode(Cabac *cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette, uint32_t k);

    public:
        Encoder(PointCloud *ppc, EncoderConfigParams *params, gpcc::Axis axis, int k = 2);
        ~Encoder();
        void encodePointCloud();
        encodeReturnData encodePointCloud2(uint32_t level, Cabac *cabacIn, uint32_t iStart = 0, uint32_t iEnd = 0, gpcc::ImageSparse *fatherSilhouette = nullptr);
        encodeReturnData encodePointCloud3(uint32_t level, Cabac *cabacIn, uint32_t iStart = 0, uint32_t iEnd = 0);
        Bitstream *encodePointCloud4(uint32_t level, Cabac *cabacIn, uint32_t iStart = 0, uint32_t iEnd = 0);

        void writeHeader(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint64_t lengthBitstreamParam, Bitstream &bstream);
        void writeHeader2(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint64_t lengthBitstreamParam, uint64_t k, Bitstream &bstream);
        Bitstream *closeBitstream(Cabac *cabac);
        Bitstream *closeBitstream2(Cabac *cabac1, Cabac *cabac2);
    };

} // namespace gpcc

#endif
