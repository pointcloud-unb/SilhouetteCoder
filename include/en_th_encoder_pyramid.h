#ifndef ENCODER_THREAD_PYRAMID_H
#define ENCODER_THREAD_PYRAMID_H

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
    } encodeThreadPyramidReturnData;

    class EncoderThreadPyramid
    {
    private:
        PointCloud *pPC;
        EncoderConfigParams *pParams;

        Cabac cabac;
        CodecParameterBitstream paramBitstream;

        std::vector<Bitstream *> acbstream;
        std::vector<uint32_t> leftBounds;

        uint32_t nbits;
        uint32_t nParallelism;
        uint32_t imSize;
        gpcc::Axis axis;
        bool useEncodePointCloud2;

        uint64_t nBitsAC;
        uint64_t nBitsHeader;
        uint64_t nBitsParamBitstream;

        gpcc::encodeThreadPyramidReturnData encodeIntervalSingleMode(Cabac *cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette, uint32_t npointcloud);
        gpcc::encodeThreadPyramidReturnData encodeIntervalSingleModeTopPyramid(Cabac* cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse* fatherSilhouette, uint32_t npointcloud);
        void writeHeader(uint16_t nBits, uint16_t axis, uint16_t algorithmChoice, uint16_t nParallelism, Bitstream &bstream);

    public:
        EncoderThreadPyramid(PointCloud *ppc, EncoderConfigParams *params, gpcc::Axis axis, uint32_t nParallelism);
        ~EncoderThreadPyramid();
        encodeThreadPyramidReturnData encodePointCloud(uint32_t npointcloud, uint32_t level, Cabac *cabacIn,
                                                       uint32_t iStart = 0, uint32_t iEnd = 0,
                                                       gpcc::ImageSparse *fatherSilhouette = nullptr);

        encodeThreadPyramidReturnData encodeTopPyramid(uint32_t level, Cabac *cabacIn,
                                                       uint32_t iStart = 0, uint32_t iEnd = 0,
                                                       gpcc::ImageSparse *fatherSilhouette = nullptr);
        void encodePointCloud2(uint32_t level, uint32_t iStart = 0, uint32_t iEnd = 0, gpcc::ImageSparse *fatherSilhouette = nullptr);
        uint32_t getResolution();
        gpcc::Axis getAxis();
        gpcc::PointCloud *getPointCloud();
        Bitstream *closeBitstream();
    };

} // namespace gpcc

#endif
