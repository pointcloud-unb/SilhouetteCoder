#ifndef DECODER_H
#define DECODER_H

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
        uint32_t decodingbitsSize;
    } decodeReturnData;

    class Decoder
    {
    private:
        uint16_t nBits;
        uint16_t nParallelism;

        gpcc::Axis axis;
        uint32_t imSize;
        uint64_t k;

        EncoderConfigParams *pParams;

        Cabac cabac;

        CodecParameterBitstream paramBitstream;

        Bitstream *acbstream;

        gpcc::PointCloud *pPC;

        void parseHeader(Bitstream &bstream);

        void deducePixels(gpcc::ImageRaster &image, gpcc::ImageSparse &fatherImage, gpcc::ImageRaster &leftImage);

        void xDecodePointCloud(uint32_t level, uint32_t iStart = 0, uint32_t iEnd = 0, gpcc::ImageSparse *fatherSilhouette = nullptr);
        void decodeIntervalSingleMode(uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette, Cabac *cabac);

    public:
        Decoder(Bitstream &pbstream, Axis axis, EncoderConfigParams *params);
        ~Decoder();
        uint32_t getNumberOfPointClouds();
        gpcc::PointCloud *decodePointCloud(uint32_t npointcloud);
        gpcc::PointCloud *decodePointCloud();
        gpcc::PointCloud *decodePointCloud2();
        gpcc::PointCloud *decodePointCloud3();
        gpcc::PointCloud *decodePointCloud4();
    };

} // namespace gpcc

#endif
