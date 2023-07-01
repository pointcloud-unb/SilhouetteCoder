#ifndef DECODER_THREAD_H
#define DECODER_THREAD_H

#include <string>

#include "ds_point_cloud.h"
#include "cg_config.h"
#include "en_codec_parameter_bitstream.h"
#include "en_cabac.h"
#include "ds_silhouette_decomposition.h"
#include "ac_bitstream.h"

namespace gpcc
{
    class DecoderThread
    {
    private:
        uint16_t nBits;
        uint16_t nParallelism;
        gpcc::Axis axis;
        uint32_t imSize;

        EncoderConfigParams *pParams;

        Cabac cabac;
        std::vector<CodecParameterBitstream> paramBitstream;

        std::vector<Bitstream *> acbstream;
        std::vector<uint32_t> leftBounds;

        std::vector<gpcc::PointCloud *> pPC;

        void parseHeader(Bitstream &bstream);

        void deducePixels(gpcc::ImageRaster &image, gpcc::ImageSparse &fatherImage, gpcc::ImageRaster &leftImage);

        void xDecodePointCloud(uint32_t npointcloud, Cabac *cabacIn, uint32_t level, uint32_t iStart = 0, uint32_t iEnd = 0, gpcc::ImageSparse *fatherSilhouette = nullptr);
        void decodeIntervalSingleMode(uint32_t npointcloud, Cabac *cabacIn, uint32_t iStart, uint32_t iEnd, gpcc::ImageSparse *fatherSilhouette);

    public:
        DecoderThread(Bitstream &pbstream, EncoderConfigParams *params);
        ~DecoderThread();
        uint32_t getNumberOfPointClouds();
        uint16_t getPCResolution();
        PointCloud *getPointCloud(uint32_t npointcloud);
        gpcc::PointCloud *decodePointCloud(uint32_t npointcloud);
        void decodePointCloud2(uint32_t npointcloud, uint32_t iStart, uint32_t iEnd);
    };

} // namespace gpcc

#endif
