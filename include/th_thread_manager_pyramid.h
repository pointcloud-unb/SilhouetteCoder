#ifndef THREAD_PYRAMID_MANAGER_H
#define THREAD_PYRAMID_MANAGER_H

#include <vector>
#include <thread>
#include <mutex>

#include "ds_point_cloud.h"
#include "cg_config.h"
#include "ac_bitstream.h"
#include "en_th_encoder_pyramid.h"
#include "en_th_decoder_pyramid.h"

namespace gpcc
{
    class ThreadManagerPyramid
    {
    private:
        uint32_t nBits;
        int32_t nthreads;
        std::vector<std::thread> _thread_list;
        std::mutex _ppc_mutex;
        EncoderConfigParams *_params;
        EncoderThreadPyramid *_enc;
        DecoderThreadPyramid *_dec;
        PointCloud *_ppc;
        uint32_t _last_block;
        uint32_t _next_block;
        std::mutex _block_mutex;
        static void decodeThreadPyramid(ThreadManagerPyramid *threadObj);
        static void encodeThreadPyramid(ThreadManagerPyramid *threadObj);

    public:
        ThreadManagerPyramid(EncoderConfigParams *params);
        void generateFatherSilhouettes();
        Bitstream *Encode(PointCloud *ppc, gpcc::Axis axis);
        PointCloud *Decode(Bitstream &pbstream);
        ~ThreadManagerPyramid();

        PointCloud *getPointCloud();
    };
}

#endif