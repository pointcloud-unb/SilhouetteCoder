#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <vector>
#include <thread>
#include <mutex>

#include "ds_point_cloud.h"
#include "cg_config.h"
#include "ac_bitstream.h"
#include "en_th_encoder.h"
#include "en_th_decoder.h"

namespace gpcc
{
    class ThreadManager
    {
    private:
        uint32_t nBits;
        int32_t nthreads;
        std::vector<std::thread> _thread_list;
        std::mutex _ppc_mutex;
        EncoderConfigParams *_params;
        EncoderThread *_enc;
        DecoderThread *_dec;
        PointCloud *_ppc;
        uint32_t _last_block;
        uint32_t _next_block;
        std::mutex _block_mutex;
        static void decodeThread(ThreadManager *threadObj);
        static void encodeThread(ThreadManager *threadObj);

    public:
        ThreadManager(EncoderConfigParams *params);
        Bitstream *Encode(PointCloud *ppc, gpcc::Axis axis);
        PointCloud *Decode(Bitstream &pbstream);
        ~ThreadManager();

        PointCloud *getPointCloud();
    };
}

#endif