#include "th_thread_manager.h"

#include <iostream>

namespace gpcc
{
    ThreadManager::ThreadManager(EncoderConfigParams *params)
    {
        this->_params = params;
    }

    ThreadManager::~ThreadManager()
    {
    }

    Bitstream *ThreadManager::Encode(PointCloud *ppc, gpcc::Axis axis)
    {
        this->nthreads = 1 << this->_params->getNThreads();
        if (this->nthreads <= 0)
            this->nthreads = std::thread::hardware_concurrency();
        this->_last_block = this->nthreads - 1;
        this->_enc = new gpcc::EncoderThread(ppc, this->_params, axis, this->nthreads);
        this->nBits = this->_enc->getResolution();
        this->_next_block = 0;

        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list.push_back(std::thread(ThreadManager::encodeThread, this));
        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list[i].join();

        return this->_enc->closeBitstream();
    }

    PointCloud *ThreadManager::Decode(Bitstream &pbstream)
    {

        int32_t nthreads = this->_params->getNThreads();
        if (nthreads <= 0)
            nthreads = std::thread::hardware_concurrency();
        //std::cout << "NThread: " << nthreads << std::endl;
        this->_ppc = new PointCloud();
        this->_dec = new gpcc::DecoderThread(pbstream, this->_params);
        this->nthreads = this->_dec->getNumberOfPointClouds();
        this->nBits = this->_dec->getPCResolution();
        this->_last_block = this->_dec->getNumberOfPointClouds() - 1;
        this->_next_block = 0;

        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list.push_back(std::thread(ThreadManager::decodeThread, this));
        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list[i].join();

        return this->_ppc;
    }

    void ThreadManager::encodeThread(ThreadManager *threadObj)
    {
        while (true)
        {
            Cabac *cabacIn = new Cabac();
            uint64_t m = 16;
            uint64_t nc2D = 6;
            // uint64_t nc3D = 9;
            uint64_t nc3D = 9;
            cabacIn->initEncoder(m, nc2D, nc3D);
            threadObj->_block_mutex.lock();
            uint32_t npointcloud = (uint32_t)threadObj->_next_block;
            threadObj->_next_block++;
            threadObj->_block_mutex.unlock();
            if (npointcloud > threadObj->_last_block)
                break;
            uint32_t blockSize = (1 << threadObj->nBits) / threadObj->nthreads;
            uint32_t iStart = npointcloud * blockSize;
            uint32_t iEnd = (npointcloud + 1) * blockSize - 1;
            // std::cout << "istart: " << iStart
            //           << " iend: " << iEnd
            //           << " npointcloud: " << npointcloud
            //           << "\n";
            threadObj->_enc->encodePointCloud(npointcloud, log2(threadObj->nthreads), cabacIn, iStart, iEnd);
        }
    }

    void ThreadManager::decodeThread(ThreadManager *threadObj)
    {
        while (true)
        {
            threadObj->_block_mutex.lock();
            uint32_t npointcloud = (uint32_t)threadObj->_next_block;
            threadObj->_next_block++;
            threadObj->_block_mutex.unlock();
            if (npointcloud > threadObj->_last_block)
                break;
            uint32_t blockSize = (1 << threadObj->nBits) / threadObj->nthreads;
            uint32_t iStart = npointcloud * blockSize;
            uint32_t iEnd = (npointcloud + 1) * blockSize - 1;
            // std::cout << " threadObj->nBits: " << threadObj->nBits
            //           << " threadObj->nthreads:" << threadObj->nthreads
            //           << " istart: " << iStart
            //           << " iend: " << iEnd
            //           << " npointcloud: " << npointcloud
            //           << "\n";
            threadObj->_dec->decodePointCloud2(npointcloud, iStart, iEnd);
            threadObj->_ppc_mutex.lock();
            gpcc::PointCloud *ppcProduced = threadObj->_dec->getPointCloud(npointcloud);
            // std::cout << "Size After: " << ppcProduced->getVoxelList().size() << std::endl;
            threadObj->_ppc->UnifyPointCloud(*ppcProduced);
            threadObj->_ppc_mutex.unlock();

            delete ppcProduced;
        }
    }

    PointCloud *ThreadManager::getPointCloud()
    {
        return this->_ppc;
    }
}
