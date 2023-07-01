#include "th_thread_manager_pyramid.h"

#include <iostream>

namespace gpcc
{
    ThreadManagerPyramid::ThreadManagerPyramid(EncoderConfigParams *params)
    {
        this->_params = params;
    }

    ThreadManagerPyramid::~ThreadManagerPyramid()
    {
    }

    Bitstream *ThreadManagerPyramid::Encode(PointCloud *ppc, gpcc::Axis axis)
    {
        this->nthreads = (1 << this->_params->getNThreads());
        if (this->nthreads <= 0)
            this->nthreads = std::thread::hardware_concurrency();
        this->_last_block = this->nthreads;
        this->_enc = new gpcc::EncoderThreadPyramid(ppc, this->_params, axis, this->nthreads);
        this->nBits = this->_enc->getResolution();
        this->_next_block = 0;

        // this->_thread_list.push_back(std::thread(ThreadManagerPyramid::encodeThreadPyramid, this));
        // this->_thread_list[0].join();

        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list.push_back(std::thread(ThreadManagerPyramid::encodeThreadPyramid, this));
        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list[i].join();

        return this->_enc->closeBitstream();
    }

    PointCloud *ThreadManagerPyramid::Decode(Bitstream &pbstream)
    {

        int32_t nthreads = (1 << this->_params->getNThreads());
        if (nthreads <= 0)
            nthreads = std::thread::hardware_concurrency();
        // std::cout << "NThread: " << nthreads << std::endl;
        this->_ppc = new PointCloud();
        this->_dec = new gpcc::DecoderThreadPyramid(pbstream, this->_params);
        this->nthreads = this->_dec->getNumberOfPointClouds();
        this->nBits = this->_dec->getPCResolution();
        this->_last_block = this->_dec->getNumberOfPointClouds() - 1;
        this->_next_block = 0;

        this->_dec->decodePointCloudRoot2();

        // this->_thread_list.push_back(std::thread(ThreadManagerPyramid::decodeThreadPyramid, this));
        // this->_thread_list[0].join();

        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list.push_back(std::thread(ThreadManagerPyramid::decodeThreadPyramid, this));
        for (uint8_t i = 0; i < this->nthreads; i++)
            this->_thread_list[i].join();

        return this->_ppc;
    }

    void ThreadManagerPyramid::encodeThreadPyramid(ThreadManagerPyramid *threadObj)
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
            uint32_t iStart = (npointcloud - 1) * blockSize;
            uint32_t iEnd = (npointcloud)*blockSize - 1;
            if (npointcloud == 0)
            {
                // std::cout << "root\n";
                threadObj->_enc->encodeTopPyramid(0, cabacIn);
            }
            else
            {
                /*
                std::cout << "istart: " << iStart
                          << " iend: " << iEnd
                          << " npointcloud: " << npointcloud
                          << "\n";
                */
                // ImageSparse *ir = threadObj->_enc->getPointCloud()->SilhouetteFromPointCloud(iStart, iEnd, threadObj->_enc->getAxis());
                threadObj->_enc->encodePointCloud(npointcloud, log2(threadObj->nthreads), cabacIn, iStart, iEnd);
            }
        }
    }

    void ThreadManagerPyramid::decodeThreadPyramid(ThreadManagerPyramid *threadObj)
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
            uint32_t iStart = (npointcloud)*blockSize;
            uint32_t iEnd = (npointcloud + 1) * blockSize - 1;
            /*
            std::cout << " threadObj->nBits: " << threadObj->nBits
                      << " threadObj->nthreads:" << threadObj->nthreads
                      << " istart: " << iStart
                      << " iend: " << iEnd
                      << " npointcloud: " << npointcloud
                      << "\n";
                      */
            threadObj->_dec->decodePointCloud2(npointcloud + 1, iStart, iEnd);
            threadObj->_ppc_mutex.lock();
            gpcc::PointCloud *ppcProduced = threadObj->_dec->getPointCloud(npointcloud + 1);
            // std::cout << "Size After: " << ppcProduced->getVoxelList().size() << std::endl;
            threadObj->_ppc->UnifyPointCloud(*ppcProduced);
            threadObj->_ppc_mutex.unlock();

            delete ppcProduced;
        }
    }

    PointCloud *ThreadManagerPyramid::getPointCloud()
    {
        return this->_ppc;
    }

    void ThreadManagerPyramid::generateFatherSilhouettes()
    {
        for (int i = 0; i < this->nthreads; i++)
        {

            gpcc::ImageSparse *fatherSilhouetteSparse = this->_dec->fatherSilhouettes[i];
            uint32_t size = 1 << this->nBits;
            gpcc::ImageRaster *fatherRaster = new gpcc::ImageRaster(fatherSilhouetteSparse, size, size, 0);
            fatherRaster->WriteBMP(std::to_string(i + 1) + "_dec");

            gpcc::ImageRaster *encSilhouette = new ImageRaster(std::to_string(i + 1) + "_enc.bmp");

            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    if (fatherRaster->getPixel(y, x) != encSilhouette->getPixel(y, x))
                    {
                        std::cout << "i: " << i << " y: " << y << " x: " << x << "  WRONG!\n";
                        std::cout << "fatherRaster: " << (int)fatherRaster->getPixel(y, x) << " encSilhouette: " << (int)encSilhouette->getPixel(y, x) << "\n";
                        return;
                    }
                }
            }
        }
    };

}
