#ifndef IMAGE_RASTER_H
#define IMAGE_RASTER_H

#include <cstddef>
#include <vector>
#include <array>
#include <tuple>
#include <string>
#include <iostream>

#include "ds_pixel.h"
#include "ds_iimage.h"
#include "ds_image_sparse.h"

namespace gpcc
{
  class ImageSparse; 
  class ImageRaster : public IImage
  {
  private:
    //! Location
    /*
     * A 2D matrix consisting of occupancy of pixels on image
     */
    std::vector<std::vector<uint8_t>> location_;

    //! Image's dimension
    Pixel<int> dimension_;

    //! Number of pixels occupied on the image
    int occupied_pixels_quantity_;

    //! Number of pixels occupied on the image
    int border_size_;

  public:
    //! Image Raster Constructor based on dimensions
    /*
     * Constructor with a initialization
     */
    ImageRaster(int first_dimension, int second_dimension);

    //! Image Raster based on pixel list
    /*
     * Constructor with a initialization
     * of image
     */
    ImageRaster(std::vector<Pixel<int>> pixel_list);

    ImageRaster(ImageSparse* image_sparse, int first_dimension, int second_dimension, int border_size);

    //! Image Raster Constructor based on .bmp file
    /*
    * Constructor to read a .bmp and transform into ImageRaster
    */
    ImageRaster(const std::string &filename);

    //! Image Raster getter
    /*
    * Returns the pixel value at location (x,y)
    */
    uint8_t getPixel(int y, int x) { return this->location_[y][x]; }

    //! Add Pixel
    /*
     * Adds a Pixel with a Pixel object
    */
    void addPixel(Pixel<int> p);

    //! Add Pixel to Image Sparse
    /*
     * Adds pixel at coordinates (x,y)
     */
    void addPixel(pix_t x, pix_t y);

    //! Remove pixel
    /*
     * Removes a pixel given a Pixel object
     */
    void removePixel(Pixel<int> p);

    //! Remove pixel
    /*
     * Removes a pixel with coordinate initialization
     */
    void removePixel(pix_t x, pix_t y);

    //! pixel Present verification function
    /*
     * Verify existence of a given pixel on image
     * by pixel object
     */
    bool PixelPresent(Pixel<int> p);

    //! pixel Present verification function
    /*
     * Verify existence of a given pixel on image
     * by pixel object
     */
    bool PixelPresent(pix_t x0, pix_t y0);

    std::tuple<int, int> Size();

    //! Number Occupied Pixels
    /*
     * Gives the quantity of pixels occupied in
     * the image
     */
    size_t NumberOccupiedPixels();

    //! Update Size
    /*
     * Updates 1st and 2st Size of Image Sparse
     */
    void updateSize(int limit_1, int limit_2);

    //! Show Image
    /*
     * Return the string form image to be printed
     */
    std::string ShowImage();

    //! Pixels accessor
    /*
     * Returns the image's pixels set 
     */
    std::vector<std::vector<uint8_t>> getLocation();

    //! A writer from image object to bmp imagem
    /*
     * Writes a bmp file given a name 
     */
    void WriteBMP(const std::string &filename);

    //! 2 Images comparator
    /*
     * If two images are equal, returns true; 
     * otherwise false
     */
    bool CompareToRaster(gpcc::ImageRaster *image_2);
  };
};

#endif