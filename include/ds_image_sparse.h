#ifndef IMAGE_SPARSE_H
#define IMAGE_SPARSE_H

#include <cstddef>
#include <set>
#include <array>
#include <string>
#include <algorithm>
#include <iostream>
#include <tuple>

#include "ds_pixel.h"
#include "ds_iimage.h"
#include "ds_image_raster.h"


namespace gpcc
{
  class ImageRaster;
  class ImageSparse : public IImage
  {
  private:
    //! Pixel List
    /*
     * List of all occupied pixels from a image
     */
    std::set<Pixel<int>> pixel_list_;

    //! Image Sparse limit dimensions
    /*
     * Minimum dimension required to create
     * the 2d image. Computed to help the Image
     * Raster construct.
     */
    Pixel<int> dimension_;

    std::set<Pixel<int>>::iterator current_pixel_;

  public:
    //! Image Sparse constructor
    /*
     * Constructs an empty ImageSparse
     */
    ImageSparse();

    //! Image Sparse constructor
    /*
     * Constructs a sparse image given a pixel list
     */
    ImageSparse(std::set<Pixel<int>> p_list);

    //! Image Sparse Constructor based on Image Raster
    /*
     * Constructor based on Image Raster Object
     */
    ImageSparse(gpcc::ImageRaster *image_raster);
    
    //! A writer from image object to bmp imagem
    /*
     * Writes a bmp file given a name 
     */
    void WriteBMP(const std::string &);

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

    //! Add Pixel list to Image Sparse
    /*
     * Adds all pixels in the list to Image Sparse
     */
    void addPixels(std::set<Pixel<int>> pixel_list);

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

    //! ImageDimension
    /*
     * Return the Image (x,y) dimension
     */
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

    //! Get location_
    /*
     *  Returns the ImageSparse location_ pixel set
     */
    std::set<Pixel<int>> getLocation();

    //! Rewinds iterator (points to the beginning of the set)
    void Rewind();

    //! Returns current iterator pixel
    Pixel<int> CurrentPixel();

    //! Moves pixel set iterator to next
    /*
     *  Set current_pixel iterator to the next pixel
     *  and returns the next pixel.
     *  If its the end, return to the beginning
     */
    Pixel<int> NextPixel();

    //! Moves pixel set iterator to previous
    /*
     *  Set current_pixel iterator to the previous pixel
     *  and returns the previous pixel.
     *  If its the beginning, return to the end
     */
    Pixel<int> PreviousPixel();

    //! Prints current iterator's pixel
    std::string ShowPixel();

    void limitsCompute();
  };
};

#endif