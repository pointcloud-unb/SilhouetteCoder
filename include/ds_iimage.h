#ifndef IMAGE_H
#define IMAGE_H

#include <cstddef>
#include <vector>
#include <array>
#include <string>
#include <iostream>


#include "ds_pixel.h"

namespace gpcc
{
   //! pix_t
   /*
   * pix_t type
   * 
   * Pixel axis unit step type
   * 
   */
   typedef int pix_t;

   class IImage // Image abstract class
   {
   public:
      // virtual friend std::ostream& operator<<(std::ostream& os, const IImage& box) = 0;
      // virtual friend std::istream& operator>>(std::istream& is, IImage& box) = 0;

      virtual ~IImage() {};

      //! Add Pixel
      /*
       * Adds a Pixel with a Pixel object
       */
      virtual void addPixel(Pixel<int> p) = 0;

      //! Add Pixel
      /*
       * Adds a Pixel with coordinate initialization
       */
      virtual void addPixel(pix_t x, pix_t y) = 0;

      //! Remove pixel
      /*
       * Removes a pixel given a Pixel object
      */
      virtual void removePixel(Pixel<int> p) = 0;

      //! Remove pixel
      /*
       * Removes a pixel with coordinate initialization
      */
      virtual void removePixel(pix_t x, pix_t y) = 0;

      //! pixel Present verification function
      /*
         * Verify existence of a given pixel on image
         * by pixel object
         */
      virtual bool PixelPresent(Pixel<int> p) = 0;

      //! pixel Present verification function
      /*
       * Verify existence of a given pixel on image
       * by pixel object
      */
      virtual bool PixelPresent(pix_t x0, pix_t y0) = 0;

      //! ImageDimension
      /*
       * Return the Image (x,y) dimension
      */
      virtual std::tuple<int,int> Size() = 0;

      //! Number Occupied Pixels
      /*
       * Gives the quantity of pixels occupied in
       * the image
       */
      virtual size_t NumberOccupiedPixels() = 0;

      //! Show Image
      /*
     * Return the string form image to be printed
     */
      virtual void WriteBMP(const std::string&) = 0;
   };
} // namespace
#endif // IMAGE_H