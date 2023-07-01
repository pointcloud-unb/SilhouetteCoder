#include "ds_image_raster.h"
#include "ds_pixel.h"
#include "tp_bmp.h"

// ============ Image Raster =============

gpcc::ImageRaster::ImageRaster(int first_dimension, int second_dimension)
{
  this->dimension_[0] = first_dimension;
  this->dimension_[1] = second_dimension;

  std::vector<uint8_t> x(first_dimension, 0);
  std::vector<std::vector<uint8_t>> x_y(second_dimension, x);
  this->occupied_pixels_quantity_ = 0;
  this->location_ = x_y;
}

gpcc::ImageRaster::ImageRaster(const std::string &filename)
{
  // Read the BMP image
  BMP silhouette_raster(filename.c_str());
  this->dimension_[0] = silhouette_raster.bmp_info_header.width;
  this->dimension_[1] = silhouette_raster.bmp_info_header.height;
  this->occupied_pixels_quantity_ = 0;

  std::vector<uint8_t> x(this->dimension_[0], 0);
  std::vector<std::vector<uint8_t>> x_y(this->dimension_[1], x);
  this->location_ = x_y;

  int value = 1;
  int pixel_sz = silhouette_raster.bmp_info_header.bit_count / 8;
  int data_size = silhouette_raster.data.size() / pixel_sz;

  for (int i = 0; i < data_size; i++)
  {
    int line = (int)(i / this->dimension_[0]);
    int column = (int)(i % this->dimension_[0]);
    if (silhouette_raster.data[pixel_sz * i] > 127)
    {
      value = 0;
    }
    else
    {
      value = 1;
    }
    this->location_[line][column] = value;
  }
}

gpcc::ImageRaster::ImageRaster(gpcc::ImageSparse *image_sparse, int first_dimension, int second_dimension, int border_size)
{
  this->dimension_[0] = first_dimension;
  this->dimension_[1] = second_dimension;

  std::vector<uint8_t> x(first_dimension, 0);
  std::vector<std::vector<uint8_t>> x_y(second_dimension, x);
  this->location_ = x_y;
  this->occupied_pixels_quantity_ = 0;

  if (image_sparse == nullptr)
  {
    return;
  }

  for (auto pixel : image_sparse->getLocation())
  {
    this->location_[pixel[1]][pixel[0]] = 1;
    this->occupied_pixels_quantity_++;
  }
}

void gpcc::ImageRaster::addPixel(Pixel<int> p)
{
  this->location_[p[1]][p[0]] = 1;
  this->occupied_pixels_quantity_++;
}

void gpcc::ImageRaster::addPixel(pix_t x, pix_t y)
{
  this->location_[y][x] = 1;
  this->occupied_pixels_quantity_++;
}

void gpcc::ImageRaster::removePixel(Pixel<int> p)
{
  this->location_[p[1]][p[0]] = 0;
  this->occupied_pixels_quantity_--;
}

void gpcc::ImageRaster::removePixel(pix_t x, pix_t y)
{
  this->location_[y][x] = 0;
  this->occupied_pixels_quantity_--;
}

bool gpcc::ImageRaster::PixelPresent(Pixel<int> p)
{
  return (this->location_[p[1]][p[0]] == 1);
}

bool gpcc::ImageRaster::PixelPresent(pix_t x0, pix_t y0)
{
  return (this->location_[y0][x0] == 1);
}

std::tuple<int, int> gpcc::ImageRaster::Size()
{
  return std::make_tuple(this->dimension_[0],
                         this->dimension_[1]);
}

size_t gpcc::ImageRaster::NumberOccupiedPixels()
{
  return this->occupied_pixels_quantity_;
}

void gpcc::ImageRaster::updateSize(int limit_1, int limit_2)
{
  this->dimension_[0] = limit_1;
  this->dimension_[1] = limit_2;
}

std::string gpcc::ImageRaster::ShowImage()
{
  std::string out = "";

  if (this->location_.empty())
  {
    out = "Empty Point Cloud\n";
  }

  for (int line = 0; line < this->dimension_[1]; line++)
  {
    for (int col = 0; col < this->dimension_[0]; col++)
    {
      if (this->location_[line][col])
      {
        out += "(" + std::to_string(col) +
               "," + std::to_string(line) + ")\n";
      }
    }
  }

  return out;
}

// gpcc::ImageRaster::ImageRaster(ImageSparse image_sparse,
//                                int border_size)
// {
//   // **** TODO ****
//   this->dimension_[0] =  0;
//   this->dimension_[1] = 0;
//   this->border_size_ = border_size;

//   std::vector<uint8_t> x(this->dimension_[0], 0);
//   std::vector<std::vector<uint8_t>> x_y(this->dimension_[1], x);
//   this->location_ = x_y;

//   for(auto pixel:image_sparse.getLocation())
//   {
//     this->location_[pixel[0]][pixel[1]] = 1;
//   }
// }

// void gpcc::ImageRaster::addPixel(pix_t x, pix_t y)
// {
//   gpcc::Pixel<int> p(x, y);
//   this->location_.push_back(p);
//   return;
// }

void gpcc::ImageRaster::WriteBMP(const std::string &name)
{
  BMP silhouette_raster(this->dimension_[0], this->dimension_[1], false);

  uint32_t channels = silhouette_raster.bmp_info_header.bit_count / 8;
  // Go through all the pixels in the raster image
  for (size_t col = 0; col < this->dimension_[0]; col++)
  {
    for (size_t line = 0; line < this->dimension_[1]; line++)
    {
      // Check if it's empty or filled pixel
      if (this->location_[line][col] == 0)
      {

        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 0] = 255;
        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 1] = 255;
        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 2] = 255;
      }
      else
      {
        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 0] = 0;
        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 1] = 0;
        silhouette_raster.data[channels * (line * silhouette_raster.bmp_info_header.width + col) + 2] = 0;
      }
    }
  }
  silhouette_raster.write((name + ".bmp").c_str());

  return;
}

std::vector<std::vector<uint8_t>> gpcc::ImageRaster::getLocation()
{
  return this->location_;
}

bool gpcc::ImageRaster::CompareToRaster(gpcc::ImageRaster *image_2)
{

  for (size_t col = 0; col < this->dimension_[0]; col++)
  {
    for (size_t line = 0; line < this->dimension_[1]; line++)
    {
      if (this->location_[line][col] != image_2->getLocation()[line][col])
      {
        return false;
      }
    }
  }
  return true;
}