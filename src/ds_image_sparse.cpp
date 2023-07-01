#include "ds_image_raster.h"
#include "ds_pixel.h"
#include "tp_bmp.h"

// ============ Image Sparse =============

gpcc::ImageSparse::ImageSparse()
{
  std::set<Pixel<int>> p_list;
  this->pixel_list_ = p_list;
  this->current_pixel_ = this->pixel_list_.begin();
  this->limitsCompute();
}

gpcc::ImageSparse::ImageSparse(std::set<Pixel<int>> p_list)
{
  this->pixel_list_ = p_list;
  this->current_pixel_ = this->pixel_list_.begin();
  this->limitsCompute();
}

gpcc::ImageSparse::ImageSparse(gpcc::ImageRaster *image_raster)
{
  std::set<Pixel<int>> p_list;
  this->pixel_list_ = p_list;
  // const int filled = 1;
  int first_dimension, second_dimension;

  std::tie(first_dimension, second_dimension) = image_raster->Size(); 
  this->dimension_[0] = first_dimension;
  this->dimension_[1] = second_dimension;

  for (int row = 0; row < first_dimension; row++)
  {
    for (int col = 0; col < second_dimension; col++)
    {
        if (image_raster->PixelPresent(col, row))
        {
            this->addPixel(col, row);
        }      
    }
  }

  this->limitsCompute();
}

bool ComparePixel(const gpcc::Pixel<int> &a, const gpcc::Pixel<int> &b)
{
  return a < b;
}

void gpcc::ImageSparse::addPixel(Pixel<int> p)
{
  if (pixel_list_.insert(p).second)
  {
    this->current_pixel_ = this->pixel_list_.insert(p).first;
  }
  return;
}

void gpcc::ImageSparse::addPixel(pix_t x, pix_t y)
{
  gpcc::Pixel<int> p(x, y);

  if (pixel_list_.insert(p).second)
  {
    this->current_pixel_ = this->pixel_list_.insert(p).first;
  }

  return;
}

void gpcc::ImageSparse::addPixels(std::set<Pixel<int>> pixel_list)
{
    for (std::set<Pixel<int>>::iterator it = pixel_list.begin(); it != pixel_list.end(); ++it)
    {
        this->addPixel(*it);
    }
        
}

void gpcc::ImageSparse::removePixel(Pixel<int> p)
{
  if(*(this->current_pixel_) == p){
    this->PreviousPixel();
  }
  this->pixel_list_.erase(p);
}

void gpcc::ImageSparse::removePixel(pix_t x0, pix_t y0)
{
  gpcc::Pixel<int> p(x0, y0);
  removePixel(p);
}

bool gpcc::ImageSparse::PixelPresent(Pixel<int> p)
{
  auto pixel_lower_bound = std::lower_bound(
      this->pixel_list_.begin(),
      this->pixel_list_.end(),
      p,
      ComparePixel);

  return *pixel_lower_bound == p;
}

bool gpcc::ImageSparse::PixelPresent(pix_t x0, pix_t y0)
{
  gpcc::Pixel<int> p(x0, y0);
  return PixelPresent(p);
}

std::tuple<int, int> gpcc::ImageSparse::Size()
{
  return std::make_tuple(this->dimension_[0],
                         this->dimension_[1]);
}

size_t gpcc::ImageSparse::NumberOccupiedPixels()
{
  return this->pixel_list_.size();
}

void gpcc::ImageSparse::updateSize(int limit_1, int limit_2)
{
  this->dimension_[0] = limit_1;
  this->dimension_[1] = limit_2;
  return;
}

std::set<gpcc::Pixel<int>> gpcc::ImageSparse::getLocation()
{
  return this->pixel_list_;
}

void gpcc::ImageSparse::WriteBMP(const std::string &name)
{
  BMP silhouette_sparse(this->dimension_[0], this->dimension_[1], false);
  int32_t channels = silhouette_sparse.bmp_info_header.bit_count / 8;
  silhouette_sparse.fill_white(); // Fill the .bmp with white before filling the black points of the silhouette
  // Go through the filled pixels to replace in the bmp data
  // int aux = 0;
  for (auto pixel : this->pixel_list_)
  {
    silhouette_sparse.data[channels * (pixel[1] * silhouette_sparse.bmp_info_header.width + pixel[0]) + 0] = 0;
    silhouette_sparse.data[channels * (pixel[1] * silhouette_sparse.bmp_info_header.width + pixel[0]) + 1] = 0;
    silhouette_sparse.data[channels * (pixel[1] * silhouette_sparse.bmp_info_header.width + pixel[0]) + 2] = 0;
  };
  silhouette_sparse.write((name + ".bmp").c_str());
}

std::string gpcc::ImageSparse::ShowImage()
{
  std::string out = "";

  if (this->pixel_list_.empty())
  {
    out = "Empty Image\n";
  }

  for (auto pixel : this->pixel_list_)
  {
    out += "(" + std::to_string(pixel[0]) +
           "," + std::to_string(pixel[1]) + ")\n";
  }

  return out;
}

void gpcc::ImageSparse::limitsCompute()
{
  return;
}

void gpcc::ImageSparse::Rewind()
{
    this->current_pixel_ = this->pixel_list_.begin();
}

gpcc::Pixel<int> gpcc::ImageSparse::CurrentPixel()
{
  return *(this->current_pixel_);
}

gpcc::Pixel<int> gpcc::ImageSparse::NextPixel()
{
  if (this->pixel_list_.size() == 0)
  {
    std::cout << "Invalid gpcc::ImageSparse::NextPixel operation" << std::endl;
    return *(this->current_pixel_);
  }

  this->current_pixel_++;

  if (this->current_pixel_ == this->pixel_list_.end())
  {
    this->current_pixel_ = this->pixel_list_.begin();
  }

  return *(this->current_pixel_);
}

gpcc::Pixel<int> gpcc::ImageSparse::PreviousPixel()
{
  if (this->pixel_list_.size() == 0)
  {
    std::cout << "Invalid gpcc::ImageSparse::NextPixel operation" << std::endl;
    return *(this->current_pixel_);
  }

  if (this->current_pixel_ == this->pixel_list_.begin())
  {
    this->current_pixel_ = (this->pixel_list_.end());
    this->current_pixel_--;
  }

  else
  {
    this->current_pixel_--;
  }

  return *(this->current_pixel_);
}