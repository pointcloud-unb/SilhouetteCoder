#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm> // std::sort

#include "ds_point_cloud.h"
#include "ds_pixel.h"
#include "tp_ffmpeg.h"
#include "ds_voxel.h"
// #define PCC_VERBOSE

gpcc::PointCloud::PointCloud()
{
  this->point_cloud_voxel_count_ = 0;
}

gpcc::PointCloud::PointCloud(std::vector<Voxel<int>> voxel_list)
{
  this->voxel_list_ = voxel_list;
  this->point_cloud_voxel_count_ = voxel_list_.size();
}

std::string gpcc::PointCloud::ShowPointCloud()
{
  std::string out = "";

  for (auto vox : this->voxel_list_)
  {
    out += "(" + std::to_string(vox[0]) +
           "," + std::to_string(vox[1]) +
           "," + std::to_string(vox[2]) + ")\n";
  }

  out += std::to_string(this->point_cloud_voxel_count_) + "\n";

  return out;
}

gpcc::ImageSparse *gpcc::PointCloud::SilhouetteFromPointCloud(vox_t slice_start,
                                                              vox_t slice_stop,
                                                              Axis axis)
{
  // Image silhouette dimension's size
  // First axis from image
  int first_dimension_size;

  // Second axis from image
  int second_dimension_size;

  // First axis dimension
  gpcc::Axis first_axis_dimension;

  // Second axis dimension
  gpcc::Axis second_axis_dimension;

  // yz plan
  if (axis == Axis::X)
  {
    first_dimension_size = this->dimension_[Axis::Y];
    second_dimension_size = this->dimension_[Axis::Z];
    first_axis_dimension = Axis::Y;
    second_axis_dimension = Axis::Z;
  }

  // zx plan
  else if (axis == Axis::Y)
  {
    first_dimension_size = this->dimension_[Axis::Z];
    second_dimension_size = this->dimension_[Axis::X];
    first_axis_dimension = Axis::Z;
    second_axis_dimension = Axis::X;
  }

  // xy plan
  else
  {
    first_dimension_size = this->dimension_[Axis::X];
    second_dimension_size = this->dimension_[Axis::Y];
    first_axis_dimension = Axis::X;
    second_axis_dimension = Axis::Y;
  }

  gpcc::Pixel<int> pixel;
  gpcc::ImageSparse *silhouette = new gpcc::ImageSparse();

  // Search every voxel inside this interval
  for (auto vox : this->voxel_list_)
  {
    if (slice_start <= vox[axis] &&
        vox[axis] <= slice_stop)
    {
      pixel[0] = vox[first_axis_dimension];
      pixel[1] = vox[second_axis_dimension];
      silhouette->addPixel(pixel);
    }
  }

  silhouette->updateSize(this->dimension_[first_axis_dimension],
                         this->dimension_[second_axis_dimension]);

  return silhouette;
}

std::tuple<int, int, int> gpcc::WrapVoxel(gpcc::Axis axis, vox_t coordinate,
                                          vox_t t0, vox_t t1)
{
  int dim1, dim2, dim3;
  // XYZ order, receives YZ image
  if (axis == gpcc::X)
  {
    dim1 = coordinate; // X
    dim2 = t0;         // Y
    dim3 = t1;         // Z
  }

  // YZX order, receives ZX image
  else if (axis == gpcc::Y)
  {

    dim1 = t1;         // X
    dim2 = coordinate; // Y
    dim3 = t0;         // Z
  }

  // ZXY order, receives XY image
  else if (axis == gpcc::Z)
  {
    dim1 = t0;         // X
    dim2 = t1;         // Y
    dim3 = coordinate; // Z
  }

  return std::make_tuple(dim1, dim2, dim3);
}

void gpcc::PointCloud::AddSilhouette(gpcc::Axis axis,
                                     vox_t coordinate,
                                     ImageSparse *image)
{
  int dim1, dim2, dim3;
  for (auto pixel : image->getLocation())
  {
    std::tie(dim1, dim2, dim3) = WrapVoxel(axis, coordinate, pixel[0], pixel[1]);
    this->addVoxel(dim1, dim2, dim3);
  }
}

void gpcc::PointCloud::RecoverVoxelsFromSilhouette(gpcc::Axis axis,
                                                   vox_t coordinate,
                                                   ImageRaster *image)
{
  int dim1, dim2, dim3;
  int heigth, width;

  std::tie(heigth, width) = image->Size();

  for (int i = 0; i < heigth; i++)
  {
    for (int j = 0; j < width; j++)
    {
      // XYZ order, receives YZ image
      if (axis == gpcc::X)
      {
        dim1 = coordinate; // X
        dim2 = i;          // Y
        dim3 = j;          // Z
      }

      // YZX order, receives ZX image
      else if (axis == gpcc::Y)
      {

        dim1 = j;          // X
        dim2 = coordinate; // Y
        dim3 = i;          // Z
      }

      // ZXY order, receives XY image
      else if (axis == gpcc::Z)
      {
        dim1 = i;          // X
        dim2 = j;          // Y
        dim3 = coordinate; // Z
      }

      if (image->getLocation()[i][j] == 1)
      {
        this->addVoxel(dim1, dim2, dim3);
      }
    }
  }
}

bool CompareVoxel(const gpcc::Voxel<int> &a, const gpcc::Voxel<int> &b)
{
  return a < b;
}

void gpcc::PointCloud::addVoxel(Voxel<int> p)
{
  this->voxel_list_.push_back(p);
  this->point_cloud_voxel_count_++;
}

void gpcc::PointCloud::addVoxel(vox_t x0, vox_t y0, vox_t z0)
{
  gpcc::Voxel<int> p(x0, y0, z0);
  this->voxel_list_.push_back(p);
  this->point_cloud_voxel_count_++;
}

bool gpcc::PointCloud::VoxelPresent(Voxel<int> p)
{
  auto voxel_lower_bound = std::lower_bound(
      this->voxel_list_.begin(),
      this->voxel_list_.end(),
      p,
      CompareVoxel);

  return *voxel_lower_bound == p;
}

bool gpcc::PointCloud::VoxelPresent(vox_t x0, vox_t y0, vox_t z0)
{
  gpcc::Voxel<int> p(x0, y0, z0);
  return VoxelPresent(p);
}

void gpcc::PointCloud::removeVoxel(Voxel<int> p)
{
  auto voxel_lower_bound = std::lower_bound(
      this->voxel_list_.begin(),
      this->voxel_list_.end(),
      p,
      CompareVoxel);

  if (*voxel_lower_bound == p)
  {
    this->voxel_list_.erase(voxel_lower_bound);
    this->point_cloud_voxel_count_--;
  }

  else
  {
    throw std::invalid_argument("gpcc::PointCloud::removeVoxel: Voxel not found");
    exit(1);
  }
}

void gpcc::PointCloud::removeVoxel(vox_t x0, vox_t y0, vox_t z0)
{
  gpcc::Voxel<int> v(x0, y0, z0);
  removeVoxel(v);
  this->point_cloud_voxel_count_--;
}

std::tuple<int, int, int> gpcc::PointCloud::Size()
{
  return std::make_tuple(this->dimension_[0],
                         this->dimension_[1],
                         this->dimension_[2]);
}

void gpcc::PointCloud::updateSize(int limit_1, int limit_2, int limit_3)
{
  this->dimension_[0] = limit_1;
  this->dimension_[1] = limit_2;
  this->dimension_[2] = limit_3;
}

size_t gpcc::PointCloud::NumberOccupiedVoxels()
{
  return this->voxel_list_.size();
}

void gpcc::PointCloud::UnifyPointCloud(PointCloud point_cloud)
{
  this->voxel_list_.insert(this->voxel_list_.end(), point_cloud.voxel_list_.begin(), point_cloud.voxel_list_.end());
}

bool gpcc::PointCloud::Load(const std::string &filename)
{

  const uint32_t PCC_UNDEFINED_INDEX = -1;

  struct PropertyNameMap
  {
    // The names of the position attributes, typically {"x", "y", "z"}
    std::array<const char *, 3> position;
  };

  PropertyNameMap attributeNames;
  attributeNames.position = {"x", "y", "z"};

  std::ifstream ifs(filename, std::ifstream::in | std::ifstream::binary); // open the file using ifstream

  if (!ifs.is_open())
  {
    return false;
  } // Check if the file is open. If it was unable to open the file, return false

  enum AttributeType
  {
    ATTRIBUTE_TYPE_FLOAT64 = 0,
    ATTRIBUTE_TYPE_FLOAT32 = 1,
    ATTRIBUTE_TYPE_UINT64 = 2,
    ATTRIBUTE_TYPE_UINT32 = 3,
    ATTRIBUTE_TYPE_UINT16 = 4,
    ATTRIBUTE_TYPE_UINT8 = 5,
    ATTRIBUTE_TYPE_INT64 = 6,
    ATTRIBUTE_TYPE_INT32 = 7,
    ATTRIBUTE_TYPE_INT16 = 8,
    ATTRIBUTE_TYPE_INT8 = 9,
  };

  struct AttributeInfo
  {
    std::string name;
    AttributeType type;
    size_t byteCount;
  };

  std::vector<AttributeInfo> attributesInfo; // Creates a vector with the AttributeInfo
  attributesInfo.reserve(16);                // Reserves enough for 16 attributes infos
  // Creates a buffer and determines the separator to get the different tokens
  const size_t MAX_BUFFER_SIZE = 4096;
  char tmp[MAX_BUFFER_SIZE];
  const char *sep = " \t\r";
  std::vector<std::string> tokens;

  /////////////////////////////////////////////////////////////////////////////////
  ///////// Reading the header ////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  // Check the first line of the ply file, returns an error if it is not "ply"
  ifs.getline(tmp, MAX_BUFFER_SIZE);
  getTokens(tmp, sep, tokens);
  if (tokens.empty() || tokens[0] != "ply")
  {
    std::cout << "Error: corrupted file!" << std::endl;
    return false;
  }
  // Check the rest of the header
  bool isAscii = false;
  double version = 1.0;
  size_t pointCount = 0;
  bool isVertexProperty = true;

  while (1)
  {
    // Check if there is an error state flag for the stream
    if (ifs.eof())
    {
      std::cout << "Error: corrupted header!" << std::endl;
      return false;
    }

    ifs.getline(tmp, MAX_BUFFER_SIZE);
    getTokens(tmp, sep, tokens);
    // if there is an empty line or a comments line, continue and jump to next line
    if (tokens.empty() || tokens[0] == "comment")
    {
      continue;
    }
    // Starts the parsing from the second line of the usual .ply file
    if (tokens[0] == "format")
    {
      if (tokens.size() != 3)
      {
        std::cout << "Error: corrupted format info!" << std::endl;
        return false;
      }
      // Get the format and the version of the ascii
      isAscii = tokens[1] == "ascii";
      version = atof(tokens[2].c_str());
    }
    // 3rd line gives the number of elements of the .ply file
    else if (tokens[0] == "element")
    {
      if (tokens.size() != 3)
      {
        std::cout << "Error: corrupted element info!" << std::endl;
        return false;
      }
      if (tokens[1] == "vertex")
      {
        pointCount = atoi(tokens[2].c_str());
      }
      else
      {
        isVertexProperty = false;
      }
    }
    // the remaining lines correspond to the properties of each point in the 3D cloud
    else if (tokens[0] == "property" && isVertexProperty)
    {
      // each property has a type and a name
      if (tokens.size() != 3)
      {
        std::cout << "Error: corrupted property info!" << std::endl;
        return false;
      }
      // get the type and the name of the attribute
      const std::string &propertyType = tokens[1];
      const std::string &propertyName = tokens[2];
      // get the current size of the vector
      const size_t attributeIndex = attributesInfo.size();
      attributesInfo.resize(attributeIndex + 1);
      AttributeInfo &attributeInfo = attributesInfo[attributeIndex];
      // put the name of the attribute in the correspond field for the correct index
      attributeInfo.name = propertyName;
      // get the correct type and the number of bytes
      if (propertyType == "float64")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT64;
        attributeInfo.byteCount = 8;
      }
      else if (propertyType == "float" || propertyType == "float32")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT32;
        attributeInfo.byteCount = 4;
      }
      else if (propertyType == "uint64")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT64;
        attributeInfo.byteCount = 8;
      }
      else if (propertyType == "uint32")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT32;
        attributeInfo.byteCount = 4;
      }
      else if (propertyType == "uint16")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT16;
        attributeInfo.byteCount = 2;
      }
      else if (propertyType == "uchar" || propertyType == "uint8")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT8;
        attributeInfo.byteCount = 1;
      }
      else if (propertyType == "int64")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_INT64;
        attributeInfo.byteCount = 8;
      }
      else if (propertyType == "int32")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_INT32;
        attributeInfo.byteCount = 4;
      }
      else if (propertyType == "int16")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_INT16;
        attributeInfo.byteCount = 2;
      }
      else if (propertyType == "char" || propertyType == "int8")
      {
        attributeInfo.type = ATTRIBUTE_TYPE_INT8;
        attributeInfo.byteCount = 1;
      }
    }
    // the last line of the header is the end_header.
    else if (tokens[0] == "end_header")
    {
      break;
    }
  }

#if PCC_VERBOSE
  std::cout << "Read the header" << std::endl;
#endif

  /////////////////////////////////////////////////////////////////////////////////
  ///////// Reading the points ////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  if (version != 1.0)
  {
    std::cout << "Error: non-supported version!" << std::endl;
    return false;
  }

  // positions
  size_t indexX = PCC_UNDEFINED_INDEX;
  size_t indexY = PCC_UNDEFINED_INDEX;
  size_t indexZ = PCC_UNDEFINED_INDEX;
  // attributes
  size_t indexR = PCC_UNDEFINED_INDEX;
  size_t indexG = PCC_UNDEFINED_INDEX;
  size_t indexB = PCC_UNDEFINED_INDEX;
  size_t indexReflectance = PCC_UNDEFINED_INDEX;
  size_t indexFrame = PCC_UNDEFINED_INDEX;
  size_t indexNX = PCC_UNDEFINED_INDEX;
  size_t indexNY = PCC_UNDEFINED_INDEX;
  size_t indexNZ = PCC_UNDEFINED_INDEX;
  // Get the number of attributes
  const size_t attributeCount = attributesInfo.size();
  // Get all the attributes indexes in the line
  for (size_t a = 0; a < attributeCount; ++a)
  {
    const auto &attributeInfo = attributesInfo[a];
    if (
        attributeInfo.name == attributeNames.position[0] && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexX = a;
    }
    else if (
        attributeInfo.name == attributeNames.position[1] && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexY = a;
    }
    else if (
        attributeInfo.name == attributeNames.position[2] && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexZ = a;
    }
    else if (attributeInfo.name == "red" && attributeInfo.byteCount == 1)
    {
      indexR = a;
    }
    else if (attributeInfo.name == "green" && attributeInfo.byteCount == 1)
    {
      indexG = a;
    }
    else if (attributeInfo.name == "blue" && attributeInfo.byteCount == 1)
    {
      indexB = a;
    }
    else if (
        (attributeInfo.name == "reflectance" || attributeInfo.name == "refc") && attributeInfo.byteCount <= 2)
    {
      indexReflectance = a;
    }
    else if (
        attributeInfo.name == "frameindex" && attributeInfo.byteCount <= 2)
    {
      indexFrame = a;
    }
    else if (
        attributeInfo.name == "nx" && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexNX = a;
    }
    else if (
        attributeInfo.name == "ny" && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexNY = a;
    }
    else if (
        attributeInfo.name == "nz" && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4))
    {
      indexNZ = a;
    }
  }

  // Check to see if any coordinates are missing
  if (
      indexX == PCC_UNDEFINED_INDEX || indexY == PCC_UNDEFINED_INDEX || indexZ == PCC_UNDEFINED_INDEX)
  {
    std::cout << "Error: missing coordinates!" << std::endl;
    return false;
  }

#if PCC_VERBOSE
  std::cout << "Checked for missing coordinates" << std::endl;
#endif

  // Check if the Point Cloud has colors/reflectance/frame_idx
  bool withColors = indexR != PCC_UNDEFINED_INDEX && indexG != PCC_UNDEFINED_INDEX && indexB != PCC_UNDEFINED_INDEX;
  bool withReflectances = indexReflectance != PCC_UNDEFINED_INDEX;
  bool withFrameIndex = indexFrame != PCC_UNDEFINED_INDEX;

  int larger_pixel = 0;

  if (isAscii)
  {
    size_t pointCounter = 0;
    // While the file is still open and the line is smaller than the total amount of points
    while (!ifs.eof() && pointCounter < pointCount)
    {
      ifs.getline(tmp, MAX_BUFFER_SIZE);
      getTokens(tmp, sep, tokens);
      // if the line is empty, continue
      if (tokens.empty())
      {
        continue;
      }
      // if the line does not contain the correct number of attributes
      if (tokens.size() < attributeCount)
      {
        return false;
      }

      vox_t X = atof(tokens[indexX].c_str());
      vox_t Y = atof(tokens[indexY].c_str());
      vox_t Z = atof(tokens[indexZ].c_str());

      this->addVoxel(X, Y, Z);

      // Update the largest pixel
      if (larger_pixel < std::max({X, Y, Z}))
      {
        larger_pixel = std::max({X, Y, Z});
      }

      pointCounter++;
    }
  }
  // Else (it is not Ascii)
  else
  {
    std::cout << "Not supported Version!!! Must be ASCII" << std::endl;
  }

#if PCC_VERBOSE
  std::cout << "Larger Pixel Value is " << larger_pixel << std::endl;
#endif

  int bit_limits = ceil(log2(larger_pixel));
  this->n_bits_ = bit_limits;

#if PCC_VERBOSE
  std::cout << "The axis bit limit is " << bit_limits << std::endl;
#endif

  int limits = pow(2, bit_limits);
  gpcc::Voxel<int> lim(limits, limits, limits);
  this->dimension_ = lim;

#if PCC_VERBOSE
  std::cout << "The axis pixel limit is " << limits << std::endl;

  std::cout << "Read the .ply \n"
            << std::endl;
#endif
  return true;
}

bool gpcc::PointCloud::Flush(const std::string &filename)
{

  // Open the file to write the .ply
  std::ofstream fout(filename, std::ofstream::out);
  if (!fout.is_open())
  {
    return false;
  }

  // Get the number of voxels in the point cloud
  const size_t pointCount = this->NumberOccupiedVoxels();

  // Starts writing the file
  fout << "ply" << std::endl;

  // Gives the format to write
  fout << "format ascii 1.0" << std::endl;

  fout << "element vertex " << pointCount << std::endl;

  fout << "property float x" << std::endl;
  fout << "property float y" << std::endl;
  fout << "property float z" << std::endl;

  fout << "end_header" << std::endl;

  // Search every voxel inside this interval
  for (auto vox : this->voxel_list_)
  {
    fout << vox[0] << " " << vox[1] << " " << vox[2];
    fout << std::endl;
  }

  fout.close();
  return true;
}

void gpcc::PointCloud::SortPointCloud()
{
  std::sort(this->voxel_list_.begin(), this->voxel_list_.end());
}

std::vector<gpcc::Voxel<int>> gpcc::PointCloud::getVoxelList()
{
  return this->voxel_list_;
}

int gpcc::PointCloud::countSliceVoxels(int start, int end, gpcc::Axis axis)
{
  // this->SortPointCloud();

  // Image silhouette dimension's size
  // First axis from image
  int first_dimension_size;

  // Second axis from image
  int second_dimension_size;

  // First axis dimension
  gpcc::Axis first_axis_dimension;

  // Second axis dimension
  gpcc::Axis second_axis_dimension;

  // yz plan
  if (axis == Axis::X)
  {
    first_dimension_size = this->dimension_[Axis::Y];
    second_dimension_size = this->dimension_[Axis::Z];
    first_axis_dimension = Axis::Y;
    second_axis_dimension = Axis::Z;
  }

  // zx plan
  else if (axis == Axis::Y)
  {
    first_dimension_size = this->dimension_[Axis::Z];
    second_dimension_size = this->dimension_[Axis::X];
    first_axis_dimension = Axis::Z;
    second_axis_dimension = Axis::X;
  }

  // xy plan
  else
  {
    first_dimension_size = this->dimension_[Axis::X];
    second_dimension_size = this->dimension_[Axis::Y];
    first_axis_dimension = Axis::X;
    second_axis_dimension = Axis::Y;
  }

  int counter = 0;

  for (gpcc::Voxel<int> v : this->voxel_list_)
  {
    if (start <= v[axis] && v[axis] <= end)
    {
      counter++;
    }
  }

  return counter;
}
