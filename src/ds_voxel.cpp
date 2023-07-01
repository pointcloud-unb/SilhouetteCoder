#include "ds_voxel.h"


using namespace gpcc;

// Print voxel
template <class T>
std::string Voxel<T>::ShowCoordinates()
{
  return std::string("{") + x + ", " + y + ", " + z + "}";
  //return "{" + std::to_string(data_[0]) + ", " + std::to_string(data_[1]) + ", " + std::to_string(data_[1]) + "}";
}


template <class T>
bool Voxel<T>::GreaterThan(Voxel<T> p)
{
  //TODO
  return true;
}

template <class T>
bool Voxel<T>::GreaterThan(Voxel<T> p, Axis a)
{
  //TODO
  return true;

}
