#ifndef Voxel_h
#define Voxel_h

#include <assert.h>
#include <type_traits>
#include <iostream>
#include <cstring>

//! GPCC namespace
namespace gpcc
{
  //! Axis
  /*
   * Axis enum class
   * 
   * Axis that point cloud operates
   * 
   */
  enum Axis
  {
    X,
    Y,
    Z
  };

  //! Voxel
  /*
   * Voxel class
   * 
   * Minimum space unit ocuppied on the point cloud
   */
  template <typename T = int>
  class Voxel
  {
    //! vox_t
    /*
     * vox_t type
     * 
     * Voxel axis unit step type
     * 
     */
    typedef T vox_t;

  private:
    //! data array
    /*
    * Voxel's attribute
    * 
    * Coordinates (x,y,z) occupied by the point cloud
    */
    T data_[3];

    template <typename U>
    friend class gpcc::Voxel;

  public:
    /// Interators ///
    T *begin() { return &data_[0]; }
    const T *begin() const { return &data_[0]; }

    T *end() { return &data_[2]; }
    const T *end() const { return &data_[2]; }

    /// Indexing operators ///
    T &operator[](size_t i)
    {
      assert(i < 3);
      return data_[i];
    }
    const T &operator[](size_t i) const
    {
      assert(i < 3);
      return data_[i];
    }
    size_t getElementCount() const { return 3; }

    /// Getters ///
    T &x() { return data_[0]; }
    T &y() { return data_[1]; }
    T &z() { return data_[2]; }

    const T &x() const { return data_[0]; }
    const T &y() const { return data_[1]; }
    const T &z() const { return data_[2]; }

    /// Assignement operators ///
    Voxel &operator=(const Voxel &rhs)
    {
      memcpy(data_, rhs.data_, sizeof(data_));
      return *this;
    }

    template <typename U>
    Voxel &operator+=(const typename gpcc::Voxel<U> &rhs)
    {
      data_[0] += rhs[0];
      data_[1] += rhs[1];
      data_[2] += rhs[2];
      return *this;
    }

    template <typename U>
    Voxel &operator-=(const typename gpcc::Voxel<U> &rhs)
    {
      data_[0] -= rhs[0];
      data_[1] -= rhs[1];
      data_[2] -= rhs[2];
      return *this;
    }

    template <typename U>
    Voxel &operator-=(U a)
    {
      data_[0] -= a;
      data_[1] -= a;
      data_[2] -= a;
      return *this;
    }

    template <typename U>
    Voxel &operator+=(U a)
    {
      data_[0] += a;
      data_[1] += a;
      data_[2] += a;
      return *this;
    }

    Voxel &operator<<=(int val)
    {
      data_[0] <<= val;
      data_[1] <<= val;
      data_[2] <<= val;
      return *this;
    }

    Voxel &operator>>=(int val)
    {
      data_[0] >>= val;
      data_[1] >>= val;
      data_[2] >>= val;
      return *this;
    }

    template <typename U>
    Voxel &operator/=(U a)
    {
      assert(a != 0);
      data_[0] /= a;
      data_[1] /= a;
      data_[2] /= a;
      return *this;
    }

    template <typename U>
    Voxel &operator*=(U a)
    {
      data_[0] *= a;
      data_[1] *= a;
      data_[2] *= a;
      return *this;
    }

    template <typename U>
    Voxel &operator=(const U a)
    {
      data_[0] = a;
      data_[1] = a;
      data_[2] = a;
      return *this;
    }

    template <typename U>
    Voxel &operator=(const U *rhs)
    {
      data_[0] = rhs[0];
      data_[1] = rhs[1];
      data_[2] = rhs[2];
      return *this;
    }

    /// Unary operators ///
    Voxel operator-() const { return Voxel<T>(-data_[0], -data_[1], -data_[2]); }

    /// Arithmetic operators ///
    // Voxel + Voxel
    template <typename U>
    friend Voxel<typename std::common_type<T, U>::type>
    operator+(const Voxel &lhs, const typename gpcc::Voxel<U> &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]);
    }

    // arithmetic const + Voxel
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator+(const U lhs, const Voxel &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs + rhs[0], lhs + rhs[1], lhs + rhs[2]);
    }

    // Voxel + artihmetic const
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator+(const Voxel &lhs, const U rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] + rhs, lhs[1] + rhs, lhs[2] + rhs);
    }

    // Voxel - Voxel
    template <typename U>
    friend Voxel<typename std::common_type<T, U>::type>
    operator-(const Voxel &lhs, const typename gpcc::Voxel<U> &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]);
    }

    // Arithmetic const - Voxel
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator-(const U lhs, const Voxel &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs - rhs[0], lhs - rhs[1], lhs - rhs[2]);
    }

    // Voxel - arithmetic const
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator-(const Voxel &lhs, const U rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] - rhs, lhs[1] - rhs, lhs[2] - rhs);
    }

    // Arithmetic const * Voxel
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator*(const U lhs, const Voxel &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs * rhs[0], lhs * rhs[1], lhs * rhs[2]);
    }

    // Voxel * Voxel = elementwise product
    template <typename U>
    friend Voxel<typename std::common_type<T, U>::type>
    operator*(const Voxel &lhs, const typename gpcc::Voxel<U> &rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] * rhs[0], lhs[1] * rhs[1], lhs[2] * rhs[2]);
    }

    // Voxel * Arithmetic const
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator*(const Voxel &lhs, const U rhs)
    {
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] * rhs, lhs[1] * rhs, lhs[2] * rhs);
    }

    // Voxel / arithmetic const
    template <typename U>
    friend Voxel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator/(const Voxel &lhs, const U rhs)
    {
      assert(rhs != 0);
      return Voxel<typename std::common_type<T, U>::type>(lhs[0] / rhs, lhs[1] / rhs, lhs[2] / rhs);
    }

    // Left shift
    friend Voxel operator<<(const Voxel &lhs, int val)
    {
      return Voxel<T>(lhs[0] << val, lhs[1] << val, lhs[2] << val);
    }

    // Right shift
    friend Voxel operator>>(const Voxel &lhs, int val)
    {
      return Voxel<T>(lhs[0] >> val, lhs[1] >> val, lhs[2] >> val);
    }

    /// Relationals
    bool operator<(const Voxel &rhs) const
    {
      if (data_[0] == rhs[0])
      {
        if (data_[1] == rhs[1])
        {
          return (data_[2] < rhs[2]);
        }
        return (data_[1] < rhs[1]);
      }
      return (data_[0] < rhs[0]);
    }

    bool operator>(const Voxel &rhs) const
    {
      if (data_[0] == rhs[0])
      {
        if (data_[1] == rhs[1])
        {
          return (data_[2] > rhs[2]);
        }
        return (data_[1] > rhs[1]);
      }
      return (data_[0] > rhs[0]);
    }

    bool operator==(const Voxel &rhs) const
    {
      return (data_[0] == rhs[0] && data_[1] == rhs[1] && data_[2] == rhs[2]);
    }

    bool operator!=(const Voxel &rhs) const
    {
      return (data_[0] != rhs[0] || data_[1] != rhs[1] || data_[2] != rhs[2]);
    }

    /// Streams ///
    friend std::ostream &operator<<(std::ostream &os, const Voxel &vox)
    {
      os << vox[0] << " " << vox[1] << " " << vox[2];
      return os;
    }

    friend std::istream &operator>>(std::istream &is, Voxel &vox)
    {
      is >> vox[0] >> vox[1] >> vox[2];
      return is;
    }

    /// Constructors ///
    Voxel(const T a) { data_[0] = data_[1] = data_[2] = a; }

    Voxel(const T x, const T y, const T z)
    {
      data_[0] = x;
      data_[1] = y;
      data_[2] = z;
    }

    template <typename U>
    Voxel(const typename gpcc::Voxel<U> &vox)
    {
      data_[0] = vox.data_[0];
      data_[1] = vox.data_[1];
      data_[2] = vox.data_[2];
    }

    Voxel() = default;
    ~Voxel(void) = default;

    //! Show Coordinates
    /*
     * Prints the Voxel coordinates
     */
    std::string ShowCoordinates();

    //! Greater Than
    /* 
     * Greater than: takes two voxels, return a bool.
     * 
     * Default case: xyz ordering.
     * 
     */
    bool GreaterThan(Voxel p);
    //! Greater Than
    /*
     * Greater than: takes two voxels, return bool.
     */
    bool GreaterThan(Voxel p, Axis a = Axis::X);

  }; // class Voxel

  // Inner product
  template <typename T, typename U>
  typename std::common_type<T, U>::type
  inner(Voxel<T> lhs, const Voxel<U> &rhs)
  {
    return (lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]);
  }

  // Squared L^2 norm
  template <typename T>
  //typename std::common_type<T, U>::type
  T normL2Squared(Voxel<T> vox)
  {
    return inner(vox, vox);
  }

  // L infinity norm
  template <typename T>
  T normInf(Voxel<T> vox)
  {
    return std::max(vox[0], std::max(vox[1], vox[2]));
  }

} // namespace
#endif // Voxel_h
