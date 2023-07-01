#ifndef Pixel_h
#define Pixel_h

#include <assert.h>
#include <type_traits>
#include <iostream>
#include <cstring>

namespace gpcc
{
  template <typename T = int>
  class Pixel
  {
  private:
    T data[2];

    template <typename U>
    friend class gpcc::Pixel;

  public:
    /// Iterators ///
    T *begin() { return &data[0]; }
    const T *begin() const { return &data[0]; }

    T *end() { return &data[1]; }
    const T *end() const { return &data[1]; }

    /// Indexing operators ///
    T &operator[](size_t i)
    {
      // TODO: throw exception instead of assert!
      assert(i < 2);
      return data[i];
    }
    const T &operator[](size_t i) const
    {
      // TODO: throw exception instead of assert!
      assert(i < 2);
      return data[i];
    }
    size_t getElementCount() const { return 2; }

    /// Getters ///
    T &x() { return data[0]; }
    T &y() { return data[1]; }
    const T &x() const { return data[0]; }
    const T &y() const { return data[1]; }

    /// Assignement operators ///
    Pixel &operator=(const Pixel &rhs)
    {
      memcpy(data, rhs.data, sizeof(data));
      return *this;
    }

    template <typename U>
    Pixel &operator+=(const typename gpcc::Pixel<U> &rhs)
    {
      data[0] += rhs[0];
      data[1] += rhs[1];
      return *this;
    }

    template <typename U>
    Pixel &operator-=(const typename gpcc::Pixel<U> &rhs)
    {
      data[0] -= rhs[0];
      data[1] -= rhs[1];
      return *this;
    }

    template <typename U>
    Pixel &operator-=(U a)
    {
      data[0] -= a;
      data[1] -= a;
      return *this;
    }

    template <typename U>
    Pixel &operator+=(U a)
    {
      data[0] += a;
      data[1] += a;
      return *this;
    }

    Pixel &operator<<=(int val)
    {
      data[0] <<= val;
      data[1] <<= val;
      return *this;
    }

    Pixel &operator>>=(int val)
    {
      data[0] >>= val;
      data[1] >>= val;
      return *this;
    }

    template <typename U>
    Pixel &operator/=(U a)
    {
      // TODO: throw exception instead of assert!
      assert(a != 0);
      data[0] /= a;
      data[1] /= a;
      return *this;
    }

    template <typename U>
    Pixel &operator*=(U a)
    {
      data[0] *= a;
      data[1] *= a;
      return *this;
    }

    template <typename U>
    Pixel &operator=(const U a)
    {
      data[0] = a;
      data[1] = a;
      return *this;
    }

    template <typename U>
    Pixel &operator=(const U *rhs)
    {
      data[0] = rhs[0];
      data[1] = rhs[1];
      return *this;
    }

    std::string ShowPixel()
    {
      std::string out = "(" + std::to_string(data[0]) + "," +
                        std::to_string(data[1]) + ")\n";
      return out;
    }

    /// Unary operators ///
    Pixel operator-() const { return Pixel<T>(-data[0], -data[1]); }

    /// Arithmetic operators ///
    // Pixel + Pixel
    template <typename U>
    friend Pixel<typename std::common_type<T, U>::type>
    operator+(const Pixel &lhs, const typename gpcc::Pixel<U> &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] + rhs[0], lhs[1] + rhs[1]);
    }

    // arithmetic const + Pixel
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator+(const U lhs, const Pixel &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs + rhs[0], lhs + rhs[1]);
    }

    // Pixel + artihmetic const
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator+(const Pixel &lhs, const U rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] + rhs, lhs[1] + rhs);
    }

    // Pixel - Pixel
    template <typename U>
    friend Pixel<typename std::common_type<T, U>::type>
    operator-(const Pixel &lhs, const typename gpcc::Pixel<U> &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] - rhs[0], lhs[1] - rhs[1]);
    }

    // Arithmetic const - Pixel
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator-(const U lhs, const Pixel &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs - rhs[0], lhs - rhs[1]);
    }

    // Pixel - arithmetic const
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator-(const Pixel &lhs, const U rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] - rhs, lhs[1] - rhs);
    }

    // Arithmetic const * Pixel
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator*(const U lhs, const Pixel &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs * rhs[0], lhs * rhs[1]);
    }

    // Pixel * Pixel = elementwise product
    // TODO: Add an inner product function?
    template <typename U>
    friend Pixel<typename std::common_type<T, U>::type>
    operator*(const Pixel &lhs, const typename gpcc::Pixel<U> &rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] * rhs[0], lhs[1] * rhs[1]);
    }

    // Pixel * Arithmetic const
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator*(const Pixel &lhs, const U rhs)
    {
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] * rhs, lhs[1] * rhs);
    }

    // Pixel / arithmetic const
    template <typename U>
    friend Pixel<typename std::enable_if<
        std::is_arithmetic<U>::value,
        typename std::common_type<T, U>::type>::type>
    operator/(const Pixel &lhs, const U rhs)
    {
      assert(rhs != 0);
      return Pixel<typename std::common_type<T, U>::type>(lhs[0] / rhs, lhs[1] / rhs);
    }

    // Left shift
    friend Pixel operator<<(const Pixel &lhs, int val)
    {
      return Pixel<T>(lhs[0] << val, lhs[1] << val);
    }

    // Right shift
    friend Pixel operator>>(const Pixel &lhs, int val)
    {
      return Pixel<T>(lhs[0] >> val, lhs[1] >> val);
    }

    /// Relationals
    bool operator<(const Pixel &rhs) const
    {
      if (data[0] == rhs[0])
      {
        return (data[1] < rhs[1]);
      }
      return (data[0] < rhs[0]);
    }

    bool operator>(const Pixel &rhs) const
    {
      if (data[0] == rhs[0])
      {
        return (data[1] > rhs[1]);
      }
      return (data[0] > rhs[0]);
    }

    bool operator==(const Pixel &rhs) const
    {
      return (data[0] == rhs[0] && data[1] == rhs[1]);
    }

    bool operator!=(const Pixel &rhs) const
    {
      return (data[0] != rhs[0] || data[1] != rhs[1]);
    }

    /// Streams ///
    friend std::ostream &operator<<(std::ostream &os, const Pixel &pix)
    {
      os << pix[0] << " " << pix[1];
      return os;
    }

    friend std::istream &operator>>(std::istream &is, Pixel &pix)
    {
      is >> pix[0] >> pix[1];
      return is;
    }

    /// Constructors ///
    Pixel(const T a) { data[0] = data[1] = a; }

    Pixel(const T x, const T y)
    {
      data[0] = x;
      data[1] = y;
    }

    template <typename U>
    Pixel(const typename gpcc::Pixel<U> &pix)
    {
      data[0] = pix.data[0];
      data[1] = pix.data[1];
    }

    Pixel() = default;
    ~Pixel(void) = default;

  }; // class Pixel

  // Inner product
  template <typename T, typename U>
  typename std::common_type<T, U>::type
  inner(Pixel<T> lhs, const Pixel<U> &rhs)
  {
    return (lhs[0] * rhs[0] + lhs[1] * rhs[1]);
  }

  // Squared L^2 norm
  template <typename T>
  //typename std::common_type<T, U>::type
  T normL2Squared(Pixel<T> pix)
  {
    return inner(pix, pix);
  }

  // L infinity norm
  template <typename T>
  T normInf(Pixel<T> pix)
  {
    return std::max(pix[0], pix[1]);
  }

} // namespace
#endif // Pixel_h
