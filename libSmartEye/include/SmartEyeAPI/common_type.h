#pragma once
#include <vector>
#include <memory> 

#ifdef _WIN32
  #ifdef SMARTEYEAPI_EXPORTS
  #define SMARTEYE_API __declspec(dllexport)
  #else
  #define SMARTEYE_API __declspec(dllimport)
  #endif
#else
  #define SMARTEYE_API
#endif

typedef enum SE_STATUS_LIST
{
    SE_STATUS_SUCCESS                   =  0,           ///< Operation was successful
    SE_STATUS_ERROR                     = -1,           ///< runtime error.
    SE_STATUS_TIMEOUT                   = -2,           ///< An operation's timeout time expired before it could be completed.     
    SE_STATUS_FAILED_LACK_CONFIG_FILES  = -3,           ///< lack config files
    SE_STATUS_FAILED_CAMERA             = -4,           ///< failed because of camera
    SE_STATUS_FAILED_PROJECTOR          = -5,           ///< failed because of projector
    SE_STATUS_FAILED_NO_DEVICE          = -6,           ///< failed because of no this device
    SE_STATUS_FAILED_CUDA               = -7,           ///< failed because of cuda
} SE_STATUS_LIST;

struct PointXYZRGB
{
    union {
        float data[4];
        struct {
            float x;
            float y;
            float z;
        };
    };

    union {
        union {
            struct {
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;
            };
            float rgb;
        };
        uint32_t rgba;
    };

    inline PointXYZRGB ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = 0;
      a = 255;
    }

    inline PointXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = _r;
      g = _g;
      b = _b;
      a = 255;
    }
};


/**
 * @brief       PointCloud is used for storing collections of 3D points.
 */

template <typename PointT>
class PointCloud
{
public:
    PointCloud() : points(), width(0), height(0) {}
    PointCloud(PointCloud<PointT> &pc) : points (), width (0), height (0) 
    {
        *this = pc;
    }

    virtual ~PointCloud() {}
    
    inline const PointT&
    at (int column, int row) const
    {
      if (this->height > 1)
        return (points.at (row * this->width + column));
      else
        throw std::exception("Can't use 2D indexing with a unorganized point cloud");
    }

    inline PointT&
    at(int column, int row)
    {
        if (this->height > 1)
            return (points.at(row * this->width + column));
        else
            throw std::exception("Can't use 2D indexing with a unorganized point cloud");
    }

    inline const PointT&
    operator () (size_t column, size_t row) const
    {
        return (points[row * this->width + column]);
    }

    inline PointT&
    operator () (size_t column, size_t row)
    {
        return (points[row * this->width + column]);
    }

    //capacity
    inline size_t size () const { return (points.size ()); }
    inline void reserve (size_t n) { points.reserve (n); }
    inline bool empty () const { return points.empty (); }

    inline void resize (size_t n) 
    { 
        points.resize (n);
        if (width * height != n)
        {
          width = static_cast<uint32_t> (n);
          height = 1;
        }
    }

    //element access
    inline const PointT& operator[] (size_t n) const { return (points[n]); }
    inline PointT& operator[] (size_t n) { return (points[n]); }
    inline const PointT& at (size_t n) const { return (points.at (n)); }
    inline PointT& at (size_t n) { return (points.at (n)); }
    inline const PointT& front () const { return (points.front ()); }
    inline PointT& front () { return (points.front ()); }
    inline const PointT& back () const { return (points.back ()); }
    inline PointT& back () { return (points.back ()); }

    inline void 
    push_back (const PointT& pt)
    {
        points.push_back (pt);
        width = static_cast<uint32_t> (points.size ());
        height = 1;
    }

    inline void 
    clear ()
    {
        points.clear ();
        width = 0;
        height = 0;
    }



    /** \brief The point data. */
    std::vector<PointT> points;
    /** \brief The point cloud width (if organized as an image-structure). */
    uint32_t width;
    /** \brief The point cloud height (if organized as an image-structure). */
    uint32_t height;
    typedef std::shared_ptr<PointCloud<PointT> > Ptr;
};

typedef PointXYZRGB PointType;
typedef PointCloud<PointType> PointCloud_SE;
typedef PointCloud_SE::Ptr PointCloud_SE_Ptr;





