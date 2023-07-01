#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <fstream>
#include <tuple>
#include <math.h>
#include <algorithm> // std::sort

#include "ds_iimage.h"
#include "ds_image_raster.h"
#include "ds_image_sparse.h"
#include "ds_voxel.h"

//! GPCC namespace
namespace gpcc
{
  //! Point Cloud
  /*
   * Minimum space ocuppied on the PC
   */

  //! vox_t
  /*
   * vox_t type
   *
   * Voxel axis unit step type
   *
   */
  typedef int vox_t;

  class PointCloud
  {

  private:
    //! Point Cloud's Voxel list
    /*
     * List of voxels
     */
    std::vector<Voxel<int>> voxel_list_;

    //! Point Cloud's voxels list size
    /*
     * List number of occupied voxels
     * from the point cloud
     */
    int point_cloud_voxel_count_;

    //! Point Cloud's Boundaries
    /*
     * Boundaries of the cloud
     */
    Voxel<int> dimension_;

    //! Point Cloud Resolution Limit
    /*
     * Determines the point cloud's limit resolution value
     */
    int point_cloud_resolution_limit_;

    //! N bits limit
    /*
     * Determines the point cloud's resolution's
     * bits quality
     */
    int n_bits_;

    //! Point Cloud 1st dim Limits
    /*
     * Pair of min and max 1st dim value in the point cloud
     * The first member is the min value, the second the max
     */
    std::pair<vox_t, vox_t> point_cloud_u_limits_;

    //! Point Cloud 2nd dim Limits
    /*
     * Pair of min and max 2nd dim value in the point cloud
     * The first member is the min value, the second the max
     */
    std::pair<vox_t, vox_t> point_cloud_v_limits_;

    //! Point Cloud 3rd dim Limits
    /*
     * Pair of min and max 3rd dim value in the point cloud
     * The first member is the min value, the second the max
     */
    std::pair<vox_t, vox_t> point_cloud_w_limits_;

  public:
    //! Point Cloud Constructor
    PointCloud();

    //! Point Cloud Constructor
    /*
     * Constructor with a initialization
     * of point cloud
     */
    PointCloud(std::vector<Voxel<int>> voxel_list);

    //! Add voxel
    /*
     * Adds a voxel with a Voxel object
     */
    void addVoxel(Voxel<int> p);

    //! Add voxel
    /*
     * Adds a voxel with a axis coordinates
     */
    void addVoxel(vox_t x, vox_t y, vox_t z);

    //! Remove voxel
    /*
     * Removes a voxel given a Voxel object.
     * The voxel list should be sorted!
     */
    void removeVoxel(Voxel<int> p);

    //! Remove voxel
    /*
     * Removes a voxel given a Voxel object
     * The voxel list should be sorted!
     */
    void removeVoxel(vox_t x0, vox_t y0, vox_t z0);

    //! Voxel Present verification function
    /*
     * Verify existence of a given voxel
     * on pointcloud by Voxel object
     */
    bool VoxelPresent(Voxel<int> p);

    //! Voxel Present verification function
    /*
     * Verify existence of a given voxel on
     * pointcloud by its coodinates
     */
    bool VoxelPresent(vox_t x0, vox_t y0, vox_t z0);

    //! Number Occupied Voxels
    /*
     * Gives the quantity of voxels occupied in
     * the point cloud
     */
    size_t NumberOccupiedVoxels();

    //! Size
    /*
     * Returns the Point Cloud x,y,z Size
     */
    std::tuple<int, int, int> Size();

    //! Update Size
    /*
     * Updates 1st, 2nd and 3rd Size of Image Sparse
     */
    void updateSize(int limit_1, int limit_2, int limit_3);

    //! Silhouette making
    /*
     * Returns a silhouette from a start and end slice,
     * on a given axis selected
     */
    ImageSparse *SilhouetteFromPointCloud(
        vox_t slice_start, vox_t slice_stop, Axis axis);

    //! Silhouette making
    /*
     * Receives a image sparse and adds the pixels
     * values to the point cloud at location coordinate
     * along axis
     * - XYZ order, receives YZ image
     * - YZX order, receives ZX image
     * - ZXY order, receives XY image
     */
    void AddSilhouette(Axis axis,
                       vox_t coordinate,
                       ImageSparse *image);

    //! Silhouette making
    /*
     * Receives a image sparse and adds the pixels
     * values to the point cloud at location coordinate
     * along axis
     * - XYZ order, receives YZ image
     * - YZX order, receives ZX image
     * - ZXY order, receives XY image
     */
    void RecoverVoxelsFromSilhouette(Axis axis,
                                     vox_t coordinate,
                                     ImageRaster *image);

    //! UnifyPointCloud
    /*
     * Receives another Point cloud and overlaps
     * the original Point cloud, generating a
     * unified Point Cloud
     */
    void UnifyPointCloud(PointCloud point_cloud);

    //! + operator
    /*
     * Receives another Point cloud and overlaps
     * the original Point cloud, returning a
     * unified Point Cloud
     */
    PointCloud operator+(PointCloud const &point_cloud);

    //! - operator
    /*
     * Receives another Point cloud and substracts
     * overlaping voxels, generating a
     * subtracted Point Cloud
     */
    void SubtractsPointCloud(PointCloud point_cloud);

    //! - operator
    /*
     * Receives another Point cloud and substracts
     * overlaping voxels, returning a
     * subtracted Point Cloud
     */
    PointCloud operator-(PointCloud const &point_cloud);

    //! Load
    /*
     * Receives a plyfile and constructs
     * the Point Cloud voxel list
     *
     * \param ply_file file's name path from which
     *  the point cloud will be loaded
     */
    bool Load(const std::string &ply_file);

    //! Flush
    /*
     * Uses the built Point Cloud voxel list
     * and writes the point cloud to a ply file
     *
     * \param ply_file file's name path where
     *  the point cloud will be written on
     */
    bool Flush(const std::string &ply_file);

    //! Show Point Clouds
    /*
     * Return the string form point cloud to be printed
     */
    std::string ShowPointCloud();

    // Returns the point cloud resolution.
    int getResolution() { return n_bits_; }

    //! Sort Point Cloud
    /*
     * Sorts the point cloud according to axis
     * X, Y and Z.
     */
    void SortPointCloud();

    //! Get Voxel List
    /*
     * Returns the Point Cloud's voxel list
     */
    std::vector<Voxel<int>> getVoxelList();

    //! Get Voxel List
    /*
     * Returns the Point Cloud's voxel list
     * from the given slice interval. Both start and 
     * end bound is inclusive.
     */
    int countSliceVoxels(int start, int end, gpcc::Axis axis);
  };

  //! Show Point Clouds
  /*
   * Return the voxel triple coordinates at t3 = coordinate, according
   * to the slice direction, given the pixel coordinates (t0, t1)
   *
   * This function transforms the pixel to the point clouds
   * spatial reference(XYZ, YZX, or ZXY)
   */
  std::tuple<int, int, int> WrapVoxel(Axis axis, vox_t coordinate,
                                      vox_t t0, vox_t t1);
} // namespace gpcc
#endif // POINT_CLOUD_H
