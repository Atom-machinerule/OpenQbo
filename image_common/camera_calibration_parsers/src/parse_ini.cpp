#include "camera_calibration_parsers/parse_ini.h"
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_file_iterator.hpp>
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/spirit/include/classic_loops.hpp>
#include <boost/typeof/typeof.hpp>
#include <iterator>
#include <fstream>

namespace camera_calibration_parsers {

/// @todo Move to new spirit
using namespace BOOST_SPIRIT_CLASSIC_NS;

/// \cond

struct SimpleMatrix
{
  int rows;
  int cols;
  const double* data;

  SimpleMatrix(int rows, int cols, const double* data)
    : rows(rows), cols(cols), data(data)
  {}
};

std::ostream& operator << (std::ostream& out, const SimpleMatrix& m)
{
  for (int i = 0; i < m.rows; ++i) {
    for (int j = 0; j < m.cols; ++j) {
      out << m.data[m.cols*i+j] << " ";
    }
    out << std::endl;
  }
  return out;
}

/// \endcond

bool writeCalibrationIni(std::ostream& out, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  // Videre INI format is legacy, only supports plumb bob distortion model.
  if (cam_info.distortion_model != sensor_msgs::distortion_models::PLUMB_BOB ||
      cam_info.D.size() != 5) {
    ROS_ERROR("Videre INI format can only save calibrations using the plumb bob distortion model. "
              "Use the YAML format instead.\n"
              "\tdistortion_model = '%s', expected '%s'\n"
              "\tD.size() = %d, expected 5", cam_info.distortion_model.c_str(),
              sensor_msgs::distortion_models::PLUMB_BOB.c_str(), (int)cam_info.D.size());
    return false;
  }
  
  out.precision(5);
  out << std::fixed;
  
  out << "# Camera intrinsics\n\n";
  /// @todo time?
  out << "[image]\n\n";
  out << "width\n" << cam_info.width << "\n\n";
  out << "height\n" << cam_info.height << "\n\n";
  out << "[" << camera_name << "]\n\n";

  out << "camera matrix\n"     << SimpleMatrix(3, 3, &cam_info.K[0]);
  out << "\ndistortion\n"      << SimpleMatrix(1, 5, &cam_info.D[0]);
  out << "\n\nrectification\n" << SimpleMatrix(3, 3, &cam_info.R[0]);
  out << "\nprojection\n"      << SimpleMatrix(3, 4, &cam_info.P[0]);

  return true;
}

bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  std::ofstream out(file_name.c_str());
  if (!out.is_open())
  {
    ROS_ERROR("Unable to open camera calibration file [%s] for writing", file_name.c_str());
    return false;
  }
  return writeCalibrationIni(out, camera_name, cam_info);
}

/// \cond
// Semantic action to store a sequence of values in an array
template <typename T>
struct ArrayAssignActor
{
  ArrayAssignActor(T* start)
    : ptr_(start)
  {}

  void operator()(T val) const
  {
    *ptr_++ = val;
  }

  mutable T* ptr_;
};

// Semantic action generator
template <typename T>
ArrayAssignActor<T> array_assign_a(T* start)
{
  return ArrayAssignActor<T>(start);
}

template <typename Iterator>
bool parseCalibrationIniRange(Iterator first, Iterator last,
                              std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  // Assume plumb bob model
  cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  cam_info.D.resize(5);
  
  // We don't actually use the [externals] info, but it's part of the format
  bool have_externals = false;
  double trans[3], rot[3];

  /// @todo separate grammar out into separate function
  
  // Image section (width, height)
  BOOST_AUTO(image,
      str_p("[image]")
      >> "width"
      >> uint_p[assign_a(cam_info.width)]
      >> "height"
      >> uint_p[assign_a(cam_info.height)]
     );

  // Optional externals section
  BOOST_AUTO(externals,
      str_p("[externals]")
      >> "translation"
      >> repeat_p(3)[real_p[array_assign_a(trans)]]
      >> "rotation"
      >> repeat_p(3)[real_p[array_assign_a(rot)]]
     );

  // Parser to save name of camera section
  BOOST_AUTO(name, confix_p('[', (*anychar_p)[assign_a(camera_name)], ']'));

  // Camera section (intrinsics)
  BOOST_AUTO(camera,
      name
      >> "camera matrix"
      >> repeat_p(9)[real_p[array_assign_a(&cam_info.K[0])]]
      >> "distortion"
      >> repeat_p(5)[real_p[array_assign_a(&cam_info.D[0])]]
      >> "rectification"
      >> repeat_p(9)[real_p[array_assign_a(&cam_info.R[0])]]
      >> "projection"
      >> repeat_p(12)[real_p[array_assign_a(&cam_info.P[0])]]
     );

  // Full grammar
  BOOST_AUTO(ini_grammar,
      image
      >> !externals[assign_a(have_externals, true)]
      >>  camera);

  // Skip whitespace and line comments
  BOOST_AUTO(skip, space_p | comment_p('#'));

  parse_info<Iterator> info = parse(first, last, ini_grammar, skip);
  return info.hit;
}
/// \endcond

bool readCalibrationIni(std::istream& in, std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  std::istream_iterator<char> first(in), last;
  return parseCalibrationIniRange(first, last, camera_name, cam_info);
}

bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info)
{
  typedef file_iterator<char> Iterator;

  Iterator first(file_name);
  if (!first) {
    ROS_ERROR("Unable to open camera calibration file [%s]", file_name.c_str());
    return false;
  }
  Iterator last = first.make_end();

  return parseCalibrationIniRange(first, last, camera_name, cam_info);
}

bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         sensor_msgs::CameraInfo& cam_info)
{
  return parseCalibrationIniRange(buffer.begin(), buffer.end(), camera_name, cam_info);
}

} //namespace camera_calibration_parsers
