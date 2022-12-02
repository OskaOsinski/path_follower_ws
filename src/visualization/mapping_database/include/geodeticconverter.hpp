//GeodeticConverter.hpp
#ifndef air_GeodeticConverter_hpp
#define air_GeodeticConverter_hpp

#include <cmath>
#include <Eigen>
#include "commonstruct.h"

/**
 * @brief The GeodeticConverter class which holds all mathematical calculation
 * to convert WGS84 coordination system into local tangent space.
 *
 * Local tangent space is tangential in point provided as constructor argument.
 * This point is also center of coordination system. There are two notation
 * which are different in order of axis in vector and direction of altidute.
 * Name of this notation system is also its definition:
 * - NED (North East Down)
 * - ENU (East North Up).this one will be used in shape file.
 * Also during conversion we use space called ECEF (Earth Centrix Earth Fixed)
 * which define position of point in cartesian XYZ coordination system with
 * root in earth center.
 */
class GeodeticConverter
{
 public:
  GeodeticConverter(CarPosition centerOfConversion);
  CarPosition getHome() const;

  Point geodeticToEcef(GeoPoint geoCord) const;
  GeoPoint ecefToGeodetic(Point ecefCord) const;

  Point ecefToNed(Point ecefCord) const;
  Point nedToEcef(Point nedCord) const;

  Point geodeticToEnu(GeoPoint geoCord) const;
  GeoPoint enuToGeodetic(Point enuCord) const;

  Point geodeticToCarSpace(GeoPoint geoCord) const;
  GeoPoint carSpaceToGeodetic(Point geoCord) const;

  static constexpr double DEG_TO_RAD(double deg)
  {
      return deg / 180.0 * M_PI;
  }

  static constexpr double RAD_TO_DEG(double rad)
  {
      return rad / M_PI * 180.0;
  }

  static inline Point rotatePointBy(double angle, Point pointToRotate)
  {
      Point afterRotation;

      afterRotation.x = pointToRotate.x*std::cos(angle) - pointToRotate.y*std::sin(angle);
      afterRotation.y = pointToRotate.x*std::sin(angle) + pointToRotate.y*std::cos(angle);
      afterRotation.z = pointToRotate.z;

      return afterRotation;
  }

  static inline double calculateAzimuth(const GeoPoint &first, const GeoPoint &second)
  {
      double y = std::sin(second.lon-first.lon) * std::cos(second.lat);
      double x = std::cos(first.lat)*std::sin(second.lat) - std::sin(first.lat)*std::cos(second.lat)*std::cos(second.lon-first.lon);
      double brng = std::fmod(std::atan2(y, x) + 2.0 * M_PI, 2.0*M_PI) ;

      return brng;
  }

private:
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;

  // Geodetic system parameters
  constexpr static double kSemimajorAxis = 6378137;
  constexpr static double kSemiminorAxis = 6356752.3142;
  constexpr static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
  constexpr static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
  constexpr static double kFlattening = 1 / 298.257223563;

  inline Matrix3x3d nRe(const double latRadians, const double lonRadians)
  {
    const double sLat = sin(latRadians);
    const double sLon = sin(lonRadians);
    const double cLat = cos(latRadians);
    const double cLon = cos(lonRadians);

    Matrix3x3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
  }

  CarPosition m_centerOfConversion;

  double m_homeLatitudeRad, m_homeLatitude;
  double m_homeLongitudeRad, m_homeLongitude;
  double m_homeAltitude;

  double m_homeEcefX;
  double m_homeEcefY;
  double m_homeEcefZ;

  Matrix3x3d m_ecefToNedMatrix;
  Matrix3x3d m_nedToEcefMatrix;

}; // class GeodeticConverter

#endif
