#include "geodeticconverter.hpp"



/**
 * @brief GeodeticConverter::GeodeticConverter initialize conversion calculation
 * with root of space system in point provided as constructor argument.
 * @param centerOfConversion home of ENU or ECF system in WGS84 system
 */
GeodeticConverter::GeodeticConverter(CarPosition centerOfConversion) :
    m_centerOfConversion(centerOfConversion),
    m_homeLatitude(m_centerOfConversion.point.lat),
    m_homeLongitude(m_centerOfConversion.point.lon),
    m_homeAltitude(m_centerOfConversion.point.h)
{
    // Save NED origin
    m_homeLatitudeRad = DEG_TO_RAD(m_homeLatitude);
    m_homeLongitudeRad = DEG_TO_RAD(m_homeLongitude);

    // Compute ECEF of NED origin
    Point nedHome = geodeticToEcef(m_centerOfConversion.point);
    m_homeEcefX = nedHome.x;
    m_homeEcefY = nedHome.y;
    m_homeEcefZ = nedHome.z;
    // Compute ECEF to NED and NED to ECEF matrices
    double phiP = atan2(m_homeEcefZ, sqrt(pow(m_homeEcefX, 2) + pow(m_homeEcefY, 2)));

    m_ecefToNedMatrix = nRe(phiP, m_homeLongitudeRad);
    m_nedToEcefMatrix = nRe(m_homeLatitudeRad, m_homeLongitudeRad).transpose();
}

/**
 * @brief GeodeticConverter::getHome return point which is root of NED and ENU
 * coordination system.
 * @return home of NED and ENU coordination system in WGS84 space
 */
CarPosition GeodeticConverter::getHome() const
{
    return m_centerOfConversion;

}

/**
 * @brief GeodeticConverter::geodeticToEcef convert WGS84 point into ECEF space
 * @param geoCord point in WGS84 space
 * @return point in ECEF space
 */
Point GeodeticConverter::geodeticToEcef(GeoPoint geoCord) const
{
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    double lat_rad = DEG_TO_RAD(geoCord.lat);
    double lon_rad = DEG_TO_RAD(geoCord.lon);
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    Point ret;
    ret.x = (kSemimajorAxis / xi + geoCord.h) * cos(lat_rad) * cos(lon_rad);
    ret.y = (kSemimajorAxis / xi + geoCord.h) * cos(lat_rad) * sin(lon_rad);
    ret.z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + geoCord.h) * sin(lat_rad);
    return ret;
}

/**
 * @brief GeodeticConverter::ecefToGeodetic conver ECEF point into WGS84 space
 * @param ecefCord point in ECEF space
 * @return point in WGS84 space
 */
GeoPoint GeodeticConverter::ecefToGeodetic(Point ecefCord) const
{
    // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
    // to geodetic coordinates," IEEE Transactions on Aerospace and
    // Electronic Systems, vol. 30, pp. 957-961, 1994.

    double r = sqrt(ecefCord.x * ecefCord.x + ecefCord.y * ecefCord.y);
    double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    double F = 54 * kSemiminorAxis * kSemiminorAxis * ecefCord.z * ecefCord.z;
    double G = r * r + (1 - kFirstEccentricitySquared) * ecefCord.z * ecefCord.z - kFirstEccentricitySquared * Esq;
    double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
    double S = cbrt(1 + C + sqrt(C * C + 2 * C));
    double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
            + sqrt(
                0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                - P * (1 - kFirstEccentricitySquared) * ecefCord.z * ecefCord.z / (Q * (1 + Q)) - 0.5 * P * r * r);
    double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + ecefCord.z * ecefCord.z);
    double V = sqrt(
                pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * ecefCord.z * ecefCord.z);
    double Z_0 = kSemiminorAxis * kSemiminorAxis * ecefCord.z / (kSemimajorAxis * V);

    GeoPoint ret;
    ret.h = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
    ret.lat = RAD_TO_DEG(atan((ecefCord.z + kSecondEccentricitySquared * Z_0) / r));
    ret.lon = RAD_TO_DEG(atan2(ecefCord.y, ecefCord.x));

    return ret;
}

/**
 * @brief GeodeticConverter::ecefToNed convert ECEF point into NED space
 * @param ecefCord point in ECEF space
 * @return point int NED space
 */
Point GeodeticConverter::ecefToNed(Point ecefCord) const
{

    Vector3d vect, ret;
    vect(0) = ecefCord.x - m_homeEcefX;
    vect(1) = ecefCord.y - m_homeEcefY;
    vect(2) = ecefCord.z - m_homeEcefZ;

    ret = m_ecefToNedMatrix * vect;

    Point retPoint;
    retPoint.x = ret(0);
    retPoint.y = ret(1);
    retPoint.z = -ret(2);

    return retPoint;
}

/**
 * @brief GeodeticConverter::nedToEcef convert NED point into ECEF space
 * @param nedCord point in NED space
 * @return point in ECEF space
 */
Point GeodeticConverter::nedToEcef(Point nedCord) const
{
    Vector3d ned;
    Vector3d ret;
    ned(0) = nedCord.x;
    ned(1) = nedCord.y;
    ned(2) = -nedCord.z;
    ret = m_nedToEcefMatrix * ned;

    Point retPoint;
    retPoint.x = ret(0) + m_homeEcefX;
    retPoint.y = ret(1) + m_homeEcefY;
    retPoint.z = ret(2) + m_homeEcefZ;
    return retPoint;
}



/**
 * @brief GeodeticConverter::geodeticToEnu convert WGS84 point into ENU space
 * @param geoCord point in WGS84 space
 * @return point in enu space
 */
Point GeodeticConverter::geodeticToEnu(GeoPoint geoCord) const
{
    Point edefPoint = geodeticToEcef(geoCord);
    Point nedPoint = ecefToNed(edefPoint);

    Point enuPoint;
    enuPoint.x = nedPoint.y;
    enuPoint.y = nedPoint.x;
    enuPoint.z = -nedPoint.z;

    return enuPoint;
}

/**
 * @brief GeodeticConverter::enuToGeodetic convert point in enu space into point
 * in WGS84 space.
 * @param enuCord Point in enu space
 * @return Point in WGS84 space
 */
GeoPoint GeodeticConverter::enuToGeodetic(Point enuCord) const
{
    Point nedCord;
    nedCord.x = enuCord.x;
    nedCord.y = enuCord.y;
    std::swap(nedCord.x, nedCord.y);
    nedCord.z = - enuCord.z;

    Point ecef = nedToEcef(nedCord);
    return ecefToGeodetic(ecef);
}

Point GeodeticConverter::geodeticToCarSpace(GeoPoint geoCord) const
{
    // Translate every point that root of coordinate system is in car middle
    //Point afterTranslation = translatePointBy( { - m_carPosition.lon, - m_carPosition.lat} , {inGeoSpace.lon, inGeoSpace.lat});
    Point inVehicle = geodeticToEnu(geoCord);

    std::swap(inVehicle.x, inVehicle.y);

    inVehicle.y = - inVehicle.y;

    // Rotate by angle to point x axis into front of car
    Point afterRotation = rotatePointBy( m_centerOfConversion.azimuthReal, inVehicle);

    return afterRotation;
}

GeoPoint GeodeticConverter::carSpaceToGeodetic(Point inCarView) const
{
    Point afterRotation = rotatePointBy( - m_centerOfConversion.azimuthReal, inCarView);

    afterRotation.y = - afterRotation.y;

    std::swap(afterRotation.x, afterRotation.y);

    // Translate every point that root of coordinate system is in car middle
    GeoPoint point = enuToGeodetic(afterRotation);

    return point;
}
