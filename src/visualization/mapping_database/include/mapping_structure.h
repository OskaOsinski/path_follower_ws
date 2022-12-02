#ifndef MAPPING_STRUCTURE_H
#define MAPPING_STRUCTURE_H

#include <memory>
#include "sqlite_orm/sqlite_orm.h"
#include "commonstruct.h"

namespace DataModel
{

struct LineDetection
{
    int id;
    int type;
    int color;
    double lon;
    double lat;
    double alt;
    double az;
    double a;
    double b;
    double c;
    double d;

    inline double calc(double x) const
    {
        return a*x*x*x + b*x*x + c*x + d;
    }

    operator CarPosition() const
    //CarPosition getPosition() const
    {
        CarPosition pos;
        pos.point.lon = lon;
        pos.point.lat = lat;
        pos.point.h = alt;
        pos.azimuthReal = az;
        return pos;
    }
};

struct Line
{
    int id;
    int type;
    int color;
};


struct LinePoint
{
    int id;
    std::shared_ptr<int>  lineId;
    double lon;
    double lat;
    double alt;

    operator GeoPoint() const
    {
        GeoPoint p;
        p.lon = lon;
        p.lat = lat;
        p.h = alt;
        return p;
    }
};

}

static auto openAndPrepareDatabase(const std::string &name)
{
    using namespace DataModel;
    namespace orm = sqlite_orm;

    auto storage = orm::make_storage(name,
                                     orm::make_table("LineDetection",
                                                orm::make_column("id",
                                                            &LineDetection::id,
                                                            orm::primary_key()),
                                                orm::make_column("type",
                                                            &LineDetection::type),
                                                orm::make_column("color",
                                                            &LineDetection::color),
                                                 orm::make_column("lon",
                                                             &LineDetection::lon),
                                                 orm::make_column("lat",
                                                             &LineDetection::lat),
                                                 orm::make_column("alt",
                                                             &LineDetection::alt),
                                                 orm::make_column("az",
                                                             &LineDetection::az),
                                                 orm::make_column("a",
                                                             &LineDetection::a),
                                                 orm::make_column("b",
                                                             &LineDetection::b),
                                                 orm::make_column("c",
                                                             &LineDetection::c),
                                                 orm::make_column("d",
                                                             &LineDetection::d)),
                                     orm::make_table("Line",
                                                orm::make_column("id",
                                                            &Line::id,
                                                            orm::primary_key()),
                                                orm::make_column("type",
                                                            &Line::type),
                                                orm::make_column("color",
                                                            &Line::color)),
                                    orm::make_table("LinePoints",
                                               orm::make_column("id",
                                                           &LinePoint::id,
                                                           orm::primary_key()),
                                               orm::make_column("lineId",
                                                           &LinePoint::lineId),
                                               orm::make_column("lon",
                                                           &LinePoint::lon),
                                               orm::make_column("lat",
                                                           &LinePoint::lat),
                                               orm::make_column("alt",
                                                           &LinePoint::alt),
                                               orm::foreign_key(&LinePoint::lineId).references(&Line::id)));
    storage.sync_schema();

    return storage;
}


#endif // MAPPING_STRUCTURE_H
