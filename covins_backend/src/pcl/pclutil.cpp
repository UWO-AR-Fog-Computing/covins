#include "covins/pcl/pclutil.hpp"

#include <string>

#include <tiledb/tiledb>

namespace pcl {

void CreateMap(std::string map_id) {
    tiledb::Context ctx;

    if (tiledb::Object::object(ctx, GetMapName(map_id)).type() == tiledb::Object::Type::Array)
        return;

    double max_dim = 1.7976931348623157e+308;
    double min_dim = -1.7976931348623157e+308;
    double dim_domain[] = {min_dim, max_dim};

    // int tile_extent = 100;

    auto dimX = tiledb::Dimension::create(ctx, "x", TILEDB_FLOAT64, dim_domain, nullptr);
    auto dimY = tiledb::Dimension::create(ctx, "y", TILEDB_FLOAT64, dim_domain, nullptr);
    auto dimZ = tiledb::Dimension::create(ctx, "z", TILEDB_FLOAT64, dim_domain, nullptr);

    tiledb::Domain domain(ctx);
    domain.add_dimension(dimX);
    domain.add_dimension(dimY);
    domain.add_dimension(dimZ);

    auto value_attr = tiledb::Attribute::create<int32_t>(ctx, "v");
    int32_t fill_value = 0;
    uint64_t value_size = sizeof(fill_value);
    value_attr.set_fill_value(&fill_value, value_size);

    auto id_attr = tiledb::Attribute::create<size_t>(ctx, "id");
    size_t fill_id = 0;
    uint64_t id_size = sizeof(size_t);
    id_attr.set_fill_value(&fill_id, id_size);

    tiledb::ArraySchema schema(ctx, TILEDB_SPARSE);
    schema.set_domain(domain);
    schema.add_attribute(value_attr);
    schema.add_attribute(id_attr);
    schema.set_tile_order(TILEDB_ROW_MAJOR);
    schema.set_cell_order(TILEDB_ROW_MAJOR);
    schema.set_capacity(1000000);
    schema.set_allows_dups(true);

    // tiledb::Filter zstd_filter(ctx, TILEDB_FILTER_ZSTD);

    // tiledb::FilterList filter_list(ctx);
    // filter_list.add_filter(zstd_filter);

    // schema.set_coords_filter_list(filter_list);

    tiledb::Array::create(GetMapName(map_id), schema);
}

void InsertMapPoint(double px, double py, double pz, size_t id, std::string map_id) {
    tiledb::Context ctx;

    std::vector<double> vpx = {px};
    std::vector<double> vpy = {py};
    std::vector<double> vpz = {pz};
    std::vector<int32_t> v = {1};
    std::vector<size_t> vid = {id};

    tiledb::Array map(ctx, GetMapName(map_id), TILEDB_WRITE);

    tiledb::Query query(ctx, map);
    query.set_layout(TILEDB_UNORDERED);
    query.set_buffer("x", vpx);
    query.set_buffer("y", vpy);
    query.set_buffer("z", vpz);
    query.set_buffer("v", v);
    query.set_buffer("id", vid);

    query.submit();

    map.close();
}

void DeleteMapPoint(size_t id, std::string map_id) {
    tiledb::Context ctx;

    tiledb::Array map(ctx, GetMapName(map_id), TILEDB_DELETE);

    tiledb::Query delete_query(ctx, map, TILEDB_DELETE);
    tiledb::QueryCondition qc(ctx);

    qc.init("id",&id, sizeof(size_t), TILEDB_EQ);
    delete_query.set_condition(qc);
    delete_query.submit();

    map.close();
}

void GetMapPoint(std::string map_id) {
    tiledb::Context ctx;

    tiledb::Array map(ctx, GetMapName(map_id), TILEDB_READ);

    tiledb::Subarray subarray(ctx, map);

    auto non_empty_domain = map.non_empty_domain<double>();
    std::cout << "Non-empty domain: ";
    std::cout << "[" << non_empty_domain[0].second.first << ","
            << non_empty_domain[0].second.second << "], ["
            << non_empty_domain[1].second.first << ","
            << non_empty_domain[1].second.second << "], ["
            << non_empty_domain[2].second.first << ","
            << non_empty_domain[2].second.second << "]"
            << non_empty_domain[3].second.first << ","
            << non_empty_domain[3].second.second << "]\n";

    subarray.add_range(0, -100.0, 100.0).add_range(1, -100.0, 100.0).add_range(2, -100.0, 100.0);

    std::vector<double> vx(100);
    std::vector<double> vy(100);
    std::vector<double> vz(100);
    std::vector<double> vv(100);
    std::vector<size_t> vid(100);

    tiledb::Query query(ctx, map, TILEDB_READ);
    query.set_subarray(subarray);
    query.set_layout(TILEDB_ROW_MAJOR);
    query.set_data_buffer("x", vx);
    query.set_data_buffer("y", vy);
    query.set_data_buffer("z", vz);
    query.set_data_buffer("v", vv);
    query.set_data_buffer("id", vid);


    query.submit();

    map.close();

    auto result_num = (int) query.result_buffer_elements()["x"].second;
    std::cout << "Result num: " << result_num << "\n";

    for (int r = 0; r < result_num; r++) {
        double x = vx[r];
        double y = vy[r];
        double z = vz[r];
        double v = vv[r];
        size_t id = vid[r];

        std::cout << "Cell (" << x << ", " << y << ", " << z << ") has data " << v << " " << id << "\n";
    }
}

void GetMapSchema(std::string map_id) {
    tiledb::Context ctx;

    tiledb::ArraySchema schema(ctx, GetMapName(map_id));

    tiledb::Domain domain = schema.domain();
    tiledb::Attribute value_attr = schema.attribute("v");
    tiledb::Attribute id_attr = schema.attribute("id");

    domain.dump(stdout);
    value_attr.dump(stdout);
    id_attr.dump(stdout);
}

std::string GetMapName(std::string map_id) {
    return MAP_PATH + map_id;
}


}

