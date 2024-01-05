
#include "include/rs_path.hpp"
#include <stdexcept>

namespace dyno
{

namespace rs
{

void add_path(
    std::vector<Path>& paths,
    const std::tuple<double, double, double>& lengths,
    const std::tuple<char, char, char>& types
    )
{
    const auto [t, u, v] = lengths;
    const auto L = std::abs(t) + std::abs(u) + std::abs(v);

    if (L >= 1000.0)  // TODO: MAX_PATH_LENGTH
    {
        return;
    }

    for (auto& ipath : paths)
    {
        const auto [it, iu, iv] = ipath.lengths;
        if (
            ipath.lengths == lengths &&
            ((it - t) + (iu - u) + (iv - v) < 0.01)    // TODO: should use abs?
        )
        {
            return;
        }
    }

    if (L < 0.01)
    {
        throw std::runtime_error("L < 0.01");
    }

    paths.emplace_back(Path{
        lengths,
        types,
        L,
        {},
        {},
        {},
        {}
    });
}

void populate_local_path_course(
    Path& path,
    double start_x,
    double start_y,
    double start_yaw,
    double max_curvature,
    double step_size
    )
{
    size_t n = path.L / step_size + 3 + 3; // Why + length(lengths) + 3 ?
    
    path.x.resize(n);
    path.y.resize(n);
    path.yaw.resize(n);
    path.directions.resize(n);

    size_t ind = 1;

    path.directions[0] = std::get<0>(path.lengths) > 0 ? 1 : -1;
    double d = std::get<0>(path.lengths) > 0 ? step_size : -step_size;

    double pd = d;
    double ll = 0.0;

    std::vector types = {std::get<0>(path.types), std::get<1>(path.types), std::get<2>(path.types)};
    std::vector lengths = {std::get<0>(path.lengths), std::get<1>(path.lengths), std::get<2>(path.lengths)};

    for (size_t i = 0; i < 3; ++i)
    {
        char type = types[i];
        double length = lengths[i];

        if (length > 0.0)
        {
            d = step_size;
        }
        else
        {
            d = -step_size;
        }

        double ox = path.x.at(ind);
        double oy = path.y.at(ind);
        double oyaw = path.yaw.at(ind);

        ind -= 1;

        if (i > 0 && (lengths[i-1]*lengths[i]) > 0)
        {
            pd = - d - ll;
        }
        else
        {
            pd = d - ll;
        }

        while (std::abs(pd) <= std::abs(length))
        {
            ind += 1;
            interpolate(ind, pd, type, max_curvature, ox, oy, oyaw, path.x, path.y, path.yaw, path.directions);
            pd += d;
        }

        ll = length - pd - d;  // calc remain length

        ind += 1;
        interpolate(ind, length, type, max_curvature, ox, oy, oyaw, path.x, path.y, path.yaw, path.directions);
    }

    while (path.x.at(path.x.size() - 1) == 0.0)
    {
        path.x.pop_back();
        path.y.pop_back();
        path.yaw.pop_back();
        path.directions.pop_back();
    }
}

void interpolate(
    size_t ind,
    double l,
    double type,
    double max_curvature,
    double ox,
    double oy,
    double oyaw,
    std::vector<double>& x,
    std::vector<double>& y,
    std::vector<double>& yaw,
    std::vector<int>& directions
    )
{

    // TODO: cos and sin can be cached?

    if (type == 'S')
    {
        x[ind] = ox + l / max_curvature * std::cos(oyaw);
        y[ind] = oy + l / max_curvature * std::sin(oyaw);
        yaw[ind] = oyaw;
    }
    else
    {
        double ldx = std::sin(l) / max_curvature;
        double ldy = 0.0;

        if (type == 'L')
        {
            ldy = (1.0 - std::cos(l)) / max_curvature;
        }
        else if (type == 'R')
        {
            ldy = (1.0 - std::cos(l)) / -max_curvature;
        }

        double gdx = std::cos(-oyaw) * ldx + std::sin(-oyaw) * ldy;
        double gdy = -std::sin(-oyaw) * ldx + std::cos(-oyaw) * ldy;

        x[ind] = ox + gdx;
        y[ind] = oy + gdy;
    }

    if (type == 'L')
    {
        yaw[ind] = oyaw + l;
    }
    else if (type == 'R')
    {
        yaw[ind] = oyaw - l;
    }

    if (l > 0.0)
    {
        directions[ind] = 1;
    }
    else
    {
        directions[ind] = -1;
    }
}

void generate_paths(
    double start_x,
    double start_y,
    double start_yaw,
    double end_x,
    double end_y,
    double end_yaw,
    double max_curvature,
    double step_size,
    std::vector<Path>& paths
    )
{
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double dyaw = end_yaw - start_yaw; // TODO: is this OK?

    double c = std::cos(start_yaw);
    double s = std::sin(start_yaw);

    double x = (c * dx + s * dy) * max_curvature;
    double y = (-s * dx + c * dy) * max_curvature;

    SCS(x, y, dyaw, paths);

    // paths = CSC(x, y, dth, paths)
    // paths = CCC(x, y, dth, paths)
    // paths = CCCC(x, y, dth, paths)
    // paths = CCSC(x, y, dth, paths)
    // paths = CCSCC(x, y, dth, paths)

    c = std::cos(-start_yaw);
    s = std::sin(-start_yaw);
    for (auto& path : paths)
    {
        populate_local_path_course(path, start_x, start_y, start_yaw, max_curvature, step_size * max_curvature); // TODO: why multiply here?
        
        // convert to global space
        for (size_t i = 0; i < path.x.size(); ++i)
        {
            const auto ix = path.x[i];
            const auto iy = path.y[i];
            const auto iyaw = path.yaw[i];

            path.x[i] = c * ix + s * iy + start_x;
            path.y[i] = -s * ix + c * iy + start_y;
            path.yaw[i] = pi2pi(iyaw + start_yaw);
        }

        // why?
        path.L = path.L / max_curvature;
        path.lengths = {
            std::get<0>(path.lengths) / max_curvature,
            std::get<1>(path.lengths) / max_curvature,
            std::get<2>(path.lengths) / max_curvature
        };
    }
}

void SCS(double x, double y, double yaw, std::vector<Path>& paths)
{
    const auto [ok0, lengths0] = SLS(x, y, yaw);

    if (ok0)
    {
        add_path(paths, lengths0, {'S', 'L', 'S'});
    }
    
    const auto [ok1, lengths1] = SLS(x, -y, -yaw);

    if (ok1)
    {
        add_path(paths, lengths1, {'S', 'R', 'S'});
    }
}

std::tuple<bool, std::tuple<double, double, double>> SLS(double x, double y, double yaw)
{
    yaw = dyno::mod2pi(yaw);
    
    if (y != 0.0 && yaw > 0.0 && yaw < 0.99*M_PI)
    {
        double xd = -y / std::tan(yaw) + x;
        double t = xd - std::tan(yaw / 2.0);
        double u = yaw;
        double v = std::sqrt(std::pow(x - xd, 2) + std::pow(y, 2)) - std::tan(yaw / 2.0);
        
        if (v < 0.0)
        {
            v *= -1;
        }

        return {true, {t, u, v}};
    }

    return {false, {0.0, 0.0, 0.0}};
}

} // namespace rs

} // namespace dyno
