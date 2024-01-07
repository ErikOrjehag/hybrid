
#include "include/rs_path.hpp"
#include <stdexcept>
#include <iostream>

namespace dyno
{

namespace rs
{

void add_path(
    std::vector<Path>& paths,
    const std::vector<double>& lengths,
    const std::vector<char>& types
    )
{
    double L = 0.0;
    for (const auto l : lengths)
    {
        L += std::abs(l);
    }

    if (L >= 1000.0)  // TODO: MAX_PATH_LENGTH
    {
        return;
    }

    if (L < 0.01)
    {
        throw std::runtime_error("L < 0.01");
    }

    for (auto& ipath : paths)
    {
        if (ipath.types == types)
        {
            double dl = 0.0;
            for (size_t i = 0; i < 3; ++i)
            {
                dl += ipath.lengths.at(i) - lengths.at(i); // TODO: should use abs?
            }
            if (dl < 0.01)
            {
                return;
            }
        }
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

    path.directions[0] = path.lengths.at(0) > 0 ? 1 : -1;
    double d = path.lengths.at(0) > 0 ? step_size : -step_size;

    double pd = d;
    double ll = 0.0;

    for (size_t i = 0; i < path.lengths.size(); ++i)
    {
        double length = path.lengths.at(i);
        char type = path.types.at(i);

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

        if (i > 0 && (path.lengths.at(i-1)*path.lengths.at(i)) > 0)
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

void shortest_path(
    double start_x,
    double start_y,
    double start_yaw,
    double end_x,
    double end_y,
    double end_yaw,
    double max_curvature,
    double step_size,
    Path& path
    )
{
    std::vector<Path> paths;
    generate_paths(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature, step_size, paths);

    double min_cost = std::numeric_limits<double>::max();
    size_t min_cost_index = 0;

    for (size_t i = 0; i < paths.size(); ++i)
    {
        double cost = paths.at(i).L;
        if (min_cost > cost)
        {
            min_cost = cost;
            min_cost_index = i;
        }
    }

    path = paths.at(min_cost_index);
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
    CSC(x, y, dyaw, paths);
    CCC(x, y, dyaw, paths);
    CCCC(x, y, dyaw, paths);
    CCSC(x, y, dyaw, paths);
    CCSCC(x, y, dyaw, paths);

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

        // TODO: why?
        path.L = path.L / max_curvature;
        for (size_t i = 0; i < path.lengths.size(); ++i)
        {
            path.lengths[i] = path.lengths[i] / max_curvature;
        }
    }
}

void SCS(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    if (SLS(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'S', 'L', 'S'});
    }

    if (SLS(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'S', 'R', 'S'});
    }
}

void CSC(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    // flag, t, u, v = LSL(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["L","S","L"])
    // end
    if (LSL(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'L', 'S', 'L'});
    }

    // flag, t, u, v = LSL(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["L","S","L"])
    // end
    if (LSL(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'L', 'S', 'L'});
    }

    // flag, t, u, v = LSL(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["R","S","R"])
    // end
    if (LSL(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'R', 'S', 'R'});
    }

    // flag, t, u, v = LSL(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["R","S","R"])
    // end
    if (LSL(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'R', 'S', 'R'});
    }

    // flag, t, u, v = LSR(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["L","S","R"])
    // end
    if (LSR(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'L', 'S', 'R'});
    }

    // flag, t, u, v = LSR(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["L","S","R"])
    // end
    if (LSR(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'L', 'S', 'R'});
    }

    // flag, t, u, v = LSR(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["R","S","L"])
    // end
    if (LSR(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'R', 'S', 'L'});
    }

    // flag, t, u, v = LSR(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["R","S","L"])
    // end
    if (LSR(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'R', 'S', 'L'});
    }

    // return paths
}

void CCC(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    // flag, t, u, v = LRL(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["L","R","L"])
    // end
    if (LRL(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'L', 'R', 'L'});
    }

    // flag, t, u, v = LRL(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["L","R","L"])
    // end
    if (LRL(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'L', 'R', 'L'});
    }

    // flag, t, u, v = LRL(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, u, v], ["R","L","R"])
    // end
    if (LRL(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, v}, {'R', 'L', 'R'});
    }

    // flag, t, u, v = LRL(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -v], ["R","L","R"])
    // end
    if (LRL(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -v}, {'R', 'L', 'R'});
    }

    // # backwards
    // xb = x*cos(phi) + y*sin(phi)
    // yb = x*sin(phi) - y*cos(phi)
    double xb = x * std::cos(yaw) + y * std::sin(yaw);
    double yb = x * std::sin(yaw) - y * std::cos(yaw);

    // flag, t, u, v = LRL(xb, yb, phi)
    // if flag
    //     paths = set_path(paths, [v, u, t], ["L","R","L"])
    // end
    if (LRL(xb, yb, yaw, t, u, v))
    {
        add_path(paths, {v, u, t}, {'L', 'R', 'L'});
    }

    // flag, t, u, v = LRL(-xb, yb, -phi)
    // if flag
    //     paths = set_path(paths, [-v, -u, -t], ["L","R","L"])
    // end
    if (LRL(-xb, yb, -yaw, t, u, v))
    {
        add_path(paths, {-v, -u, -t}, {'L', 'R', 'L'});
    }
    
    // flag, t, u, v = LRL(xb, -yb, -phi)
    // if flag
    //     paths = set_path(paths, [v, u, t], ["R","L","R"])
    // end
    if (LRL(xb, -yb, -yaw, t, u, v))
    {
        add_path(paths, {v, u, t}, {'R', 'L', 'R'});
    }

    // flag, t, u, v = LRL(-xb, -yb, phi)
    // if flag
    //     paths = set_path(paths, [-v, -u, -t], ["R","L","R"])
    // end
    if (LRL(-xb, -yb, yaw, t, u, v))
    {
        add_path(paths, {-v, -u, -t}, {'R', 'L', 'R'});
    }
}

void CCCC(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    // flag, t, u, v = LRLRn(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, u, -u, v], ["L","R","L","R"])
    // end
    if (LRLRn(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, -u, v}, {'L', 'R', 'L', 'R'});
    }

    // flag, t, u, v = LRLRn(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, u, -v], ["L","R","L","R"])
    // end
    if (LRLRn(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, -u, u, -v}, {'L', 'R', 'L', 'R'});
    }

    // flag, t, u, v = LRLRn(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, u, -u, v], ["R","L","R","L"])
    // end
    if (LRLRn(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, -u, v}, {'R', 'L', 'R', 'L'});
    }

    // flag, t, u, v = LRLRn(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, u, -v], ["R","L","R","L"])
    // end
    if (LRLRn(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, -u, u, -v}, {'R', 'L', 'R', 'L'});
    }

    // flag, t, u, v = LRLRp(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, u, u, v], ["L","R","L","R"])
    // end
    if (LRLRp(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, u, u, v}, {'L', 'R', 'L', 'R'});
    }

    // flag, t, u, v = LRLRp(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -u, -v], ["L","R","L","R"])
    // end
    if (LRLRp(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -u, -v}, {'L', 'R', 'L', 'R'});
    }

    // flag, t, u, v = LRLRp(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, u, u, v], ["R","L","R","L"])
    // end
    if (LRLRp(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, u, u, v}, {'R', 'L', 'R', 'L'});
    }

    // flag, t, u, v = LRLRp(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, -u, -u, -v], ["R","L","R","L"])
    // end
    if (LRLRp(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, -u, -u, -v}, {'R', 'L', 'R', 'L'});
    }
}

void CCSC(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    // flag, t, u, v = LRSL(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","L"])
    // end
    if (LRSL(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, v}, {'L', 'R', 'S', 'L'});
    }

    // flag, t, u, v = LRSL(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","L"])
    // end
    if (LRSL(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, -v}, {'L', 'R', 'S', 'L'});
    }

    // flag, t, u, v = LRSL(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","R"])
    // end
    if (LRSL(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, v}, {'R', 'L', 'S', 'R'});
    }

    // flag, t, u, v = LRSL(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","R"])
    // end
    if (LRSL(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, -v}, {'R', 'L', 'S', 'R'});
    }

    // flag, t, u, v = LRSR(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","R"])
    // end
    if (LRSR(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, v}, {'L', 'R', 'S', 'R'});
    }

    // flag, t, u, v = LRSR(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","R"])
    // end
    if (LRSR(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, -v}, {'L', 'R', 'S', 'R'});
    }

    // flag, t, u, v = LRSR(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","L"])
    // end
    if (LRSR(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, v}, {'R', 'L', 'S', 'L'});
    }

    // flag, t, u, v = LRSR(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","L"])
    // end
    if (LRSR(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, -v}, {'R', 'L', 'S', 'L'});
    }

    // # backwards
    // xb = x*cos(phi) + y*sin(phi)
    // yb = x*sin(phi) - y*cos(phi)
    double xb = x * std::cos(yaw) + y * std::sin(yaw);
    double yb = x * std::sin(yaw) - y * std::cos(yaw);

    // flag, t, u, v = LRSL(xb, yb, phi) 
    // if flag
    //     paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","R","L"])
    // end
    if (LRSL(xb, yb, yaw, t, u, v))
    {
        add_path(paths, {v, u, -0.5*M_PI, t}, {'L', 'S', 'R', 'L'});
    }

    // flag, t, u, v = LRSL(-xb, yb, -phi) 
    // if flag
    //     paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","R","L"])
    // end
    if (LRSL(-xb, yb, -yaw, t, u, v))
    {
        add_path(paths, {-v, -u, 0.5*M_PI, -t}, {'L', 'S', 'R', 'L'});
    }

    // flag, t, u, v = LRSL(xb, -yb, -phi) 
    // if flag
    //     paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","L","R"])
    // end
    if (LRSL(xb, -yb, -yaw, t, u, v))
    {
        add_path(paths, {v, u, -0.5*M_PI, t}, {'R', 'S', 'L', 'R'});
    }

    // flag, t, u, v = LRSL(-xb, -yb, phi) 
    // if flag
    //     paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","L","R"])
    // end
    if (LRSL(-xb, -yb, yaw, t, u, v))
    {
        add_path(paths, {-v, -u, 0.5*M_PI, -t}, {'R', 'S', 'L', 'R'});
    }

    // flag, t, u, v = LRSR(xb, yb, phi) 
    // if flag
    //     paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","R","L"])
    // end
    if (LRSR(xb, yb, yaw, t, u, v))
    {
        add_path(paths, {v, u, -0.5*M_PI, t}, {'R', 'S', 'R', 'L'});
    }

    // flag, t, u, v = LRSR(-xb, yb, -phi) 
    // if flag
    //     paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","R","L"])
    // end
    if (LRSR(-xb, yb, -yaw, t, u, v))
    {
        add_path(paths, {-v, -u, 0.5*M_PI, -t}, {'R', 'S', 'R', 'L'});
    }

    // flag, t, u, v = LRSR(xb, -yb, -phi) 
    // if flag
    //     paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","L","R"])
    // end
    if (LRSR(xb, -yb, -yaw, t, u, v))
    {
        add_path(paths, {v, u, -0.5*M_PI, t}, {'L', 'S', 'L', 'R'});
    }

    // flag, t, u, v = LRSR(-xb, -yb, phi) 
    // if flag
    //     paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","L","R"])
    // end
    if (LRSR(-xb, -yb, yaw, t, u, v))
    {
        add_path(paths, {-v, -u, 0.5*M_PI, -t}, {'L', 'S', 'L', 'R'});
    }
}

void CCSCC(double x, double y, double yaw, std::vector<Path>& paths)
{
    double t, u, v;

    // flag, t, u, v = LRSLR(x, y, phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["L","R","S","L","R"])
    // end
    if (LRSLR(x, y, yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, -0.5*M_PI, v}, {'L', 'R', 'S', 'L', 'R'});
    }

    // flag, t, u, v = LRSLR(-x, y, -phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["L","R","S","L","R"])
    // end
    if (LRSLR(-x, y, -yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, 0.5*M_PI, -v}, {'L', 'R', 'S', 'L', 'R'});
    }

    // flag, t, u, v = LRSLR(x, -y, -phi) 
    // if flag
    //     paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["R","L","S","R","L"])
    // end
    if (LRSLR(x, -y, -yaw, t, u, v))
    {
        add_path(paths, {t, -0.5*M_PI, u, -0.5*M_PI, v}, {'R', 'L', 'S', 'R', 'L'});
    }

    // flag, t, u, v = LRSLR(-x, -y, phi) 
    // if flag
    //     paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["R","L","S","R","L"])
    // end
    if (LRSLR(-x, -y, yaw, t, u, v))
    {
        add_path(paths, {-t, 0.5*M_PI, -u, 0.5*M_PI, -v}, {'R', 'L', 'S', 'R', 'L'});
    }
}

bool SLS(double x, double y, double yaw, double& t, double& u, double& v)
{
    yaw = dyno::mod2pi(yaw);

    if (y > 0.0 && yaw > 0.0 && yaw < 0.99*M_PI)
    {
        double xd = -y / std::tan(yaw) + x;
        t = xd - std::tan(yaw / 2.0);
        u = yaw;
        v = std::sqrt(std::pow(x - xd, 2) + std::pow(y, 2)) - std::tan(yaw / 2.0);
        return true;
    } else if (y < 0.0 && yaw > 0.0 && yaw < 0.99*M_PI)
    {
        double xd = -y / std::tan(yaw) + x;
        t = xd - std::tan(yaw / 2.0);
        u = yaw;
        v = -std::sqrt(std::pow(x - xd, 2) + std::pow(y, 2)) - std::tan(yaw / 2.0);
        return true;
    }

    return false;

    // phi = mod2pi(phi)
    // if y > 0.0 && phi > 0.0 && phi < pi*0.99
    //     xd = - y/tan(phi) + x
    //     t =  xd - tan(phi/2.0)
    //     u = phi
    //     v = sqrt((x-xd)^2+y^2)-tan(phi/2.0)
    //     return true, t, u, v
    // elseif y < 0.0 && phi > 0.0 && phi < pi*0.99
    //     xd = - y/tan(phi) + x
    //     t =  xd - tan(phi/2.0)
    //     u = phi
    //     v = -sqrt((x-xd)^2+y^2)-tan(phi/2.0)
    //     return true, t, u, v
    // end

    // return false, 0.0, 0.0, 0.0
}

bool LSL(double x, double y, double yaw, double& t, double& u, double& v)
{
    const auto [r, theta] = polar(x - std::sin(yaw), y - 1.0 + cos(yaw));

    u = r;
    t = theta;

    if (t >= 0.0)
    {
        v = mod2pi(yaw - t);
        if (v >= 0.0)
        {
            return true;
        }
    }

    return false;
}

bool LSR(double x, double y, double yaw, double& t, double& u, double& v)
{
    auto [r, theta] = polar(x + std::sin(yaw), y - 1.0 - std::cos(yaw));

    double r2 = std::pow(r, 2);
    
    if (r2 >= 4.0)
    {
        // TODO: inefficient sqrt?
        u = std::sqrt(r2 - 4.0);
        double th = std::atan2(2.0, u);
        t = mod2pi(theta + th);
        v = mod2pi(t - yaw);

        if (t >= 0.0 && v >= 0.0)
        {
            return true;
        }
    }

    return false;
}

bool LRL(double x, double y, double yaw, double& t, double& u, double& v)
{
    auto [r, theta] = polar(x - std::sin(yaw), y - 1.0 + std::cos(yaw));

    if (r <= 4.0)
    {
        u = -2.0 * std::asin(0.25 * r);
        t = mod2pi(theta + 0.5 * u + M_PI);
        v = mod2pi(yaw - t + u);

        if (t >= 0.0 && u <= 0.0)
        {
            return true;
        }
    }

    return false;
}

bool LRLRn(double x, double y, double yaw, double& t, double& u, double& v)
{
    double xi = x + std::sin(yaw);
    double eta = y - 1.0 - std::cos(yaw);
    double rho = 0.25 * (2.0 + std::sqrt(std::pow(xi, 2) + std::pow(eta, 2)));

    if (rho <= 1.0)
    {
        u = std::acos(rho);
        auto [tau, omega] = calc_tau_omega(u, -u, xi, eta, yaw);
        t = tau;
        v = omega;

        if (t >= 0.0 && v <= 0.0)
        {
            return true;
        }
    }

    return false;
    // xi = x + sin(phi)
    // eta = y - 1.0 - cos(phi)
    // rho = 0.25 * (2.0 + sqrt(xi*xi + eta*eta))

    // if rho <= 1.0
    //     u = acos(rho)
    //     t, v = calc_tauOmega(u, -u, xi, eta, phi);
    //     if t >= 0.0 && v <= 0.0
    //         return true, t, u, v
    //     end
    // end

    // return false, 0.0, 0.0, 0.0
}

bool LRLRp(double x, double y, double yaw, double& t, double& u, double& v)
{
    double xi = x + std::sin(yaw);
    double eta = y - 1.0 - std::cos(yaw);
    double rho = (20.0 - std::pow(xi, 2) - std::pow(eta, 2)) / 16.0;

    if (rho >= 0.0 && rho <= 1.0)
    {
        u = -std::acos(rho);
        auto [tau, omega] = calc_tau_omega(u, u, xi, eta, yaw);
        t = tau;
        v = omega;

        if (t >= 0.0 && v >= 0.0)
        {
            return true;
        }
    }

    return false;
    // xi = x + sin(phi)
    // eta = y - 1.0 - cos(phi)
    // rho = (20.0 - xi*xi - eta*eta) / 16.0;

    // if (rho>=0.0 && rho<=1.0)
    //     u = -acos(rho);
    //     if (u >= -0.5 * pi)
    //         t, v = calc_tauOmega(u, u, xi, eta, phi);
    //         if t >= 0.0 && v >= 0.0
    //             return true, t, u, v
    //         end
    //     end
    // end

    // return false, 0.0, 0.0, 0.0
}

std::tuple<double, double> calc_tau_omega(double u, double v, double xi, double eta, double phi)
{
    double delta = mod2pi(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;

    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;

    double tau = t2 < 0 ? mod2pi(t1 + M_PI) : mod2pi(t1);
    double omega = mod2pi(tau - u + v - phi);

    return {tau, omega};
    // delta = mod2pi(u-v)
    // A = sin(u) - sin(delta)
    // B = cos(u) - cos(delta) - 1.0

    // t1 = atan(eta*A - xi*B, xi*A + eta*B)
    // t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    // if t2 < 0
    //     tau = mod2pi(t1+pi)
    // else
    //     tau = mod2pi(t1)
    // end
    // omega = mod2pi(tau - u + v - phi)

    // return tau, omega
}

bool LRSL(double x, double y, double yaw, double& t, double& u, double& v)
{
    double xi = x - std::sin(yaw);
    double eta = y - 1.0 + std::cos(yaw);
    const auto [rho, theta] = polar(xi, eta);

    if (rho >= 2.0)
    {
        double r = std::sqrt(std::pow(rho, 2) - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + std::atan2(r, -2.0));
        v = mod2pi(yaw - 0.5 * M_PI - t);

        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            return true;
        }
    }

    return false;

    // xi = x - sin(phi)
    // eta = y - 1.0 + cos(phi)
    // rho, theta = polar(xi, eta)

    // if rho >= 2.0
    //     r = sqrt(rho*rho - 4.0);
    //     u = 2.0 - r;
    //     t = mod2pi(theta + atan(r, -2.0));
    //     v = mod2pi(phi - 0.5*pi - t);
    //     if t >= 0.0 && u<=0.0 && v<=0.0
    //         return true, t, u, v
    //     end
    // end

    // return false, 0.0, 0.0, 0.0
}

bool LRSR(double x, double y, double yaw, double& t, double& u, double& v)
{
    double xi = x + std::sin(yaw);
    double eta = y - 1.0 - std::cos(yaw);
    const auto [rho, theta] = polar(-eta, xi);

    if (rho >= 2.0)
    {
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5 * M_PI - yaw);
        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            return true;
        }
    }

    return false;
    // xi = x + sin(phi)
    // eta = y - 1.0 - cos(phi)
    // rho, theta = polar(-eta, xi)

    // if rho >= 2.0
    //     t = theta
    //     u = 2.0 - rho
    //     v = mod2pi(t + 0.5*pi - phi)
    //     if t >= 0.0 && u <= 0.0 && v <=0.0
    //         return true, t, u, v
    //     end
    // end

    // return false, 0.0, 0.0, 0.0
}

bool LRSLR(double x, double y, double yaw, double& t, double& u, double& v)
{
    double xi = x + std::sin(yaw);
    double eta = y - 1.0 - std::cos(yaw);
    const auto [rho, theta] = polar(xi, eta);

    if (rho >= 2.0)
    {
        u = 4.0 - std::sqrt(std::pow(rho, 2) - 4.0);

        if (u <= 0.0)
        {
            t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = mod2pi(t - yaw);

            if (t >= 0.0 && v >= 0.0)
            {
                return true;
            }
        }
    }

    return false;
    // # formula 8.11 *** TYPO IN PAPER ***
    // xi = x + sin(phi)
    // eta = y - 1.0 - cos(phi)
    // rho, theta = polar(xi, eta)
    // if rho >= 2.0
    //     u = 4.0 - sqrt(rho*rho - 4.0)
    //     if u <= 0.0
    //         t = mod2pi(atan((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta));
    //         v = mod2pi(t - phi);

    //         if t >= 0.0 && v >=0.0
    //             return true, t, u, v
    //         end
    //     end
    // end

    // return false, 0.0, 0.0, 0.0
}

} // namespace rs

} // namespace dyno
