// Hybrid A* + Snap-to-Constraints Smoother (C++17 + Eigen)
// ------------------------------------------------------------------
// This file adds a compact Hybrid A* search and runs the previously
// implemented snapper smoother as a post-processing step.
//
// Pipeline:
//   1) Build ESDF grid (clearance map)
//   2) Run Hybrid A* on a SE(2) lattice (x,y with heading bins) using
//      forward motion primitives with ±kappa_max, 0 curvature.
//   3) Convert the resulting lattice path to a Catmull–Rom guide.
//   4) Run the snap-to-constraints optimizer with endpoint taper &
//      curvature regularization to get a smooth, feasible path.
//   5) Write outputs: snapped.csv, guide.csv, esdf.csv, meta.json
//
// Notes:
//   - Start & goal pose are enforced as HARD constraints in the smoother.
//   - Collision checking during search uses ESDF > robot_radius along each
//     primitive (discrete samples).
//
// Dependencies: Eigen (header-only)
//   Ubuntu/Debian: sudo apt-get install libeigen3-dev
// Build:
//   g++ -O2 -std=c++17 snap_to_constraints.cpp -I /usr/include/eigen3 -o plan
// Run:
//   ./plan > snapped.csv
// ------------------------------------------------------------------

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <cassert>

using std::vector; using std::size_t;
using Eigen::MatrixXd; using Eigen::VectorXd; using Eigen::Vector2d;

// --------------------- Utility ---------------------
static inline double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }
static inline double wrapAngle(double a){ while(a >  M_PI) a -= 2*M_PI; while(a < -M_PI) a += 2*M_PI; return a; }
static inline double smoothstep01(double x) { x = clamp(x, 0.0, 1.0); return x*x*(3.0 - 2.0*x); }
static inline double ramp_up(double s, double taper)   { if(taper <= 1e-9) return 1.0; return smoothstep01(s / taper); }
static inline double ramp_down(double s, double taper) { if(taper <= 1e-9) return 1.0; return smoothstep01(s / taper); }

struct Pose { double x,y,psi; };

// --------------------- ESDF ---------------------
struct DistanceField2D { virtual double distGrad(double x, double y, Vector2d& grad) const = 0; virtual ~DistanceField2D() = default; };
struct ESDFGrid : public DistanceField2D {
    int W,H; double res; VectorXd d; // row-major distances
    ESDFGrid(int W_, int H_, double res_) : W(W_), H(H_), res(res_), d(W_*H_) { d.setConstant(10.0); }
    inline int idx(int ix, int iy) const { return iy*W + ix; }
    void setCircleObstacle(double cx, double cy, double radius){
        for(int iy=0; iy<H; ++iy){ for(int ix=0; ix<W; ++ix){ double x=(ix+0.5)*res, y=(iy+0.5)*res; d[idx(ix,iy)] = std::sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy)) - radius; }} }
    double distGrad(double x, double y, Vector2d& grad) const override {
        double gx = x / res - 0.5, gy = y / res - 0.5; int ix = (int)std::floor(gx), iy=(int)std::floor(gy);
        if(ix<0 || iy<0 || ix+1>=W || iy+1>=H){ grad.setZero(); return 10.0; }
        double tx=gx-ix, ty=gy-iy; auto val=[&](int i,int j){ return d[idx(i,j)]; };
        double v00=val(ix,iy), v10=val(ix+1,iy), v01=val(ix,iy+1), v11=val(ix+1,iy+1);
        double v0=(1-tx)*v00 + tx*v10; double v1=(1-tx)*v01 + tx*v11; double v=(1-ty)*v0 + ty*v1;
        double dvx=((1-ty)*(v10-v00) + ty*(v11-v01)) / res; double dvy=((1-tx)*(v01-v00) + tx*(v11-v10)) / res;
        grad = Vector2d(dvx,dvy); return v;
    }
};

// --------------------- Catmull–Rom Guide ---------------------
struct GuidePath {
    vector<Vector2d> ctrl; // control points
    void bounds(double &umin, double &umax) const { umin = 1.0; umax = std::max(1.0, (double)ctrl.size()-3.0); }
    void eval(double u, Vector2d &pos, Vector2d &tangent) const {
        int i = (int)std::floor(u); double t = u - i; i = std::max(1, std::min((int)ctrl.size()-3, i));
        const Vector2d &P0=ctrl[i-1], &P1=ctrl[i], &P2=ctrl[i+1], &P3=ctrl[i+2];
        double t2=t*t, t3=t2*t;
        pos = 0.5*((2.0*P1) + (-P0+P2)*t + (2.0*P0-5.0*P1+4.0*P2-P3)*t2 + (-P0+3.0*P1-3.0*P2+P3)*t3);
        tangent = 0.5*((-P0+P2) + 2.0*(2.0*P0-5.0*P1+4.0*P2-P3)*t + 3.0*(-P0+3.0*P1-3.0*P2+P3)*t2);
    }
    vector<double> u_samples; vector<double> s_accum; // arc-length LUT
    void buildArcTable(int samples=1000){
        u_samples.clear(); s_accum.clear(); double umin,umax; bounds(umin,umax); int S=std::max(samples,10);
        Vector2d p_prev,t_prev; eval(umin,p_prev,t_prev); double acc=0.0;
        for(int k=0;k<=S;++k){ double u=umin + (umax-umin)*(double)k/S; Vector2d p,t; eval(u,p,t);
            if(k>0) acc += (p - p_prev).norm(); u_samples.push_back(u); s_accum.push_back(acc); p_prev=p; }
    }
    double totalLength() const { return s_accum.empty()?0.0:s_accum.back(); }
    double uFromS(double s) const { s = clamp(s,0.0,totalLength()); int lo=0, hi=(int)s_accum.size()-1; while(hi-lo>1){int mid=(lo+hi)/2; if(s_accum[mid]<s) lo=mid; else hi=mid;} double t=(s-s_accum[lo])/std::max(1e-9,s_accum[hi]-s_accum[lo]); return u_samples[lo]*(1-t)+u_samples[hi]*t; }
    double nearestU(const Vector2d& q) const { double best_u=u_samples.front(), best_d=1e9; for(size_t i=0;i<u_samples.size();++i){ Vector2d p,t; eval(u_samples[i],p,t); double d=(p-q).squaredNorm(); if(d<best_d){best_d=d; best_u=u_samples[i];}} return best_u; }
};

// --------------------- Snapper Params & State ---------------------
struct SnapParams {
    int    N        = 150;   double h = 0.06;  double kappa_max=0.35;
    double w_d=20.0, w_psi=5.0, w_kin=50.0;   // adherence & kinematics
    double sigma_w=10.0, w_ddkappa=2.0, w_kappa=0.1, w_dpsi=0.2; // curvature/heading smoothness
    double endpoint_taper=1.0; // meters
    double w_clr=20.0, d_safe=0.4; // clearance
    int max_iter=30;
};
struct SnapState { vector<double> x,y,psi,kappa; };

// --------------------- Snapper Solver ---------------------
struct SnapSolver {
    const GuidePath& guide; const DistanceField2D& df; const SnapParams p; Pose start_fix{0,0,0}, goal_fix{0,0,0}; bool have_goal=false;
    SnapSolver(const GuidePath& g, const DistanceField2D& d, const SnapParams& sp):guide(g),df(d),p(sp){}
    SnapState initialize(const Pose& start, const Pose* goalOpt) const {
        double u0 = guide.nearestU(Vector2d(start.x,start.y)); int idx=0; double best=1e9; for(size_t i=0;i<guide.u_samples.size();++i){ double du=std::abs(guide.u_samples[i]-u0); if(du<best){best=du; idx=(int)i;}} double s0=guide.s_accum[idx];
        double s_goal = s0 + p.h*(p.N); if(goalOpt){ double ug=guide.nearestU(Vector2d(goalOpt->x,goalOpt->y)); int idxg=idx; double bestg=1e9; for(size_t i=0;i<guide.u_samples.size();++i){ double du=std::abs(guide.u_samples[i]-ug); if(du<bestg){bestg=du; idxg=(int)i;}} s_goal=guide.s_accum[idxg]; if(s_goal<=s0) s_goal=s0+p.h*(p.N);} 
        SnapState st; st.x.resize(p.N+1); st.y.resize(p.N+1); st.psi.resize(p.N+1); st.kappa.resize(p.N+1);
        st.x[0]=start.x; st.y[0]=start.y; st.psi[0]=start.psi; st.kappa[0]=0.0;
        for(int k=1;k<=p.N;++k){ double s=s0 + (s_goal-s0)*(double)k/(double)p.N; Vector2d gp,gt; guide.eval(guide.uFromS(s),gp,gt); double psi_g=std::atan2(gt.y(),gt.x()); st.x[k]=gp.x(); st.y[k]=gp.y(); st.psi[k]=psi_g; st.kappa[k-1]=0.0; }
        if(goalOpt){ st.x[p.N]=goalOpt->x; st.y[p.N]=goalOpt->y; st.psi[p.N]=goalOpt->psi; }
        st.kappa[p.N]=0.0; return st; }

    double assemble(const SnapState& st, VectorXd& r, MatrixXd& J) const {
        const int V=4*(p.N+1);
        int M= 3*p.N + 2*(p.N+1) + p.N + (p.N+1) + (p.N+1) + (p.N>0? (p.N-1) : 0) + p.N;
        r.setZero(M); J.setZero(M,V); int row=0; auto vidx=[&](int k){return 4*k;};
        // 1) Kinematics
        for(int k=0;k<p.N;++k){ int i0=vidx(k), i1=vidx(k+1); double x0=st.x[k], y0=st.y[k], psi0=st.psi[k]; double x1=st.x[k+1], y1=st.y[k+1], psi1=st.psi[k+1]; double h=p.h;
            r[row] = x1 - x0 - h*std::cos(psi0); J(row,i0+0)=-1; J(row,i1+0)=1; J(row,i0+2)=h*std::sin(psi0); r[row]*=std::sqrt(p.w_kin); J.block(row,0,1,V)*=std::sqrt(p.w_kin); row++;
            r[row] = y1 - y0 - h*std::sin(psi0); J(row,i0+1)=-1; J(row,i1+1)=1; J(row,i0+2)=-h*std::cos(psi0); r[row]*=std::sqrt(p.w_kin); J.block(row,0,1,V)*=std::sqrt(p.w_kin); row++;
            r[row] = wrapAngle(psi1 - psi0 - h*st.kappa[k]); J(row,i0+2)=-1; J(row,i1+2)=1; J(row,i0+3)=-h; r[row]*=std::sqrt(p.w_kin); J.block(row,0,1,V)*=std::sqrt(p.w_kin); row++; }
        // 2) Guide (with endpoint taper)
        for (int k = 0; k <= p.N; ++k) {
            int i = vidx(k); double s_k = k * p.h; double s_to_end = (p.N - k) * p.h;
            double edge = ramp_up(s_k, p.endpoint_taper) * ramp_down(s_to_end, p.endpoint_taper);
            double w_dk   = std::sqrt(p.w_d   * edge);
            double w_psik = std::sqrt(p.w_psi * edge);
            Vector2d q(st.x[k], st.y[k]); double u = guide.nearestU(q); Vector2d gp, gt; guide.eval(u, gp, gt);
            double psi_g = std::atan2(gt.y(), gt.x()); Vector2d t = gt.normalized(); Vector2d n(-t.y(), t.x());
            double e_lat = n.dot(q - gp);
            r[row] = e_lat; J(row, i+0) = n.x(); J(row, i+1) = n.y(); r[row] *= w_dk; J.block(row,0,1,V)*=w_dk; row++;
            double epsi = wrapAngle(st.psi[k] - psi_g);
            r[row] = epsi;  J(row, i+2) = 1.0; r[row] *= w_psik; J.block(row,0,1,V)*=w_psik; row++;
        }
        // 3) Δkappa
        for(int k=0;k<p.N;++k){ int i0=vidx(k), i1=vidx(k+1); r[row]=st.kappa[k+1]-st.kappa[k]; J(row,i0+3)=-1; J(row,i1+3)=1; double w=std::sqrt(p.sigma_w); r[row]*=w; J.block(row,0,1,V)*=w; row++; }
        // 3b) Δ²kappa
        for (int k = 1; k < p.N; ++k) { int im1=vidx(k-1), i0=vidx(k), ip1=vidx(k+1); double d2k = st.kappa[k+1]-2.0*st.kappa[k]+st.kappa[k-1]; r[row]=d2k; J(row,im1+3)=-1; J(row,i0+3)=2; J(row,ip1+3)=-1; double w=std::sqrt(p.w_ddkappa); r[row]*=w; J.block(row,0,1,V)*=w; row++; }
        // 4) |kappa|
        for(int k=0;k<=p.N;++k){ int i=vidx(k); r[row]=st.kappa[k]; J(row,i+3)=1.0; double w=std::sqrt(p.w_kappa); r[row]*=w; J.block(row,0,1,V)*=w; row++; }
        // 2b) Δpsi
        for (int k = 0; k < p.N; ++k) { int i0=vidx(k), i1=vidx(k+1); double dpsi = wrapAngle(st.psi[k+1]-st.psi[k]); r[row]=dpsi; J(row,i0+2)=-1; J(row,i1+2)=1; double w=std::sqrt(p.w_dpsi); r[row]*=w; J.block(row,0,1,V)*=w; row++; }
        // 5) Clearance
        for(int k=0;k<=p.N;++k){ int i=vidx(k); Vector2d grad; double d=df.distGrad(st.x[k],st.y[k],grad); double c=p.d_safe - d; if(c>0){ r[row]=c; J(row,i+0)=-grad.x(); J(row,i+1)=-grad.y(); double w=std::sqrt(p.w_clr); r[row]*=w; J.block(row,0,1,V)*=w; } else { r[row]=0.0; } row++; }
        return r.squaredNorm(); }

    void projectBounds(SnapState& st) const { for(int k=0;k<=p.N;++k){ st.kappa[k]=clamp(st.kappa[k], -p.kappa_max, p.kappa_max);} }

    double lineSearch(const VectorXd& dx, SnapState& st, const VectorXd& x0, double f0){
        double alpha=1.0; auto apply = [&](double a){ SnapState s=st; for(int k=0;k<=p.N;++k){ int o=4*k; s.x[k]=x0[o+0]+a*dx[o+0]; s.y[k]=x0[o+1]+a*dx[o+1]; s.psi[k]=wrapAngle(x0[o+2]+a*dx[o+2]); s.kappa[k]=x0[o+3]+a*dx[o+3]; }
            s.x[0]=start_fix.x; s.y[0]=start_fix.y; s.psi[0]=start_fix.psi; if(have_goal){ s.x[p.N]=goal_fix.x; s.y[p.N]=goal_fix.y; s.psi[p.N]=goal_fix.psi; }
            projectBounds(s); VectorXd r; MatrixXd J; r.resize(3*p.N + 2*(p.N+1) + p.N + (p.N+1) + (p.N+1) + (p.N>0? (p.N-1) : 0) + p.N); J.resize(r.size(), 4*(p.N+1)); double f=assemble(s,r,J); return std::make_pair(f,s); };
        double c1=1e-4; VectorXd r0; MatrixXd J0; r0.resize(3*p.N + 2*(p.N+1) + p.N + (p.N+1) + (p.N+1) + (p.N>0? (p.N-1) : 0) + p.N); J0.resize(r0.size(), 4*(p.N+1)); assemble(st,r0,J0); double g0=(J0.transpose()*r0).dot(dx);
        SnapState bestS=st; double bestF=f0; for(int it=0; it<12; ++it){ auto [f,s]=apply(alpha); if(f<=f0 + c1*alpha*g0){ st=s; return f; } if(f<bestF){ bestF=f; bestS=s;} alpha*=0.5;} st=bestS; return bestF; }

    SnapState solve(const Pose& start, const Pose* goalOpt=nullptr){
        start_fix=start; have_goal=(goalOpt!=nullptr); if(have_goal) goal_fix=*goalOpt; SnapState st=initialize(start,goalOpt); projectBounds(st);
        for(int iter=0; iter<p.max_iter; ++iter){ VectorXd r; MatrixXd J; r.resize(3*p.N + 2*(p.N+1) + p.N + (p.N+1) + (p.N+1) + (p.N>0? (p.N-1) : 0) + p.N); J.resize(r.size(), 4*(p.N+1)); double f=assemble(st,r,J); MatrixXd H=J.transpose()*J; VectorXd g=J.transpose()*r; H.diagonal().array() += 1e-6; VectorXd dx = H.ldlt().solve(-g);
            int o0=0; dx[o0+0]=0; dx[o0+1]=0; dx[o0+2]=0; if(have_goal){ int oN=4*p.N; dx[oN+0]=0; dx[oN+1]=0; dx[oN+2]=0; }
            VectorXd x0(4*(p.N+1)); for(int k=0;k<=p.N;++k){ int o=4*k; x0[o+0]=st.x[k]; x0[o+1]=st.y[k]; x0[o+2]=st.psi[k]; x0[o+3]=st.kappa[k]; }
            double fnew=lineSearch(dx,st,x0,f); double step=dx.norm(); std::cout<<"iter "<<iter<<" f="<<fnew<<" step="<<step<<"\n"; if(step<1e-4) break; }
        st.x[0]=start_fix.x; st.y[0]=start_fix.y; st.psi[0]=start_fix.psi; if(have_goal){ st.x[p.N]=goal_fix.x; st.y[p.N]=goal_fix.y; st.psi[p.N]=goal_fix.psi; }
        return st; }
};

// --------------------- Hybrid A* (compact) ---------------------
struct HybridAstarParams {
    double ds = 0.2;          // step length per primitive (m)
    int    N_theta = 72;      // heading bins
    double kappa_max = 0.35;  // max curvature [1/m]
    double robot_radius = 0.3;// collision radius (m)
    int    check_subsamples = 5; // collision subsamples along a primitive
    double goal_xy_tol = 0.25;   // goal tolerance (m)
    double goal_psi_tol = 15.0*M_PI/180.0; // rad
};

struct HAState { double x,y,psi; int ix,iy,it; double g,f; int parent=-1; int steer_idx=0; };

struct HybridAstar {
    const ESDFGrid& esdf; HybridAstarParams p;
    HybridAstar(const ESDFGrid& e, const HybridAstarParams& hp): esdf(e), p(hp) {}

    inline int thetaBin(double psi) const { double t = (psi + M_PI) / (2*M_PI); int b = (int)std::floor(t * p.N_theta); if(b<0) b=0; if(b>=p.N_theta) b=p.N_theta-1; return b; }

    bool freeAlong(const Pose& from, double kappa, double ds) const {
        // Sample along primitive and require ESDF > robot_radius
        int S = std::max(2, p.check_subsamples);
        double h = ds / S, x=from.x, y=from.y, psi=from.psi;
        for(int s=0;s<S;++s){
            x += h*std::cos(psi); y += h*std::sin(psi); psi = wrapAngle(psi + h*kappa);
            Vector2d grad; double d = esdf.distGrad(x,y,grad);
            if(d < p.robot_radius) return false;
        }
        return true;
    }

    double heuristic(const Pose& a, const Pose& g) const {
        double dx=g.x-a.x, dy=g.y-a.y; double d=std::sqrt(dx*dx+dy*dy);
        double dh = std::abs(wrapAngle(g.psi - a.psi));
        return d + 0.25 * dh / std::max(1e-6, p.kappa_max); // simple heading term
    }

    vector<Pose> plan(const Pose& start, const Pose& goal){
        // Discretize grid for closed-set bookkeeping
        const int W=esdf.W, H=esdf.H; double res=esdf.res;
        auto toGrid=[&](double v){ return (int)std::floor(v/res); };
        auto inside=[&](int ix,int iy){ return ix>=0 && iy>=0 && ix<W && iy<H; };

        vector<HAState> nodes; nodes.reserve(200000);
        auto cmp = [&](int a, int b){ return nodes[a].f > nodes[b].f; };
        std::priority_queue<int, vector<int>, decltype(cmp)> open(cmp);
        // closed set: 3D occupancy (W*H*N_theta)
        vector<float> closed((size_t)W*H*p.N_theta, std::numeric_limits<float>::infinity());
        auto key = [&](int ix,int iy,int it){ return ((size_t)it * H + iy) * W + ix; };

        // Steering set: {-kappa_max, 0, +kappa_max}
        const double Ks[3] = {-p.kappa_max, 0.0, p.kappa_max};

        // Seed
        HAState s0; s0.x=start.x; s0.y=start.y; s0.psi=start.psi; s0.ix=toGrid(start.x); s0.iy=toGrid(start.y); s0.it=thetaBin(start.psi); s0.g=0; s0.f=heuristic(start,goal); s0.parent=-1; nodes.push_back(s0); open.push(0);
        if(!inside(s0.ix,s0.iy)){ std::cerr<<"Start outside grid!\n"; return {}; }

        int goal_idx=-1; int expansions=0;
        while(!open.empty()){
            int i = open.top(); open.pop(); HAState cur = nodes[i];
            // Goal check
            double dx=goal.x-cur.x, dy=goal.y-cur.y; if(std::sqrt(dx*dx+dy*dy) < p.goal_xy_tol && std::abs(wrapAngle(goal.psi-cur.psi)) < p.goal_psi_tol){ goal_idx=i; break; }
            size_t ck = key(cur.ix,cur.iy,cur.it); if(cur.g >= closed[ck]) continue; closed[ck]=(float)cur.g;
            // Expand
            for(int si=0; si<3; ++si){ double K = Ks[si]; if(!freeAlong({cur.x,cur.y,cur.psi}, K, p.ds)) continue;
                Pose nxt; nxt.x = cur.x + p.ds*std::cos(cur.psi); nxt.y = cur.y + p.ds*std::sin(cur.psi); nxt.psi = wrapAngle(cur.psi + p.ds*K);
                HAState n; n.x=nxt.x; n.y=nxt.y; n.psi=nxt.psi; n.ix=toGrid(nxt.x); n.iy=toGrid(nxt.y); n.it=thetaBin(nxt.psi); if(!inside(n.ix,n.iy)) continue;
                double step_cost = p.ds * (1.0 + 0.1*(si!=1)); // slight penalty for turning
                n.g = cur.g + step_cost; n.f = n.g + heuristic(nxt, goal); n.parent=i; n.steer_idx=si;
                size_t nk = key(n.ix,n.iy,n.it); if(n.g >= closed[nk]) continue; int nid = (int)nodes.size(); nodes.push_back(n); open.push(nid);
            }
            if(++expansions > 200000){ std::cerr<<"Hybrid A* abort: too many expansions\n"; break; }
        }

        if(goal_idx<0){ std::cerr<<"Hybrid A* failed to reach goal tolerance\n"; return {}; }
        // Reconstruct
        vector<Pose> path_rev; for(int i=goal_idx; i!=-1; i=nodes[i].parent){ path_rev.push_back({nodes[i].x,nodes[i].y,nodes[i].psi}); }
        std::reverse(path_rev.begin(), path_rev.end());
        return path_rev;
    }
};

// --------------------- IO helpers ---------------------
static void write_snapped_csv(const std::string& path, const SnapState& s, int N){ std::ofstream f(path); f<<"x,y,psi,kappa\n"; for(int k=0;k<=N;++k){ f<<s.x[k]<<","<<s.y[k]<<","<<s.psi[k]<<","<<s.kappa[k]<<"\n"; } }
static void write_guide_csv(const std::string& path, const GuidePath& g){ std::ofstream f(path); f<<"x,y\n"; for(double u: g.u_samples){ Vector2d p,t; g.eval(u,p,t); f<<p.x()<<","<<p.y()<<"\n"; } }
static void write_esdf_csv(const std::string& path, const ESDFGrid& e){ std::ofstream f(path); f<<"x,y,dist\n"; for(int iy=0; iy<e.H; ++iy){ for(int ix=0; ix<e.W; ++ix){ double x=(ix+0.5)*e.res; double y=(iy+0.5)*e.res; double d=e.d[iy*e.W+ix]; f<<x<<","<<y<<","<<d<<"\n"; } } }
static void write_meta_json(const std::string& path, const Pose& start, const Pose& goal, const ESDFGrid& e){ std::ofstream f(path); f<<"{\n";
    f<<"  \"start\": { \"x\": "<<start.x<<", \"y\": "<<start.y<<", \"psi\": "<<start.psi<<" },\n";
    f<<"  \"goal\": { \"x\": "<<goal.x<<", \"y\": "<<goal.y<<", \"psi\": "<<goal.psi<<" },\n";
    f<<"  \"grid\": { \"W\": "<<e.W<<", \"H\": "<<e.H<<", \"res\": "<<e.res<<" }\n";
    f<<"}\n"; }

// --------------------- Guide from lattice path ---------------------
static GuidePath guideFromLattice(const vector<Pose>& P){
    GuidePath g; if(P.size()<2){ // trivial
        if(P.empty()){ g.ctrl = { {0,0}, {0,0}, {0,0}, {0,0} }; }
        else{ Vector2d p(P.front().x,P.front().y); g.ctrl = {p,p,p,p}; }
    } else {
        // Downsample a bit to avoid overfitting tiny lattice wiggles
        vector<Vector2d> C; C.reserve(P.size()); const double MIN_DIST=0.15; Vector2d last(P[0].x,P[0].y); C.push_back(last);
        for(size_t i=1;i<P.size();++i){ Vector2d q(P[i].x,P[i].y); if((q-last).norm()>MIN_DIST){ C.push_back(q); last=q; } }
        if(C.size()<4){ // pad endpoints
            while(C.size()<4) C.push_back(C.back());
        }
        // Add two phantom points for Catmull–Rom boundary behavior
        Vector2d pre = C[0] + (C[0]-C[1]); Vector2d post = C.back() + (C.back()-C[C.size()-2]);
        g.ctrl.clear(); g.ctrl.push_back(pre); for(auto& c: C) g.ctrl.push_back(c); g.ctrl.push_back(post);
    }
    g.buildArcTable(1000);
    return g;
}

// --------------------- Demo main ---------------------
int main(){
    // Grid & ESDF
    ESDFGrid esdf(220, 220, 0.05); // 11x11 m
    esdf.setCircleObstacle(5.5, 2.2, 0.6);

    // Start/Goal
    Pose start{0.4, 0.2, 0.0};
    Pose goal {8.0, 3.0, 0.7};

    // 1) Hybrid A*
    HybridAstarParams hap; hap.kappa_max = 0.35; hap.ds = 0.2; hap.robot_radius = 0.3; hap.N_theta = 72;
    HybridAstar hsearch(esdf, hap);
    auto lattice_path = hsearch.plan(start, goal);
    if(lattice_path.size()<2){ std::cerr<<"No lattice path found.\n"; return 1; }

    Pose start2{1.0, 1.0, 0.0};
    Pose goal2{8.0, 6.0, 0.7};
    auto lattice_path_guide = hsearch.plan(start2, goal2);

    // 2) Build guide from lattice path
    GuidePath guide = guideFromLattice(lattice_path_guide);

    // 3) Snap/smooth
    SnapParams sp; sp.kappa_max = hap.kappa_max; sp.endpoint_taper=1.2; sp.w_ddkappa=2.0; sp.w_dpsi=0.2; sp.h=0.06; sp.N=150; sp.d_safe=0.4;
    SnapSolver solver(guide, esdf, sp);
    SnapState s = solver.solve(start, &goal);

    // 4) Outputs
    write_snapped_csv("snapped.csv", s, sp.N);
    write_guide_csv("guide.csv", guide);
    write_esdf_csv("esdf.csv", esdf);
    write_meta_json("meta.json", start, goal, esdf);

    // Also print snapped to stdout
    std::cout << "x,y,psi,kappa\n";
    for(int k=0;k<=sp.N;++k){ std::cout<<s.x[k]<<","<<s.y[k]<<","<<s.psi[k]<<","<<s.kappa[k]<<"\n"; }
    return 0;
}
