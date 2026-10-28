#include <SlicingTrajPlan/VirtualBaffle.h>
#include <SlicingTrajPlan/UniformSlicer.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
namespace ModelTrajPlan {
namespace {
constexpr double eps=1e-8;
using V2=Eigen::Vector2d;
double cross(const V2& a,const V2& b){return a.x()*b.y()-a.y()*b.x();}
std::vector<Eigen::Vector3d> sample(const BoundaryLine& line,size_t n) {
    std::vector<Eigen::Vector3d> p;for(const auto& a:line.points) p.push_back(a.position);
    if(p.size()>1 && (p.front()-p.back()).norm()<eps)p.pop_back();
    std::vector<double> length{0};for(size_t i=0;i<p.size();++i)length.push_back(length.back()+(p[(i+1)%p.size()]-p[i]).norm());
    if(length.back()<eps)return {};
    std::vector<Eigen::Vector3d> out;
    for(size_t i=0;i<n;++i){double d=length.back()*i/n;auto it=std::upper_bound(length.begin(),length.end(),d);size_t j=std::min(size_t(it-length.begin()-1),p.size()-1);double l=length[j+1]-length[j];out.push_back(p[j]+(p[(j+1)%p.size()]-p[j])*(l>eps?(d-length[j])/l:0));}
    return out;
}
bool overlap(const std::vector<V2>& a,const std::vector<V2>& b){
    if(a.empty()||b.empty())return false;
    for(const auto* poly:{&a,&b})for(size_t i=0;i<poly->size();++i){V2 edge=(*poly)[(i+1)%poly->size()]-(*poly)[i];
        for(const V2& axis:{V2(-edge.y(),edge.x()),V2(edge)}){if(axis.norm()<eps)continue;
            double loA=1e300,hiA=-1e300,loB=1e300,hiB=-1e300;
            for(const auto& p:a){loA=std::min(loA,p.dot(axis));hiA=std::max(hiA,p.dot(axis));}for(const auto& p:b){loB=std::min(loB,p.dot(axis));hiB=std::max(hiB,p.dot(axis));}
            if(hiA<loB-eps*axis.norm() || hiB<loA-eps*axis.norm())return false;
        }
    }return true;
}
}
bool VirtualBaffle::apertureContains(const V2& p) const {
    bool inside=false;
    for(const auto& ring:contours)for(size_t i=0,j=ring.size()-1;i<ring.size();j=i++){
        const auto &a=ring[j],&b=ring[i];const V2 e=b-a;
        if(std::abs(cross(e,p-a))<eps*std::max(eps,e.norm()) && (p-a).dot(p-b)<=eps*eps)return false; // boundary is forbidden
        if((a.y()>p.y())!=(b.y()>p.y()) && p.x()<(b.x()-a.x())*(p.y()-a.y())/(b.y()-a.y())+a.x())inside=!inside;
    }return inside;
}
// Exact scanline decomposition of the finite display window minus the aperture.
// Collision invokes this with a window enclosing the queried primitive, so its semantics are infinite.
std::vector<VirtualBaffle::Triangle> VirtualBaffle::displayTriangles(double extent) const {
    std::vector<Triangle> out;if(!enabled || !(extent>0))return out;
    std::vector<double> xs{-extent,extent};for(const auto& ring:contours)for(const auto& p:ring)if(p.x()>-extent && p.x()<extent)xs.push_back(p.x());
    std::sort(xs.begin(),xs.end());xs.erase(std::unique(xs.begin(),xs.end(),[](double a,double b){return std::abs(a-b)<eps;}),xs.end());
    struct Cut {double mid,left,right;};
    for(size_t k=1;k<xs.size();++k){double l=xs[k-1],r=xs[k],m=(l+r)/2;if(r-l<eps)continue;
        std::vector<Cut> cuts{{-extent,-extent,-extent},{extent,extent,extent}};
        for(const auto& ring:contours)for(size_t i=0;i<ring.size();++i){auto a=ring[i],b=ring[(i+1)%ring.size()];if((a.x()<m)==(b.x()<m))continue;
            const auto y=[&](double x){return a.y()+(b.y()-a.y())*(x-a.x())/(b.x()-a.x());};double ym=y(m);if(ym>-extent && ym<extent)cuts.push_back({ym,std::clamp(y(l),-extent,extent),std::clamp(y(r),-extent,extent)});
        }
        std::sort(cuts.begin(),cuts.end(),[](const auto& a,const auto& b){return a.mid<b.mid;});
        for(size_t i=1;i<cuts.size();++i){if(apertureContains(V2(m,(cuts[i-1].mid+cuts[i].mid)/2)))continue;
            const auto point=[&](double x,double y)->Eigen::Vector3d{return center+x*axisX+y*axisY;};
            auto a=point(l,cuts[i-1].left),b=point(r,cuts[i-1].right),c=point(r,cuts[i].right),d=point(l,cuts[i].left);out.push_back({a,b,c});out.push_back({a,c,d});
        }
    }return out;
}
bool VirtualBaffle::intersectsTriangle(const Triangle& t,double clearance) const {
    if(!enabled)return false;
    // L-infinity inflation in the baffle basis conservatively enforces Euclidean clearance.
    double h=thickness/2+clearance;
    std::vector<Eigen::Vector3d> poly(t.begin(),t.end());
    for(int sign:{-1,1}){std::vector<Eigen::Vector3d> next;for(size_t i=0;i<poly.size();++i){const auto &a=poly[i],&b=poly[(i+1)%poly.size()];double da=sign*normal.dot(a-center)-h,db=sign*normal.dot(b-center)-h;if(da<=eps)next.push_back(a);if((da>eps)!=(db>eps))next.push_back(a+(b-a)*(da/(da-db)));}poly=std::move(next);if(poly.empty())return false;}
    std::vector<V2> projected;double extent=1;
    for(const auto& p:poly){V2 v(axisX.dot(p-center),axisY.dot(p-center));projected.push_back(v);extent=std::max(extent,v.cwiseAbs().maxCoeff()+1);if(!apertureContains(v))return true;}
    // Boundary contact and crossing, including concave apertures and holes.
    for(const auto& ring:contours)for(size_t i=0;i<ring.size();++i){const V2 a=ring[i],b=ring[(i+1)%ring.size()];
        if(overlap(projected,{a,b}))return true;
        if(clearance>0){V2 n=V2(-(b-a).y(),(b-a).x()).normalized()*clearance;V2 along=(b-a).normalized()*clearance;
            if(overlap(projected,{a-along+n,b+along+n,b+along-n,a-along-n}))return true;}
    }
    return false;
}
VirtualBaffle buildVirtualBaffle(const ModelData& model,double thickness,double maxFitResidual){
    VirtualBaffle out;out.thickness=thickness;
    if(!std::isfinite(thickness)||thickness<=0||!std::isfinite(maxFitResidual)||maxFitResidual<=0){out.diagnostic="invalid_baffle_parameters";return out;}
    BoundaryData data;Epiphyseal ignored;BoundaryDetector detector;
    if(!detector.detectBoundaries(model,data,ignored,&out.topology)){
        std::ostringstream msg;msg<<"boundary_topology: expected exactly two closed unbranched rims; found "<<out.topology.loopCount<<", branch_vertices="<<out.topology.branchVertices<<", open_vertices="<<out.topology.openVertices<<", nonmanifold_edges="<<out.topology.nonManifoldEdges;out.diagnostic=msg.str();return out;}
    const size_t n=256;auto a=sample(data.lowerBoundary_,n),b=sample(data.upperBoundary_,n);
    if(a.size()!=n||b.size()!=n){out.diagnostic="degenerate_boundary";return out;}
    // Match both winding directions and all cyclic phases after arclength resampling.
    double best=std::numeric_limits<double>::infinity();int direction=1;size_t phase=0;
    Eigen::Vector3d ca=Eigen::Vector3d::Zero(),cb=ca;for(size_t i=0;i<n;++i){ca+=a[i]/n;cb+=b[i]/n;}
    double scaleA=0,scaleB=0;for(size_t i=0;i<n;++i){scaleA+=(a[i]-ca).squaredNorm()/n;scaleB+=(b[i]-cb).squaredNorm()/n;}scaleA=std::sqrt(scaleA);scaleB=std::sqrt(scaleB);
    if(scaleA<eps || scaleB<eps){out.diagnostic="degenerate_boundary_scale";return out;}
    for(int dir:{-1,1})for(size_t shift=0;shift<n;++shift){double cost=0;for(size_t i=0;i<n;++i)cost+=((a[i]-ca)/scaleA-(b[(shift+n+dir*int(i))%n]-cb)/scaleB).squaredNorm();if(cost<best){best=cost;phase=shift;direction=dir;}}
    out.correspondenceRms=std::sqrt(best/n);
    std::vector<Eigen::Vector3d> mid;for(size_t i=0;i<n;++i)mid.push_back((a[i]+b[(phase+n+direction*int(i))%n])/2);
    for(const auto& p:mid)out.center+=p/n;
    Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();for(const auto& p:mid)covariance+=(p-out.center)*(p-out.center).transpose()/n;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
    if(eig.info()!=Eigen::Success || eig.eigenvalues()[1]<1e-12){out.diagnostic="degenerate_midpoint_plane";return out;}
    out.normal=eig.eigenvectors().col(0);out.axisX=eig.eigenvectors().col(2);out.axisY=out.normal.cross(out.axisX).normalized();
    for(const auto& p:mid){double d=std::abs(out.normal.dot(p-out.center));out.rmsResidual+=d*d/n;out.maxResidual=std::max(out.maxResidual,d);}out.rmsResidual=std::sqrt(out.rmsResidual);
    if(out.maxResidual>maxFitResidual){out.diagnostic="nonplanar_midpoints: fit maximum residual exceeds configured tolerance";return out;}
    // Cross-rim correspondence must progress through a nonzero separated band.
    // Rims need not be congruent (tapered and asymmetric parts are valid).
    // Correspondence is a bijective, order-preserving arclength map; the fitted
    // residual and real mesh section, not shape similarity, decide usability.
    if(!std::isfinite(out.correspondenceRms)){out.diagnostic="invalid_boundary_correspondence";return out;}
    SlicePlane plane{out.center,out.normal};size_t invalid=0;
    const auto sections=UniformSlicer::traceContoursFromIntersections(model,plane,true,&invalid);
    if(invalid){out.diagnostic="branched_or_invalid_plane_section";return out;}
    for(const auto& line:sections){if(line.points.size()<4 || (line.points.front().position-line.points.back().position).norm()>1e-6){out.diagnostic="open_plane_section: cannot define an unambiguous aperture";return out;}
        std::vector<V2> ring;for(size_t i=0;i+1<line.points.size();++i){const auto p=line.points[i].position-out.center;ring.emplace_back(out.axisX.dot(p),out.axisY.dot(p));}out.contours.push_back(std::move(ring));}
    if(out.contours.empty()){out.diagnostic="missing_plane_section";return out;}
    out.enabled=true;out.diagnostic=out.maxResidual>thickness?"enabled_with_fit_warning: residual exceeds slab thickness; infinite slab uses the least-squares plane":"enabled: infinite slab, closed-section even-odd aperture";return out;
}
}
