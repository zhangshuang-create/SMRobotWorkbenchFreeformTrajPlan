#include "GpuMeshProjection.h"

#include <cuda_runtime.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <mutex>

namespace SprayTrajPlan
{
    namespace
    {
        struct Vec3f { float x, y, z; };
        struct Triangle { int a, b, c, sourceFace; };
        struct Node { Vec3f lower, upper; int left, right, first, count; };
        struct Result { Vec3f point; float distance; int face; };

        inline Vec3f toFloat(const Eigen::Vector3d& point) {
            return {static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
        }
        inline Vec3f add(Vec3f a, Vec3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
        inline Vec3f scale(Vec3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
        inline Vec3f minimum(Vec3f a, Vec3f b) { return {std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z)}; }
        inline Vec3f maximum(Vec3f a, Vec3f b) { return {std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z)}; }

        struct HostBvh {
            std::vector<Vec3f> vertices;
            std::vector<Triangle> triangles;
            std::vector<Node> nodes;
            int build(int first, int count) {
                const int id = static_cast<int>(nodes.size());
                nodes.push_back({{FLT_MAX,FLT_MAX,FLT_MAX},{-FLT_MAX,-FLT_MAX,-FLT_MAX},-1,-1,first,count});
                for (int i = first; i < first + count; ++i) {
                    const Triangle& t = triangles[i];
                    nodes[id].lower = minimum(nodes[id].lower, minimum(vertices[t.a], minimum(vertices[t.b], vertices[t.c])));
                    nodes[id].upper = maximum(nodes[id].upper, maximum(vertices[t.a], maximum(vertices[t.b], vertices[t.c])));
                }
                if (count <= 8) return id;
                const Vec3f extent{nodes[id].upper.x - nodes[id].lower.x, nodes[id].upper.y - nodes[id].lower.y, nodes[id].upper.z - nodes[id].lower.z};
                const int axis = extent.y > extent.x ? (extent.z > extent.y ? 2 : 1) : (extent.z > extent.x ? 2 : 0);
                const int middle = first + count / 2;
                std::nth_element(triangles.begin() + first, triangles.begin() + middle, triangles.begin() + first + count,
                    [&](const Triangle& l, const Triangle& r) {
                        const Vec3f lc = scale(add(add(vertices[l.a],vertices[l.b]),vertices[l.c]), 1.0f / 3.0f);
                        const Vec3f rc = scale(add(add(vertices[r.a],vertices[r.b]),vertices[r.c]), 1.0f / 3.0f);
                        return axis == 0 ? lc.x < rc.x : (axis == 1 ? lc.y < rc.y : lc.z < rc.z);
                    });
                nodes[id].left = build(first, middle - first);
                nodes[id].right = build(middle, first + count - middle);
                nodes[id].count = 0;
                return id;
            }
        };

        struct DeviceCache {
            const void* vertexKey = nullptr; const void* faceKey = nullptr;
            size_t vertexCount = 0, faceCount = 0;
            Vec3f* vertices = nullptr; Triangle* triangles = nullptr; Node* nodes = nullptr;
            size_t triangleCount = 0, nodeCount = 0;
            ~DeviceCache() { cudaFree(vertices); cudaFree(triangles); cudaFree(nodes); }
            void clear() { cudaFree(vertices); cudaFree(triangles); cudaFree(nodes); vertices = nullptr; triangles = nullptr; nodes = nullptr; }
        };
        DeviceCache cache;
        std::mutex cacheMutex;

        __device__ Vec3f dsub(Vec3f a, Vec3f b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
        __device__ Vec3f dadd(Vec3f a, Vec3f b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
        __device__ Vec3f dscale(Vec3f a, float s) { return {a.x*s,a.y*s,a.z*s}; }
        __device__ float dot(Vec3f a, Vec3f b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
        __device__ float squared(Vec3f a) { return dot(a,a); }
        __device__ float length(Vec3f a) { return sqrtf(squared(a)); }
        __device__ Vec3f normalized(Vec3f a) { const float n=length(a); return n>1.0e-12f?dscale(a,1.0f/n):Vec3f{0,0,0}; }
        __device__ void addDirected(Vec3f p, const Vec3f* leader, int begin, int count,
            float target, float gain, Vec3f& force) {
            if (count <= 0) return;
            int best=begin; float best2=FLT_MAX;
            for (int i=begin;i<begin+count;++i) { const float q=squared(dsub(p,leader[i])); if(q<best2){best2=q;best=i;} }
            const float distance=sqrtf(best2); if(distance<=1.0e-8f) return;
            const float error=(distance-target)/target; const float t=fminf(fabsf(error)/0.015f,1.0f);
            const float dead=t*t*(3.0f-2.0f*t);
            const float magnitude=(distance<target?6.0f*fminf(target/distance-1.0f,4.0f):-6.0f*fminf(distance/target-1.0f,1.0f))*dead*gain*0.5f;
            force=dadd(force,dscale(normalized(dsub(p,leader[best])),magnitude));
        }
        __global__ void blankKernel(const Vec3f* samples, int sampleCount, const Vec3f* points, int pointCount,
            const Vec3f* fixed, int fixedCount, float* coverage) {
            const int index=blockIdx.x*blockDim.x+threadIdx.x; if(index>=sampleCount) return;
            float best=FLT_MAX; const Vec3f p=samples[index];
            for(int i=0;i<pointCount;++i) best=fminf(best,length(dsub(p,points[i])));
            for(int i=0;i<fixedCount;++i) best=fminf(best,length(dsub(p,fixed[i])));
            coverage[index]=best;
        }
        __global__ void forceKernel(const Vec3f* points, Vec3f* output, const int* offsets, const int* owners,
            const unsigned char* movable, int pointCount, int lineCount, const Vec3f* fixed, int fixedCount,
            const Vec3f* seam, int seamCount, int region, float d, Vec3f blank, float blankCoverage) {
            const int index=blockIdx.x*blockDim.x+threadIdx.x; if(index>=pointCount) return;
            const int line=owners[index], begin=offsets[line], end=offsets[line+1], local=index-begin;
            const Vec3f p=points[index]; if(!movable[line] || local==0 || index+1==end){output[index]=p;return;}
            Vec3f force=dscale(dadd(dadd(points[index-1],points[index+1]),dscale(p,-2.0f)),region==0?0.80f:2.40f);
            for(int q=begin;q<end;++q) { if(abs(q-index)<=6) continue; const Vec3f delta=dsub(p,points[q]); const float distance=length(delta); if(distance>1.0e-8f && distance<1.8f*d) force=dadd(force,dscale(delta,0.30f*(1.0f-distance/(1.8f*d))/distance)); }
            if(region==0) {
                if(line>0) addDirected(p,points,offsets[line-1],offsets[line]-offsets[line-1],d,0.680f,force);
                if(line+1<lineCount) addDirected(p,points,offsets[line+1],offsets[line+2]-offsets[line+1],d,0.680f,force);
                if((line==0 || line+1==lineCount) && seamCount>0) addDirected(p,seam,0,seamCount,d,2.04f,force);
                else if(line==0 && lineCount>1) addDirected(p,points,offsets[lineCount-1],offsets[lineCount]-offsets[lineCount-1],2.0f*d,0.680f,force);
                else if(line+1==lineCount && lineCount>1) addDirected(p,points,offsets[0],offsets[1],2.0f*d,0.680f,force);
                if(blankCoverage>0.5f*d) { const Vec3f delta=dsub(blank,p); const float distance=length(delta); const float t=fminf(fmaxf((distance-0.5f*d)/d,0.0f),1.0f); if(distance>1.0e-8f) force=dadd(force,dscale(delta,0.05f*t*t*(3.0f-2.0f*t)/distance)); }
            } else if(line>0) addDirected(p,points,offsets[line-1],offsets[line]-offsets[line-1],d,0.680f,force);
            else addDirected(p,fixed,0,fixedCount,d,0.680f,force);
            const float magnitude=length(force); output[index]=magnitude>1.0e-8f?dadd(p,dscale(force,fminf(0.12f*d,magnitude)/magnitude)):p;
        }
        __device__ Vec3f closestTriangle(Vec3f p, Vec3f a, Vec3f b, Vec3f c) {
            const Vec3f ab=dsub(b,a), ac=dsub(c,a), ap=dsub(p,a); const float d1=dot(ab,ap), d2=dot(ac,ap);
            if (d1<=0.f && d2<=0.f) return a;
            const Vec3f bp=dsub(p,b); const float d3=dot(ab,bp), d4=dot(ac,bp); if (d3>=0.f && d4<=d3) return b;
            const float vc=d1*d4-d3*d2; if (vc<=0.f && d1>=0.f && d3<=0.f) return dadd(a,dscale(ab,d1/(d1-d3)));
            const Vec3f cp=dsub(p,c); const float d5=dot(ab,cp), d6=dot(ac,cp); if (d6>=0.f && d5<=d6) return c;
            const float vb=d5*d2-d1*d6; if (vb<=0.f && d2>=0.f && d6<=0.f) return dadd(a,dscale(ac,d2/(d2-d6)));
            const float va=d3*d6-d5*d4; if (va<=0.f && d4-d3>=0.f && d5-d6>=0.f) return dadd(b,dscale(dsub(c,b),(d4-d3)/((d4-d3)+(d5-d6))));
            const float inverse=1.f/(va+vb+vc); return dadd(a,dadd(dscale(ab,vb*inverse),dscale(ac,vc*inverse)));
        }
        __device__ float boxDistance(Vec3f p, const Node& node) {
            const float x=p.x<node.lower.x ? node.lower.x-p.x : (p.x>node.upper.x ? p.x-node.upper.x : 0.f);
            const float y=p.y<node.lower.y ? node.lower.y-p.y : (p.y>node.upper.y ? p.y-node.upper.y : 0.f);
            const float z=p.z<node.lower.z ? node.lower.z-p.z : (p.z>node.upper.z ? p.z-node.upper.z : 0.f); return x*x+y*y+z*z;
        }
        __global__ void closestKernel(const Vec3f* vertices, const Triangle* triangles, const Node* nodes,
            const Vec3f* queries, Result* results, int queryCount) {
            const int index=blockIdx.x*blockDim.x+threadIdx.x; if(index>=queryCount) return;
            const Vec3f query=queries[index]; float best=FLT_MAX; Vec3f point{0,0,0}; int face=-1; int stack[64]; int top=0; stack[top++]=0;
            while(top) { const Node node=nodes[stack[--top]]; if(boxDistance(query,node)>best) continue;
                if(node.left<0) for(int i=node.first;i<node.first+node.count;++i) { const Triangle tri=triangles[i]; const Vec3f q=closestTriangle(query,vertices[tri.a],vertices[tri.b],vertices[tri.c]); const float d=squared(dsub(query,q)); if(d<best) {best=d;point=q;face=tri.sourceFace;} }
                else { const float left=boxDistance(query,nodes[node.left]), right=boxDistance(query,nodes[node.right]); if(left<right) {stack[top++]=node.right;stack[top++]=node.left;} else {stack[top++]=node.left;stack[top++]=node.right;} }
            }
            results[index]={point,sqrtf(best),face};
        }
        bool ready(cudaError_t status) { return status == cudaSuccess; }
        bool upload(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& faces) {
            if (cache.vertexKey == &vertices && cache.faceKey == &faces && cache.vertexCount == vertices.size() && cache.faceCount == faces.size()) return true;
            HostBvh host; host.vertices.reserve(vertices.size()); for(const auto& point:vertices) host.vertices.push_back(toFloat(point));
            host.triangles.reserve(faces.size()); for(size_t i=0;i<faces.size();++i) { const auto& f=faces[i]; if(f.minCoeff()>=0 && f.maxCoeff()<static_cast<int>(vertices.size())) host.triangles.push_back({f[0],f[1],f[2],static_cast<int>(i)}); }
            if(host.triangles.empty()) return false; host.nodes.reserve(host.triangles.size()*2); host.build(0,static_cast<int>(host.triangles.size())); cache.clear();
            if(!ready(cudaMalloc(&cache.vertices,host.vertices.size()*sizeof(Vec3f))) || !ready(cudaMalloc(&cache.triangles,host.triangles.size()*sizeof(Triangle))) || !ready(cudaMalloc(&cache.nodes,host.nodes.size()*sizeof(Node)))) { cache.clear(); return false; }
            if(!ready(cudaMemcpy(cache.vertices,host.vertices.data(),host.vertices.size()*sizeof(Vec3f),cudaMemcpyHostToDevice)) || !ready(cudaMemcpy(cache.triangles,host.triangles.data(),host.triangles.size()*sizeof(Triangle),cudaMemcpyHostToDevice)) || !ready(cudaMemcpy(cache.nodes,host.nodes.data(),host.nodes.size()*sizeof(Node),cudaMemcpyHostToDevice))) { cache.clear(); return false; }
            cache.vertexKey=&vertices; cache.faceKey=&faces; cache.vertexCount=vertices.size(); cache.faceCount=faces.size(); cache.triangleCount=host.triangles.size(); cache.nodeCount=host.nodes.size(); return true;
        }
    }

    bool projectClosestPointsCuda(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& faces,
        const std::vector<Eigen::Vector3d>& queries, std::vector<Eigen::Vector3d>& closest, std::vector<double>& distances, std::vector<int>& faceIds) {
        if(queries.empty()) { closest.clear(); distances.clear(); faceIds.clear(); return true; }
        std::lock_guard<std::mutex> guard(cacheMutex); if(!upload(vertices,faces)) return false;
        std::vector<Vec3f> hostQueries; hostQueries.reserve(queries.size()); for(const auto& q:queries) hostQueries.push_back(toFloat(q)); Vec3f* deviceQueries=nullptr; Result* deviceResults=nullptr;
        if(!ready(cudaMalloc(&deviceQueries,hostQueries.size()*sizeof(Vec3f))) || !ready(cudaMalloc(&deviceResults,queries.size()*sizeof(Result)))) { cudaFree(deviceQueries); cudaFree(deviceResults); return false; }
        const bool copied=ready(cudaMemcpy(deviceQueries,hostQueries.data(),hostQueries.size()*sizeof(Vec3f),cudaMemcpyHostToDevice)); if(copied) closestKernel<<<(static_cast<int>(queries.size())+255)/256,256>>>(cache.vertices,cache.triangles,cache.nodes,deviceQueries,deviceResults,static_cast<int>(queries.size()));
        std::vector<Result> results(queries.size()); const bool completed=copied && ready(cudaGetLastError()) && ready(cudaMemcpy(results.data(),deviceResults,results.size()*sizeof(Result),cudaMemcpyDeviceToHost)); cudaFree(deviceQueries); cudaFree(deviceResults); if(!completed) return false;
        closest.resize(results.size()); distances.resize(results.size()); faceIds.resize(results.size()); for(size_t i=0;i<results.size();++i) { closest[i]=Eigen::Vector3d(results[i].point.x,results[i].point.y,results[i].point.z); distances[i]=results[i].distance; faceIds[i]=results[i].face; } return true;
    }

    bool relaxEnergyRegionCuda(const std::vector<std::vector<Eigen::Vector3d>>& lines,
        const std::vector<bool>& movable, const std::vector<Eigen::Vector3d>& fixed,
        const std::vector<Eigen::Vector3d>& seam, const std::vector<Eigen::Vector3d>& samples,
        int regionCode, double spacing, std::vector<std::vector<Eigen::Vector3d>>& updated)
    {
        if(lines.empty() || lines.size()!=movable.size() || samples.empty()) return false;
        std::vector<Vec3f> points, hostFixed, hostSeam, hostSamples;
        std::vector<int> offsets{0}, owners; std::vector<unsigned char> hostMovable;
        for(size_t line=0;line<lines.size();++line){ if(lines[line].size()<2) return false; hostMovable.push_back(movable[line]?1:0); for(const auto& p:lines[line]){points.push_back(toFloat(p));owners.push_back(static_cast<int>(line));} offsets.push_back(static_cast<int>(points.size())); }
        for(const auto& p:fixed) hostFixed.push_back(toFloat(p)); for(const auto& p:seam) hostSeam.push_back(toFloat(p)); for(const auto& p:samples) hostSamples.push_back(toFloat(p));
        Vec3f *dPoints=nullptr,*dOutput=nullptr,*dFixed=nullptr,*dSeam=nullptr,*dSamples=nullptr; int *dOffsets=nullptr,*dOwners=nullptr; unsigned char* dMovable=nullptr; float* dCoverage=nullptr;
        auto copy=[&](auto** device,const auto& host){ const size_t bytes=host.size()*sizeof(host[0]); return host.empty() || (ready(cudaMalloc(reinterpret_cast<void**>(device),bytes)) && ready(cudaMemcpy(*device,host.data(),bytes,cudaMemcpyHostToDevice))); };
        const bool uploaded=copy(&dPoints,points)&&copy(&dFixed,hostFixed)&&copy(&dSeam,hostSeam)&&copy(&dSamples,hostSamples)&&copy(&dOffsets,offsets)&&copy(&dOwners,owners)&&copy(&dMovable,hostMovable)
            && ready(cudaMalloc(&dOutput,points.size()*sizeof(Vec3f)))
            && (regionCode != 0 || ready(cudaMalloc(&dCoverage,hostSamples.size()*sizeof(float))));
        if(!uploaded){cudaFree(dPoints);cudaFree(dOutput);cudaFree(dFixed);cudaFree(dSeam);cudaFree(dSamples);cudaFree(dOffsets);cudaFree(dOwners);cudaFree(dMovable);cudaFree(dCoverage);return false;}
        const int blocksPoints=(static_cast<int>(points.size())+255)/256, blocksSamples=(static_cast<int>(hostSamples.size())+255)/256;
        Vec3f blank{0,0,0}; float blankCoverage=0.0f; bool ok=true;
        if(regionCode == 0) {
            blankKernel<<<blocksSamples,256>>>(dSamples,static_cast<int>(hostSamples.size()),dPoints,static_cast<int>(points.size()),dFixed,static_cast<int>(hostFixed.size()),dCoverage);
            std::vector<float> coverage(hostSamples.size()); ok=ready(cudaGetLastError())&&ready(cudaMemcpy(coverage.data(),dCoverage,coverage.size()*sizeof(float),cudaMemcpyDeviceToHost));
            size_t blankIndex=0; if(ok) for(size_t i=1;i<coverage.size();++i) if(coverage[i]>coverage[blankIndex]) blankIndex=i;
            if(ok) { blank=hostSamples[blankIndex]; blankCoverage=coverage[blankIndex]; }
        }
        if(ok) forceKernel<<<blocksPoints,256>>>(dPoints,dOutput,dOffsets,dOwners,dMovable,static_cast<int>(points.size()),static_cast<int>(lines.size()),dFixed,static_cast<int>(hostFixed.size()),dSeam,static_cast<int>(hostSeam.size()),regionCode,static_cast<float>(spacing),blank,blankCoverage);
        std::vector<Vec3f> result(points.size()); ok=ok&&ready(cudaGetLastError())&&ready(cudaMemcpy(result.data(),dOutput,result.size()*sizeof(Vec3f),cudaMemcpyDeviceToHost));
        cudaFree(dPoints);cudaFree(dOutput);cudaFree(dFixed);cudaFree(dSeam);cudaFree(dSamples);cudaFree(dOffsets);cudaFree(dOwners);cudaFree(dMovable);cudaFree(dCoverage); if(!ok) return false;
        updated.clear(); updated.resize(lines.size()); for(size_t line=0;line<lines.size();++line){updated[line].reserve(lines[line].size());for(int i=offsets[line];i<offsets[line+1];++i)updated[line].emplace_back(result[i].x,result[i].y,result[i].z);} return true;
    }
}
