#include <SlicingTrajPlan/BoundaryDetector.h>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/BoundaryPlaneGeometry.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <unordered_set>

bool Vector3dLexicographicalCompare::operator()(const Eigen::Vector3d& left, const Eigen::Vector3d& right) const
{
    constexpr double epsilon = 1.0e-9;
    if(std::abs(left.x() - right.x()) > epsilon) return left.x() < right.x();
    if(std::abs(left.y() - right.y()) > epsilon) return left.y() < right.y();
    if(std::abs(left.z() - right.z()) > epsilon) return left.z() < right.z();
    return false;
}

bool PairOfVector3dCompare::operator()(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& left,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& right) const
{
    Vector3dLexicographicalCompare compare;
    if(compare(left.first, right.first)) return true;
    if(compare(right.first, left.first)) return false;
    return compare(left.second, right.second);
}namespace ModelTrajPlan
{
    namespace
    {
        constexpr size_t kReflectionSampleCount = 600;

        std::vector<Eigen::Vector3d> resampleClosedPolyline(
            const std::vector<Eigen::Vector3d>& points, size_t count)
        {
            std::vector<Eigen::Vector3d> uniquePoints;
            uniquePoints.reserve(points.size());
            for (const auto& point : points) {
                if (uniquePoints.empty() ||
                    (point - uniquePoints.back()).squaredNorm() > 1e-18) {
                    uniquePoints.push_back(point);
                }
            }
            if (uniquePoints.size() > 1 &&
                (uniquePoints.front() - uniquePoints.back()).squaredNorm() <= 1e-18) {
                uniquePoints.pop_back();
            }
            if (uniquePoints.size() < 3 || count < 3) {
                return {};
            }

            std::vector<double> cumulative(uniquePoints.size() + 1, 0.0);
            for (size_t i = 0; i < uniquePoints.size(); ++i) {
                cumulative[i + 1] = cumulative[i] +
                    (uniquePoints[(i + 1) % uniquePoints.size()] - uniquePoints[i]).norm();
            }
            const double length = cumulative.back();
            if (length <= 1e-12) {
                return {};
            }

            std::vector<Eigen::Vector3d> samples;
            samples.reserve(count);
            size_t segment = 0;
            for (size_t i = 0; i < count; ++i) {
                const double distance = length * static_cast<double>(i) / static_cast<double>(count);
                while (segment + 1 < cumulative.size() - 1 && distance > cumulative[segment + 1]) {
                    ++segment;
                }
                const Eigen::Vector3d& start = uniquePoints[segment];
                const Eigen::Vector3d& end = uniquePoints[(segment + 1) % uniquePoints.size()];
                const double segmentLength = cumulative[segment + 1] - cumulative[segment];
                samples.push_back(start + (distance - cumulative[segment]) / segmentLength * (end - start));
            }
            return samples;
        }

        double pointToSampleCloudSquaredDistance(
            const Eigen::Vector3d& query, const std::vector<Eigen::Vector3d>& samples)
        {
            double minimum = std::numeric_limits<double>::infinity();
            for (const auto& sample : samples) {
                minimum = std::min(minimum, (query - sample).squaredNorm());
            }
            return minimum;
        }

        bool fitReflectionSymmetryNormal(
            const BoundaryLine& upperBoundary, Eigen::Vector3d& normalOut)
        {
            std::vector<Eigen::Vector3d> topPoints;
            topPoints.reserve(upperBoundary.points.size());
            for (const auto& point : upperBoundary.points) {
                topPoints.push_back(point.position);
            }
            const auto samples = resampleClosedPolyline(topPoints, kReflectionSampleCount);
            if (samples.size() != kReflectionSampleCount) {
                return false;
            }

            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto& point : samples) center += point;
            center /= static_cast<double>(samples.size());

            std::vector<Eigen::Vector3d> centeredSamples;
            centeredSamples.reserve(samples.size());
            for (const auto& point : samples) centeredSamples.push_back(point - center);

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const auto& delta : centeredSamples) {
                covariance += delta * delta.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
            if (eigensolver.info() != Eigen::Success) {
                return false;
            }
            const Eigen::Vector3d xApprox = eigensolver.eigenvectors().col(2);
            const Eigen::Vector3d zApprox = eigensolver.eigenvectors().col(0);
            const Eigen::Vector3d yApprox = zApprox.cross(xApprox).normalized();

            auto evaluate = [&](const Eigen::Vector3d& parameters) {
                const Eigen::Vector3d inPlane = std::cos(parameters.x()) * xApprox +
                    std::sin(parameters.x()) * yApprox;
                Eigen::Vector3d normal = std::cos(parameters.y()) * inPlane +
                    std::sin(parameters.y()) * zApprox;
                if (normal.norm() <= 1e-12) return std::numeric_limits<double>::infinity();
                normal.normalize();
                double squaredSum = 0.0;
                for (const auto& point : centeredSamples) {
                    const Eigen::Vector3d reflected = point - 2.0 * normal.dot(point) * normal;
                    squaredSum += pointToSampleCloudSquaredDistance(reflected, centeredSamples);
                }
                return squaredSum / static_cast<double>(samples.size());
            };

            std::array<std::pair<double, Eigen::Vector3d>, 120> seeds;
            for (size_t i = 0; i < seeds.size(); ++i) {
                Eigen::Vector3d parameters(2.0 * M_PI * static_cast<double>(i) / seeds.size(), 0.0, 0.0);
                seeds[i] = { evaluate(parameters), parameters };
            }
            std::sort(seeds.begin(), seeds.end(),
                [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

            double bestScore = std::numeric_limits<double>::infinity();
            Eigen::Vector3d bestParameters = Eigen::Vector3d::Zero();
            for (size_t seedIndex = 0; seedIndex < 1; ++seedIndex) {
                Eigen::Vector3d parameters = seeds[seedIndex].second;
                double score = seeds[seedIndex].first;
                Eigen::Vector2d step(0.10, 0.05);
                for (int iteration = 0; iteration < 100; ++iteration) {
                    bool improved = false;
                    for (int dimension = 0; dimension < 2; ++dimension) {
                        for (double sign : { -1.0, 1.0 }) {
                            Eigen::Vector3d candidate = parameters;
                            candidate[dimension] += sign * step[dimension];
                            const double candidateScore = evaluate(candidate);
                            if (candidateScore < score) {
                                parameters = candidate;
                                score = candidateScore;
                                improved = true;
                            }
                        }
                    }
                    if (!improved) step *= 0.5;
                    if (step.maxCoeff() < 1e-6) break;
                }
                if (score < bestScore) {
                    bestScore = score;
                    bestParameters = parameters;
                }
            }
            if (!std::isfinite(bestScore)) {
                return false;
            }

            const Eigen::Vector3d inPlane = std::cos(bestParameters.x()) * xApprox +
                std::sin(bestParameters.x()) * yApprox;
            normalOut = std::cos(bestParameters.y()) * inPlane +
                std::sin(bestParameters.y()) * zApprox;
            if (normalOut.norm() <= 1e-12) {
                return false;
            }
            normalOut.normalize();
            return true;
        }

        bool appendProjectedOuterContour(const ModelData& modelData,
            const Eigen::Vector3d& planeOrigin, const Eigen::Vector3d& planeNormal,
            const BoundaryLine& lowerBoundary, const BoundaryLine& upperBoundary,
            Epiphyseal& projectedContourLines,
            std::vector<BoundaryLine>& referenceLines)
        {
            const size_t triangleCount = STLProcessor::getTriangleCount(modelData.mainModelData_);
            if(triangleCount == 0 || planeNormal.squaredNorm() <= 1e-18) return false;

            const Eigen::Vector3d normal = planeNormal.normalized();
            Eigen::Vector3d height = upperBoundary.center - planeOrigin;
            height -= normal * height.dot(normal);
            if(height.squaredNorm() <= 1e-18) return false;
            height.normalize();
            const Eigen::Vector3d lateral = height.cross(normal).normalized();

            struct ProjectedTriangle { Eigen::Vector2d vertices[3]; };
            std::vector<ProjectedTriangle> triangles;
            triangles.reserve(triangleCount);
            double minX = std::numeric_limits<double>::infinity();
            double maxX = -minX;
            double minY = minX;
            double maxY = -minX;
            for(size_t index = 0; index < triangleCount; ++index) {
                Eigen::Vector3d vertices[3];
                STLProcessor::getTriangleVertices(modelData.mainModelData_, index, vertices);
                ProjectedTriangle triangle;
                for(int vertex = 0; vertex < 3; ++vertex) {
                    const Eigen::Vector3d delta = vertices[vertex] - planeOrigin;
                    triangle.vertices[vertex] = { delta.dot(lateral), delta.dot(height) };
                    minX = std::min(minX, triangle.vertices[vertex].x());
                    maxX = std::max(maxX, triangle.vertices[vertex].x());
                    minY = std::min(minY, triangle.vertices[vertex].y());
                    maxY = std::max(maxY, triangle.vertices[vertex].y());
                }
                triangles.push_back(triangle);
            }
            if(maxX - minX <= 1e-10 || maxY - minY <= 1e-10) return false;

            constexpr int kScanLineCount = 600;
            std::vector<Eigen::Vector3d> left;
            std::vector<Eigen::Vector3d> right;
            left.reserve(kScanLineCount);
            right.reserve(kScanLineCount);
            for(int row = 0; row < kScanLineCount; ++row) {
                const double y = minY + (maxY - minY) * (static_cast<double>(row) + 0.5) /
                    static_cast<double>(kScanLineCount);
                double leftmost = std::numeric_limits<double>::infinity();
                double rightmost = -leftmost;
                for(const auto& triangle : triangles) {
                    double intersections[3];
                    int count = 0;
                    for(int edge = 0; edge < 3; ++edge) {
                        const Eigen::Vector2d& a = triangle.vertices[edge];
                        const Eigen::Vector2d& b = triangle.vertices[(edge + 1) % 3];
                        if((a.y() <= y && y < b.y()) || (b.y() <= y && y < a.y())) {
                            intersections[count++] = a.x() +
                                (y - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
                        }
                    }
                    if(count == 2) {
                        leftmost = std::min(leftmost, std::min(intersections[0], intersections[1]));
                        rightmost = std::max(rightmost, std::max(intersections[0], intersections[1]));
                    }
                }
                if(std::isfinite(leftmost) && std::isfinite(rightmost)) {
                    left.push_back(planeOrigin + leftmost * lateral + y * height);
                    right.push_back(planeOrigin + rightmost * lateral + y * height);
                }
            }
            if(left.size() < 2 || right.size() < 2) return false;

            // The scan envelope also includes the sloping end caps. Remove
            // those BEFORE pairing; otherwise both curves finish at the same
            // extreme cap vertex and their midpoint collapses onto one side.
            std::vector<std::vector<Eigen::Vector3d>> rims;
            for (const BoundaryLine* boundary : { &lowerBoundary, &upperBoundary }) {
                std::vector<Eigen::Vector3d> rim;
                for (const auto& point : boundary->points)
                    rim.push_back(point.position - normal.dot(point.position - planeOrigin) * normal);
                if (rim.size() < 2) return false;
                rims.push_back(std::move(rim));
            }
            const double capTolerance = std::max(1e-4,
                std::hypot(maxX - minX, maxY - minY) * 1e-3);
            const auto onCap = [&](const Eigen::Vector3d& p) {
                for (const auto& rim : rims) {
                    for (size_t i = 0; i < rim.size(); ++i) {
                        const auto& a = rim[i];
                        const Eigen::Vector3d edge = rim[(i + 1) % rim.size()] - a;
                        const double t = edge.squaredNorm() > 1e-18 ?
                            std::clamp((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0) : 0.0;
                        if ((p - a - t * edge).norm() <= capTolerance) return true;
                    }
                }
                return false;
            };
            const auto trimSide = [&](std::vector<Eigen::Vector3d>& side) {
                // Follow the longest continuous side run. Any return onto a
                // rim terminates the run, including cap fragments that leave
                // the tolerance band again near an extreme vertex.
                size_t first = 0, last = 0, runStart = 0;
                double bestLength = -1.0, runLength = 0.0;
                for (size_t i = 0; i <= side.size(); ++i) {
                    if (i == side.size() || onCap(side[i])) {
                        if (i - runStart >= 2 && runLength > bestLength) {
                            first = runStart; last = i; bestLength = runLength;
                        }
                        runStart = i + 1;
                        runLength = 0.0;
                    } else if (i > runStart) {
                        runLength += (side[i] - side[i - 1]).norm();
                    }
                }
                if (last - first < 2) return false;
                std::vector<Eigen::Vector3d> trimmed(side.begin() + first, side.begin() + last);
                // Extend only the local outgoing tangent to the first nearby
                // rim intersection. Never connect to a global rim extremum:
                // that can reverse direction and create a false corner.
                const auto endpoint = [&](bool lower) -> Eigen::Vector3d {
                    const size_t end = lower ? 0 : trimmed.size() - 1;
                    const size_t inward = lower ? std::min<size_t>(3, trimmed.size() - 1)
                                                : trimmed.size() - 1 - std::min<size_t>(3, trimmed.size() - 1);
                    const Eigen::Vector3d p = trimmed[end];
                    Eigen::Vector3d direction = p - trimmed[inward];
                    const double localLength = direction.norm();
                    if (localLength <= 1e-10) return p;
                    direction /= localLength;
                    double bestDistance = 2.0 * localLength + capTolerance;
                    Eigen::Vector3d result = p;
                    const auto& rim = rims[lower ? 0 : 1];
                    for (size_t i = 0; i < rim.size(); ++i) {
                        const Eigen::Vector3d a = rim[i];
                        const Eigen::Vector3d edge = rim[(i + 1) % rim.size()] - a;
                        const double denominator = direction.cross(edge).dot(normal);
                        if (std::abs(denominator) < 1e-12) continue;
                        const double distance = (a - p).cross(edge).dot(normal) / denominator;
                        const double t = (a - p).cross(direction).dot(normal) / denominator;
                        if (distance >= 0.0 && distance < bestDistance && t >= -1e-8 && t <= 1.0 + 1e-8) {
                            bestDistance = distance;
                            result = p + distance * direction;
                        }
                    }
                    return result;
                };
                const Eigen::Vector3d lowerEnd = endpoint(true);
                const Eigen::Vector3d upperEnd = endpoint(false);
                if ((lowerEnd - trimmed.front()).norm() > 1e-8) trimmed.insert(trimmed.begin(), lowerEnd);
                if ((upperEnd - trimmed.back()).norm() > 1e-8) trimmed.push_back(upperEnd);
                side.swap(trimmed);
                return true;
            };
            if (!trimSide(left) || !trimSide(right)) return false;

            // Align endpoints, then pair equal fractions of each side's length.
            const auto orientLowerToUpper = [&](std::vector<Eigen::Vector3d>& contour) {
                if ((contour.front() - planeOrigin).dot(height) >
                    (contour.back() - planeOrigin).dot(height)) {
                    std::reverse(contour.begin(), contour.end());
                }
            };
            std::vector<Eigen::Vector3d> alignedLeft = left;
            std::vector<Eigen::Vector3d> alignedRight = right;
            orientLowerToUpper(alignedLeft);
            orientLowerToUpper(alignedRight);

            const auto resampleNormalized = [&](const std::vector<Eigen::Vector3d>& contour,
                const size_t sampleCount) {
                std::vector<Eigen::Vector3d> samples;
                samples.reserve(sampleCount);
                std::vector<double> arc(contour.size(), 0.0);
                for (size_t i = 1; i < contour.size(); ++i)
                    arc[i] = arc[i - 1] + (contour[i] - contour[i - 1]).norm();
                size_t segment = 0;
                for(size_t sample = 0; sample < sampleCount; ++sample) {
                    const double target = arc.back() * static_cast<double>(sample) /
                        static_cast<double>(sampleCount - 1);
                    while(segment + 1 < contour.size() - 1 &&
                        arc[segment + 1] < target) ++segment;
                    const double startHeight = arc[segment];
                    const double endHeight = arc[segment + 1];
                    const double heightDelta = endHeight - startHeight;
                    const double ratio = heightDelta > 1e-12 ?
                        (target - startHeight) / heightDelta : 0.0;
                    samples.push_back((1.0 - ratio) * contour[segment] + ratio * contour[segment + 1]);
                }
                samples.front() = contour.front();
                samples.back() = contour.back();
                return samples;
            };
            const size_t pairedSampleCount = std::max(alignedLeft.size(), alignedRight.size());
            const std::vector<Eigen::Vector3d> pairedLeft =
                resampleNormalized(alignedLeft, pairedSampleCount);
            const std::vector<Eigen::Vector3d> pairedRight =
                resampleNormalized(alignedRight, pairedSampleCount);

            projectedContourLines.clear();
            projectedContourLines.resize(3);
            for(size_t index = 0; index < pairedSampleCount; ++index) {
                LinePoint leftPoint;
                leftPoint.position = pairedLeft[index];
                leftPoint.index = index;
                projectedContourLines[0].push_back(leftPoint);

                LinePoint rightPoint;
                rightPoint.position = pairedRight[index];
                rightPoint.index = index;
                projectedContourLines[1].push_back(rightPoint);

                LinePoint centerPoint;
                centerPoint.position = 0.5 * (pairedLeft[index] + pairedRight[index]);
                centerPoint.index = index;
                projectedContourLines[2].push_back(centerPoint);
            }

            Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            Eigen::Vector3d maximum = -minimum;
            for(const auto& point : left) { minimum = minimum.cwiseMin(point); maximum = maximum.cwiseMax(point); }
            for(const auto& point : right) { minimum = minimum.cwiseMin(point); maximum = maximum.cwiseMax(point); }
            const double diagonal = (maximum - minimum).norm();
            const Eigen::Vector3d redOffset = normal * std::max(1e-4, diagonal * 1e-4);
            const Eigen::Vector3d yellowOffset = redOffset;
            const auto appendLine = [&](const std::vector<Eigen::Vector3d>& points,
                const Eigen::Vector3f& color, const char* name) {
                if(points.size() < 2) return;
                BoundaryLine line{};
                line.color = color;
                line.name = name;
                line.Type = BoundaryLineType::SlicePlane;
                line.points.reserve(points.size());
                for(size_t index = 0; index < points.size(); ++index) {
                    line.points.push_back({ points[index], 0.0, index });
                }
                referenceLines.push_back(std::move(line));
            };
            const auto appendRedSide = [&](const std::vector<Eigen::Vector3d>& side, bool reverse) {
                std::vector<Eigen::Vector3d> visible;
                visible.reserve(side.size());
                for(size_t index = 0; index < side.size(); ++index) {
                    visible.push_back(side[reverse ? (side.size() - 1 - index) : index] + redOffset);
                }
                appendLine(visible, Eigen::Vector3f(1.0f, 0.0f, 0.0f), "Projected outer contour");
            };
            appendRedSide(left, false);
            appendRedSide(right, true);
            std::vector<Eigen::Vector3d> center;
            center.reserve(pairedSampleCount);
            for(size_t index = 0; index < pairedSampleCount; ++index) {
                center.push_back(0.5 * (pairedLeft[index] + pairedRight[index]) + yellowOffset);
            }
            appendLine(center, Eigen::Vector3f(1.0f, 1.0f, 0.0f),
                "Projected contour center reference");
            return true;
        }

    }


    BoundaryDetector::BoundaryDetector()
    {
        // [DESIGN] Pure algorithm class - no internal data storage
    }

    BoundaryDetector::~BoundaryDetector() = default;

    std::map<std::pair<Eigen::Vector3d, Eigen::Vector3d>, int, PairOfVector3dCompare>
        BoundaryDetector::getEdgeCountMap(const ModelData& modelData) {
        std::map<std::pair<Eigen::Vector3d, Eigen::Vector3d>, int, PairOfVector3dCompare> edgeCount;
        size_t triCount = STLProcessor::getTriangleCount(modelData.mainModelData_);

        for (size_t i = 0; i < triCount; ++i) {
            Eigen::Vector3d v[3];
            STLProcessor::getTriangleVertices(modelData.mainModelData_, i, v);
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3d v1 = v[j];
                Eigen::Vector3d v2 = v[(j + 1) % 3];
                Vector3dLexicographicalCompare comp;
                if (comp(v2, v1)) std::swap(v1, v2);
                edgeCount[std::make_pair(v1, v2)]++;
            }
        }
        return edgeCount;
    }
    //Construct the intersection of the model and the new plane by cutting the model with horizontal planes from top to bottom
    bool BoundaryDetector::calculatePlaneIntersection(
        const ModelData& modelData,
        const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary,
        Epiphyseal& boundary2,
        Eigen::Vector3d* phasePlaneNormal)
    {
        boundary2.clear();

        Eigen::Vector3d planeNormal;
        Eigen::Vector3d upperCenter = upperBoundary.center;
        Eigen::Vector3d lowerCenter = lowerBoundary.center;
        const bool upperCircular = fitBoundaryCircle(upperBoundary, upperCenter);
        const bool lowerCircular = fitBoundaryCircle(lowerBoundary, lowerCenter);
        bool fitSucceeded = false;
        if(upperCircular && lowerCircular) {
            fitSucceeded = circularBoundaryPlane(lowerBoundary, upperBoundary,
                lowerCenter, upperCenter, planeNormal);
        } else {
            // One circular rim: analyze only the noncircular rim. Two
            // noncircular rims: use the consistently assigned second rim.
            const auto& source = upperCircular ? lowerBoundary : upperBoundary;
            fitSucceeded = fitReflectionSymmetryNormal(source, planeNormal);
            const Eigen::Vector3d axis = upperCenter - lowerCenter;
            if(fitSucceeded && axis.squaredNorm() > 1e-18) {
                planeNormal -= axis * planeNormal.dot(axis) / axis.squaredNorm();
                fitSucceeded = planeNormal.norm() > 1e-8;
                if(fitSucceeded) planeNormal.normalize();
            } else fitSucceeded = false;
        }
        if (!fitSucceeded) {
            return false;
        }
        if (phasePlaneNormal) {
            *phasePlaneNormal = planeNormal;
        }

        // ==========================================================
        // Plane equation: ax + by + cz + d = 0
        // ==========================================================
        double a = planeNormal.x();
        double b = planeNormal.y();
        double c = planeNormal.z();
        const Eigen::Vector3d& planeOrigin = lowerCenter;
        double d = -(a * planeOrigin.x() + b * planeOrigin.y() + c * planeOrigin.z());

        // ==========================================================
        // Get model boundary edges
        // ==========================================================
        auto edgeCount = getEdgeCountMap(modelData);
        std::set<Eigen::Vector3d, Vector3dLexicographicalCompare> boundaryVertices;
        for (const auto& [edge, count] : edgeCount) {
            if (count == 1) {
                boundaryVertices.insert(edge.first);
                boundaryVertices.insert(edge.second);
            }
        }

        // ==========================================================
        // Compute intersections between plane and mesh triangles
        // ==========================================================
        std::vector<CheckedSegment> validSegments;
        size_t triCount = STLProcessor::getTriangleCount(modelData.mainModelData_);
        const double eps = 1e-9;

        for (size_t i = 0; i < triCount; ++i) {
            Eigen::Vector3d vPts[3];
            STLProcessor::getTriangleVertices(modelData.mainModelData_, i, vPts);

            // Distances from triangle vertices to plane
            double dists[3];
            for (int j = 0; j < 3; ++j) {
                dists[j] = a * vPts[j].x() + b * vPts[j].y() + c * vPts[j].z() + d;
                if (std::abs(dists[j]) < eps) {
                    dists[j] = 0;
                }
            }

            // Collect the triangle's unique intersection points.  A fitted
            // symmetry plane often follows a mesh symmetry edge exactly, so
            // the old "exactly two points" case dropped valid intersections
            // whenever it hit a vertex or a coplanar edge.
            std::vector<PointInfo> pts;
            std::vector<CheckedSegment> coplanarEdges;
            const auto appendUnique = [&](const PointInfo& point) {
                for (const auto& existing : pts) {
                    if ((existing.pos - point.pos).squaredNorm() <= eps * eps) {
                        return;
                    }
                }
                pts.push_back(point);
            };
            for (int j = 0; j < 3; ++j) {
                int next = (j + 1) % 3;
                const bool firstOnPlane = dists[j] == 0.0;
                const bool secondOnPlane = dists[next] == 0.0;

                if (firstOnPlane && secondOnPlane) {
                    const bool firstBoundary = boundaryVertices.find(vPts[j]) != boundaryVertices.end();
                    const bool secondBoundary = boundaryVertices.find(vPts[next]) != boundaryVertices.end();
                    if (!(firstBoundary && secondBoundary)) {
                        coplanarEdges.push_back({ { vPts[j], firstBoundary },
                            { vPts[next], secondBoundary } });
                    }
                    continue;
                }

                // Vertex lies on plane
                if (firstOnPlane) {
                    bool isBoundary = (boundaryVertices.find(vPts[j]) != boundaryVertices.end());
                    appendUnique({ vPts[j], isBoundary });
                }

                // Edge intersects plane
                if (dists[j] * dists[next] < -eps) {
                    double t = std::abs(dists[j]) / (std::abs(dists[j]) + std::abs(dists[next]));
                    Eigen::Vector3d intersectPos = vPts[j] + t * (vPts[next] - vPts[j]);

                    Eigen::Vector3d ev1 = vPts[j], ev2 = vPts[next];
                    Vector3dLexicographicalCompare vcomp;
                    if (vcomp(ev2, ev1)) {
                        std::swap(ev1, ev2);
                    }

                    bool isBoundary = (edgeCount[{ev1, ev2}] == 1);
                    appendUnique({ intersectPos, isBoundary });
                }
            }

            if (!coplanarEdges.empty()) {
                validSegments.insert(validSegments.end(), coplanarEdges.begin(), coplanarEdges.end());
                continue;
            }

            // A vertex hit can leave three unique candidates on a triangle.
            // The farthest pair is its actual plane-crossing chord.
            if (pts.size() >= 2) {
                size_t first = 0;
                size_t second = 1;
                double maximumDistance = -1.0;
                for (size_t firstIndex = 0; firstIndex < pts.size(); ++firstIndex) {
                    for (size_t secondIndex = firstIndex + 1; secondIndex < pts.size(); ++secondIndex) {
                        const double distance = (pts[firstIndex].pos - pts[secondIndex].pos).squaredNorm();
                        if (distance > maximumDistance) {
                            maximumDistance = distance;
                            first = firstIndex;
                            second = secondIndex;
                        }
                    }
                }
                if (maximumDistance <= eps * eps ||
                    (pts[first].isBoundary && pts[second].isBoundary)) {
                    continue;
                }
                validSegments.push_back({ pts[first], pts[second] });
            }
        }

        // ==========================================================
        // Build adjacency graph and extract paths
        // ==========================================================
        std::map<Eigen::Vector3d, std::vector<Eigen::Vector3d>, Vector3dLexicographicalCompare> adjacency;
        std::map<Eigen::Vector3d, bool, Vector3dLexicographicalCompare> isBoundaryMap;

        for (const auto& seg : validSegments) {
            adjacency[seg.p1.pos].push_back(seg.p2.pos);
            adjacency[seg.p2.pos].push_back(seg.p1.pos);
            isBoundaryMap[seg.p1.pos] = seg.p1.isBoundary;
            isBoundaryMap[seg.p2.pos] = seg.p2.isBoundary;
        }

        std::set<Eigen::Vector3d, Vector3dLexicographicalCompare> visited;
        std::vector<std::vector<Eigen::Vector3d>> finalPaths;

        // Start from boundary points
        std::vector<Eigen::Vector3d> startNodes;
        for (const auto& [pos, isBoundary] : isBoundaryMap) {
            if (isBoundary) {
                startNodes.push_back(pos);
            }
        }

        // Trace paths from boundary to boundary
        for (const auto& startPos : startNodes) {
            if (visited.count(startPos)) {
                continue;
            }

            std::vector<Eigen::Vector3d> currentPath;
            Eigen::Vector3d curr = startPos;
            bool pathValid = false;

            while (true) {
                visited.insert(curr);
                currentPath.push_back(curr);

                // Stop at another boundary point
                if (currentPath.size() > 1 && isBoundaryMap[curr]) {
                    pathValid = true;
                    break;
                }

                bool foundNext = false;
                if (adjacency.count(curr)) {
                    for (const auto& nextNode : adjacency[curr]) {
                        if (!visited.count(nextNode)) {
                            if (currentPath.size() == 1 && isBoundaryMap[nextNode]) {
                                continue;
                            }
                            curr = nextNode;
                            foundNext = true;
                            break;
                        }
                    }
                }

                if (!foundNext) {
                    break;
                }
            }

            if (pathValid && currentPath.size() >= 2) {
                finalPaths.push_back(currentPath);
            }
        }

        // ==========================================================
        // Convert paths to Epiphyseal format
        // ==========================================================
        for (const auto& path : finalPaths) {
            std::vector<LinePoint> linePoints;
            for (size_t i = 0; i < path.size(); ++i) {
                LinePoint lp;
                lp.position = path[i];
                lp.index = i;
                lp.normal = planeNormal;
                lp.curvature = 0.0;

                if (i < path.size() - 1) {
                    lp.tangent = (path[i + 1] - path[i]).normalized();
                }
                else if (i > 0) {
                    lp.tangent = (path[i] - path[i - 1]).normalized();
                }

                linePoints.push_back(lp);
            }
            boundary2.push_back(linePoints);
        }

        // Compute line attributes
        if (!boundary2.empty()) {
            computeLineAttributes(boundary2);
        }

        return !boundary2.empty();
    }
    //computer feature normal or tangent
    void BoundaryDetector::computeLineAttributes(Epiphyseal& lines)
    {
        if (lines.empty()) return;

        // ============================
        // If two lines exist, synthesize a middle line
        // ============================
        if (lines.size() == 2)
        {
            auto& line1 = lines[0];
            auto& line2 = lines[1];

            if (line1.size() >= 2 && line2.size() >= 2)
            {
                // ---------- Direction alignment ----------
                if ((line1.front().position - line2.front().position).norm() >
                    (line1.front().position - line2.back().position).norm())
                {
                    std::reverse(line2.begin(), line2.end());
                }

                // ---------- Arc length ----------
                auto computeArc = [](const std::vector<LinePoint>& line)
                {
                    std::vector<double> arc(line.size());
                    arc[0] = 0.0;
                    for (size_t i = 1; i < line.size(); ++i)
                    {
                        arc[i] = arc[i - 1] +
                            (line[i].position - line[i - 1].position).norm();
                    }
                    return arc;
                };

                auto arc1 = computeArc(line1);
                auto arc2 = computeArc(line2);

                double len1 = arc1.back();
                double len2 = arc2.back();
                double maxLen = std::max(len1, len2);

                // ---------- Interpolation ----------
                auto interp = [](const std::vector<LinePoint>& line,
                    const std::vector<double>& arc,
                    double s) -> Eigen::Vector3d
                {
                    if (s <= 0) return line.front().position;
                    if (s >= arc.back()) return line.back().position;

                    for (size_t i = 0; i < arc.size() - 1; ++i)
                    {
                        if (s >= arc[i] && s <= arc[i + 1])
                        {
                            double t = (s - arc[i]) / (arc[i + 1] - arc[i]);

                            return ((1 - t) * line[i].position + t * line[i + 1].position).eval();
                        }
                    }

                    return line.back().position;
                };

                // ---------- Uniform sampling (2 mm) ----------
                const double step = 0.001;
                size_t sampleN = static_cast<size_t>(maxLen / step) + 1;

                std::vector<LinePoint> centerLine;

                for (size_t i = 0; i < sampleN; ++i)
                {
                    double s = i * step;

                    Eigen::Vector3d p1 = interp(line1, arc1, s * len1 / maxLen);
                    Eigen::Vector3d p2 = interp(line2, arc2, s * len2 / maxLen);

                    LinePoint mid;
                    mid.position = 0.5 * (p1 + p2);
                    mid.index = i;

                    centerLine.push_back(mid);
                }

                // Append generated center line
                lines.push_back(centerLine);
            }
        }

        // ============================
        // Compute per-point tangent/curvature/normal for all lines
        // ============================
        for (auto& line : lines)
        {
            const size_t n = line.size();
            if (n < 2) continue;

            // ===== tangent =====
            for (size_t i = 0; i < n; ++i)
            {
                Eigen::Vector3d t;

                if (i == 0)
                    t = line[i + 1].position - line[i].position;
                else if (i == n - 1)
                    t = line[i].position - line[i - 1].position;
                else
                    t = line[i + 1].position - line[i - 1].position;

                if (t.norm() > 1e-8) t.normalize();
                line[i].tangent = t;
            }

            // ===== curvature =====
            for (size_t i = 0; i < n; ++i)
            {
                double curvature = 0.0;

                if (i > 0 && i < n - 1)
                {
                    Eigen::Vector3d v1 = line[i].position - line[i - 1].position;
                    Eigen::Vector3d v2 = line[i + 1].position - line[i].position;

                    double len1 = v1.norm();
                    double len2 = v2.norm();

                    if (len1 > 1e-8 && len2 > 1e-8)
                    {
                        double angle = std::acos(
                            std::clamp(v1.normalized().dot(v2.normalized()), -1.0, 1.0)
                        );

                        curvature = angle / (0.5 * (len1 + len2));
                    }
                }

                line[i].curvature = curvature;
            }

            // ===== normal (continuity-stabilized) =====
            Eigen::Vector3d lastNormal(0, 0, 1);

            for (size_t i = 0; i < n; ++i)
            {
                Eigen::Vector3d normal = Eigen::Vector3d::Zero();

                if (i > 0 && i < n - 1)
                {
                    Eigen::Vector3d dt =
                        line[i + 1].tangent - line[i - 1].tangent;

                    if (dt.norm() > 1e-8)
                        normal = dt.normalized();
                }

                if (normal.norm() < 1e-8)
                    normal = lastNormal;

                // Prevent normal flipping (critical for continuity)
                if (normal.dot(lastNormal) < 0)
                    normal = -normal;

                line[i].normal = normal;
                lastNormal = normal;
            }
        }
    }

    //Point generation based on curvature
    bool BoundaryDetector::generateAdaptiveNodes(
        const Epiphyseal& lines,
        Epiphyseal& Pointnode,
        const double epsilon,
        const double min_step
    )
    {
        Pointnode.clear();
        if (lines.empty()) {
            return false;
        }

        constexpr double k_threshold = 1e-3; // fixed straight-segment curvature threshold
        constexpr double k_cornerCosThreshold = 0.996194698; // 5 degrees

        // Iterate over each polyline
        for (const auto& line : lines)
        {
            if (line.size() < 2) continue;

            const auto isSharpCorner = [&](size_t index) {
                if (index == 0 || index + 1 >= line.size()) return false;
                Eigen::Vector3d incoming = line[index].position - line[index - 1].position;
                Eigen::Vector3d outgoing = line[index + 1].position - line[index].position;
                if (incoming.norm() <= 1e-8 || outgoing.norm() <= 1e-8) return false;
                incoming.normalize();
                outgoing.normalize();
                return incoming.dot(outgoing) < k_cornerCosThreshold;
            };

            std::vector<LinePoint> nodes;

            size_t i = 0;

            while (i < line.size())
            {
                // ============================
                // Classify current run as straight or curved
                // ============================
                // A polyline corner must remain a node even when its sampled
                // curvature falls below the absolute threshold (which happens
                // on long straight segments).  Otherwise the two segments are
                // collapsed into one straight run and no cutting plane can be
                // constructed at their junction.
                bool isStraight = line[i].curvature < k_threshold && !isSharpCorner(i);

                size_t start = i;

                // Find maximal contiguous run with same type
                while (i < line.size() &&
                    ((line[i].curvature < k_threshold && !isSharpCorner(i)) == isStraight))
                {
                    ++i;
                }

                size_t end = i - 1;

                // ============================
                // Straight run handling
                // ============================
                if (isStraight)
                {
                    // An AA straight section is represented by its two
                    // bounding nodes. UniformSlicer converts that pair into
                    // one middle cutting plane, so min_step must not split a
                    // straight section into additional planes.
                    const size_t runStart = start > 0 && isSharpCorner(start - 1)
                        ? start - 1 : start;
                    const size_t runEnd = end + 1 < line.size() && isSharpCorner(end + 1)
                        ? end + 1 : end;
                    nodes.push_back(line[runStart]);
                    if (runEnd != runStart) {
                        nodes.push_back(line[runEnd]);
                    }
                }
                // ============================
                // Curved run handling (adaptive sampling)
                // ============================
                else
                {
                    size_t idx = start;

                    nodes.push_back(line[idx]); // run start

                    while (idx < end)
                    {
                        double k = std::max(line[idx].curvature, 1e-6);

                        double step = std::sqrt(8.0 * epsilon / k);

                        if (step < min_step)
                            step = min_step;

                        // ============================
                        // Advance along arc length (distance approximation)
                        // ============================
                        double accum_dist = 0.0;
                        size_t next = idx;

                        while (next + 1 <= end)
                        {
                            double d = (line[next + 1].position - line[next].position).norm();

                            if (accum_dist + d >= step)
                                break;

                            accum_dist += d;
                            ++next;
                        }

                        // Safety guard to avoid infinite loop
                        if (next == idx)
                            ++next;

                        nodes.push_back(line[next]);
                        idx = next;
                    }
                }
            }

            // A shared corner can be appended by both adjacent straight
            // sections (and by its single-point corner run). Keep exactly one
            // node there so the junction produces one cutting plane.
            std::vector<LinePoint> uniqueNodes;
            uniqueNodes.reserve(nodes.size());
            for (const LinePoint& node : nodes) {
                if (uniqueNodes.empty() ||
                    (node.position - uniqueNodes.back().position).norm() > 1e-8) {
                    uniqueNodes.push_back(node);
                }
            }
            if (uniqueNodes.size() >= 2) {
                Pointnode.push_back(std::move(uniqueNodes));
            }
        }

        return !Pointnode.empty();
    }

    // Build new slicing nodes by correspondence across three lines.
    bool BoundaryDetector::buildSlicingNodes(
        const Epiphyseal& lines,        // original dense lines: lines[0], lines[1], lines[2]
        const Epiphyseal& Pointnode,    // adaptive sparse points from side lines
        Epiphyseal& slicingnode,
        const double min_step,
        const int maxSliceCount)
    {
        slicingnode.clear();

        if (lines.size() < 3 || Pointnode.size() < 2)
            return false;

        // Projected envelopes have exact scan-row correspondence. Preserve it:
        // independent arc-length interpolation moves selected corners and pairs
        // side points from different heights.
        bool paired = lines[0].size() == lines[2].size() &&
            lines[1].size() == lines[2].size() && lines[2].size() >= 2;
        for (size_t i = 0; paired && i < lines[2].size(); ++i) {
            paired = (lines[2][i].position - 0.5 *
                (lines[0][i].position + lines[1][i].position)).norm() < 1e-7;
        }
        if (paired) {
            const auto& center = lines[2];
            std::vector<double> arc(center.size(), 0.0);
            for (size_t i = 1; i < center.size(); ++i)
                arc[i] = arc[i - 1] + (center[i].position - center[i - 1].position).norm();
            std::vector<size_t> candidates;
            for (size_t i = 1; i + 1 < center.size(); ++i) candidates.push_back(i);
            const auto turn = [&](size_t i) {
                const Eigen::Vector3d a = center[i].position - center[i - 1].position;
                const Eigen::Vector3d b = center[i + 1].position - center[i].position;
                return 1.0 - std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0);
            };
            std::stable_sort(candidates.begin(), candidates.end(), [&](size_t a, size_t b) {
                const bool ac = turn(a) > 1.0 - 0.996194698;
                const bool bc = turn(b) > 1.0 - 0.996194698;
                if (ac != bc) return ac;
                if (ac && std::abs(turn(a) - turn(b)) > 1e-12) return turn(a) > turn(b);
                return center[a].curvature > center[b].curvature;
            });
            std::vector<size_t> selected{0, center.size() - 1};
            for (size_t i : candidates) {
                bool adaptiveCandidate = false;
                if (Pointnode.size() > 2) {
                    for (const auto& node : Pointnode[2])
                        if ((node.position - center[i].position).norm() < 1e-7) adaptiveCandidate = true;
                }
                if (!adaptiveCandidate && turn(i) <= 1.0 - 0.996194698) continue;
                if (center[i].curvature < 1e-3 && turn(i) < 1.0 - 0.996194698) continue;
                bool separated = true;
                for (size_t j : selected)
                    if (std::abs(arc[i] - arc[j]) < std::max(1e-6, min_step)) separated = false;
                if (!separated) continue;
                selected.push_back(i);
                if (maxSliceCount > 0 && selected.size() >= static_cast<size_t>(maxSliceCount) + 2) break;
            }
            std::sort(selected.begin(), selected.end());
            slicingnode.resize(3);
            for (size_t i : selected)
                for (size_t side = 0; side < 3; ++side) slicingnode[side].push_back(lines[side][i]);
            return true;
        }

        // ============================
        // 1) Use the fitted center line (Pointnode[2]) as the primary
        // reference. It represents the center of the ribbon and produces
        // better-balanced slice placement than either side boundary alone.
        // Keep the side-line choice only as a fallback for incomplete input.
        // ============================
        int ref_id = 0;
        if (Pointnode.size() > 2 && Pointnode[2].size() >= 2) {
            ref_id = 2;
        } else if (Pointnode[1].size() >= Pointnode[0].size()) {
            ref_id = 1;
        }
        std::vector<LinePoint> refSparse = Pointnode[ref_id];

        if (refSparse.size() < 2)
            return false;

        // ============================
        // 2) Remove near-duplicate points in reference line
        // ============================
        const double dedupThreshold = std::max(1e-6, 0.2 * min_step);
        std::vector<LinePoint> cleanedRef;
        cleanedRef.reserve(refSparse.size());
        cleanedRef.push_back(refSparse[0]);

        for (size_t i = 1; i < refSparse.size(); ++i) {
            if ((refSparse[i].position - cleanedRef.back().position).norm() > dedupThreshold) {
                cleanedRef.push_back(refSparse[i]);
            }
        }

        refSparse = cleanedRef;  // use deduplicated reference points

        if (refSparse.size() < 2)
            return false;

        if (maxSliceCount > 0 && refSparse.size() > 2) {
            struct SliceCandidate {
                size_t index;
                bool isCorner;
                double turnSeverity;
                double curvature;
            };

            constexpr double kCornerCosThreshold = 0.996194698; // 5 degrees
            std::vector<SliceCandidate> candidates;
            candidates.reserve(refSparse.size() - 2);

            for (size_t i = 1; i + 1 < refSparse.size(); ++i) {
                Eigen::Vector3d incoming = refSparse[i].position - refSparse[i - 1].position;
                Eigen::Vector3d outgoing = refSparse[i + 1].position - refSparse[i].position;
                double cosine = 1.0;
                if (incoming.norm() > 1e-8 && outgoing.norm() > 1e-8) {
                    incoming.normalize();
                    outgoing.normalize();
                    cosine = std::clamp(incoming.dot(outgoing), -1.0, 1.0);
                }

                candidates.push_back({
                    i,
                    cosine < kCornerCosThreshold,
                    1.0 - cosine,
                    std::max(0.0, refSparse[i].curvature)
                });
            }

            std::stable_sort(candidates.begin(), candidates.end(), [](const SliceCandidate& lhs,
                                                                         const SliceCandidate& rhs) {
                if (lhs.isCorner != rhs.isCorner) return lhs.isCorner > rhs.isCorner;
                if (lhs.isCorner && std::abs(lhs.turnSeverity - rhs.turnSeverity) > 1e-12) {
                    return lhs.turnSeverity > rhs.turnSeverity;
                }
                if (std::abs(lhs.curvature - rhs.curvature) > 1e-12) {
                    return lhs.curvature > rhs.curvature;
                }
                if (std::abs(lhs.turnSeverity - rhs.turnSeverity) > 1e-12) {
                    return lhs.turnSeverity > rhs.turnSeverity;
                }
                return lhs.index < rhs.index;
            });

            const size_t selectedCount = std::min(
                candidates.size(), static_cast<size_t>(maxSliceCount));
            std::vector<size_t> selectedIndices;
            selectedIndices.reserve(selectedCount + 2);
            selectedIndices.push_back(0);
            for (size_t i = 0; i < selectedCount; ++i) {
                selectedIndices.push_back(candidates[i].index);
            }
            selectedIndices.push_back(refSparse.size() - 1);
            std::sort(selectedIndices.begin(), selectedIndices.end());

            std::vector<LinePoint> limitedRef;
            limitedRef.reserve(selectedIndices.size());
            for (const size_t index : selectedIndices) {
                limitedRef.push_back(refSparse[index]);
            }
            refSparse.swap(limitedRef);
        }

        // ============================
        // 3) Fetch three dense lines and align their direction
        // ============================
        std::vector<LinePoint> line1 = lines[0];
        std::vector<LinePoint> line2 = lines[1];
        std::vector<LinePoint> line3 = lines[2];

        auto removeNearDuplicatePoints = [](std::vector<LinePoint>& line)
        {
            if (line.empty()) return;
            std::vector<LinePoint> filtered;
            filtered.reserve(line.size());
            filtered.push_back(line.front());
            for (size_t i = 1; i < line.size(); ++i) {
                if ((line[i].position - filtered.back().position).norm() > 1e-8) {
                    filtered.push_back(line[i]);
                }
            }
            line.swap(filtered);
        };

        auto alignLineDirection = [](std::vector<LinePoint>& line, const std::vector<LinePoint>& ref)
        {
            if (line.empty() || ref.empty()) return;
            const double dFront = (line.front().position - ref.front().position).norm();
            const double dBack = (line.back().position - ref.front().position).norm();
            if (dBack < dFront) {
                std::reverse(line.begin(), line.end());
            }
        };

        removeNearDuplicatePoints(line1);
        removeNearDuplicatePoints(line2);
        removeNearDuplicatePoints(line3);

        if (line1.size() < 2 || line2.size() < 2 || line3.size() < 2) {
            return false;
        }

        // Align all dense lines to the deduplicated reference orientation
        alignLineDirection(line1, refSparse);
        alignLineDirection(line2, refSparse);
        alignLineDirection(line3, refSparse);

        // ============================
        // 4) Compute arc length for each dense line
        // ============================
        auto computeArc = [](const std::vector<LinePoint>& line) -> std::vector<double>
        {
            std::vector<double> arc(line.size());
            arc[0] = 0.0;
            for (size_t i = 1; i < line.size(); ++i) {
                arc[i] = arc[i - 1] + (line[i].position - line[i - 1].position).norm();
            }
            return arc;
        };

        auto arc1 = computeArc(line1);
        auto arc2 = computeArc(line2);
        auto arc3 = computeArc(line3);

        double len1 = arc1.back();
        double len2 = arc2.back();
        double len3 = arc3.back();

        // Arc length for deduplicated sparse reference line
        auto arcRef = computeArc(refSparse);
        double lenRef = arcRef.back();

        // ============================
        // 5) Normalize sparse reference arc parameter into [0, 1]
        // ============================
        std::vector<double> normalizedParams(refSparse.size());
        for (size_t i = 0; i < refSparse.size(); ++i) {
            normalizedParams[i] = arcRef[i] / lenRef;
        }

        // ============================
        // 6) Interpolation helper on dense line by arc length
        // ============================
        auto interpPosition = [](const std::vector<LinePoint>& line,
            const std::vector<double>& arc,
            double s) -> Eigen::Vector3d
        {
            if (s <= 0) return line.front().position;
            if (s >= arc.back()) return line.back().position;

            for (size_t i = 0; i < arc.size() - 1; ++i) {
                if (s >= arc[i] && s <= arc[i + 1]) {
                    double segLen = arc[i + 1] - arc[i];
                    if (segLen < 1e-10) {
                        return line[i].position;
                    }
                    double t = (s - arc[i]) / segLen;
                    return (1.0 - t) * line[i].position + t * line[i + 1].position;
                }
            }
            return line.back().position;
        };

        // ============================
        // 7) Map normalized parameters to all three lines
        // ============================
        std::vector<Eigen::Vector3d> positions1, positions2, positions3;
        positions1.reserve(refSparse.size());
        positions2.reserve(refSparse.size());
        positions3.reserve(refSparse.size());

        for (size_t i = 0; i < refSparse.size(); ++i) {
            double t = normalizedParams[i];  // normalized [0, 1]

            positions1.push_back(interpPosition(line1, arc1, t * len1));
            positions2.push_back(interpPosition(line2, arc2, t * len2));
            positions3.push_back(interpPosition(line3, arc3, t * len3));
        }

        // ============================
        // 8) Recover geometric attributes from nearest dense points
        // ============================
        auto findNearestAttributes = [](const Eigen::Vector3d& pos,
            const std::vector<LinePoint>& denseLine) -> LinePoint
        {
            double minDist = std::numeric_limits<double>::max();
            size_t minIdx = 0;

            for (size_t i = 0; i < denseLine.size(); ++i) {
                double dist = (pos - denseLine[i].position).norm();
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = i;
                }
            }

            // Copy nearest attributes
            LinePoint lp = denseLine[minIdx];
            lp.position = pos;  // overwrite with mapped position
            return lp;
        };

        // ============================
        // 9) Build final output with one-to-one correspondence
        // ============================
        std::vector<LinePoint> slice1, slice2, slice3;
        slice1.reserve(positions1.size());
        slice2.reserve(positions2.size());
        slice3.reserve(positions3.size());

        for (size_t i = 0; i < positions1.size(); ++i) {
            LinePoint lp1 = findNearestAttributes(positions1[i], line1);
            lp1.index = i;
            slice1.push_back(lp1);
        }

        for (size_t i = 0; i < positions2.size(); ++i) {
            LinePoint lp2 = findNearestAttributes(positions2[i], line2);
            lp2.index = i;
            slice2.push_back(lp2);
        }

        for (size_t i = 0; i < positions3.size(); ++i) {
            LinePoint lp3 = findNearestAttributes(positions3[i], line3);
            lp3.index = i;
            slice3.push_back(lp3);
        }

        // ============================
        // 10) Output result
        // ============================
        slicingnode.push_back(slice1);
        slicingnode.push_back(slice2);
        slicingnode.push_back(slice3);
        return !slicingnode.empty();
    }

    // Newell method for the original boundary-loop normal calculation.
    static Eigen::Vector3d computePolygonNormal(const std::vector<Eigen::Vector3d>& pts)
    {
        Eigen::Vector3d n(0, 0, 0);
        if (pts.size() < 3) {
            return Eigen::Vector3d(0, 0, 1);
        }
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& p0 = pts[i];
            const auto& p1 = pts[(i + 1) % pts.size()];
            n.x() += (p0.y() - p1.y()) * (p0.z() + p1.z());
            n.y() += (p0.z() - p1.z()) * (p0.x() + p1.x());
            n.z() += (p0.x() - p1.x()) * (p0.y() + p1.y());
        }
        const double len = n.norm();
        if (len < 1e-8) {
            return Eigen::Vector3d(0, 0, 1);
        }
        return n / len;
    }

    // Original algorithm entry, with its old rendering-data output removed.
    bool BoundaryDetector::detectBoundaries(const ModelData& modelData, BoundaryData& boundaryData, Epiphyseal& intersectionPoints, BoundaryTopologyDiagnostics* topology)
    {
        if(topology) *topology = {};
        // [ALGORITHM] Detect boundaries from STL model
        // Input: modelData - STL model data
        // Output: boundaryData - detected upper/lower boundaries

        if (STLProcessor::getTriangleCount(modelData.mainModelData_) == 0) {
            return false;
        }

        try {
            // Trace complete free-edge loops first. End labels and reference
            // plane selection use rim geometry, never a world-Z partition.
            const auto edgeCount = getEdgeCountMap(modelData);
            std::map<Eigen::Vector3d, std::vector<Eigen::Vector3d>, Vector3dLexicographicalCompare> adjacency;
            std::set<std::pair<Eigen::Vector3d, Eigen::Vector3d>, PairOfVector3dCompare> unvisitedEdges;
            const auto edgeKey = [](Eigen::Vector3d a, Eigen::Vector3d b) {
                Vector3dLexicographicalCompare compare;
                if (compare(b, a)) std::swap(a, b);
                return std::make_pair(a, b);
            };
            for (const auto& [edge, count] : edgeCount) {
                if(topology && count>2) ++topology->nonManifoldEdges;
                if (count != 1) continue;
                adjacency[edge.first].push_back(edge.second);
                adjacency[edge.second].push_back(edge.first);
                unvisitedEdges.insert(edgeKey(edge.first, edge.second));
            }

            if(topology) for(const auto& entry:adjacency) {
                if(entry.second.size()>2) ++topology->branchVertices;
                if(entry.second.size()<2) ++topology->openVertices;
            }
            std::vector<std::vector<Eigen::Vector3d>> boundaryLoops;
            while (!unvisitedEdges.empty()) {
                const Eigen::Vector3d start = unvisitedEdges.begin()->first;
                Eigen::Vector3d current = start;
                std::vector<Eigen::Vector3d> loop;

                while (true) {
                    loop.push_back(current);
                    const auto adjacencyIt = adjacency.find(current);
                    if (adjacencyIt == adjacency.end()) break;

                    bool foundNext = false;
                    Eigen::Vector3d next;
                    for (const auto& candidate : adjacencyIt->second) {
                        const auto key = edgeKey(current, candidate);
                        if (unvisitedEdges.find(key) != unvisitedEdges.end()) {
                            next = candidate;
                            unvisitedEdges.erase(key);
                            foundNext = true;
                            break;
                        }
                    }
                    if (!foundNext || next.isApprox(start, 1e-9)) break;
                    current = next;
                    if (loop.size() > edgeCount.size() + 1) break;
                }

                if (loop.size() >= 3) {
                    boundaryLoops.push_back(std::move(loop));
                }
            }

            if(topology) {
                topology->loopCount=boundaryLoops.size();
                if(!topology->usable()) return false;
            }
            if (boundaryLoops.size() >= 2) {
                const auto loopCenter = [](const std::vector<Eigen::Vector3d>& loop) {
                    Eigen::Vector3d center = Eigen::Vector3d::Zero();
                    for (const auto& point : loop) center += point;
                    return center / static_cast<double>(loop.size());
                };
                const auto perimeter = [](const std::vector<Eigen::Vector3d>& loop) {
                    double length = 0.0;
                    for(size_t i = 0; i < loop.size(); ++i)
                        length += (loop[(i + 1) % loop.size()] - loop[i]).norm();
                    return length;
                };
                // Large/small ends are geometric labels, independent of Z.
                // Equal-size ends retain their traced order.
                // Baffle callers require two unordered rims; legacy end labeling is not needed.
                if(!topology) std::stable_sort(boundaryLoops.begin(), boundaryLoops.end(),
                    [&](const auto& lhs, const auto& rhs) { return perimeter(lhs) > perimeter(rhs); });

                const auto fillBoundary = [&](const std::vector<Eigen::Vector3d>& loop,
                    BoundaryLine& boundary, const Eigen::Vector3f& color,
                    const char* name, BoundaryLineType type) {
                    const Eigen::Vector3d center = loopCenter(loop);
                    boundary.points.clear();
                    boundary.points.reserve(loop.size());
                    for (size_t i = 0; i < loop.size(); ++i) {
                        BasicPoint point;
                        point.position = loop[i];
                        point.index = i;
                        boundary.points.push_back(point);
                    }
                    boundary.center = center;
                    boundary.z_level = center.z();
                    boundary.normal = computePolygonNormal(loop);
                    boundary.color = color;
                    boundary.name = name;
                    boundary.Type = type;
                    Eigen::Vector3d circleCenter;
                    if(fitBoundaryCircle(boundary, circleCenter)) {
                        boundary.center = circleCenter;
                        boundary.z_level = circleCenter.z();
                    }
                };

                fillBoundary(boundaryLoops.front(), boundaryData.lowerBoundary_,
                    Eigen::Vector3f(0.0f, 0.0f, 1.0f), "Lower Boundary",
                    BoundaryLineType::LowerBoundary);
                fillBoundary(boundaryLoops.back(), boundaryData.upperBoundary_,
                    Eigen::Vector3f(1.0f, 0.0f, 0.0f), "Upper Boundary",
                    BoundaryLineType::UpperBoundary);
            }

            if(topology) return !boundaryData.upperBoundary_.points.empty() && !boundaryData.lowerBoundary_.points.empty();
            if (!boundaryData.upperBoundary_.points.empty() && !boundaryData.lowerBoundary_.points.empty())
            {
                // Clear previous intersection/slicing artifacts before rebuilding.
                intersectionPoints.clear();
                boundaryData.projectedContourLines_.clear();
                boundaryData.phaseReferenceLine_.clear();
                boundaryData.phaseReferencePlaneNormal_.setZero();
                boundaryData.referenceLines_.clear();
                boundaryData.LinePoints_.clear();
                boundaryData.Mainslicingnode.clear();

                // The legacy intersection pass is retained solely to fit the
                // mirror-plane normal.  Its traced curves are neither drawn
                // nor used as slicing references.
                calculatePlaneIntersection(
                    modelData,
                    boundaryData.upperBoundary_,
                    boundaryData.lowerBoundary_,
                    intersectionPoints,
                    &boundaryData.phaseReferencePlaneNormal_
                );

                // Projection only needs the fitted plane. Tracing a mesh-plane
                // intersection can fail on coplanar edges; that must not suppress
                // the independently computed projected envelope.
                const bool projectedContourSuccess = appendProjectedOuterContour(
                    modelData, boundaryData.lowerBoundary_.center,
                    boundaryData.phaseReferencePlaneNormal_, boundaryData.lowerBoundary_,
                    boundaryData.upperBoundary_, boundaryData.projectedContourLines_,
                    boundaryData.referenceLines_);

                if (projectedContourSuccess && boundaryData.projectedContourLines_.size() == 3) {
                    // All three curves must share lower-to-upper orientation so
                    // their indices remain paired when building cutting planes.
                    const auto& centerLine = boundaryData.projectedContourLines_[2];
                    if (centerLine.size() >= 2 &&
                        (centerLine.front().position - boundaryData.lowerBoundary_.center).norm() >
                        (centerLine.back().position - boundaryData.lowerBoundary_.center).norm()) {
                        for (auto& line : boundaryData.projectedContourLines_) {
                            std::reverse(line.begin(), line.end());
                        }
                    }
                    computeLineAttributes(boundaryData.projectedContourLines_);

                    // The projected center drives slice placement only. For
                    // trajectory phase select the shorter *raw* direct
                    // mesh/mirror-plane intersection. It is the user's
                    // cutting line, so sample 0 on every contour lies there.
                    const size_t directLineCount = std::min<size_t>(2, intersectionPoints.size());
                    double shortestLength = std::numeric_limits<double>::infinity();
                    size_t shortestIndex = 0;
                    for (size_t lineIndex = 0; lineIndex < directLineCount; ++lineIndex) {
                        const auto& line = intersectionPoints[lineIndex];
                        if (line.size() < 2) continue;
                        double length = 0.0;
                        for (size_t pointIndex = 1; pointIndex < line.size(); ++pointIndex) {
                            length += (line[pointIndex].position - line[pointIndex - 1].position).norm();
                        }
                        if (length < shortestLength) {
                            shortestLength = length;
                            shortestIndex = lineIndex;
                        }
                    }
                    boundaryData.phaseReferenceLine_ = std::isfinite(shortestLength)
                        ? intersectionPoints[shortestIndex]
                        : boundaryData.projectedContourLines_[2];

                    // Use the projected contour center line for corner-first,
                    // curvature-second cutting-plane placement.
                    const bool adaptiveOk =
                        generateAdaptiveNodes(
                            boundaryData.projectedContourLines_,
                            boundaryData.LinePoints_,
                            boundaryData.epsilon_,
                            boundaryData.minStep_);
                    const bool slicingNodeOk =
                        buildSlicingNodes(
                            boundaryData.projectedContourLines_,
                            boundaryData.LinePoints_,
                            boundaryData.Mainslicingnode,
                            boundaryData.minStep_);
                    if (!adaptiveOk || !slicingNodeOk) {
                        boundaryData.Mainslicingnode.clear();
                    }
                }
            }

            return !boundaryData.upperBoundary_.points.empty() || !boundaryData.lowerBoundary_.points.empty();
        }
        catch (const std::exception& e) {
            std::cerr << "Error detecting boundaries: " << e.what() << std::endl;
            return false;
        }
    }

    std::vector<Eigen::Vector3d> BoundaryDetector::extractBoundaryEdges(const ModelData& modelData, bool isUpper, double z_min, double z_max)
    {
        // [ALGORITHM] Extract boundary edges from model and build ordered boundary sequence
        // Input: modelData - STL model
        //        isUpper - extract upper (true) or lower (false) boundary
        //        z_min, z_max - Z range for boundary classification
        // Output: vector<Vector3d> - ordered boundary edge points (using edge connectivity)

        std::vector<Eigen::Vector3d> orderedPoints;
        if (STLProcessor::getTriangleCount(modelData.mainModelData_) == 0) {
            return orderedPoints;
        }

        // Use midZ to classify free edges into upper/lower boundaries
        const double midZ = 0.5 * (z_max + z_min);
        constexpr double epsilon = 1e-6;

        // Count undirected edges; boundary edges appear exactly once
        std::map<std::pair<Eigen::Vector3d, Eigen::Vector3d>, int, PairOfVector3dCompare> edgeCount;
        for (size_t triangleIndex = 0; triangleIndex < STLProcessor::getTriangleCount(modelData.mainModelData_); ++triangleIndex) {
            Eigen::Vector3d vertices[3];
            STLProcessor::getTriangleVertices(modelData.mainModelData_, triangleIndex, vertices);

            for (int i = 0; i < 3; ++i) {
                Eigen::Vector3d v1 = vertices[i];
                Eigen::Vector3d v2 = vertices[(i + 1) % 3];
                Vector3dLexicographicalCompare comp;
                if (comp(v2, v1)) std::swap(v1, v2);
                edgeCount[std::make_pair(v1, v2)]++;
            }
        }

        // Keep only boundary edges (count==1) on the correct Z side of midZ
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> boundaryEdges;
        for (const auto& kv : edgeCount) {
            if (kv.second != 1) continue;
            const auto& e = kv.first;
            const bool upperSide = (e.first.z() > midZ) && (e.second.z() > midZ);
            const bool lowerSide = (e.first.z() < midZ) && (e.second.z() < midZ);
            if ((isUpper && upperSide) || (!isUpper && lowerSide)) {
                boundaryEdges.push_back(e);
            }
        }

        if (boundaryEdges.empty()) {
            return orderedPoints;
        }

        // Build adjacency from edge endpoints
        std::map<Eigen::Vector3d, std::vector<Eigen::Vector3d>, Vector3dLexicographicalCompare> adjacencyMap;
        for (const auto& edge : boundaryEdges) {
            adjacencyMap[edge.first].push_back(edge.second);
            adjacencyMap[edge.second].push_back(edge.first);
        }

        if (adjacencyMap.empty()) {
            return orderedPoints;
        }

        // Prefer a degree-1 vertex as polyline start; else arbitrary
        Eigen::Vector3d startPoint;
        bool foundStart = false;
        for (const auto& kv : adjacencyMap) {
            if (kv.second.size() == 1) {
                startPoint = kv.first;
                foundStart = true;
                break;
            }
        }
        if (!foundStart) {
            startPoint = adjacencyMap.begin()->first;
        }

        std::set<std::pair<Eigen::Vector3d, Eigen::Vector3d>, PairOfVector3dCompare> visitedEdges;

        auto isEdgeVisited = [&](const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) -> bool {
            Vector3dLexicographicalCompare comp;
            Eigen::Vector3d e1 = v1, e2 = v2;
            if (comp(e2, e1)) std::swap(e1, e2);
            return visitedEdges.find(std::make_pair(e1, e2)) != visitedEdges.end();
        };

        auto markEdgeVisited = [&](const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
            Vector3dLexicographicalCompare comp;
            Eigen::Vector3d e1 = v1, e2 = v2;
            if (comp(e2, e1)) std::swap(e1, e2);
            visitedEdges.insert(std::make_pair(e1, e2));
        };

        Eigen::Vector3d currentPoint = startPoint;
        orderedPoints.push_back(currentPoint);

        while (true) {
            auto it = adjacencyMap.find(currentPoint);
            if (it == adjacencyMap.end()) {
                break;
            }

            Eigen::Vector3d nextPoint;
            bool foundNext = false;
            for (const auto& neighbor : it->second) {
                if (!isEdgeVisited(currentPoint, neighbor)) {
                    nextPoint = neighbor;
                    foundNext = true;
                    markEdgeVisited(currentPoint, neighbor);
                    break;
                }
            }

            if (!foundNext) {
                break;
            }

            currentPoint = nextPoint;
            orderedPoints.push_back(currentPoint);

            if (orderedPoints.size() > boundaryEdges.size() * 2) {
                break;
            }
        }

        return orderedPoints;
    }



    void BoundaryDetector::computeOffsetBoundaries(BoundaryData& boundaryData, double upperOffset, double lowerOffset)
    {
        // [ALGORITHM] Compute offset boundaries
        // Input: boundaryData.upperBoundary_, boundaryData.lowerBoundary_ - original boundaries
        //        upperOffset, lowerOffset - offset distances
        // Output: boundaryData.upperBoundaryOffset_, boundaryData.lowerBoundaryOffset_ - offset boundaries

        // Clear old offset boundary data
        boundaryData.upperBoundaryOffset_.points.clear();
        boundaryData.lowerBoundaryOffset_.points.clear();

        if (upperOffset != 0.0 && !boundaryData.upperBoundary_.points.empty()) {
            boundaryData.upperBoundaryOffset_ = computeOffsetForBoundary(boundaryData.upperBoundary_, upperOffset);
            boundaryData.upperBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f); // Yellow
            boundaryData.upperBoundaryOffset_.Type = BoundaryLineType::UpperBoundaryOffset;
        }
        if (lowerOffset != 0.0 && !boundaryData.lowerBoundary_.points.empty()) {
            boundaryData.lowerBoundaryOffset_ = computeOffsetForBoundary(boundaryData.lowerBoundary_, lowerOffset);
            boundaryData.lowerBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f); // Yellow
            boundaryData.lowerBoundaryOffset_.Type = BoundaryLineType::LowerBoundaryOffset;
        }

        // boundaryLines_ is filled elsewhere (e.g. BurTrajPlan::generateMATLABTrajectory,
        // TrajectoryGenerator::generateReferenceLinesFromSliceResult).
    }

    BoundaryLine BoundaryDetector::computeOffsetForBoundary(const BoundaryLine& boundary, double offset)
    {
        BoundaryLine offsetBoundary;
        if (boundary.points.size() < 2) {
            return offsetBoundary;
        }

        // Offset along Z by shifting each point's Z
        offsetBoundary.points.reserve(boundary.points.size());

        for (size_t i = 0; i < boundary.points.size(); ++i) {
            const auto& bp_curr = boundary.points[i];

            BasicPoint offsetPoint;
            offsetPoint.position = Eigen::Vector3d(
                bp_curr.position.x(),
                bp_curr.position.y(),
                bp_curr.position.z() + offset
            );

            offsetPoint.angle = bp_curr.angle;
            offsetPoint.index = bp_curr.index;

            offsetBoundary.points.push_back(offsetPoint);
        }

        offsetBoundary.center = boundary.center;
        offsetBoundary.center.z() += offset;

        offsetBoundary.z_level = boundary.z_level + offset;

        offsetBoundary.normal = boundary.normal;

        offsetBoundary.color = boundary.color;
        offsetBoundary.name = boundary.name;

        return offsetBoundary;
    }

    void BoundaryDetector::clear(BoundaryData& boundaryData)
    {
        // [ALGORITHM] Clear boundary data
        // Input/Output: boundaryData - boundary data to clear

        // Clear original boundary data
        boundaryData.upperBoundary_.points.clear();
        boundaryData.upperBoundary_.center = Eigen::Vector3d::Zero();
        boundaryData.upperBoundary_.z_level = 0.0;
        boundaryData.upperBoundary_.color = Eigen::Vector3f(1.0f, 0.2f, 0.2f);
        boundaryData.upperBoundary_.name = "Upper Boundary";
        boundaryData.upperBoundary_.Type = BoundaryLineType::UpperBoundary;

        boundaryData.lowerBoundary_.points.clear();
        boundaryData.lowerBoundary_.center = Eigen::Vector3d::Zero();
        boundaryData.lowerBoundary_.z_level = 0.0;
        boundaryData.lowerBoundary_.color = Eigen::Vector3f(0.2f, 0.2f, 1.0f);
        boundaryData.lowerBoundary_.name = "Lower Boundary";
        boundaryData.lowerBoundary_.Type = BoundaryLineType::LowerBoundary;

        // Clear offset boundary data
        boundaryData.upperBoundaryOffset_.points.clear();
        boundaryData.upperBoundaryOffset_.center = Eigen::Vector3d::Zero();
        boundaryData.upperBoundaryOffset_.z_level = 0.0;
        boundaryData.upperBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        boundaryData.upperBoundaryOffset_.name = "Upper Boundary Offset";
        boundaryData.upperBoundaryOffset_.Type = BoundaryLineType::UpperBoundaryOffset;

        boundaryData.lowerBoundaryOffset_.points.clear();
        boundaryData.lowerBoundaryOffset_.center = Eigen::Vector3d::Zero();
        boundaryData.lowerBoundaryOffset_.z_level = 0.0;
        boundaryData.lowerBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        boundaryData.lowerBoundaryOffset_.name = "Lower Boundary Offset";
        boundaryData.lowerBoundaryOffset_.Type = BoundaryLineType::LowerBoundaryOffset;

        boundaryData.z_max_boundary_ = 0.0;
        boundaryData.z_min_boundary_ = 0.0;
        boundaryData.boundaryLines_.clear();
        boundaryData.intersectionPoints_.clear();
        boundaryData.projectedContourLines_.clear();
        boundaryData.phaseReferenceLine_.clear();
        boundaryData.phaseReferencePlaneNormal_.setZero();
        boundaryData.referenceLines_.clear();
        boundaryData.LinePoints_.clear();
        boundaryData.Mainslicingnode.clear();

        // Note: No longer clean up render models, managed by RenderData
    }

    void BoundaryDetector::clearOffsetBoundaries(BoundaryData& boundaryData)
    {
        // [ALGORITHM] Clear only offset boundary data, preserve original boundaries
        // Input/Output: boundaryData - boundary data to clear offsets from

        boundaryData.upperBoundaryOffset_.points.clear();
        boundaryData.upperBoundaryOffset_.center = Eigen::Vector3d::Zero();
        boundaryData.upperBoundaryOffset_.z_level = 0.0;
        boundaryData.upperBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        boundaryData.upperBoundaryOffset_.name = "Upper Boundary Offset";
        boundaryData.upperBoundaryOffset_.Type = BoundaryLineType::UpperBoundaryOffset;

        boundaryData.lowerBoundaryOffset_.points.clear();
        boundaryData.lowerBoundaryOffset_.center = Eigen::Vector3d::Zero();
        boundaryData.lowerBoundaryOffset_.z_level = 0.0;
        boundaryData.lowerBoundaryOffset_.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        boundaryData.lowerBoundaryOffset_.name = "Lower Boundary Offset";
        boundaryData.lowerBoundaryOffset_.Type = BoundaryLineType::LowerBoundaryOffset;

        boundaryData.boundaryLines_.clear();

        // Note: No longer clean up render models, managed by RenderData
    }

}
