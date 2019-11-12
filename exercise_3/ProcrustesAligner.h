#pragma once

#include "SimpleMesh.h"

class ProcrustesAligner {
public:
    Matrix4f estimatePose(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints) {
        ASSERT(sourcePoints.size() == targetPoints.size() &&
               "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

        // We estimate the pose between source and target points using Procrustes algorithm.
        // Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
        // from source points to target points.

        auto sourceMean = computeMean(sourcePoints);
        auto targetMean = computeMean(targetPoints);

        Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
        Vector3f translation = computeTranslation(sourceMean, targetMean);

        // TODO: Compute the transformation matrix by using the computed rotation and translation.
        // Note: The final translation is not equal to the translation of the means. Refer to the exercise sheet for more details.
        // You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
        Matrix4f estimatedPose = Matrix4f::Identity();

        // -R * xMean + t + xMean = -R * (yMean - t) + (yMean - xMean) + xMean = R * t + R * yMean + yMean
        estimatedPose.block(0, 0, 3, 3) = rotation;
        estimatedPose.block(0, 3, 3, 1) = rotation * translation - rotation * targetMean + targetMean;

        return estimatedPose;
    }

private:
    Vector3f computeMean(const std::vector<Vector3f> &points) {
        // TODO: Compute the mean of input points.
        Vector3f mean = Vector3f::Zero();

        for (unsigned int i = 0; i < points.size(); i++) mean += (points[i] / points.size());

        return mean;
    }

    Matrix3f estimateRotation(const std::vector<Vector3f> &sourcePoints, const Vector3f &sourceMean,
                              const std::vector<Vector3f> &targetPoints, const Vector3f &targetMean) {
        // TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
        // To compute the singular value decomposition you can use JacobiSVD() from Eigen.
        // Important: The covariance matrices should contain mean-centered source/target points.
        Matrix3f rotation = Matrix3f::Identity();
        MatrixXf source(3, sourcePoints.size());
        MatrixXf target(3, targetPoints.size());

        unsigned int max_iter = std::max(sourcePoints.size(), targetPoints.size());

        for (unsigned int i = 0; i < max_iter; i++) {
            if (i < sourcePoints.size()){
                Vector3f p = sourcePoints[i] - sourceMean; // mean-centered
                source(0, i) = p.x();
                source(1, i) = p.y();
                source(2, i) = p.z();
            }
            if (i < targetPoints.size()){
                Vector3f p = targetPoints[i] - targetMean; // Mean-centered
                target(0, i) = p.x();
                target(1, i) = p.y();
                target(2, i) = p.z();
            }

        }

        JacobiSVD<MatrixXf> svd(target * source.transpose(), ComputeFullV | ComputeFullU);
        rotation = svd.matrixU() * svd.matrixV().transpose();

        return rotation;
    }

    Vector3f computeTranslation(const Vector3f &sourceMean, const Vector3f &targetMean) {
        // TODO: Compute the translation vector from source to target points.
        Vector3f translation = Vector3f::Zero();

        // t = yMean - xMean
        translation = targetMean - sourceMean;

        return translation;
    }
};