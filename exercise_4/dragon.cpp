#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction {
    RegistrationCostFunction(const Point2D &point1_, const Point2D &point2_, const Weight &weight_)
            : p(point1_), q(point2_), w(weight_) {
    }

    template<typename T>
    bool operator()(const T *const deg, const T *const tx, const T *const ty, T *residual) const {
        residual[0] = T(w.w) * (pow(T(p.x) * cos(deg[0]) - T(p.y) * sin(deg[0]) + tx[0] - T(q.x), 2) +
                                    pow(T(p.x) * sin(deg[0]) + T(p.y) * cos(deg[0]) + ty[0] - T(q.y), 2));

        return true;
    }

private:
    const Point2D p;
    const Point2D q;
    const Weight w;

};


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // TODO: Read data points and the weights. Define the parameters of the problem
    const std::string file_path_1 = "../data/points_dragon_1.txt";
    const std::string file_path_2 = "../data/points_dragon_2.txt";
    const std::string file_path_weights = "../data/weights_dragon.txt";

    const auto points1 = read_points_from_file<Point2D>(file_path_1);
    const auto points2 = read_points_from_file<Point2D>(file_path_2);
    const auto weights = read_points_from_file<Weight>(file_path_weights);

    ceres::Problem problem;

    // TODO: For each weighted correspondence create one residual block

    const double deg_initial = 45.0;
    const double tx_initial = 1.0;
    const double ty_initial = 1.0;

    double deg = deg_initial;
    double tx = tx_initial;
    double ty = ty_initial;

    for (int i = 0; i < points1.size(); ++i) {
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
                        new RegistrationCostFunction(points1[i], points2[i], weights[i])),
                nullptr, &deg, &tx, &ty
        );
    }


    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // TODO: Output the final values of the translation and rotation (in degree)
    std::cout << "Initial deg: " << deg_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
    std::cout << "Final deg: " << deg << "\ttx: " << tx << "\tty: " << ty << std::endl;

    system("pause");
    return 0;
}