#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include "cubic_spline.hpp"
#include "lbfgs.hpp"
#include "sdqp.hpp"
#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace path_smoother
{

    class PathSmoother
    {
    private:
        cubic_spline::CubicSpline cubSpline;

        int pieceN;
        Eigen::Matrix2Xd diskObstacle;
        double penaltyWeight;
        Eigen::Vector2d headP;
        Eigen::Vector2d tailP;
        Eigen::Matrix2Xd points;
        Eigen::Matrix2Xd gradByPoints;

        lbfgs::lbfgs_parameter_t lbfgs_params;

    private:
        static inline double costFunction(void *ptr,
                                          const Eigen::VectorXd &x,
                                          Eigen::VectorXd &g)
        {
            PathSmoother &obj = *(PathSmoother *)ptr;
            const int N = obj.pieceN;
            Eigen::Map<const Eigen::Matrix2Xd> innerP(x.data(), 2, N - 1);
            Eigen::Map<Eigen::Matrix2Xd> gradInnerP(g.data(), 2, N - 1);

            double cost;
            obj.cubSpline.setInnerPoints(innerP);
            obj.cubSpline.getStretchEnergy(cost);
            obj.cubSpline.getGrad(gradInnerP);

            // We use nonsmooth cost formed by potential + energy here 
            // to test the applicability of our solver.
//            for (int i = 0; i < N - 1; i++)
//            {
//                Eigen::Vector2d delta;
//                //innerP.col(i) - obj.diskObstacle.head<2>();
//                const double signdist = obj.calDistance(innerP.col(i), delta, obj.diskObstacle);
//                const double dist = delta.norm() + DBL_EPSILON;
//                if (signdist != 0.0)
//                {
//                    cost += -signdist * obj.penaltyWeight;
//                    gradInnerP.col(i) += -delta / dist * obj.penaltyWeight;
//                }
//            }

            return cost;
        }

    public:
        inline bool setup(const Eigen::Vector2d &initialP,
                          const Eigen::Vector2d &terminalP,
                          const int &pieceNum,
                          const Eigen::Matrix2Xd &diskObs,
                          const double penaWeight)
        {
            pieceN = pieceNum;
            diskObstacle = diskObs;
            penaltyWeight = penaWeight;
            headP = initialP;
            tailP = terminalP;

            cubSpline.setConditions(headP, tailP, pieceN);

            points.resize(2, pieceN - 1);
            gradByPoints.resize(2, pieceN - 1);

            return true;
        }

        inline double optimize(CubicCurve &curve,
                               const Eigen::Matrix2Xd &iniInPs,
                               const double &relCostTol)
        {
            Eigen::VectorXd x(pieceN * 2 - 2);
            Eigen::Map<Eigen::Matrix2Xd> innerP(x.data(), 2, pieceN - 1);
            innerP = iniInPs;

            double minCost;
            lbfgs_params.mem_size = 64;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCost,
                                            &PathSmoother::costFunction,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                cubSpline.setInnerPoints(innerP);
                cubSpline.getCurve(curve);
            }
            else
            {
                curve.clear();
                minCost = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCost;
        }

        inline double calDistance(const Eigen::Matrix2Xd innerPoint, Eigen::Vector2d &minDistance, const Eigen::Matrix2Xd conv)
        {
//            int m = conv.cols();
//            Eigen::Matrix<double, 2, 2> Q;
//            Eigen::Matrix<double, 2, 1> c;
//            Eigen::Matrix<double, 2, 1> x;        // decision variables
//            Eigen::Matrix<double, -1, 3> A(m, 2); // constraint matrix
//            Eigen::VectorXd b(m);                 // constraint bound
//            Q << 1.0, 0, 0, 1.0;
//            c << 0, 0;
//            for(int i = 0; i<m; i++)
//            {
//                A(i,0) = conv(0, i) - innerPoint(0,0);
//                A(i,1) = conv(1, i) - innerPoint(1,0);
//                b(i) = 1;
//            }
//
//            double minobj = sdqp::sdqp<2>(Q, c, A, b, x);
//
//            minDistance = x * (1/(x.transpose().dot(x)));
//            double bias = minDistance.dot(innerPoint);
//            double signdist = 0.0;
//            for(int i = 0; i<m; i++)
//            {
//                if((minDistance(0) *  conv(0, i) + minDistance(1) *  conv(1, i)) > bias)
//                {
//                    signdist = minDistance.norm() + DBL_EPSILON;
//                    return signdist;
//                }
//            }
            return 0;

        }
    };

}

#endif
