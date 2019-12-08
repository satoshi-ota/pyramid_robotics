#ifndef PYRAMID_CONTROL_OPTIMIZER_H
#define PYRAMID_CONTROL_OPTIMIZER_H

#include <Eigen/Core>

#include <ros/ros.h>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

namespace ifopt
{
using Eigen::Vector2d;


class ExVariables : public VariableSet
{
public:
    ExVariables() : ExVariables("var_set1") {};
    ExVariables(const std::string& name) : VariableSet(6, name)
    {
        // the initial values where the NLP starts iterating from
        x0_ = 0.0;
        x1_ = 0.0;
        x2_ = 0.0;
        x3_ = 0.0;
        x4_ = 0.0;
        x5_ = 0.0;
    }

    void SetVariables(const VectorXd& x) override
    {
        x0_ = x(0);
        x1_ = x(1);
        x2_ = x(2);
        x3_ = x(3);
        x4_ = x(4);
        x5_ = x(5);
    };

    VectorXd GetValues() const override
    {
        Eigen::VectorXd vec;
        vec.resize(6, 1);
        vec << x0_, x1_, x2_, x3_, x4_, x5_;

        return vec;
    };

    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        bounds.at(0) = Bounds(-inf, inf);
        bounds.at(1) = Bounds(-inf, inf);
        bounds.at(2) = Bounds(-inf, inf);
        bounds.at(3) = Bounds(-inf, inf);
        bounds.at(4) = Bounds(-inf, inf);
        bounds.at(5) = Bounds(-inf, inf);
        return bounds;
    }

private:
    //unsigned int variable_num_;
    double x0_, x1_, x2_, x3_, x4_, x5_;
};


class ExConstraint : public ConstraintSet
{
public:
    ExConstraint(const std::string& name, const Eigen::MatrixXd& _kernel, const Eigen::VectorXd& _tau)
        :ConstraintSet(12, name),
         kernel_(_kernel),
         tau_(_tau){ }

    // The constraint value minus the constant value "1", moved to bounds.
    VectorXd GetValues() const override
    {
        VectorXd g(GetRows());
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
        g = tau_ + kernel_ * x;
        return g;
    };

    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
        b.at(0) = Bounds(0.0, inf);
        b.at(1) = Bounds(0.0, inf);
        b.at(2) = Bounds(0.0, inf);
        b.at(3) = Bounds(0.0, inf);
        b.at(4) = Bounds(0.0, inf);
        b.at(5) = Bounds(0.0, inf);
        b.at(6) = Bounds(0.0, inf);
        b.at(7) = Bounds(0.0, inf);
        b.at(8) = Bounds(0.0, inf);
        b.at(9) = Bounds(0.0, inf);
        b.at(10) = Bounds(0.0, inf);
        b.at(11) = Bounds(0.0, inf);
        return b;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        if (var_set == "var_set1") {
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            // printf("%d, %d\n", jac_block.rows(), jac_block.cols());

            jac_block.coeffRef(0, 0) = kernel_(0, 0);
            jac_block.coeffRef(0, 1) = kernel_(0, 1);
            jac_block.coeffRef(0, 2) = kernel_(0, 2);
            jac_block.coeffRef(0, 3) = kernel_(0, 3);
            jac_block.coeffRef(0, 4) = kernel_(0, 4);
            jac_block.coeffRef(0, 5) = kernel_(0, 5);

            jac_block.coeffRef(1, 0) = kernel_(1, 0);
            jac_block.coeffRef(1, 1) = kernel_(1, 1);
            jac_block.coeffRef(1, 2) = kernel_(1, 2);
            jac_block.coeffRef(1, 3) = kernel_(1, 3);
            jac_block.coeffRef(1, 4) = kernel_(1, 4);
            jac_block.coeffRef(1, 5) = kernel_(1, 5);

            jac_block.coeffRef(2, 0) = kernel_(2, 0);
            jac_block.coeffRef(2, 1) = kernel_(2, 1);
            jac_block.coeffRef(2, 2) = kernel_(2, 2);
            jac_block.coeffRef(2, 3) = kernel_(2, 3);
            jac_block.coeffRef(2, 4) = kernel_(2, 4);
            jac_block.coeffRef(2, 5) = kernel_(2, 5);

            jac_block.coeffRef(3, 0) = kernel_(3, 0);
            jac_block.coeffRef(3, 1) = kernel_(3, 1);
            jac_block.coeffRef(3, 2) = kernel_(3, 2);
            jac_block.coeffRef(3, 3) = kernel_(3, 3);
            jac_block.coeffRef(3, 4) = kernel_(3, 4);
            jac_block.coeffRef(3, 5) = kernel_(3, 5);

            jac_block.coeffRef(4, 0) = kernel_(4, 0);
            jac_block.coeffRef(4, 1) = kernel_(4, 1);
            jac_block.coeffRef(4, 2) = kernel_(4, 2);
            jac_block.coeffRef(4, 3) = kernel_(4, 3);
            jac_block.coeffRef(4, 4) = kernel_(4, 4);
            jac_block.coeffRef(4, 5) = kernel_(4, 5);

            jac_block.coeffRef(5, 0) = kernel_(5, 0);
            jac_block.coeffRef(5, 1) = kernel_(5, 1);
            jac_block.coeffRef(5, 2) = kernel_(5, 2);
            jac_block.coeffRef(5, 3) = kernel_(5, 3);
            jac_block.coeffRef(5, 4) = kernel_(5, 4);
            jac_block.coeffRef(5, 5) = kernel_(5, 5);

            jac_block.coeffRef(6, 0) = kernel_(6, 0);
            jac_block.coeffRef(6, 1) = kernel_(6, 1);
            jac_block.coeffRef(6, 2) = kernel_(6, 2);
            jac_block.coeffRef(6, 3) = kernel_(6, 3);
            jac_block.coeffRef(6, 4) = kernel_(6, 4);
            jac_block.coeffRef(6, 5) = kernel_(6, 5);

            jac_block.coeffRef(7, 0) = kernel_(7, 0);
            jac_block.coeffRef(7, 1) = kernel_(7, 1);
            jac_block.coeffRef(7, 2) = kernel_(7, 2);
            jac_block.coeffRef(7, 3) = kernel_(7, 3);
            jac_block.coeffRef(7, 4) = kernel_(7, 4);
            jac_block.coeffRef(7, 5) = kernel_(7, 5);

            jac_block.coeffRef(8, 0) = kernel_(8, 0);
            jac_block.coeffRef(8, 1) = kernel_(8, 1);
            jac_block.coeffRef(8, 2) = kernel_(8, 2);
            jac_block.coeffRef(8, 3) = kernel_(8, 3);
            jac_block.coeffRef(8, 4) = kernel_(8, 4);
            jac_block.coeffRef(8, 5) = kernel_(8, 5);

            jac_block.coeffRef(9, 0) = kernel_(9, 0);
            jac_block.coeffRef(9, 1) = kernel_(9, 1);
            jac_block.coeffRef(9, 2) = kernel_(9, 2);
            jac_block.coeffRef(9, 3) = kernel_(9, 3);
            jac_block.coeffRef(9, 4) = kernel_(9, 4);
            jac_block.coeffRef(9, 5) = kernel_(9, 5);

            jac_block.coeffRef(10, 0) = kernel_(10, 0);
            jac_block.coeffRef(10, 1) = kernel_(10, 1);
            jac_block.coeffRef(10, 2) = kernel_(10, 2);
            jac_block.coeffRef(10, 3) = kernel_(10, 3);
            jac_block.coeffRef(10, 4) = kernel_(10, 4);
            jac_block.coeffRef(10, 5) = kernel_(10, 5);

            jac_block.coeffRef(11, 0) = kernel_(11, 0);
            jac_block.coeffRef(11, 1) = kernel_(11, 1);
            jac_block.coeffRef(11, 2) = kernel_(11, 2);
            jac_block.coeffRef(11, 3) = kernel_(11, 3);
            jac_block.coeffRef(11, 4) = kernel_(11, 4);
            jac_block.coeffRef(11, 5) = kernel_(11, 5);

            // for(int i = 0; i < 4; i++)
            // {
            //     for(int j = 0; j < 4; i++)
            //     {
            //         jac_block.coeffRef(i, j) = kernel_(i, j);
            //     }
            // }
            ROS_INFO_ONCE("GOODOPT");
        }
    }

private:
    Eigen::VectorXd tau_;
    Eigen::MatrixXd kernel_;
};


class ExCost: public CostTerm {
public:
    ExCost() : ExCost("cost_term1") {}
    ExCost(const std::string& name) : CostTerm(name) {}

    double GetCost() const override
    {
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
        return std::pow(x(0),2)
              +std::pow(x(1),2)
              +std::pow(x(2),2)
              +std::pow(x(3),2)
              +std::pow(x(4),2)
              +std::pow(x(5),2);
    };

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_set1") {
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            jac.coeffRef(0, 0) = 2.0*x(0);
            jac.coeffRef(0, 1) = 2.0*x(1);
            jac.coeffRef(0, 2) = 2.0*x(2);
            jac.coeffRef(0, 3) = 2.0*x(3);
            jac.coeffRef(0, 4) = 2.0*x(4);
            jac.coeffRef(0, 5) = 2.0*x(5);
        }
    }
};

} // namespace opt

#endif //PYRAMID_CONTROL_OPTIMIZER_H
