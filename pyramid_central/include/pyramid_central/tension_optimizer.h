#ifndef PYRAMID_CENTRAL_TENSION_OPTIMIZER_H
#define PYRAMID_CENTRAL_TENSION_OPTIMIZER_H

#include <Eigen/Core>

#include <ros/ros.h>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

namespace ifopt {
using Eigen::Vector2d;


class ExVariables : public VariableSet
{
public:
    ExVariables() : ExVariables("var_set1") {};
    ExVariables(const std::string& name) : VariableSet(2, name)
    {
        x0_ = 0.0;
        x1_ = 0.0;
    }

    void SetVariables(const VectorXd& x) override
    {
        x0_ = x(0);
        x1_ = x(1);
    };

    VectorXd GetValues() const override
    {
        return Vector2d(x0_, x1_);
    };

    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        bounds.at(0) = Bounds(-inf, inf);
        bounds.at(1) = Bounds(-inf, inf);
        return bounds;
    }

private:
    //unsigned int variable_num_;
    double x0_, x1_;
};


class ExConstraint : public ConstraintSet
{
public:
    ExConstraint(const std::string& name, const Eigen::MatrixXd& _kernel, const Eigen::Vector4d& _tau)
        :ConstraintSet(4, name),
         kernel_(_kernel),
         tau_(_tau){ }

    // The constraint value minus the constant value "1", moved to bounds.
    VectorXd GetValues() const override
    {
        VectorXd g(GetRows());
        Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
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
        return b;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        if (var_set == "var_set1") {
            Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

            jac_block.coeffRef(0, 0) = kernel_(0, 0); jac_block.coeffRef(0, 1) = kernel_(0, 1);
            jac_block.coeffRef(1, 0) = kernel_(1, 0); jac_block.coeffRef(1, 1) = kernel_(1, 1);
            jac_block.coeffRef(2, 0) = kernel_(2, 0); jac_block.coeffRef(2, 1) = kernel_(2, 1);
            jac_block.coeffRef(3, 0) = kernel_(3, 0); jac_block.coeffRef(3, 1) = kernel_(3, 1);
        }
    }

private:
    Eigen::Vector4d tau_;
    Eigen::Matrix<double, 4, 2> kernel_;
};


class ExCost: public CostTerm {
public:
    ExCost() : ExCost("cost_term1") {}
    ExCost(const std::string& name) : CostTerm(name) {}

    double GetCost() const override
    {
        Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
        return std::pow(x(0),2) + std::pow(x(1),2);
    };

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_set1") {
            Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

            jac.coeffRef(0, 0) = 2.0*x(0);
            jac.coeffRef(0, 1) = 2.0*x(1);
        }
    }
};

} // namespace opt

#endif //PYRAMID_CENTRAL_TENSION_OPTIMIZER_H
