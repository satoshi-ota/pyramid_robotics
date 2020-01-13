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
    ExVariables(const std::string& name,
                const Eigen::VectorXd& initVal,
                const int dimKer)
        :VariableSet(dimKer, name),
         lambda_(initVal){ }

    void SetVariables(const VectorXd& x) override
    {
        lambda_ = x;
    };

    VectorXd GetValues() const override
    {
        return lambda_;
    };

    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        for (int i = 0; i < GetRows(); ++i)
            bounds.at(i) = Bounds(-inf, inf);
        return bounds;
    }

private:
    Eigen::VectorXd lambda_;
};


class ExConstraint : public ConstraintSet
{
public:
    ExConstraint(const std::string& name,
                 const int constraintNum,
                 const Eigen::MatrixXd& kernel,
                 const Eigen::VectorXd& tau,
                 const Eigen::VectorXd& upperLimmit)
        :ConstraintSet(constraintNum, name),
         kernel_(kernel),
         tau_(tau),
         upperLimmit_(upperLimmit){ }

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
        for (int i = 0; i < GetRows(); ++i)
            b.at(i) = Bounds(0.0, inf);
        return b;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        if (var_set == "var_set1") {
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            for (int i = 0; i < kernel_.rows(); ++i)
            {
                jac_block.startVec(i);
                for (int j = 0; j < kernel_.cols(); ++j)
                {
                    jac_block.insertBack(i, j) = kernel_(i, j);
                }
            }
        }
    }

private:
    Eigen::VectorXd tau_;
    Eigen::MatrixXd kernel_;
    Eigen::VectorXd upperLimmit_;
};


class ExCost: public CostTerm {
public:
    ExCost() : ExCost("cost_term1") {}
    ExCost(const std::string& name) : CostTerm(name) {}

    double GetCost() const override
    {
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

        double cost = 0;
        for (int i = 0; i < x.rows(); ++i)
            cost += std::pow(x(i),2);

        return cost;
    };

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_set1") {
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            jac.startVec(0);

            for (int i = 0; i < x.rows(); ++i)
            {
                jac.insertBack(0, i) = 2.0*x(i);
            }
        }
    }
};

} // namespace opt

#endif //PYRAMID_CONTROL_OPTIMIZER_H
