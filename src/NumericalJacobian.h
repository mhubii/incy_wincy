#pragma once

#include <boost/numeric/ublas/matrix.hpp>


// Some typedefs.
using vector_type = boost::numeric::ublas::vector< double >;
using matrix_type = boost::numeric::ublas::matrix< double >;


// Function pointer of the system we want to describe.
using system_func_ptr = auto(*)(const vector_type&) -> vector_type;


// Physical description of the system.
class System
{
public:
    System(system_func_ptr system) : system_(system) {  }; // takes a function pointer to the physical description

    auto operator() (const vector_type& x, vector_type& dxdt, double t) -> void {

        dxdt = system_(x);
    }; // Performs a function call on system function pointer

    // Physics of the system are described by this function pointer.
    system_func_ptr system_;
};


// Find the Jacobian of the system of interest.
class NumericalJacobian : public System
{
public:
    NumericalJacobian(system_func_ptr system) : System(system) {   };

    auto operator() (const vector_type& x, matrix_type& J , const double& t, vector_type& dfdt) -> void {

        // Determine the Jacobian.
        uint c = x.size();

        for (uint i = 0; i < c; i++) {

            column(J, i) = 0.5/h_*(system_(x + boost::numeric::ublas::unit_vector<double>(c, i)*h_) - system_(x - boost::numeric::ublas::unit_vector<double>(c, i)*h_)); // compute finite differences
        }
    };

    double h_ = std::sqrt(std::numeric_limits<double>::min());
};

