/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "inverse.h"
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <iostream>

namespace ublas = boost::numeric::ublas;
using namespace boost::numeric::ublas;

bool InvertMatrix(const matrix<double> & input, matrix<double> & inverse){
  //create a working copy of the input
  matrix<double> A(input);

  //create a permutation matrix for the LU-factorization
  permutation_matrix<std::size_t> pm(A.size1());

  //perform LU-factorization
  int res = boost::numeric::ublas::lu_factorize(A,pm);

  if(res!=0){
        std::cout<<"Singularity"<<std::endl;
        return false;
  }

  //create identity matrix of "inverse"
  inverse.assign(identity_matrix<double>(A.size1()));

  //backsubstitute to get the inverse
  lu_substitute(A, pm, inverse);

  return true;
}

bool PseudoInverse(const matrix<double> & input, matrix<double> & inverse){
      int n=input.size1();
      int m=input.size2();
      if(n>m){
	std::cout<<"Wrong matrix dimensions!"<<std::endl;
        return false;
      }

      matrix<double> B=matrix<double>(m,n);
      matrix<double> C=matrix<double>(n,n);
      matrix<double> D=matrix<double>(n,n);

      B=trans(input);
      C=prod(input,B);
      InvertMatrix(C,D);
      inverse=prod(B,D);

      return 1;
}

