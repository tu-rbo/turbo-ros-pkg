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
#ifndef _COUT_
#define _COUT_
#include <boost/numeric/ublas/matrix.hpp>
#include "feature.h"
#include <vector>
#include <iostream>

/*! \brief Brief additional << operators for feature/vector<feature> etc.
 *
 */

/**
 * @brief ostream operator to use with cout << boost::numeric::ublas::matrix<double>
 *
 * @param outputStream
 * @param input
 */
std::ostream &operator<<(std::ostream &outputStream, boost::numeric::ublas::matrix<double> &input);

//std::ostream &operator<<(std::ostream &outputStream, Feature &input);

/**
 * @brief ostream operator to use with cout << std::vector<Feature>
 *
 * @param outputStream
 * @param input
 */
std::ostream &operator<<(std::ostream &outputStream, std::vector< vision::Feature> &input);

/**
 * @brief ostream operator to use with cout << std::vector< std::vector<Feature> >
 *
 * @param outputStream
 * @param input
 */
std::ostream &operator<<(std::ostream &outputStream, std::vector< std::vector<vision::Feature> > &input);

/**
 * @brief ostream operator to use with cout << std::vector< boost::numeric::ublas::matrix<double> >
 *
 * @param outputStream
 * @param input
 */
std::ostream &operator<<(std::ostream &outputStream, std::vector< boost::numeric::ublas::matrix<double> > &input);

template <typename T>
/**
 * @brief
 *
 * @param outputStream
 * @param input
 * @return std::ostream & operator
 */
std::ostream &operator<<(std::ostream &outputStream, std::vector< std::vector<T> > &input)
{
    for(unsigned int i=0;i<input.size();i++){
        outputStream << "[" ;
        for(unsigned int k=0;k<input.at(i).size();k++){
            outputStream << input.at(i).at(k) << " ";
        }
        outputStream << "]" << std::endl;
    }
    return outputStream;
}

template <typename T>
/**
 * @brief
 *
 * @param outputStream
 * @param input
 * @return std::ostream & operator
 */
std::ostream &operator<<(std::ostream &outputStream, std::vector< T > &input)
{
    for(unsigned int i=0;i<input.size();i++){
        outputStream << input.at(i) << " ";
    }
    return outputStream;
}

#endif
