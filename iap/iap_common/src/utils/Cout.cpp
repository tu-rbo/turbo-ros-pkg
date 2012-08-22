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
#include "Cout.h"

std::ostream &operator<<(std::ostream &outputStream, boost::numeric::ublas::matrix<double> &input)
{
	for(unsigned int i=0;i<input.size1();i++){
		outputStream << " ";
		for(unsigned int j=0;j<input.size2();j++){
			outputStream << input(i,j) << " ";
		}
		outputStream << ";" << std::endl;
	}
	return outputStream;
}
/*
std::ostream &operator<<(std::ostream &outputStream, Feature &input)
{
	outputStream << input.getX() << " "<< input.getY() << " "<< input.getZ();
	return outputStream;
}*/

std::ostream &operator<<(std::ostream &outputStream, std::vector<vision::Feature> &input)
{
	for(unsigned int i=0;i<input.size();i++){
		outputStream << "" << input.at(i).getX() << " "<< input.at(i).getY() << " "<< input.at(i).getZ() << ";" << std::endl;
	}
	return outputStream;
}


std::ostream &operator<<(std::ostream &outputStream, std::vector< std::vector<vision::Feature> > &input)
{
	for(unsigned int i=0;i<input.size();i++){
		outputStream << input.at(i) << std::endl;
	}
	return outputStream;
}

std::ostream &operator<<(std::ostream &outputStream, std::vector< boost::numeric::ublas::matrix<double> > &input)
{
	for(unsigned int i=0;i<input.size();i++){
		outputStream << input.at(i) << std::endl;
	}
	return outputStream;

}

