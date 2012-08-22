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
/*
 * JointFactory.h
 *
 *  Class is a singleton, derived classes must be, too
 *
 * You need to care for correct instantiation of your JointFactory.
 * Be sure you store world instance your generating in
 * m_pInstance. E.g. you could write a create method as follows
 * <code>
 * JointFactory* MyJointFactory::getInstance() {
    if (m_instance == NULL) {
        m_instance = new MyJointFactory;
    }
    return m_instance;
   }
 *
 *  Created on: Apr 20, 2012
 *      Author: roberto
 */

#ifndef JOINTFACTORY_H_
#define JOINTFACTORY_H_

#include "Joint.h"

class JointFactory
{
public:
  virtual ~JointFactory();
  virtual std::vector<JointPtr> generateJoints(double min_motion) const = 0;
protected:
  //static JointFactory* m_instance;
  static JointFactory* m_instance_tb;
  static JointFactory* m_instance_r;
  static JointFactory* m_instance_p;
  static JointFactory* m_instance_fb;
  JointFactory();
};

#endif /* JOINTFACTORY_H_ */
