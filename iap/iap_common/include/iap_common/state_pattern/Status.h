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
 * Lightweight implementation of the state pattern,
 * see http://en.wikipedia.org/wiki/State_pattern
 *
 * How to use: Create a new class for every state and let it inherit from Status.
 * In the step method of the Status class the actual state transition is implemented.
 *
 * Then let the owner of the states inherit from StatusContext. You can also give
 * your status friend access to the StatusContext in order to provide it with information
 * necessary for state transitions.
 *
 *  Created on: Feb 18, 2012
 *      Author: shoefer
 */

#ifndef STATE_PATTERN_STATUS_H_
#define STATE_PATTERN_STATUS_H_

#include <exception>
#include <iostream>
#include <string>


namespace iap_common {

namespace state_pattern {

class Status;

/**
 * @brief StatusContext class
 *
 * A class which works as the executor of the state machine must implement
 * the abstract class StatusContext.
 */
class StatusContext
{
  Status* currentStatus;

public:
  StatusContext() : currentStatus(0) {}

  virtual Status* getCurrentStatus() const {
    return currentStatus;
  }

  virtual void setCurrentStatus(Status* s) {
    Status* tmp = 0;

    if (currentStatus != 0)
      tmp = currentStatus;
    currentStatus = s;

    // currentStatus must never be zero (thread safety!)
    if (tmp != 0)
      delete tmp;
  }

  friend class Status;
};

/**
 * @brief Status interface class
 *
 * Every state has to implement this abstract class.
 */
class Status
{
protected:
  StatusContext* context;

public:
  Status(StatusContext* ctx) : context(ctx) {
    if (context == 0) {
      std::cout << "[Status]: Context must not be NULL" << std::endl;
      throw std::exception();
    }
  }

  virtual void step() = 0;
  virtual std::string name() = 0;
};

}
}

#endif /* STATE_PATTERN_STATUS_H_ */
