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
/** ***************************************************************************
 * CloseUpDisplay Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/03/10 15:00:59 $
 * $Revision: 1.1 $
 *****************************************************************************/
#include "closeUpDisplay.h"
#include "stFeature.h"
#include "image.h"

#include "highgui.h"
#include <iostream>

using namespace vision;

CloseUpDisplay::CloseUpDisplay(int window, int groupId) {
    _lastImg = 0;
    _window = window;
    _groupId = groupId;
    cvNamedWindow("ft", 0);
}
// ============================================================================

CloseUpDisplay::~CloseUpDisplay() {
    cvDestroyWindow("ft");
    if (_lastImg)
        delete _lastImg;
}
// ============================================================================

void CloseUpDisplay::notifyReset(const std::vector<FeaturePtr> &features) {
}
// ============================================================================

void CloseUpDisplay::notifyStep(const Image &img,
        const std::vector<FeaturePtr> &features) {
    if (_lastImg)
        delete _lastImg;
    _lastImg = new Image(_window * 2 + 1, _window * 2 + 1, 3);
    int i0, j0;
    int height = img.getHeight();
    //int width = img.getWidth();

    if (!features.empty()) {
        double x, y;
        std::vector<FeaturePtr>::const_iterator featuresIt;
        for (featuresIt = features.begin(); featuresIt != features.end(); featuresIt++) {
            FeaturePtr feat = *(featuresIt);

            x = feat->getX();
            y = feat->getY();

            if (!feat->isLost() && (feat->getGroupId() == _groupId)) {
                i0 = height - (int) y - 1;
                j0 = (int) x;

                for (int i = -_window; i < _window; i++) {
                    for (int j = -_window; j < _window; j++) {
                        Pixel px = img.get2D(i0 + i, j0 + j);
                        _lastImg->set2D(_window + i, _window + j, px);
                    }
                }
                //cvCircle( iimg, cvPointFrom32f(cvPoint2D32f(x, img.getHeight() - y - 1)), 3,
                //        CV_RGB(0,255,0), -1, 8,0);

            }
        }
    }
    cvShowImage("ft", _lastImg->getIplImage());
    //cvWaitKey(10);
}
/** ***************************************************************************
 End of File
 ******************************************************************************/
