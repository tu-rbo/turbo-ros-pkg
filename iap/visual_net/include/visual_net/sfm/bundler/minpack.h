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
/* minpack.h */
/* Prototypes for FORTRAN minpack routines */

#ifndef __minpack_h__
#define __minpack_h__

void lmdif1_(void *fcn, const int *m, const int *n, double *x, double *fvec,
	     double *tol, int *info, int *iwa, double *wa, const int *lwa);

void lmdif_(void *fcn, int *m, int *n, double *x, double *fvec, double *ftol, 
	    double *xtol, double *gtol, int *maxfev, double *epsfcn, double *diag,
	    int *mode, double *factor, int *nprint, int *info, int *nfev, double *fjac,
	    int *ldfjac, int *ipvt, double *qtf, double *wa1, double *wa2, double *wa3,
	    double *wa4);

#endif /* __minpack_h__ */
