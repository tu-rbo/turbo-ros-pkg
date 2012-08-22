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
#ifndef __lapack_h__
#define __lapack_h__

/* Routines for inverting matrices */
void dgetrf_(int *m, int *n, double *A, int *lda, int *ipiv, int *info);
void dgetri_(int *n, double *A, int *lda, int *ipiv, double *work, 
	     int *lwork, int *info);

/* Routines for computing the least-squares solution to Ax = b */
void dgelss_(int *m, int *n, int *nrhs, double *A, int *lda, double *b, int *ldb, 
	     double *s, double *rcond, int *rank, double *work, int *lwork, int *info);
void dgelsy_(int *m, int *n, int *nrhs, double *A, int *lda, double *b, int *ldb,
	     int *jpvt, double *rcond, int *rank, double *work, int *lwork, int *info);

void dgesv_(int *n, int *nrhs, double *A, int *lda, int *ipiv, double *b,
	    int *ldb, int *info);

void dgetrs_(char *trans, int *n, int *nrhs, double *A, int *lda, int *ipiv, 
             double *b, int *ldb, int *info);

/* Routine for computing the eigenvalues / eigenvectors of a matrix */
void dgeev_(char *jobvl, char *jobvr, int *n, double *A, int *lda, double *wr, double *wi,
	    double *vl, int *ldvl, double *vr, int *ldvr, double *work, int *lwork,
	    int *info);

/* Routine for singular value decomposition */
void dgesvd_(char *jobu, char *jobvt, int *m, int *n, double *A, int *lda, 
	     double *S, double *U, int *ldu, double *VT, int *ldvt,
	     double *work, int *lwork, int *info);

/* Routine for Cholesky decomposition */
void dpotrf_(char *uplo, int *n, double *A, int *lda, int *info);

/* Routine for QR factorization */
void dgeqrf_(int *m, int *n, double *A, int *lda, double *tau, double *work, 
	     int *lwork, int *info);

/* Routine for RQ factorization */
void dgerqf_(int *m, int *n, double *A, int *lda, double *tau, double *work, 
	     int *lwork, int *info);

#endif /* __lapack_h__ */
