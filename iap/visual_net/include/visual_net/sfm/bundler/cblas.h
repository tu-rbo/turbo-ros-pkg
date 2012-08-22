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
/* cblas.h */
/* cblas headers */

#ifndef __CBLAS_H__
#define __CBLAS_H__

#define CBLAS_INDEX size_t  /* this may vary between platforms */

typedef enum {
    CblasRowMajor=101, CblasColMajor=102
} CBLAS_ORDER;

typedef enum {
    CblasNoTrans=111, CblasTrans=112, CblasConjTrans=113
} CBLAS_TRANSPOSE;

typedef enum {
    CblasUpper=121, CblasLower=122
} CBLAS_UPLO;

typedef enum {
    CblasNonUnit=131, CblasUnit=132
} CBLAS_DIAG;

typedef enum {
    CblasLeft=141, CblasRight=142
} CBLAS_SIDE;

void cblas_dgemm(const  CBLAS_ORDER Order, const  CBLAS_TRANSPOSE TransA,
                 const  CBLAS_TRANSPOSE TransB, 
                 const int M, const int N,
                 const int K, const double alpha, const double *A,
                 const int lda, const double *B, const int ldb,
                 const double beta, double *C, const int ldc);

#endif /* __CBLAS_H__ */
