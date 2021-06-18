/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file matrix_alg.cpp
 *
 * Matrix algebra on raw arrays
 */

#include "matrix_alg.h"
#include <px4_platform_common/defines.h>

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A n*m
 *    @param     B,           Matrix B n*m
 *    @param     n,           row dimemsion
 *    @param     m,           col dimemsion
 *    @param     positive,    +/-
 *    @returns                plused matrix i.e. A+B
 */

bool mat_plus(float *out, float *A, float *B, uint8_t n, uint8_t m, bool positive)
{
    bool ret = true;
    memset(out, 0.0f, n * m * sizeof(float));

    if(positive)
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < m; j++) {
                out[i * m + j] = A[i * m + j] + B[i * m + j];
            }
        }
    else
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < m; j++) {
                out[i * m + j] = A[i * m + j] - B[i * m + j];
            }
        }

    return ret;
}

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A
 *    @param     B,           Matrix B
 *    @param     n,           dimemsion of square matrices
 *    @returns                multiplied matrix i.e. A*B
 */

bool mat_mul(float *out, float *A, float *B, uint8_t n)
{
    bool ret = true;
    memset(out, 0.0f, n * n * sizeof(float));

	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			for (uint8_t k = 0; k < n; k++) {
                out[i * n + j] += A[i * n + k] * B[k * n + j];
			}
		}
	}

	return ret;
}

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A
 *    @param     B,           Matrix B
 *    @param     n,           dimemsion of matrix A n*m
 *    @param     m,           dimemsion
 *    @param     l,           dimemsion of matrix B m*l
 *    @returns                multiplied matrix i.e. A*B
 */

bool mat_mul(float *out, float *A, float *B, uint8_t n, uint8_t m, uint8_t l)
{
    bool ret = true;
    memset(out, 0.0f, n * l * sizeof(float));

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < l; j++) {
            for (uint8_t k = 0; k < m; k++) {
                out[i * n + j] += A[i * n + k] * B[k * l + j];
            }
        }
    }

    return ret;
}

static inline void swap(float &a, float &b)
{
	float c;
	c = a;
	a = b;
	b = c;
}

/*
 *    calculates pivot matrix such that all the larger elements in the row are on diagonal
 *
 *    @param     A,           input matrix matrix
 *    @param     pivot
 *    @param     n,           dimenstion of square matrix
 *    @returns                false = matrix is Singular or non positive definite, true = matrix inversion successful
 */

static void mat_pivot(float *A, float *pivot, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			pivot[i * n + j] = (i == j);
		}
	}

	for (uint8_t i = 0; i < n; i++) {
		uint8_t max_j = i;

		for (uint8_t j = i; j < n; j++) {
			if (fabsf(A[j * n + i]) > fabsf(A[max_j * n + i])) {
				max_j = j;
			}
		}

		if (max_j != i) {
			for (uint8_t k = 0; k < n; k++) {
				swap(pivot[i * n + k], pivot[max_j * n + k]);
			}
		}
	}
}

/*
 *    calculates matrix inverse of Lower trangular matrix using forward substitution
 *
 *    @param     L,           lower triangular matrix
 *    @param     out,         Output inverted lower triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_forward_sub(float *L, float *out, uint8_t n)
{
	// Forward substitution solve LY = I
	for (int i = 0; i < n; i++) {
		out[i * n + i] = 1 / L[i * n + i];

		for (int j = i + 1; j < n; j++) {
			for (int k = i; k < j; k++) {
				out[j * n + i] -= L[j * n + k] * out[k * n + i];
			}

			out[j * n + i] /= L[j * n + j];
		}
	}
}

/*
 *    calculates matrix inverse of Upper trangular matrix using backward substitution
 *
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_back_sub(float *U, float *out, uint8_t n)
{
	// Backward Substitution solve UY = I
	for (int i = n - 1; i >= 0; i--) {
		out[i * n + i] = 1 / U[i * n + i];

		for (int j = i - 1; j >= 0; j--) {
			for (int k = i; k > j; k--) {
				out[j * n + i] -= U[j * n + k] * out[k * n + i];
			}

			out[j * n + i] /= U[j * n + j];
		}
	}
}

/*
 *    Decomposes square matrix into Lower and Upper triangular matrices such that
 *    A*P = L*U, where P is the pivot matrix
 *    ref: http://rosettacode.org/wiki/LU_decomposition
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

static void mat_LU_decompose(float *A, float *L, float *U, float *P, uint8_t n)
{
	memset(L, 0, n * n * sizeof(float));
	memset(U, 0, n * n * sizeof(float));
	memset(P, 0, n * n * sizeof(float));
	mat_pivot(A, P, n);

    float *APrime = new float[n*n];
    mat_mul(APrime, P, A, n);

	for (uint8_t i = 0; i < n; i++) {
		L[i * n + i] = 1;
	}

	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
			if (j <= i) {
				U[j * n + i] = APrime[j * n + i];

				for (uint8_t k = 0; k < j; k++) {
					U[j * n + i] -= L[j * n + k] * U[k * n + i];
				}
			}

			if (j >= i) {
				L[j * n + i] = APrime[j * n + i];

				for (uint8_t k = 0; k < i; k++) {
					L[j * n + i] -= L[j * n + k] * U[k * n + i];
				}

				L[j * n + i] /= U[i * n + i];
			}
		}
	}

	delete[] APrime;
}

/*
 *    matrix inverse code for any square matrix using LU decomposition
 *    inv = inv(U)*inv(L)*P, where L and U are triagular matrices and P the pivot matrix
 *    ref: http://www.cl.cam.ac.uk/teaching/1314/NumMethods/supporting/mcmaster-kiruba-ludecomp.pdf
 *    @param     m,           input 4x4 matrix
 *    @param     inv,      Output inverted 4x4 matrix
 *    @param     n,           dimension of square matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */
bool mat_inverse(float *A, float *inv, uint8_t n)
{
	float *L, *U, *P;
	bool ret = true;
	L = new float[n * n];
	U = new float[n * n];
	P = new float[n * n];
	mat_LU_decompose(A, L, U, P, n);

	float *L_inv = new float[n * n];
	float *U_inv = new float[n * n];

	memset(L_inv, 0, n * n * sizeof(float));
	mat_forward_sub(L, L_inv, n);

	memset(U_inv, 0, n * n * sizeof(float));
	mat_back_sub(U, U_inv, n);

	// decomposed matrices no longer required
	delete[] L;
	delete[] U;

    float *inv_unpivoted = new float[n*n];
    mat_mul(inv_unpivoted, U_inv, L_inv, n);
    float *inv_pivoted = new float[n*n];
    mat_mul(inv_pivoted, inv_unpivoted, P, n);

	//check sanity of results
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < n; j++) {
            if (!PX4_ISFINITE(inv_pivoted[i * n + j])) {
				ret = false;
			}
		}
	}

	memcpy(inv, inv_pivoted, n * n * sizeof(float));

	//free memory
	delete[] inv_pivoted;
	delete[] inv_unpivoted;
	delete[] P;
	delete[] U_inv;
	delete[] L_inv;
	return ret;
}

bool inverse4x4(float m[], float invOut[])
{
	float inv[16], det;
	uint8_t i;

	inv[0] = m[5]  * m[10] * m[15] -
		 m[5]  * m[11] * m[14] -
		 m[9]  * m[6]  * m[15] +
		 m[9]  * m[7]  * m[14] +
		 m[13] * m[6]  * m[11] -
		 m[13] * m[7]  * m[10];

	inv[4] = -m[4]  * m[10] * m[15] +
		 m[4]  * m[11] * m[14] +
		 m[8]  * m[6]  * m[15] -
		 m[8]  * m[7]  * m[14] -
		 m[12] * m[6]  * m[11] +
		 m[12] * m[7]  * m[10];

	inv[8] = m[4]  * m[9] * m[15] -
		 m[4]  * m[11] * m[13] -
		 m[8]  * m[5] * m[15] +
		 m[8]  * m[7] * m[13] +
		 m[12] * m[5] * m[11] -
		 m[12] * m[7] * m[9];

	inv[12] = -m[4]  * m[9] * m[14] +
		  m[4]  * m[10] * m[13] +
		  m[8]  * m[5] * m[14] -
		  m[8]  * m[6] * m[13] -
		  m[12] * m[5] * m[10] +
		  m[12] * m[6] * m[9];

	inv[1] = -m[1]  * m[10] * m[15] +
		 m[1]  * m[11] * m[14] +
		 m[9]  * m[2] * m[15] -
		 m[9]  * m[3] * m[14] -
		 m[13] * m[2] * m[11] +
		 m[13] * m[3] * m[10];

	inv[5] = m[0]  * m[10] * m[15] -
		 m[0]  * m[11] * m[14] -
		 m[8]  * m[2] * m[15] +
		 m[8]  * m[3] * m[14] +
		 m[12] * m[2] * m[11] -
		 m[12] * m[3] * m[10];

	inv[9] = -m[0]  * m[9] * m[15] +
		 m[0]  * m[11] * m[13] +
		 m[8]  * m[1] * m[15] -
		 m[8]  * m[3] * m[13] -
		 m[12] * m[1] * m[11] +
		 m[12] * m[3] * m[9];

	inv[13] = m[0]  * m[9] * m[14] -
		  m[0]  * m[10] * m[13] -
		  m[8]  * m[1] * m[14] +
		  m[8]  * m[2] * m[13] +
		  m[12] * m[1] * m[10] -
		  m[12] * m[2] * m[9];

	inv[2] = m[1]  * m[6] * m[15] -
		 m[1]  * m[7] * m[14] -
		 m[5]  * m[2] * m[15] +
		 m[5]  * m[3] * m[14] +
		 m[13] * m[2] * m[7] -
		 m[13] * m[3] * m[6];

	inv[6] = -m[0]  * m[6] * m[15] +
		 m[0]  * m[7] * m[14] +
		 m[4]  * m[2] * m[15] -
		 m[4]  * m[3] * m[14] -
		 m[12] * m[2] * m[7] +
		 m[12] * m[3] * m[6];

	inv[10] = m[0]  * m[5] * m[15] -
		  m[0]  * m[7] * m[13] -
		  m[4]  * m[1] * m[15] +
		  m[4]  * m[3] * m[13] +
		  m[12] * m[1] * m[7] -
		  m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] +
		  m[0]  * m[6] * m[13] +
		  m[4]  * m[1] * m[14] -
		  m[4]  * m[2] * m[13] -
		  m[12] * m[1] * m[6] +
		  m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
		 m[1] * m[7] * m[10] +
		 m[5] * m[2] * m[11] -
		 m[5] * m[3] * m[10] -
		 m[9] * m[2] * m[7] +
		 m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		 m[0] * m[7] * m[10] -
		 m[4] * m[2] * m[11] +
		 m[4] * m[3] * m[10] +
		 m[8] * m[2] * m[7] -
		 m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		  m[0] * m[7] * m[9] +
		  m[4] * m[1] * m[11] -
		  m[4] * m[3] * m[9] -
		  m[8] * m[1] * m[7] +
		  m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		  m[0] * m[6] * m[9] -
		  m[4] * m[1] * m[10] +
		  m[4] * m[2] * m[9] +
		  m[8] * m[1] * m[6] -
		  m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (fabsf(det) < 1.1755e-38f) {
		return false;
	}

	det = 1.0f / det;

	for (i = 0; i < 16; i++) {
		invOut[i] = inv[i] * det;
	}

	return true;
}

/*
 *    Cholesky decomposition
 *    A=chol*chol', where chol is a lower triagular matrix
 *
 *    @param     A,           input matrix
 *    @param     chol,        Output decomposed matrix
 *    @param     n,           dimension of square matrix
 *    @returns                false = matrix is Singular, true = matrix decomposition successful
 */
bool mat_cholesky(float *A, float *chol, uint8_t n)
{
    float *L;
    L = new float[n * n];

    for(int i=0;i<n;i++)
        for(int j=0;j<=i;j++){
            float sum = 0.0f;
            for(int k=0; k<j; k++)
                sum += L[i*n+k] * L[j*n+k];

            if(A[i*n+i] - sum < 1e-12f)
                return false;

            L[i*n+j] = (i == j)?
                        (float)sqrt(A[i*n+i] - sum) :
                        (1.0f / L[j*n+j] * (A[i*n+j] - sum));
        }

    memcpy(chol, L, n * n * sizeof(float));
    delete[] L;
    return true;
}

bool mat_cholesky(double *A, double *chol, uint8_t n)
{
    double *L;
    L = new double[n * n];

    for(int i=0;i<n;i++)
        for(int j=0;j<=i;j++){
            double sum = 0.0;
            for(int k=0; k<j; k++)
                sum += L[i*n+k] * L[j*n+k];

            if(A[i*n+i] - sum < 1e-12)
                return false;

            L[i*n+j] = (i == j)?
                        sqrt(A[i*n+i] - sum) :
                        (1.0 / L[j*n+j] * (A[i*n+j] - sum));
        }

    memcpy(chol, L, n * n * sizeof(double));
    delete[] L;
    return true;
}

/*
 *    Matrix Transpose
 *
 *    @param     A,           input matrix n*m
 *    @param     n,           dimension of matrix
 *    @param     m,           dimension of matrix
 *    @returns                transposed matrix
 */
bool mat_transpose(float *out, float *A, uint8_t n, uint8_t m)
{
    bool ret = true;
    memset(out, 0.0f, m * n * sizeof(float));

    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < n; j++)
                out[i * n + j] = A[j * m + i];
    return ret;
}

bool mat_transpose(double *out, double *A, uint8_t n, uint8_t m)
{
    bool ret = true;
    memset(out, 0.0, m * n * sizeof(double));

    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < n; j++)
                out[i * n + j] = A[j * m + i];
    return ret;
}
