
#include <unit_test/unit_test.h>

#include <matrix/math.hpp>
#include <matrix/filter.hpp>
#include <matrix/integration.hpp>
#include <matrix/Quaternion.hpp>

using namespace matrix;

class MatrixTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool attitudeTests();
	bool filterTests();
	bool helperTests();
	bool integrationTests();
	bool inverseTests();
	bool matrixAssignmentTests();
	bool matrixMultTests();
	bool matrixScalarMultTests();
	bool setIdentityTests();
	bool sliceTests();
	bool squareMatrixTests();
	bool transposeTests();
	bool vectorTests();
	bool vector2Tests();
	bool vector3Tests();
	bool vectorAssignmentTests();
	bool dcmRenormTests();
};

bool MatrixTest::run_tests()
{
	ut_run_test(attitudeTests);
	ut_run_test(filterTests);
	ut_run_test(helperTests);
	ut_run_test(integrationTests);
	ut_run_test(inverseTests);
	ut_run_test(matrixAssignmentTests);
	ut_run_test(matrixMultTests);
	ut_run_test(matrixScalarMultTests);
	ut_run_test(setIdentityTests);
	ut_run_test(sliceTests);
	ut_run_test(squareMatrixTests);
	ut_run_test(transposeTests);
	ut_run_test(vectorTests);
	ut_run_test(vector2Tests);
	ut_run_test(vector3Tests);
	ut_run_test(vectorAssignmentTests);
	ut_run_test(dcmRenormTests);

	return (_tests_failed == 0);
}


ut_declare_test_c(test_matrix, MatrixTest)

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Eulerf;
using matrix::Vector3f;

bool MatrixTest::attitudeTests()
{
	double eps = 1e-6;

	// check data
	Eulerf euler_check(0.1f, 0.2f, 0.3f);
	Quatf q_check(0.98334744f, 0.0342708f, 0.10602051f, .14357218f);
	float dcm_data[] =  {
		0.93629336f, -0.27509585f,  0.21835066f,
		0.28962948f,  0.95642509f, -0.03695701f,
		-0.19866933f,  0.0978434f,  0.97517033f
	};
	Dcmf dcm_check(dcm_data);

	// euler ctor
	ut_test(isEqual(euler_check, Vector3f(0.1f, 0.2f, 0.3f)));


	// euler default ctor
	Eulerf e;
	Eulerf e_zero = zeros<float, 3, 1>();
	ut_test(isEqual(e, e_zero));
	ut_test(isEqual(e, e));

	// euler vector ctor
	Vector<float, 3> v;
	v(0) = 0.1f;
	v(1) = 0.2f;
	v(2) = 0.3f;
	Eulerf euler_copy(v);
	ut_test(isEqual(euler_copy, euler_check));

	// quaternion ctor
	Quatf q0(1, 2, 3, 4);
	Quatf q(q0);
	ut_test(fabs(q(0) - 1) < eps);
	ut_test(fabs(q(1) - 2) < eps);
	ut_test(fabs(q(2) - 3) < eps);
	ut_test(fabs(q(3) - 4) < eps);

	// quat normalization
	q.normalize();
	ut_test(isEqual(q, Quatf(0.18257419f,  0.36514837f,
				 0.54772256f,  0.73029674f)));
	ut_test(isEqual(q0.unit(), q));

	// quat default ctor
	q = Quatf();
	ut_test(isEqual(q, Quatf(1, 0, 0, 0)));

	// euler to quaternion
	q = Quatf(euler_check);
	ut_test(isEqual(q, q_check));

	// euler to dcm
	Dcmf dcm(euler_check);
	ut_test(isEqual(dcm, dcm_check));

	// quaternion to euler
	Eulerf e1(q_check);
	ut_test(isEqual(e1, euler_check));

	// quaternion to dcm
	Dcmf dcm1(q_check);
	ut_test(isEqual(dcm1, dcm_check));

	// dcm default ctor
	Dcmf dcm2;
	SquareMatrix<float, 3> I = eye<float, 3>();
	ut_test(isEqual(dcm2, I));

	// dcm to euler
	Eulerf e2(dcm_check);
	ut_test(isEqual(e2, euler_check));

	// dcm to quaterion
	Quatf q2(dcm_check);
	ut_test(isEqual(q2, q_check));

	// constants
	double deg2rad = M_PI / 180.0;
	double rad2deg = 180.0 / M_PI;

	// euler dcm round trip check
	for (int roll = -90; roll <= 90; roll += 90) {
		for (int pitch = -90; pitch <= 90; pitch += 90) {
			for (int yaw = -179; yaw <= 180; yaw += 90) {
				// note if theta = pi/2, then roll is set to zero
				int roll_expected = roll;
				int yaw_expected = yaw;

				if (pitch == 90) {
					roll_expected = 0;
					yaw_expected = yaw - roll;

				} else if (pitch == -90) {
					roll_expected = 0;
					yaw_expected = yaw + roll;
				}

				if (yaw_expected < -180) { yaw_expected += 360; }

				if (yaw_expected > 180) { yaw_expected -= 360; }

				//printf("roll:%d pitch:%d yaw:%d\n", roll, pitch, yaw);
				Euler<double> euler_expected(
					deg2rad * double(roll_expected),
					deg2rad * double(pitch),
					deg2rad * double(yaw_expected));
				Euler<double> euler(
					deg2rad * double(roll),
					deg2rad * double(pitch),
					deg2rad * double(yaw));
				Dcm<double> dcm_from_euler(euler);
				//dcm_from_euler.print();
				Euler<double> euler_out(dcm_from_euler);
				ut_test(isEqual(rad2deg * euler_expected, rad2deg * euler_out));

				Eulerf eulerf_expected(
					float(deg2rad)*float(roll_expected),
					float(deg2rad)*float(pitch),
					float(deg2rad)*float(yaw_expected));
				Eulerf eulerf(float(deg2rad)*float(roll),
					      float(deg2rad)*float(pitch),
					      float(deg2rad)*float(yaw));
				Dcm<float> dcm_from_eulerf(eulerf);
				Euler<float> euler_outf(dcm_from_eulerf);
				ut_test(isEqual(float(rad2deg)*eulerf_expected,
						float(rad2deg)*euler_outf));
			}
		}
	}

	// quaterion copy ctors
	float data_v4[] = {1, 2, 3, 4};
	Vector<float, 4> v4(data_v4);
	Quatf q_from_v(v4);
	ut_test(isEqual(q_from_v, v4));

	Matrix<float, 4, 1> m4(data_v4);
	Quatf q_from_m(m4);
	ut_test(isEqual(q_from_m, m4));

	// quaternion derivate
	Vector<float, 4> q_dot = q.derivative1(Vector3f(1, 2, 3));

	// quaternion product
	Quatf q_prod_check(0.93394439f, 0.0674002f, 0.20851f, 0.28236266f);
	ut_test(isEqual(q_prod_check, q_check * q_check));
	q_check *= q_check;
	ut_test(isEqual(q_prod_check, q_check));

	// Quaternion scalar multiplication
	float scalar = 0.5;
	Quatf q_scalar_mul(1.0f, 2.0f, 3.0f, 4.0f);
	Quatf q_scalar_mul_check(1.0f * scalar, 2.0f * scalar,
				 3.0f * scalar,  4.0f * scalar);
	Quatf q_scalar_mul_res = scalar * q_scalar_mul;
	ut_test(isEqual(q_scalar_mul_check, q_scalar_mul_res));
	Quatf q_scalar_mul_res2 = q_scalar_mul * scalar;
	ut_test(isEqual(q_scalar_mul_check, q_scalar_mul_res2));
	Quatf q_scalar_mul_res3(q_scalar_mul);
	q_scalar_mul_res3 *= scalar;
	ut_test(isEqual(q_scalar_mul_check, q_scalar_mul_res3));

	// quaternion inverse
	q = q_check.inversed();
	ut_test(fabs(q_check(0) - q(0)) < eps);
	ut_test(fabs(q_check(1) + q(1)) < eps);
	ut_test(fabs(q_check(2) + q(2)) < eps);
	ut_test(fabs(q_check(3) + q(3)) < eps);

	q = q_check;
	q.invert();
	ut_test(fabs(q_check(0) - q(0)) < eps);
	ut_test(fabs(q_check(1) + q(1)) < eps);
	ut_test(fabs(q_check(2) + q(2)) < eps);
	ut_test(fabs(q_check(3) + q(3)) < eps);

	// rotate quaternion (nonzero rotation)
	Quatf qI(1.0f, 0.0f, 0.0f, 0.0f);
	Vector<float, 3> rot;
	rot(0) = 1.0f;
	rot(1) = rot(2) = 0.0f;
	qI.rotate(rot);
	Quatf q_true(cosf(1.0f / 2), sinf(1.0f / 2), 0.0f, 0.0f);
	ut_test(fabs(qI(0) - q_true(0)) < eps);
	ut_test(fabs(qI(1) - q_true(1)) < eps);
	ut_test(fabs(qI(2) - q_true(2)) < eps);
	ut_test(fabs(qI(3) - q_true(3)) < eps);

	// rotate quaternion (zero rotation)
	qI = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	rot(0) = 0.0f;
	rot(1) = rot(2) = 0.0f;
	qI.rotate(rot);
	q_true = Quatf(cosf(0.0f), sinf(0.0f), 0.0f, 0.0f);
	ut_test(fabs(qI(0) - q_true(0)) < eps);
	ut_test(fabs(qI(1) - q_true(1)) < eps);
	ut_test(fabs(qI(2) - q_true(2)) < eps);
	ut_test(fabs(qI(3) - q_true(3)) < eps);

	// get rotation axis from quaternion (nonzero rotation)
	q = Quatf(cosf(1.0f / 2), 0.0f, sinf(1.0f / 2), 0.0f);
	rot = q.to_axis_angle();
	ut_test(fabs(rot(0)) < eps);
	ut_test(fabs(rot(1) - 1.0f) < eps);
	ut_test(fabs(rot(2)) < eps);

	// get rotation axis from quaternion (zero rotation)
	q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	rot = q.to_axis_angle();
	ut_test(fabs(rot(0)) < eps);
	ut_test(fabs(rot(1)) < eps);
	ut_test(fabs(rot(2)) < eps);

	// from axis angle (zero rotation)
	rot(0) = rot(1) = rot(2) = 0.0f;
	q.from_axis_angle(rot, 0.0f);
	q_true = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	ut_test(fabs(q(0) - q_true(0)) < eps);
	ut_test(fabs(q(1) - q_true(1)) < eps);
	ut_test(fabs(q(2) - q_true(2)) < eps);
	ut_test(fabs(q(3) - q_true(3)) < eps);

	return true;
}

bool MatrixTest::filterTests()
{
	const size_t n_x = 6;
	const size_t n_y = 5;
	SquareMatrix<float, n_x> P = eye<float, n_x>();
	SquareMatrix<float, n_y> R = eye<float, n_y>();
	Matrix<float, n_y, n_x> C;
	C.setIdentity();
	float data[] = {1, 2, 3, 4, 5};
	Vector<float, n_y> r(data);

	Vector<float, n_x> dx;
	SquareMatrix<float, n_x> dP;
	float beta = 0;
	kalman_correct<float, 6, 5>(P, C, R, r, dx, dP, beta);

	float data_check[] = {0.5, 1, 1.5, 2, 2.5, 0};
	Vector<float, n_x> dx_check(data_check);
	ut_test(isEqual(dx, dx_check));

	return true;
}

bool MatrixTest::helperTests()
{
	ut_test(fabs(wrap_pi(4.0) - (4.0 - 2 * M_PI)) < 1e-5);
	ut_test(fabs(wrap_pi(-4.0) - (-4.0 + 2 * M_PI)) < 1e-5);
	ut_test(fabs(wrap_pi(3.0) - (3.0)) < 1e-3);
	wrap_pi(NAN);

	Vector3f a(1, 2, 3);
	Vector3f b(4, 5, 6);
	ut_test(isEqual(a, a));

	return true;
}


Vector<float, 6> f(float t, const Matrix<float, 6, 1> &y, const Matrix<float, 3, 1> &u);

Vector<float, 6> f(float t, const Matrix<float, 6, 1> &y, const Matrix<float, 3, 1> &u)
{
	float v = -sinf(t);
	return v * ones<float, 6, 1>();
}

bool MatrixTest::integrationTests()
{
	Vector<float, 6> y = ones<float, 6, 1>();
	Vector<float, 3> u = ones<float, 3, 1>();
	float t0 = 0;
	float tf = 2;
	float h = 0.001f;
	integrate_rk4(f, y, u, t0, tf, h, y);
	float v = 1 + cosf(tf) - cosf(t0);
	ut_test(isEqual(y, (ones<float, 6, 1>()*v)));

	return true;
}

template class matrix::SquareMatrix<float, 3>;

bool MatrixTest::inverseTests()
{
	float data[9] = {0, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	float data_check[9] = {
		-0.4f, -0.8f,  0.6f,
		-0.4f,  4.2f, -2.4f,
		0.6f, -2.8f,  1.6f
	};

	SquareMatrix<float, 3> A(data);
	SquareMatrix<float, 3> A_I = inv(A);
	SquareMatrix<float, 3> A_I_check(data_check);

	float eps = 1e-5;

	ut_test((A_I - A_I_check).abs().max() < eps);

	SquareMatrix<float, 3> zero_test = zeros<float, 3, 3>();
	inv(zero_test);

	return true;
}

bool MatrixTest::matrixAssignmentTests()
{
	Matrix3f m;
	m.setZero();
	m(0, 0) = 1;
	m(0, 1) = 2;
	m(0, 2) = 3;
	m(1, 0) = 4;
	m(1, 1) = 5;
	m(1, 2) = 6;
	m(2, 0) = 7;
	m(2, 1) = 8;
	m(2, 2) = 9;

	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f m2(data);

	double eps = 1e-6f;

	for (int i = 0; i < 9; i++) {
		ut_test(fabs(data[i] - m2.data()[i]) < eps);
	}

	float data_times_2[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f m3(data_times_2);

	ut_test(isEqual(m, m2));
	ut_test(!isEqual(m, m3));

	m2 *= 2;
	ut_test(isEqual(m2, m3));

	m2 /= 2;
	m2 -= 1;
	float data_minus_1[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ut_test(isEqual(Matrix3f(data_minus_1), m2));

	m2 += 1;
	ut_test(isEqual(Matrix3f(data), m2));

	m3 -= m2;

	ut_test(isEqual(m3, m2));

	float data_row_02_swap[9] = {
		7, 8, 9,
		4, 5, 6,
		1, 2, 3,
	};

	float data_col_02_swap[9] = {
		3, 2, 1,
		6, 5, 4,
		9, 8, 7
	};

	Matrix3f m4(data);

	ut_test(isEqual(-m4, m4 * (-1)));

	m4.swapCols(0, 2);
	ut_test(isEqual(m4, Matrix3f(data_col_02_swap)));
	m4.swapCols(0, 2);
	m4.swapRows(0, 2);
	ut_test(isEqual(m4, Matrix3f(data_row_02_swap)));
	ut_test(fabs(m4.min() - 1) < 1e-5);

	Scalar<float> s;
	s = 1;
	ut_test(fabs(s - 1) < 1e-5);

	Matrix<float, 1, 1> m5 = s;
	ut_test(fabs(m5(0, 0) - s) < 1e-5);

	Matrix<float, 2, 2> m6;
	m6.setRow(0, Vector2f(1, 1));
	m6.setCol(0, Vector2f(1, 1));

	return true;
}

bool MatrixTest::matrixMultTests()
{
	float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
	Matrix3f A(data);
	float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
	Matrix3f A_I(data_check);
	Matrix3f I;
	I.setIdentity();
	Matrix3f R = A * A_I;
	ut_test(isEqual(R, I));

	Matrix3f R2 = A;
	R2 *= A_I;
	ut_test(isEqual(R2, I));


	Matrix3f A2 = eye<float, 3>() * 2;
	Matrix3f B = A2.emult(A2);
	Matrix3f B_check = eye<float, 3>() * 4;
	ut_test(isEqual(B, B_check));

	return true;
}

bool MatrixTest::matrixScalarMultTests()
{
	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f A(data);
	A = A * 2;
	float data_check[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f A_check(data_check);
	ut_test(isEqual(A, A_check));

	return true;
}


template class matrix::Matrix<float, 3, 3>;

bool MatrixTest::setIdentityTests()
{
	Matrix3f A;
	A.setIdentity();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j) {
				ut_test(fabs(A(i, j) -  1) < 1e-7);

			} else {
				ut_test(fabs(A(i, j) -  0) < 1e-7);
			}
		}
	}

	return true;
}

bool MatrixTest::sliceTests()
{
	float data[9] = {0, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	float data_check[6] = {
		4, 5, 6,
		7, 8, 10
	};
	SquareMatrix<float, 3> A(data);
	Matrix<float, 2, 3> B_check(data_check);
	Matrix<float, 2, 3> B(A.slice<2, 3>(1, 0));
	ut_test(isEqual(B, B_check));

	float data_2[4] = {
		11, 12,
		13, 14
	};

	Matrix<float, 2, 2> C(data_2);
	A.set(C, 1, 1);

	float data_2_check[9] = {
		0, 2, 3,
		4, 11, 12,
		7, 13, 14
	};
	Matrix<float, 3, 3> D(data_2_check);
	ut_test(isEqual(A, D));

	return true;
}


bool MatrixTest::squareMatrixTests()
{
	float data[9] = {1, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	SquareMatrix<float, 3> A(data);
	Vector3<float> diag_check(1, 5, 10);

	ut_test(isEqual(A.diag(), diag_check));

	float data_check[9] = {
		1.01158503f,  0.02190432f,  0.03238144f,
		0.04349195f,  1.05428524f,  0.06539627f,
		0.07576783f,  0.08708946f,  1.10894048f
	};

	float dt = 0.01f;
	SquareMatrix<float, 3> eA = expm(SquareMatrix<float, 3>(A * dt), 5);
	SquareMatrix<float, 3> eA_check(data_check);

	float eps = 1e-3;
	ut_test((eA - eA_check).abs().max() < eps);

	return true;
}

bool MatrixTest::transposeTests()
{
	float data[6] = {1, 2, 3, 4, 5, 6};
	Matrix<float, 2, 3> A(data);
	Matrix<float, 3, 2> A_T = A.transpose();
	float data_check[6] = {1, 4, 2, 5, 3, 6};
	Matrix<float, 3, 2> A_T_check(data_check);
	ut_test(isEqual(A_T, A_T_check));

	return true;
}

bool MatrixTest::vectorTests()
{
	float data1[] = {1, 2, 3, 4, 5};
	float data2[] = {6, 7, 8, 9, 10};
	Vector<float, 5> v1(data1);
	ut_test(fabs(v1.norm() - 7.416198487095663f) < 1e-5);
	Vector<float, 5> v2(data2);
	ut_test(fabs(v1.dot(v2) - 130.0f) < 1e-5);
	v2.normalize();
	Vector<float, 5> v3(v2);
	ut_test(isEqual(v2, v3));
	float data1_sq[] = {1, 4, 9, 16, 25};
	Vector<float, 5> v4(data1_sq);
	ut_test(isEqual(v1, v4.pow(0.5)));

	return true;
}

bool MatrixTest::vector2Tests()
{
	Vector2f a(1, 0);
	Vector2f b(0, 1);
	ut_test(fabs(a % b - 1.0f) < 1e-5);

	Vector2f c;
	ut_test(fabs(c(0) - 0) < 1e-5);
	ut_test(fabs(c(1) - 0) < 1e-5);

	Matrix<float, 2, 1> d(a);
	ut_test(fabs(d(0, 0) - 1) < 1e-5);
	ut_test(fabs(d(1, 0) - 0) < 1e-5);

	Vector2f e(d);
	ut_test(fabs(e(0) - 1) < 1e-5);
	ut_test(fabs(e(1) - 0) < 1e-5);

	float data[] = {4, 5};
	Vector2f f(data);
	ut_test(fabs(f(0) - 4) < 1e-5);
	ut_test(fabs(f(1) - 5) < 1e-5);
	return true;
}

bool MatrixTest::vector3Tests()
{
	Vector3f a(1, 0, 0);
	Vector3f b(0, 1, 0);
	Vector3f c = a.cross(b);
	ut_test(isEqual(c, Vector3f(0, 0, 1)));
	c = a % b;
	ut_test(isEqual(c, Vector3f(0, 0, 1)));
	Matrix<float, 3, 1> d(c);
	Vector3f e(d);
	ut_test(isEqual(e, d));
	float data[] = {4, 5, 6};
	Vector3f f(data);
	ut_test(isEqual(f, Vector3f(4, 5, 6)));
	return true;
}

bool MatrixTest::vectorAssignmentTests()
{
	Vector3f v;
	v(0) = 1;
	v(1) = 2;
	v(2) = 3;

	static const float eps = 1e-7f;

	ut_test(fabsf(v(0) - 1) < eps);
	ut_test(fabsf(v(1) - 2) < eps);
	ut_test(fabsf(v(2) - 3) < eps);

	Vector3f v2(4, 5, 6);

	ut_test(fabsf(v2(0) - 4) < eps);
	ut_test(fabsf(v2(1) - 5) < eps);
	ut_test(fabsf(v2(2) - 6) < eps);

	SquareMatrix<float, 3> m = diag(Vector3f(1, 2, 3));
	ut_test(fabsf(m(0, 0) - 1) < eps);
	ut_test(fabsf(m(1, 1) - 2) < eps);
	ut_test(fabsf(m(2, 2) - 3) < eps);

	return true;
}

bool MatrixTest::dcmRenormTests()
{
	bool verbose = true;

	Dcm<float> A = eye<float, 3>();
	Euler<float> euler(0.1f, 0.2f, 0.3f);
	Dcm<float> R(euler);

	// demonstrate need for renormalization
	for (int i = 0; i < 1000; i++) {
		A = R * A;
	}

	float err = 0.0f;

	if (verbose) {
		for (int row = 0; row < 3; row++) {
			matrix::Vector3f rvec(A._data[row]);
			err += fabsf(1.0f - rvec.length());
		}

		printf("error: %e\n", (double)err);
	}

	A.renormalize();

	err = 0.0f;

	for (int row = 0; row < 3; row++) {
		matrix::Vector3f rvec(A._data[row]);
		err += fabsf(1.0f - rvec.length());
	}

	if (verbose) {
		printf("renorm error: %e\n", (double)err);
	}

	static const float eps = 1e-6f;
	ut_test(err < eps);

	return true;
}
