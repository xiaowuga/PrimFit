/* This file is part of PyMesh. Copyright (c) 2015 by Qingnan Zhou */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

//namespace PyMesh {

//typedef float Float;
typedef Eigen::VectorXf VectorF;
typedef Eigen::VectorXd VectorD;
typedef Eigen::VectorXi VectorI;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::Vector4i Vector4I;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3I;
typedef Eigen::Vector2f Vector2F;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector2i Vector2I;

typedef Eigen::MatrixXf MatrixF;
typedef Eigen::MatrixXd MatrixD;
typedef Eigen::Matrix2f Matrix2F;
typedef Eigen::Matrix2d Matrix2D;
typedef Eigen::Matrix3f Matrix3F;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Matrix4f Matrix4F;
typedef Eigen::Matrix4d Matrix4D;
typedef Eigen::MatrixXi MatrixI;
typedef Eigen::Matrix2i Matrix2I;
typedef Eigen::Matrix3i Matrix3I;
typedef Eigen::Matrix4i Matrix4I;

typedef Eigen::Matrix<int  , Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixIr;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixFr;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixDr;


typedef Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor> Matrix2Ir;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> Matrix2Fr;
typedef Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> Matrix2Dr;
typedef Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> Matrix3Ir;
typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Matrix3Fr;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> Matrix3Dr;
typedef Eigen::Matrix<int, Eigen::Dynamic, 4, Eigen::RowMajor> Matrix4Ir;
typedef Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> Matrix4Fr;
typedef Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor> Matrix4Dr;
//}
