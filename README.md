# Kalman-Filter-In-C#
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace EthSignTranslator
{
    public class kalmanf
    {
        private Matrix<double> transitionMatrix;
        private Matrix<double> measurementMatrix;
        private Matrix<double> processNoise;
        private Matrix<double> measurementNoise;
        private Matrix<double> IdentityMatrix;
        public kalmanf()
        {
            transitionMatrix = Matrix.Build.DenseIdentity(2);
            measurementMatrix = Matrix.Build.DenseIdentity(2); ;

            processNoise = Matrix.Build.Diagonal(2, 2, 1.0e-4);
            measurementNoise = Matrix.Build.Diagonal(2, 2, 1.0e-1);
            IdentityMatrix = Matrix.Build.DenseIdentity(2);
        }
        public void kalmanFilter(Matrix<double> measurement, ref Matrix<double> state, ref Matrix<double> errorCovariancePost)
        {

            //predict
            state = transitionMatrix.Multiply(state);
            errorCovariancePost = transitionMatrix.Multiply(errorCovariancePost).Multiply(transitionMatrix.Transpose()) + processNoise;

            //update
            Matrix<double> S = (measurementMatrix.Multiply(errorCovariancePost)).Multiply(measurementMatrix.Transpose()) + measurementNoise;
            Matrix<double> K = (errorCovariancePost.Multiply(measurementMatrix.Transpose())).Multiply(S.Inverse());
            state = state + K.Multiply(measurement - measurementMatrix.Multiply(state));
            errorCovariancePost = (IdentityMatrix - (K.Multiply(measurementMatrix)).Multiply(errorCovariancePost));
        }
    }
}
