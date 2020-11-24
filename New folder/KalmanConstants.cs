using Emgu.CV;
using Emgu.CV.Structure;
using System;
using System.Collections.Generic;
using System.Text;

namespace KalmanFilter
{
    public class KalmanConstants
    {
        #region fields
        public Matrix<double> transitionMatrix;
        public Matrix<double> measurementMatrix;
        public Matrix<double> processNoise;
        public Matrix<double> measurementNoise;
        public Matrix<double> identityMatrix;
        #endregion
        #region constructor
        public KalmanConstants()
        {
            transitionMatrix = new Matrix<double>(new double[,]
                   {
                        {1, 0, 1, 0},  // x-pos, y-pos, x-velocity, y-velocity
                        {0, 1, 0, 1},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1}
                   });
            identityMatrix = new Matrix<double>(new double[,]
                    {
                        {1, 0, 0, 0},  // x-pos, y-pos, x-velocity, y-velocity
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1}
                    });
            measurementMatrix = new Matrix<double>(new double[,]
                    {
                        { 1, 0, 0, 0 },
                        { 0, 1, 0, 0 }
                    });
            measurementMatrix.SetIdentity();
            processNoise = new Matrix<double>(4, 4); //Linked to the size o the transition matrix
            processNoise.SetIdentity(new MCvScalar(1.0e-4)); //The smaller the value the more resistance to noise 
            measurementNoise = new Matrix<double>(2, 2); //ixed accordiong to input data 
            measurementNoise.SetIdentity(new MCvScalar(1.0e-1));
        }
        #endregion
        #region properties
        public Matrix<double> TransitionMatrix
        {
            get { return transitionMatrix; }
        }
        public Matrix<double> MeasurementMatrix
        {
            get { return measurementMatrix; }
        }
        public Matrix<double> MeasurementNoise
        {
            get { return measurementNoise; }
        }
        public Matrix<double> ProcessNoise
        {
            get { return processNoise; }
        }
        public Matrix<double> IdentityMatrix
        {
            get { return IdentityMatrix; }
        }
        #endregion
    }
}
