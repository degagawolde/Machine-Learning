using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using DocumentFormat.OpenXml.Drawing;
using DocumentFormat.OpenXml;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using System.IO;

namespace KalmanFilter
{
    public class Kalman
    {
        //instantiate the KalmanConstants class to use the matrices 
        KalmanConstants con = new KalmanConstants();

        /// <summary>
        /// the implimentation of kalman filter
        /// </summary>
        /// <param name="measurement"></param>
        /// <param name="state"></param>
        /// <param name="errorCovariancePost"></param>
        public void kalmanf(Matrix<double> measurement, ref Matrix<double> state,ref Matrix<double> errorCovariancePost) {
           
            Matrix<double> TT = new Matrix<double>(4, 4);   
            Matrix<double> A = new Matrix<double>(4, 4);
            Matrix<double> mT = new Matrix<double>(4, 2);
            Matrix<double> y, K;

            //predict
            CvInvoke.Transpose(con.TransitionMatrix, TT);
            state = con.TransitionMatrix * state;
            errorCovariancePost = con.TransitionMatrix * errorCovariancePost * TT + con.ProcessNoise;

            //update
            CvInvoke.Transpose(con.MeasurementMatrix, mT);
            Matrix<double> B = (con.MeasurementMatrix * errorCovariancePost * mT) + con.MeasurementNoise;

            CvInvoke.Invert(B, A, 0);
            K = (errorCovariancePost * mT) * A;
            y = con.MeasurementMatrix * measurement;
            state = state + K * (y - con.MeasurementMatrix * state);
            errorCovariancePost = (con.IdentityMatrix - K * con.MeasurementMatrix) * errorCovariancePost;
        }
    }
}
