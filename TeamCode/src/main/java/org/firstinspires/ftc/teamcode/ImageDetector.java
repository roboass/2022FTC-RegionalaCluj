package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.CopyOnWriteArraySet;


public class ImageDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public static String position = "FIRST";
    public double firstT, secondT, thirdT;
    public int rows = 720, cols = 1080;
    //int rows = firstFrame.rows(), cols = firstFrame.cols();

    int HEIGHT = 100, WIDTH = 200;
    Point p1ss = new Point(0, 0),
          p2ss = new Point(400, 0),
          p3ss = new Point(800, 0);


    Point p1dj = new Point(p1ss.x + HEIGHT, p1ss.y + WIDTH),
          p2dj = new Point(p2ss.x + HEIGHT, p2ss.y + WIDTH),
          p3dj = new Point(p3ss.x + HEIGHT, p3ss.y + WIDTH);

    Mat posOne, posTwo, posThree;

    public ImageDetector() {
    }

    @Override
    public void init(Mat firstFrame)
    {
        posOne = firstFrame.submat(new Rect(p1ss, p1dj));
        posTwo = firstFrame.submat(new Rect(p2ss, p2dj));
        posThree = firstFrame.submat(new Rect(p3ss, p3dj));
    }
    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
//        posOne = input.submat(new Rect(p1ss, p1dj));
//        posTwo = input.submat(new Rect(p2ss, p2dj));
//        posThree = input.submat(new Rect(p3ss, p3dj));
        //Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2XYZ);
        //Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_XYZ2BGR);

        //Mat matFourRings = workingMatrix.submat(147, 167, 0, 70);

        /*
        Imgproc.rectangle(workingMatrix, new Rect(p1xss+0, p1yss+0, p1ydj - p1yss, p1xdj - p1xss), new Scalar(255, 0, 0));
        Imgproc.rectangle(workingMatrix, new Rect(p2xss, p2yss, p2ydj - p2yss, p2xdj - p2xss), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(p3xss, p3yss, p3ydj - p3yss, p3xdj - p3xss), new Scalar(0, 0, 255));
        */
        Imgproc.rectangle(workingMatrix, p1ss, p1dj, new Scalar(255, 0, 0), 4);
        Imgproc.rectangle(workingMatrix, p2ss, p2dj, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(workingMatrix, p3ss, p3dj, new Scalar(0, 0, 255), 4);

        firstT = Core.sumElems(posOne).val[0];
        secondT = Core.sumElems(posTwo).val[0];
        thirdT = Core.sumElems(posThree).val[0];

        double firstmax = Math.max(firstT, secondT),
               lastmax = Math.max(firstmax, thirdT);

        if(lastmax == firstT) position = "FIRST";
        else if(lastmax == secondT) position = "SECOND";
        else position = "THIRD";
        //oneRingTotal = Core.sumElems(matOneRing).val[2];

        //fourRingsTotal = Core.sumElems(matFourRings).val[2];
/*
       double thresholdOne = 62500, thresholdFour = 135000; //old thresholdFour = 130000
       if(fourRingsTotal < thresholdFour) count = "FOUR";
       else if(oneRingTotal < thresholdOne) count = "ONE";
       else count = "ZERO";
       /*if(oneRingTotal < thresholdOne)
       {
           if(fourRingsTotal < thresholdFour)
           {
               count = "FOUR";
           }
           else
               count = "ONE";
       } else count = "ZERO";
 */
        return workingMatrix;
    }
    public String getPosition()
    {
        return position;
    }
    public double getFirstT()
    {
        return firstT;
    }
    public double getSecondT()
    {
        return secondT;
    }
    public double getThirdT() { return thirdT; }

}
