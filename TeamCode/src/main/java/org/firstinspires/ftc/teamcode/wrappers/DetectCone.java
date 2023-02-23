package org.firstinspires.ftc.teamcode.wrappers;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class DetectCone extends OpenCvPipeline {
    private static final Scalar
        lowerBlue = new Scalar(100, 100, 100),
        upperBlue = new Scalar(120, 255, 255),
        lowerRed = new Scalar(175, 100, 100),
        upperRed = new Scalar(185, 255, 255);
        private Mat red = new Mat(), blue = new Mat(), hsv = new Mat();
        private double maxArea;
        private Rect rect;
        private boolean team;
        public volatile double x=0, y=0, width=0, height=0;
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public DetectCone(boolean blueTeam) { team = blueTeam; }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowerBlue, upperBlue, blue);
        Core.inRange(hsv, lowerRed, upperRed, red);
        contours.clear();
        if (team) {
            Imgproc.findContours(blue, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            if (contours.size() > 0) {
                maxArea = 0;
                for (int i = 0; i<contours.size(); i++) {
                    if (maxArea < Imgproc.contourArea(contours.get(i))) {
                        maxArea = Imgproc.contourArea(contours.get(i));
                        rect = Imgproc.boundingRect(contours.get(i));
                    }
                }
            }
        } else {
            Imgproc.findContours(red, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            if (contours.size() > 0) {
                maxArea = 0;
                for (int i = 0; i<contours.size(); i++) {
                    if (maxArea < Imgproc.contourArea(contours.get(i))) {
                        maxArea = Imgproc.contourArea(contours.get(i));
                        rect = Imgproc.boundingRect(contours.get(i));
                    }
                }
            }
        }
        if (maxArea>100) {
            Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x+rect.width, rect.y+rect.height), new Scalar(0, 255, 0), 2);
            x = rect.x;
            y = rect.y;
            width = rect.width;
            height = rect.height;
        }
        return input;
    }
}
