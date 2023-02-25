package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.BaseAuto;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="ColorDetection")
public class color extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, false);

        ColorDetection colorDetection = new ColorDetection();
        camera.setPipeline(colorDetection);

        while(!isStarted()){
            telemetry.addData("ColorHSV", colorDetection.getColor());
            telemetry.update();
        }
    }

    static class ColorDetection extends OpenCvPipeline{
        Mat hsv;
        double[] color = new double[3];
        @Override
        public Mat processFrame(Mat input) {
            // draw circle in center then collect center color hsv value
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Imgproc.circle(input, new Point(0,0), 10, new Scalar(0, 255, 0), 2);
            // find color
            color = hsv.get(hsv.height()/2, hsv.width()/2);
            return input;
        }
        public String getColor(){
            return String.format("H: %f, S: %f, V: %f", color[0], color[1], color[2]);
        }
    }

}
