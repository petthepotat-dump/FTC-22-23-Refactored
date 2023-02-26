package org.firstinspires.ftc.teamcode.wrappers;

public class Controller {
    private final double DEGREES_TO_RADIANS = Math.PI/180f;
    private final double MAX_VELOCITY = 3.6;
    private final double MIN_VELOCITY = 0.05;
    private final double ACCELERATE = 0.5;

    private final double ANGLE_MAX_VELOCITY = 220;
    private final double ANGLE_MIN_VELOCITY = 2;
    private final double ANGLE_ACCELERATE = 40;

    private final double WHEEL_ANGLE = 678.0/892.0;
    private final double ROBOT_LENGTH = 0.114;
    private final double ROBOT_WIDTH = 0.119;

    private final double BRAKE_DISTANCE = 0.7;
    private final double BRAKE_DISTANCE_RATIO = (MAX_VELOCITY-MIN_VELOCITY)/BRAKE_DISTANCE;
    private final double BRAKE_ANGLE = 30;
    private final double BRAKE_ANGLE_RATIO = (ANGLE_MAX_VELOCITY-ANGLE_MIN_VELOCITY)/BRAKE_ANGLE;

    private MecanumChassis robot;
    private Position pos;
    private Controller.ControllerThread controllerThread;
    private double x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone;
    private boolean velocityControl;
    public volatile boolean finished = true;

    public Controller(MecanumChassis robot, Position position) {
        this.robot = robot;
        this.pos = position;
        controllerThread = new ControllerThread();
    }
    public void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.angleSpeed = angleSpeed;
        this.distanceDeadzone = distanceDeadzone;
        this.angleDeadzone = angleDeadzone;
        this.speed = speed;
        this.velocityControl = velocityControl;
        this.finished = false;
        controllerThread.start();
    }
    public void updatePos(double x, double y, double angle, boolean velocityControl) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velocityControl = velocityControl;
    }
    public void stop() {
        controllerThread.interrupt();
        finished = true;
    }
    private double heading(double theta) {
        if (Math.abs(theta)%360>180) return Math.signum(theta)*(Math.abs(theta)%360-360);
        else return Math.signum(theta)*(Math.abs(theta)%360);
    }
    private void setPower(double fr, double fl, double br, double bl) {
        double mag = 1/Math.max(Math.max(Math.max(Math.max(fr, fl),br),bl),MAX_VELOCITY);
        robot.fr.setPower(fr*mag);
        robot.fl.setPower(fl*mag);
        robot.br.setPower(br*mag);
        robot.bl.setPower(bl*mag);
    }
    private class ControllerThread extends Thread {
        public ControllerThread() {}
        @Override
        public void run() {
            double distanceError, angleError, theta, vx, vy, power, anglePower, prevAnglePower=0, prevPower=0;
            try {
                while (!isInterrupted() && (!finished)) {
                    distanceError = Math.sqrt((x-pos.x)*(x-pos.x)+(y-pos.y)*(y-pos.y));
                    angleError = heading(angle-pos.angle);
                    if (distanceError<distanceDeadzone && Math.abs(angleError)<angleDeadzone) {
                        finished = true;
                        setPower(0,0,0,0);
                    } else {
                        if (velocityControl) {
                            if (distanceError<(speed-MIN_VELOCITY)/BRAKE_DISTANCE_RATIO) power = Math.abs(distanceError)*BRAKE_DISTANCE_RATIO+MIN_VELOCITY;
                            else power = prevPower+ACCELERATE;
                            if (power>speed) power = speed;
                            if (power<MIN_VELOCITY) power = MIN_VELOCITY;
                        } else power = speed;
                        if (Math.abs(angleError)<(angleSpeed-ANGLE_MIN_VELOCITY)/BRAKE_ANGLE_RATIO) anglePower = Math.signum(angleError)*(Math.abs(angleError)*BRAKE_ANGLE_RATIO+ANGLE_MIN_VELOCITY);
                        else anglePower = Math.signum(angleError)*(Math.abs(prevAnglePower)+ANGLE_ACCELERATE);
                        if (Math.abs(anglePower) > Math.abs(angleSpeed)) anglePower = Math.signum(anglePower) * Math.abs(angleSpeed);
                        if (Math.abs(anglePower) < ANGLE_MIN_VELOCITY)  anglePower = Math.signum(anglePower) * ANGLE_MIN_VELOCITY;
                        prevAnglePower = anglePower;
                        prevPower = power;
                        anglePower *= DEGREES_TO_RADIANS*(ROBOT_WIDTH*WHEEL_ANGLE+ROBOT_LENGTH)/WHEEL_ANGLE;
                        theta = Math.atan2(x-pos.x,y-pos.y);
                        vx = -power*Math.sin(theta-pos.angle*DEGREES_TO_RADIANS);
                        vy = -power*Math.cos(theta-pos.angle*DEGREES_TO_RADIANS);
                        setPower(
                                vy-vx/WHEEL_ANGLE+anglePower,
                                vy+vx/WHEEL_ANGLE-anglePower,
                                vy+vx/WHEEL_ANGLE+anglePower,
                                vy-vx/WHEEL_ANGLE-anglePower
                        );
                        Thread.sleep(10);
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}