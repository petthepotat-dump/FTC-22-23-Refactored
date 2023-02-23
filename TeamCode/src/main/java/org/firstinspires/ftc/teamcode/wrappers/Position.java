package org.firstinspires.ftc.teamcode.wrappers;

public class Position {
    private static final double STRAFE_RATIO = 678f/892f;
    private static final double TICKS_TO_METERS = 1.3*0.38/658f;
    private static final double DEGREES_TO_RADIANS = Math.PI/180f;
    private MecanumChassis robot;
    private Position.PositionThread positionThread;

    private double fr, fl, bl, br;
    public volatile double angle, x, y;

    public Position(MecanumChassis robot) {
        this.robot = robot;
        positionThread = new PositionThread();
        positionThread.start();
    }
    public void updatePos(double x, double y) {
        this.x = x;
        this.y = y;
    }
    private class PositionThread extends Thread {
        public PositionThread() {
            fr = robot.fr.getCurrentPosition();
            fl = robot.fl.getCurrentPosition();
            br = robot.br.getCurrentPosition();
            bl = robot.bl.getCurrentPosition();
            x = 0; y = 0;
            angle = robot.getHeading();
        }
        @Override
        public void run() {
            double a, b, dx, dy;
            try {
                while (!isInterrupted()) {
                    angle = robot.getHeading();
                    a = (robot.fr.getCurrentPosition()+robot.bl.getCurrentPosition()-fr-bl)*TICKS_TO_METERS;
                    b = (robot.fl.getCurrentPosition()+robot.br.getCurrentPosition()-fl-br)*TICKS_TO_METERS;
                    fr = robot.fr.getCurrentPosition();
                    fl = robot.fl.getCurrentPosition();
                    br = robot.br.getCurrentPosition();
                    bl = robot.bl.getCurrentPosition();
                    dx = (a-b)*0.25*STRAFE_RATIO;
                    dy = (a+b)*0.25;
                    x+=dx*Math.cos(angle*DEGREES_TO_RADIANS)-dy*Math.sin(angle*DEGREES_TO_RADIANS);
                    y+=-dx*Math.sin(angle*DEGREES_TO_RADIANS)-dy*Math.cos(angle*DEGREES_TO_RADIANS);
                    Thread.sleep(10);
                }
            } catch (Exception e) {}
        }
    }
}
