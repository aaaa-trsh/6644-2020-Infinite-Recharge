package frc.robot.Classes;

import java.awt.geom.Point2D;


public class BezierCurve 
{
    Point2D endPoint, control1, control2;

    public BezierCurve(Point2D _control1, Point2D _control2, Point2D _endPoint) 
    {
        control1 = new Point2D.Double(_control1.getX(), _control1.getY());
        control2 = new Point2D.Double(_control2.getX(), _control2.getY());
        endPoint = new Point2D.Double(_endPoint.getX(), _endPoint.getY());
    }

    public Point2D getPos(double t) {
        //return ((1 - t) * (1 - t) * (1 - t)) * 0 + 3 * ((1 - t) * (1 - t)) * t * control1.getY()
        //        + 3 * (1 - t) * (t * t) * control2.getY() + (t * t * t) * endPoint.getY();
        return new Point2D.Double(
                (1 - t) * ((1 - t) * ((1 - t) * 0 + t * control1.getX())
                        + t * ((1 - t) * control1.getX() + t * control2.getX()))
                        + t * ((1 - t) * ((1 - t) * control1.getX() + t * control2.getX())
                                + t * ((1 - t) * control2.getX() + t * endPoint.getX())),
                (1 - t) * ((1 - t) * ((1 - t) * 0 + t * control1.getY())
                        + t * ((1 - t) * control1.getY() + t * control2.getY()))
                        + t * ((1 - t) * ((1 - t) * control1.getY() + t * control2.getY())
                                + t * ((1 - t) * control2.getY() + t * endPoint.getY())));
    }

    public double getDX(double t) {
        return Math.pow(3 * (1 - t), 2) * (control1.getX() - 0) + 6 * (1 - t) * t * (control2.getX() - control1.getX())
                + 3 * Math.pow(t, 2) * (endPoint.getX() - control2.getX());
    }

    public double getDY(double t) {
        return Math.pow(3 * (1 - t), 2) * (control1.getY() - 0) + 6 * (1 - t) * t * (control2.getY() - control1.getY())
                + 3 * Math.pow(t, 2) * (endPoint.getY() - control2.getY());
    }

    public double getAngle(double t) 
    {
        return Math.toDegrees(Math.atan2(getDY(t), getDX(t)));
    }

    /*public double getLength(double samples) 
    {
        double length = 0;
        double unit = 1 / samples;

        for (double t = unit; t <= 1; t += unit) {
            double prev = t - unit;
            length += Math.sqrt(Math.pow(getPos(t).getX() - getPos(prev).getX(), 2)
                    + Math.pow(getPos(t).getY() - getPos(prev).getY(), 2));
        }
        return length;
    }*/

    public double getLength(int samples)
    {
        int len = samples;
        double[] arcLengths = new double[len + 1];
        arcLengths[0] = 0;

        double ox = getPos(0).getX();
        double oy = getPos(0).getY();
        double clen = 0;

        for (int i = 1; i <= len; i += 1) 
        {
            double x = getPos(i * 0.01).getX();
            double y = getPos(i * 0.01).getY();
            double dx = ox - x, dy = oy - y;
            clen += Math.sqrt(dx * dx + dy * dy);
            arcLengths[i] = clen;
            ox = x;
            oy = y;
        }
        return clen;
    }

    int integral, previous_error, error;
    //AD gyro;
/*
    public double TurnPID(double target, double P, double I, double D) {
        error = target - gyro.getAngle(); // Error = Target - Actual
        integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
        // IterativeRobot)
        double derivative = (error - previous_error) / .02;
        return P * error + I * integral + D * derivative;
    }*/
}
