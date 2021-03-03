package org.firstinspires.ftc.teamcode.NopeRopeLibs.auto;

public class MovementConstants {
    public double[][] constants;
    public double xTarget;
    public double yTarget;
    public double zTarget;

    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;

    private static final int kp = 0;
    private static final int ki = 1;
    private static final int kd = 2;

    public double timeout;

    public MovementConstants(double x, double y, double z, double timeout){
        constants = new double[3][3];
        this.xTarget = x;
        this.yTarget = y;
        this.zTarget = z;
        this.timeout = timeout;
    }

    public void setValue(int r, int c, double val){
        constants[r][c] = val;
    }

    public void initConstants(double[]...values){
        for (double[] v: values){
            setValue((int)v[0], (int)v[1], v[2]);
        }
    }

}
