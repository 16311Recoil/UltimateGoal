package org.firstinspires.ftc.teamcode.NopeRopeLibs.auto;

import java.util.ArrayList;

public class AutoBuilder {
    private ArrayList<MovementConstants> scan, redA, redB, redC;
    private ArrayList<MovementConstants> current;

    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;

    private static final int kp = 0;
    private static final int ki = 1;
    private static final int kd = 2;

    private static final double FORWARD = Math.PI/2;
    private static final double BACKWARD = 3 * Math.PI/2;
    private static final double LEFT = Math.PI;
    private static final double RIGHT = 2 * Math.PI;

    private boolean readyToScan;
    private int stackSize = -1;
    private boolean handOff;

    public AutoBuilder(){

        MovementConstants movement1 = new MovementConstants(75.0, 0,  BACKWARD, 4 * (1000));
        MovementConstants movement2 = new MovementConstants(0.0, -25,  LEFT, 3 * (1000));

        double[] c_1 = {X, kp, (0.90 / 70.0)};
        double[] c_2 = {X, kd, 0.1};

        movement1.initConstants(c_1, c_2);

        double[] c_3 = {Y, kp,  0.9 / 20};
        double[] c_4 = {Z, kp, 0.035};
        double[] c_5 = {Z, kd, 0.25};

        movement2.initConstants(c_3, c_4, c_5);
        scan.add(movement1);
        scan.add(movement2);

        readyToScan = false;
        handOff = false;

        if (stackSize < 0)
            current = scan;
    }

    public void setStackSize(int stackSize){
        this.stackSize = stackSize;
        if (stackSize == 0)
            current = redA;
        else if (stackSize == 1)
            current = redB;
        else if (stackSize >= 3)
            current = redC;
        else
            current = redA;
    }

    public ArrayList<MovementConstants> getCurrent(){
        return current;
    }

    public ArrayList<MovementConstants> score(){return null;}

    public void setRedB(){
        MovementConstants movement3 = new MovementConstants(0.0, 5,  RIGHT, 3 * (1000));

        movement3.setValue(Y, kp, (0.90 / 20.0));
        movement3.setValue(Z, kp, 0.035);
        movement3.setValue(Z, kd, 0.15);

        MovementConstants movement4 = new MovementConstants(95.0, 0,  BACKWARD, 4 * (1000));

        movement4.setValue(X, kp, (0.90/25.0));
        movement4.setValue(X, kd, 0.1);
        movement4.setValue(Z, kp, 0.02);

        MovementConstants movement5 = new MovementConstants(-3.25, 0,  FORWARD, 5 * (1000));


        movement5.setValue(X, kp, (0.95 / 90.0));
        movement5.setValue(X, kd, 0.04);
        movement5.setValue(Z, kp, 0.02);

        MovementConstants movement6 = new MovementConstants(0, -30,  LEFT, 3 * (1000));

        movement6.setValue(Y, kp, 0.6/30.0);
        movement6.setValue(Z, kp, 0.035);
        movement6.setValue(Z, kd, 0.25);

        MovementConstants movement7 = new MovementConstants(0, 5,  RIGHT, 3 * (1000));


        movement7.setValue(Y, kp, 0.9 / 30);
        movement7.setValue(Z, kp, 0.035);
        movement7.setValue(Z, kp, 0.15);

        MovementConstants movement8 = new MovementConstants(0, 5,  RIGHT, 3 * (1000));

        movement8.setValue(X, kp, 0.90/80.0);
        movement8.setValue(X, kd, 0.1);
        movement8.setValue(Z, kp, 0.02);

        MovementConstants movement9 = new MovementConstants(-70.0, 0,  FORWARD, 3 * (1000));

        redB.add(movement3); //strafe back
        redB.add(movement4); //backward to box
        redB.add(movement5); //back to original
        redB.add(movement6); //Strafe left to grab
        redB.add(movement7); //Strafe right to grab
        redB.add(movement8);
    }



}
