package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;
@Autonomous(name = "GyroTest", group = "Test")
public class GyroTest extends LinearOpMode {
    private NopeRope nopeRope;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void runOpMode() throws InterruptedException {
        nopeRope = new NopeRope(this);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(!isStopRequested()){
            packet = new TelemetryPacket();
            telemetry.addData("First Angle", nopeRope.getSensors().getRawExternalHeading());
            telemetry.addData("First AngleTD", Math.toDegrees(nopeRope.getSensors().getRawExternalHeading()));
            telemetry.addData("Second Angle", nopeRope.getSensors().getSecondAngle());
            telemetry.addData("Third Angle", nopeRope.getSensors().getThirdAngle());


            packet.put("First Angle", nopeRope.getSensors().getRawExternalHeading());
            packet.put("First AngleTD", Math.toDegrees(nopeRope.getSensors().getRawExternalHeading()));
            packet.put("Second Angle", nopeRope.getSensors().getSecondAngle());
            packet.put("Third Angle", nopeRope.getSensors().getThirdAngle());

            dashboard.sendTelemetryPacket(packet);

        }




    }
}
