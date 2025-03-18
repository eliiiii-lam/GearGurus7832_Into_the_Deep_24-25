package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "sample")
public class sample extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(-51, 61.5, Math.toRadians(360));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        Arm sig = new Arm(hardwareMap);
        //  resetRuntime();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "Arm");
        DcMotorEx uppies = hardwareMap.get(DcMotorEx.class, "uppies");


        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)

                .stopAndAdd(arm.closeClaw())
                .afterTime(0,arm.SetPosition(-600))
                //.splineToLinearHeading(new Pose2d(-75,60, Math.toRadians(320)), Math.toRadians(320))
                .strafeToLinearHeading(new Vector2d(-77,62.5), Math.toRadians(315))

                .afterTime(0, sig.SetPosition1(2500))

                .afterTime(1,arm.SetPosition(-300))
                .waitSeconds(1.3)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(-1110))
                .afterTime(0.5, sig.SetPosition1(1150))
////////////////////////////////////////////////////////////////////////////////////////////////////////////

                .strafeToLinearHeading(new Vector2d(-72.75,55), Math.toRadians(280))
                .stopAndAdd(arm.extendIn())
                .waitSeconds(1)
                .stopAndAdd(arm.grab())
                .waitSeconds(1)
                .stopAndAdd(arm.transferPos())
                .strafeToLinearHeading(new Vector2d(-77,62.5), Math.toRadians(315))
                .afterTime(0.8, sig.SetPosition1(940))
                .waitSeconds(1)
                .stopAndAdd(arm.transfer())
                .afterTime(1, sig.SetPosition1(2800))
                .afterTime(1.6,arm.SetPosition(-200))
                .waitSeconds(2)
                .stopAndAdd(arm.lockIntake())
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(-1110))
                .afterTime(0.5, sig.SetPosition1(1150))

////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-78,55.5), Math.toRadians(280))
                .stopAndAdd(arm.extendIn())
                .waitSeconds(1)
                .stopAndAdd(arm.grab())
                .waitSeconds(1)
                .stopAndAdd(arm.transferPos())
                .strafeToLinearHeading(new Vector2d(-77,63), Math.toRadians(315))
                .afterTime(0.8, sig.SetPosition1(940))
                .waitSeconds(1)
                .stopAndAdd(arm.transfer())
                .afterTime(1, sig.SetPosition1(2800))
                .afterTime(1.6,arm.SetPosition(-200))
                .waitSeconds(2)
                .stopAndAdd(arm.lockIntake())
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(-1110))
                .afterTime(0.5, sig.SetPosition1(1150))



                .splineToLinearHeading(new Pose2d(-67,49.75, Math.toRadians(230)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.clawTilt())
                .stopAndAdd(arm.extendIn())
                .waitSeconds(1)
                .stopAndAdd(arm.grab())
                .waitSeconds(1)
                .stopAndAdd(arm.transferPos())
                .strafeToLinearHeading(new Vector2d(-77,63), Math.toRadians(315))
                .afterTime(0.8, sig.SetPosition1(940))
                .waitSeconds(1)
                .stopAndAdd(arm.transfer())
                .afterTime(1, sig.SetPosition1(2800))
                .afterTime(1.6,arm.SetPosition(-200))
                .waitSeconds(2)
                .stopAndAdd(arm.lockIntake())
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(-700))
                .afterTime(0.5, sig.SetPosition1(1150))
                .splineToLinearHeading(new Pose2d(-67,10, Math.toRadians(360)), Math.toRadians(360))
                .splineToLinearHeading(new Pose2d(-30,10, Math.toRadians(360)), Math.toRadians(360))

                .afterTime(0,sig.SetPosition(800))







                ;














        Action TrajectoryClose = spec1.endTrajectory().build();


        if (isStopRequested()) return;




        Actions.runBlocking(


                new ParallelAction(
                        arm.lockIntake(),
                        spec1.build(),
                        arm.UpdatePID(),
                        sig.UpdatePID1()


                )


        );
    }
}