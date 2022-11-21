/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

@TeleOp(name="FtcTeleOp", group="Ftc3543")

/*


    Autoassisted Teleop sequence
        *Driver drives to the substation
        *Driver holds B, initiating pickup Sequence
            *if driver releases B, automated pickup sequence quits -> control manually
        *Driver clicks left button -> robot drives to the nearest high pole, raises its elevator
        *Driver holds B, initiating the scoring sequence
            *if driver releases B, automated scoring sequence stops -> manual control

    Driver Controls:
    - LeftStickX (Rotation)
    - RightStickX (Strafe Left/Right), RightStickY (Forward/Backward)
    - LeftBumper (Change DriveOrientation: Robot/Field/Inverted)
    - RightBummer (Hold for slow drive speed)
    - Y (Toggle Pivot turn mode)

    Operator Controls:
    - Turret: LeftTrigger (anti-clockwise), RightTrigger (clockwise),
              DPadLeft (anti-clockwise preset), DPadRight (clockwise preset)
    - Elevator: RightStickY (Up/Down), DPadUp (preset up), DPadDown (preset down)
    - Arm: LeftStickY (Up/Down)
    - Retract Everything: B (prepare for pickup
    - Expand Everything: (x), Prepare for scoring
        - raise elevator to high pole height
         - turn turret to right side
         - extend arm to parallel
    - Intake: Hold A (Dump), Hold Y (Pickup)
    - Picking up cones - Operator Right bumper
        -Prereqs: assumes intake is right above cone/ conestack
        -while operator holds it, elevator goes down with intake spinning, stops
    - FD
*/

public class FtcTeleOp extends FtcOpMode
{
    private static final String moduleName = "FtcTeleOp";
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private TrcDriveBase.DriveOrientation driveOrientation = TrcDriveBase.DriveOrientation.ROBOT;
    private double drivePowerScale = 1.0;
    private boolean turretSlowModeOn = false;
    private boolean autoMovement = false;
    private boolean pivotTurnMode = false;
    private boolean manualOverride = false;
    private boolean atScoringLocation = false;
    private boolean lockOnPole = false;
    private boolean lockOnCone = false;
    private TrcPidController coneAlignPidCtrl;
    private Double coneAngle;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Unknown_TeleOp";
            robot.globalTracer.openTraceLog(RobotParams.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::operatorButtonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);

        if (robot.robotDrive != null)
        {
            robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
        }
        if(robot.vision != null){
            robot.vision.frontEocvVision.setDetectObjectType(
                    FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
        }
        coneAlignPidCtrl = new TrcPidController(
                "turnPidCtrl", RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, RobotParams.TURN_SETTLING,
                RobotParams.TURN_STEADY_STATE_ERR, RobotParams.TURN_STALL_ERRRATE_THRESHOLD, this::findConeAlignAngle);

    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.dashboard.clearDisplay();
        if (robot.globalTracer.isTraceLogEnabled())
        {
            robot.globalTracer.traceInfo(moduleName, "***** Starting TeleOp *****");
        }
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        updateDriveModeLeds();
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.isTraceLogOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically at a slow rate. Typically, you put code that doesn't require frequent
     * update here. For example, TeleOp joystick code or status display code can be put here since human responses
     * are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void slowPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        if (robot.robotDrive != null)
        {
            double[] inputs = driverGamepad.getDriveInputs(RobotParams.ROBOT_DRIVE_MODE, true, drivePowerScale);
            boolean turnOnly = inputs[0] == 0.0 && inputs[1] == 0.0;

            if (pivotTurnMode && turnOnly)
            {
                double leftPower, rightPower;
                if (inputs[2] >= 0.0)
                {
                    leftPower = inputs[2];
                    rightPower = 0.0;
                }
                else
                {
                    leftPower = 0.0;
                    rightPower = -inputs[2];
                }
                robot.robotDrive.driveBase.tankDrive(leftPower, rightPower);
            }
            else if (robot.robotDrive.driveBase.supportsHolonomicDrive())
            {
                robot.robotDrive.driveBase.holonomicDrive(
                    null, inputs[0], inputs[1], inputs[2],
                    robot.robotDrive.driveBase.getDriveGyroAngle(driveOrientation));
            }
            else
            {
                double turnPower = inputs[2];
                //if we see the cone and lockOnCone is enabled, turn based on coneAlign pid controller, otherwise turn clockwise
//                if (robot.vision != null && lockOnCone)
//                {
//                    findConeAlignAngle() is a Double so it returns null if no cone found
//                    coneAngle = robot.vision.getConeAngle();
//                    if(coneAngle != null && Math.abs(coneAngle) > 1.0){
//                        turnPower = coneAlignPidCtrl.getOutput();
//                    }
//                    else if(coneAngle != null){
//                        turnPower = 0;
//                    }
//                    else{
//                        turnPower = -0.25;
//                    }
//                }
                robot.robotDrive.driveBase.arcadeDrive(inputs[1], turnPower);
            }

            robot.dashboard.displayPrintf(2, "Pose:%s", robot.robotDrive.driveBase.getFieldPosition());
        }
        //
        // Other subsystems.
        //
        if (robot.elevator != null)
        {
            double elevatorPower = operatorGamepad.getRightStickY(true);

            if (elevatorPower < 0.0)
            {
                // Elevator is going down, gravity is helping here so we can scale the down power way down.
                elevatorPower *= RobotParams.ELEVATOR_DOWN_POWER_SCALE;
            }

            if (manualOverride)
            {
                robot.elevator.setPower(elevatorPower);
            }
            else
            {
                robot.elevator.setPidPower(elevatorPower, true);
            }

            robot.dashboard.displayPrintf(
                3, "Elevator: power=%.2f, pos=%.1f, LimitSW=%s",
                elevatorPower, robot.elevator.getPosition(), robot.elevator.isLowerLimitSwitchActive());
        }

        if (robot.arm != null)
        {
            double armPower = -operatorGamepad.getLeftStickY(true);
            if (manualOverride)
            {
                robot.arm.setPower(armPower);
            }
            else
            {
                robot.arm.setPidPower(armPower, false);
            }

            robot.dashboard.displayPrintf(
                4, "Arm: power=%.2f, pos=%.1f, LimitSW=%s",
                armPower, robot.arm.getPosition(), robot.arm.isLowerLimitSwitchActive());
        }

        if (robot.turret != null)
        {
            if (robot.vision != null && lockOnPole)
            {
                Double poleAngle = robot.vision.getPoleAngle();

                if (poleAngle != null && Math.abs(poleAngle) >= 1.0)
                {
                    robot.turret.setTarget(robot.turret.getPosition() - poleAngle);
                }
            }
            else
            {
                double turretPower = -operatorGamepad.getTrigger(true) * RobotParams.TURRET_POWER_SCALE;
                robot.turret.setPower(turretPower, !manualOverride);
//                double turretX = operatorGamepad.getLeftStickX();
//                double turretY = operatorGamepad.getLeftStickY();
//                double turretPower = operatorGamepad.getMagnitude(turretX, turretY);
//                double turretDirDegrees = 90.0 - operatorGamepad.getDirectionDegrees(turretX, turretY);
//                if (turretDirDegrees < 0.0)
//                {
//                    turretDirDegrees += 360.0;
//                }
//                robot.turret.setTarget(turretDirDegrees, turretPower);

                robot.dashboard.displayPrintf(
                    5, "Turret: power=%.2f, pos=%.1f, LimitSW=%s/%s",
                    turretPower, robot.turret.getPosition(),
                    robot.turret.isZeroPosSwitchActive(), robot.turret.isCalDirSwitchActive());

            }
        }

        if (robot.intake != null)
        {
            robot.dashboard.displayPrintf(
                6, "Intake: power=%.2f, sensor=%.2f", robot.intake.getPower(), robot.intake.getSensorValue());
        }
        else if (robot.grabber != null)
        {
            robot.dashboard.displayPrintf(
                6, "Grabber: pos=%.2f, sensor=%.2f", robot.grabber.getPosition(), robot.grabber.getSensorValue());
        }
    }   //slowPeriodic

    /**
     * This method updates the blinkin LEDs to show the drive orientation mode.
     */
    private void updateDriveModeLeds()
    {
        if (robot.blinkin != null)
        {
            robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_FIELD, false);
            robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_ROBOT, false);
            robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_INVERTED, false);
            switch (driveOrientation)
            {
                case FIELD:
                    robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_FIELD, true);
                    break;

                case ROBOT:
                    robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_ROBOT, true);
                    break;

                case INVERTED:
                    robot.blinkin.setPatternState(Vision.DRIVE_ORIENTATION_INVERTED, true);
                    break;
            }
        }
    }   //updateDriveModeLeds

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");
        robot.dashboard.displayPrintf(8, "Drive Mode:%s", driveOrientation.toString());

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    robot.robotDrive.gridDrive.cancel();
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if(pressed){
                    if(!robot.intake.hasObject()){
                        robot.cyclingTask.doTeleopPickup(TaskCyclingCones.VisionType.CONE_AND_POLE_VISION, 1, null);
                    }
                    else{
                        robot.cyclingTask.scoreCone(TaskCyclingCones.VisionType.CONE_AND_POLE_VISION, null);
                    }
                }
                else{
                    robot.cyclingTask.cancel();
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    atScoringLocation = !atScoringLocation;
                    TrcPose2D endPoint = robot.robotDrive.gridDrive.gridCellToPose(
                        atScoringLocation ?
                            robot.robotDrive.getAutoTargetCell(
                                RobotParams.SCORE_LOCATION_RED_LEFT, FtcAuto.autoChoices) :
                            robot.robotDrive.getAutoTargetCell(
                                RobotParams.SUBSTATION_RED_LEFT, FtcAuto.autoChoices));
                    robot.robotDrive.gridDrive.driveToEndPoint(endPoint);
                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (pressed)
                {
                    pivotTurnMode = !pivotTurnMode;
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (pressed)
                {
                    driveOrientation = TrcDriveBase.DriveOrientation.nextDriveOrientation(driveOrientation);
                    updateDriveModeLeds();
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.SLOW_DRIVE_POWER_SCALE: 1.0;
                autoMovement = pressed;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    robot.robotDrive.gridDrive.setRelativeYGridTarget(1);
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    robot.robotDrive.gridDrive.setRelativeYGridTarget(-1);
                }
                break;

            //drives to the nearest high pole if it has a cone, or the nearest triangle substation to pick up a cone
            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if(pressed && robot.robotDrive.gridDrive != null) {
                    if (autoMovement) {
                        TrcPose2D endPoint = null;
                        if (robot.intake.hasObject()) {
                            endPoint = getNearestHighPole();
                        } else {
                            endPoint = autoPickupPoint(FtcAuto.autoChoices.alliance, 0);
                        }
                        robot.robotDrive.gridDrive.driveToEndPoint(robot.robotDrive.gridDrive.gridCellToPose(endPoint));
                    } else {
                        robot.robotDrive.gridDrive.setRelativeXGridTarget(-1);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    robot.robotDrive.gridDrive.setRelativeXGridTarget(1);
                }
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (pressed && robot.robotDrive.gridDrive != null)
                {
                    robot.robotDrive.gridDrive.resetGridCellCenter();
                }
                break;
            case FtcGamepad.GAMEPAD_LSTICK_BTN:
                lockOnCone = pressed;
                break;

        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (pressed)
                {
                    if (robot.intake != null)
                    {
                        robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP);
                    }
                    else if (robot.grabber != null)
                    {
//                        robot.grabber.autoAssist(null, 0.0, null, 0.0);
                        robot.grabber.open();
                    }
                }
                break;
            // Prepare Pickup: Extend arm, turn turret to face front, lower elevator after half a second
            case FtcGamepad.GAMEPAD_B:
                if (pressed && robot.arm != null && robot.turret != null && robot.elevator != null && robot.robotDrive.gridDrive!= null)
                {
//                    TrcPose2D endPoint = robot.robotDrive.gridDrive.gridCellToPose(
//                                    robot.robotDrive.getAutoTargetCell(
//                                            RobotParams.SCORE_LOCATION_RED_LEFT, FtcAuto.autoChoices));
//                    robot.robotDrive.gridDrive.driveToEndPoint(endPoint);
                    robot.arm.setTarget(RobotParams.ARM_MAX_POS);
                    robot.turret.setTarget(RobotParams.TURRET_FRONT);
                    robot.elevator.setTarget(0.5, RobotParams.ELEVATOR_CONE_GRAB_HEIGHT, true, 1.0, null, null, 0.0);

                }

                break;
            //Prepares high pole scoring: raise elevator, turn turret to the left, set arm to parallel
            case FtcGamepad.GAMEPAD_X:
                if (pressed && robot.arm != null && robot.elevator != null && robot.turret != null && robot.robotDrive.gridDrive != null)
                {
//                    TrcPose2D endPoint = robot.robotDrive.gridDrive.gridCellToPose(
//                            robot.robotDrive.getAutoTargetCell(
//                                    RobotParams.SUBSTATION_RED_LEFT, FtcAuto.autoChoices));
//                    robot.robotDrive.gridDrive.driveToEndPoint(endPoint);
                    robot.turret.setTarget(
                       RobotParams.TURRET_LEFT, 1.0, null, null, 0.0,
                       RobotParams.ELEVATOR_SCORING_HEIGHT, 33.0);

                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (pressed)
                {
                    if (robot.intake != null)
                    {
                        robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP);
                    }
                    else if (robot.grabber != null)
                    {
                        robot.grabber.close();
                    }
                }
//                if (pressed && robot.elevator != null)
//                {
//                    robot.elevator.setTarget(RobotParams.ELEVATOR_MIN_POS);
//                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                manualOverride = pressed;
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                if(pressed){
                    turretSlowModeOn = !turretSlowModeOn;
                    if(turretSlowModeOn){
                        robot.turret.getPidActuator().getPidController().setOutputLimit(0.5);
                    }
                    else{
                        robot.turret.getPidActuator().getPidController().setOutputLimit(1.0);
                    }
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (pressed && robot.elevator != null)
                {
                    robot.elevator.presetPositionUp();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (pressed && robot.elevator != null)
                {
                    robot.elevator.presetPositionDown();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (pressed && robot.turret != null)
                {
                    robot.turret.presetPositionUp();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (pressed && robot.turret != null)
                {
                    robot.turret.presetPositionDown();
                }
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (pressed && robot.turret != null)
                {
                    robot.turret.zeroCalibrate();
                }
                break;

            case FtcGamepad.GAMEPAD_LSTICK_BTN:
                lockOnPole = pressed;
            break;

        }

    }   //operatorButtonEvent

    private Double findConeAlignAngle(){
        return coneAngle != null? coneAngle : 0.0;
    }

    /*buttonIndex:
    0: left substation
    1: right substation
    2: left cone stack
    3: right cone stack
    */
    private TrcPose2D autoPickupPoint(FtcAuto.Alliance alliance, int buttonIndex) {
        TrcPose2D point = null;
        if (alliance == FtcAuto.Alliance.RED_ALLIANCE) {
            point = RobotParams.PICKUP_POINTS_RED[buttonIndex];
        }
        else {
            point = RobotParams.PICKUP_POINTS_BLUE[buttonIndex];
        }
        return point;
    }

    private TrcPose2D getNearestHighPole() {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        TrcPose2D robotGridCell = robot.robotDrive.gridDrive.poseToGridCell(robotPose);
        robotGridCell = robot.robotDrive.gridDrive.adjustGridCellCenter(robotGridCell);
        double minDistance = Double.MAX_VALUE;
        TrcPose2D closest = null;
        for(TrcPose2D pole : RobotParams.SCORING_POINTS) {
            double distance = Math.pow(robotGridCell.x - pole.x, 2) + Math.pow(robotGridCell.y - pole.y, 2);
            if(distance < minDistance) {
                closest = pole;
                minDistance = distance;
            }
        }
        //TODO: Calculate offset to scoring position
        return closest;
    }

}   //class FtcTeleOp
