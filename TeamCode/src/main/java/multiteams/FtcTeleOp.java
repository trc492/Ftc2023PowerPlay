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

package multiteams;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This class contains the TeleOp Mode program.
 */
public class FtcTeleOp extends FtcOpMode
{
    public enum DriveOrientation
    {
        ROBOT(0), FIELD(1), INVERTED(2);

        int value;

        DriveOrientation(int val)
        {
            this.value = val;
        }

        void increment()
        {
            value = (value + 1) % 3;
        }
    }   //enum DriveOrientation

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale = 1.0;
    private final DriveOrientation driveOrientation = DriveOrientation.FIELD;

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
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::operatorButtonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
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
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
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
            double[] inputs = getDriveInputs();

            if (robot.robotDrive.driveBase.supportsHolonomicDrive())
            {
                robot.robotDrive.driveBase.holonomicDrive(null, inputs[0], inputs[1], inputs[2], getDriveGyroAngle());
            }
            else
            {
                robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
            }

            robot.dashboard.displayPrintf(2, "Pose:%s", robot.robotDrive.driveBase.getFieldPosition());
        }
    }   //slowPeriodic

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs()
    {
        double x = 0.0, y = 0.0, rot = 0.0;
        double mag;
        double newMag;

        switch (RobotParams.robotDriveMode)
        {
            case HOLONOMIC_MODE:
                x = driverGamepad.getLeftStickX(false);
                y = driverGamepad.getRightStickY(false);
                rot = (driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true));
                robot.dashboard.displayPrintf(1, "Holonomic:x=%.1f,y=%.1f,rot=%.1f", x, y, rot);
                break;

            case ARCADE_MODE:
                x = driverGamepad.getRightStickX(false);
                y = driverGamepad.getRightStickY(false);
                rot = driverGamepad.getLeftStickX(true);
                robot.dashboard.displayPrintf(1, "Arcade:x=%.1f,y=%.1f,rot=%.1f", x, y, rot);
                break;

            case TANK_MODE:
                double leftPower = driverGamepad.getLeftStickY(false);
                double rightPower = driverGamepad.getRightStickY(false);
                x = 0.0;
                y = (leftPower + rightPower)/2.0;
                rot = (leftPower - rightPower)/2.0;
                robot.dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f", leftPower, rightPower);
                break;
        }
        mag = TrcUtil.magnitude(x, y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
            mag = 1.0;
        }
        newMag = Math.pow(mag, 3);

        newMag *= drivePowerScale;
        rot *= drivePowerScale;

        if (mag != 0.0)
        {
            x *= newMag / mag;
            y *= newMag / mag;
        }

        return new double[] { x, y, rot };
    }   //getDriveInput

    /**
     * This method returns robot heading to be maintained in teleop drive according to drive orientation mode.
     *
     * @return robot heading to be maintained.
     */
    private double getDriveGyroAngle()
    {
        switch (driveOrientation)
        {
            case ROBOT:
                return 0.0;

            case INVERTED:
                return 180.0;

            default:
            case FIELD:
                return robot.robotDrive.driveBase.getHeading();
        }
    }   //getDriveGyroAngle

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
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (pressed)
                {
                    driveOrientation.increment();
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.slowDrivePowerScale: 1.0;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;

            case FtcGamepad.GAMEPAD_BACK:
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
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;

            case FtcGamepad.GAMEPAD_BACK:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
