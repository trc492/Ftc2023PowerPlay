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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";

    private final TrcRevBlinkin.Pattern[] ledPatternPriorities = {};

    private final Robot robot;
    private final TrcDbgTrace tracer;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision eocvVision;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        this.tracer = TrcDbgTrace.getGlobalTracer();

        if (RobotParams.Preferences.useEasyOpenCV)
        {
            OpenCvCamera webcam =
                OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM), cameraViewId);
            eocvVision = new EocvVision(
                "EocvVision", RobotParams.IMAGE_WIDTH, RobotParams.IMAGE_HEIGHT,
                RobotParams.cameraRect, RobotParams.worldRect, webcam, OpenCvCameraRotation.UPRIGHT,
                RobotParams.Preferences.showEocvView, null);
        }
        else
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        robot.blinkin.setPatternPriorities(ledPatternPriorities);
    }   //setupBlinkin

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        tensorFlowVision.shutdown();
        tensorFlowVision = null;
    }   //tensorFlowShutdown

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing object distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller distance to image center than b, 0 if a and b have equal distance to
     * image center, positive value if a has larger distance to image center than b.
     */
    private int compareDistanceFromCamera(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.distanceFromCamera.y) - Math.abs(b.distanceFromCamera.y))*1000);
    }   //compareDistanceFromCamera

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing camera angle.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller camera angle than b, 0 if a and b have equal camera angle, positive
     *         value if a has larger camera angle than b.
     */
    private int compareCameraAngle(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.horizontalAngle) - Math.abs(b.horizontalAngle))*1000);
    }   //compareCameraAngle

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing object size.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller area than b, 0 if a and b have equal area, positive value if a has
     *         larger area than b.
     */
    private int compareObjectSize(
        TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject> b)
    {
        return a.detectedObj.rect.width * a.detectedObj.rect.height -
               b.detectedObj.rect.width * b.detectedObj.rect.height;
    }   //compareObjectSize

}   //class Vision
