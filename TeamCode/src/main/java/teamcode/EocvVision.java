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

import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcFtcLib.ftclib.FtcEocvAprilTagPipeline;
import TrcFtcLib.ftclib.FtcEocvColorBlobPipeline;
import TrcFtcLib.ftclib.FtcEocvDetector;

/**
 * This class implements EOCV Vision that provides the capability to detect AprilTag or color blobs and return their
 * detected info.
 */
public class EocvVision extends FtcEocvDetector
{
    private static final double[] colorThresholdsRedCone = {100.0, 255.0, 0.0, 100.0, 0.0, 60.0};
    private static final double[] colorThresholdsBlueCone = {0.0, 60.0, 0.0, 100.0, 100, 255.0};
    private static final double[] colorThresholdsYellowPole = {128.0, 255.0, 128.0, 255.0, 0.0, 120.0};

    public enum ObjectType
    {
        APRIL_TAG, RED_CONE, BLUE_CONE, YELLOW_POLE;

        static ObjectType nextObjectType(ObjectType objType)
        {
            ObjectType nextObjType;

            switch (objType)
            {
                case APRIL_TAG:
                    nextObjType = RED_CONE;
                    break;

                case RED_CONE:
                    nextObjType = BLUE_CONE;
                    break;

                case BLUE_CONE:
                    nextObjType = YELLOW_POLE;
                    break;

                default:
                case YELLOW_POLE:
                    nextObjType = APRIL_TAG;
                    break;
            }

            return nextObjType;
        }   //nextObjectType

    }   //enum ObjectType

    private final TrcDbgTrace tracer;
    private final FtcEocvAprilTagPipeline aprilTagPipeline;
    private final FtcEocvColorBlobPipeline redConePipeline;
    private final FtcEocvColorBlobPipeline blueConePipeline;
    private final FtcEocvColorBlobPipeline yellowPolePipeline;
    private ObjectType objectType = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the camera image width.
     * @param imageHeight specifies the camera image height.
     * @param cameraRect specifies the homography camera pixel rectangle, can be null if not provided.
     * @param worldRect specifies the homography world coordinate rectangle, can be null if not provided.
     * @param openCvCam specifies the OpenCV camera object.
     * @param cameraRotation specifies the camera orientation.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public EocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation, TrcDbgTrace tracer)
    {
        super(instanceName, openCvCam, imageWidth, imageHeight, cameraRotation, cameraRect, worldRect, tracer);

        this.tracer = tracer;
        TrcOpenCvColorBlobPipeline.FilterContourParams redConeFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(500)
                .setMinPerimeter(100.0)
                .setWidthRange(50.0, 1000.0)
                .setHeightRange(50.0, 1000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);
        TrcOpenCvColorBlobPipeline.FilterContourParams blueConeFilterContourParams =
                new TrcOpenCvColorBlobPipeline.FilterContourParams()
                        .setMinArea(1000)
                        .setMinPerimeter(100.0)
                        .setWidthRange(50.0, 1000.0)
                        .setHeightRange(20.0, 1000.0)
                        .setSolidityRange(0.0, 100.0)
                        .setVerticesRange(0.0, 1000.0)
                        .setAspectRatioRange(0.0, 1000.0);
        TrcOpenCvColorBlobPipeline.FilterContourParams poleFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(10000.0)
                .setMinPerimeter(500.0)
                .setWidthRange(100.0, 1000.0)
                .setHeightRange(500.0, 1000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);
        aprilTagPipeline = new FtcEocvAprilTagPipeline(
            AprilTagDetectorJNI.TagFamily.TAG_36h11, RobotParams.APRILTAG_SIZE,
            RobotParams.FRONTCAM_FX, RobotParams.FRONTCAM_FY, RobotParams.FRONTCAM_CX, RobotParams.FRONTCAM_CY,
            tracer);
        redConePipeline = new FtcEocvColorBlobPipeline(
            "redConePipeline", false, colorThresholdsRedCone, redConeFilterContourParams, tracer);
        blueConePipeline = new FtcEocvColorBlobPipeline(
            "blueConePipeline", false, colorThresholdsBlueCone, blueConeFilterContourParams, tracer);
        yellowPolePipeline = new FtcEocvColorBlobPipeline(
            "yellowPolePipeliine", false, colorThresholdsYellowPole, poleFilterContourParams, tracer);
        if (instanceName.equals("frontEocvVision"))
        {
            setDetectObjectType(ObjectType.APRIL_TAG);
        }
        else
        {
            setDetectObjectType(ObjectType.YELLOW_POLE);
        }
    }   //EocvVision

    /**
     * This method updates the pipeline to detect the currently selected object type.
     */
    private void updatePipeline()
    {
        if (tracer != null)
        {
            tracer.traceInfo("updatePipeline", "objType=%s", objectType);
        }

        switch (objectType)
        {
            case APRIL_TAG:
                setPipeline(aprilTagPipeline);
                break;

            case RED_CONE:
                setPipeline(redConePipeline);
                break;

            case BLUE_CONE:
                setPipeline(blueConePipeline);
                break;

            case YELLOW_POLE:
                setPipeline(yellowPolePipeline);
                break;
        }
    }   //updatePipeline

    /**
     * This method sets the object type to detect.
     *
     * @param objType specifies the object type to detect.
     */
    public void setDetectObjectType(ObjectType objType)
    {
        objectType = objType;
        updatePipeline();
    }   //setDetectObjectType

    /**
     * This method sets the detect object type to the next type.
     */
    public void setNextObjectType()
    {
        setDetectObjectType(ObjectType.nextObjectType(objectType));
    }   //setNextObjectType

    /**
     * This method returns the detect object type.
     *
     * @return detect object type.
     */
    public ObjectType getDetectObjectType()
    {
        return objectType;
    }   //getDetectObjectType

    /**
     * This method toggles the colorblob pipeline to display either the annotated input or the color filter output.
     * This is mainly for debugging the color filtering of the pipeline so one can see what the color filtering output
     * looks like.
     */
    public void toggleColorFilterOutput()
    {
        TrcOpenCvPipeline<?> pipeline = getPipeline();

        if (pipeline == redConePipeline || pipeline == blueConePipeline || pipeline == yellowPolePipeline)
        {
            ((FtcEocvColorBlobPipeline) pipeline).toggleColorFilterOutput();
        }
    }   //toggleColorFilterOutput

}   //class EocvVision
