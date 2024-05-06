// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.SimPhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Translation2dProto;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import static frc.robot.Constants.Vision.*;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import org.ejml.simple.SimpleMatrix;
import org.opencv.osgi.OpenCVInterface;

/** Add your docs here. */
public class Vision {
    public BreakerPhotonCamera backRightCam, backLeftCam, frontRightCam, frontLeftCam;
    private BreakerPhotonVisionPoseEstimator backRightPosSrc, backLeftPosSrc, frontRightPosSrc, frontLeftPosSrc;
    private BreakerPhotonVisionPoseEstimator[] poseSources;
    private ArrayList<BreakerEstimatedPose> estimatedPoses;
    private VisionSystemSim visionSim;

    public Vision() {

        backRightCam = new BreakerPhotonCamera(BACK_RIGHT_CAMERA_NAME, BACK_RIGHT_CAMERA_TRANS);
        backLeftCam = new BreakerPhotonCamera(BACK_LEFT_CAMERA_NAME, BACK_LEFT_CAMERA_TRANS);
        frontRightCam = new BreakerPhotonCamera(FRONT_RIGHT_CAMERA_NAME, FRONT_RIGHT_CAMERA_TRANS);
        frontLeftCam = new BreakerPhotonCamera(FRONT_LEFT_CAMERA_NAME, FRONT_LEFT_CAMERA_TRANS);

        backRightPosSrc = backRightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        backLeftPosSrc = backLeftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        frontRightPosSrc = frontRightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        frontLeftPosSrc = frontLeftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        poseSources = new BreakerPhotonVisionPoseEstimator[]{backLeftPosSrc, backRightPosSrc, frontRightPosSrc, frontLeftPosSrc};//
        estimatedPoses = new ArrayList<>();
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
            // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
            

            //cameraProp.setCalibration(960, 720, camIntrinsics, distCoeff);
            cameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
            cameraProp.setCalibError(0.15, 0.05);
            cameraProp.setFPS(35);
            cameraProp.setAvgLatencyMs(25);
            cameraProp.setLatencyStdDevMs(0.0);

            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            PhotonCameraSim flSim = new PhotonCameraSim(frontLeftCam.getBaseCamera(), cameraProp);
            flSim.setMaxSightRange(6.0);
            PhotonCameraSim frSim = new PhotonCameraSim(frontRightCam.getBaseCamera(), cameraProp);
            frSim.setMaxSightRange(6.0);
            PhotonCameraSim blSim = new PhotonCameraSim(backLeftCam.getBaseCamera(), cameraProp);
            blSim.setMaxSightRange(6.0);
            PhotonCameraSim brSim = new PhotonCameraSim(backRightCam.getBaseCamera(), cameraProp);
            brSim.setMaxSightRange(6.0);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(flSim, FRONT_LEFT_CAMERA_TRANS);
            // flSim.enableDrawWireframe(true);
            visionSim.addCamera(frSim, FRONT_RIGHT_CAMERA_TRANS);
            // //frSim.enableDrawWireframe(true);
            visionSim.addCamera(blSim, BACK_LEFT_CAMERA_TRANS);
            // //blSim.enableDrawWireframe(true);
            visionSim.addCamera(brSim, BACK_RIGHT_CAMERA_TRANS);
            //brSim.enableDrawWireframe(true);
    }

    public void estimateRobotPose(SwerveDrive drive, GtsamInterface gtsamInterface) {
            var loopStart = WPIUtilJNI.now();
            long tagDetTime = 0;
            tagDetTime = loopStart - 25000;
            estimatedPoses.clear();
            for (BreakerPhotonVisionPoseEstimator est: poseSources) {
                Optional<BreakerEstimatedPose> posOpt = est.getEstimatedPose(PoseOrigin.ofGlobal());
                if (posOpt.isPresent()) {
                    BreakerEstimatedPose pos = posOpt.get();
                    List<PhotonTrackedTarget> targets = est.getTrackedTargets();
                    if (targets.size() == 1) {
                        PhotonTrackedTarget tgt = targets.get(0);
                        if (tgt.getPoseAmbiguity() <= 0.25) {
                            continue;
                        }
                        final Pose3d actual = pos.estimatedPose;
                        final double fieldBorderMargin = 0.5;
                        final double zMargin = 0.75;

                        if (actual.getX() < -fieldBorderMargin
                            || actual.getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() + fieldBorderMargin
                            || actual.getY() < -fieldBorderMargin
                            || actual.getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth() + fieldBorderMargin
                            || actual.getZ() < -zMargin
                            || actual.getZ() > zMargin) {
                                continue;
                        }
                    }      
                    if (!pos.estimationStandardDevations.get().isEqual(VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), 1e-6))  {
                        estimatedPoses.add(pos);
                    }             
                    
                }
                gtsamInterface.setCamIntrinsics(est.getBaseCamera().getName(), est.getBaseCamera().getCameraMatrix());
                ArrayList<TagDetection> dets = new ArrayList<>();
                for (PhotonTrackedTarget tt : est.getTrackedTargets()) {
                    dets.add(new TagDetection(tt.getFiducialId(), tt.getDetectedCorners()));
                }
                gtsamInterface.sendVisionUpdate(est.getBaseCamera().getName(), tagDetTime, dets, est.getPoseEstimator().getRobotToCameraTransform());
            }


            sortByStandardDeviation(estimatedPoses);
            tagDetTime = loopStart - 25 * 1000;
            for (BreakerEstimatedPose estPos : estimatedPoses) {
                drive.addVisionMeasurement(estPos.estimatedPose.toPose2d(), tagDetTime / 1e6, estPos.estimationStandardDevations.get());
            }

        gtsamInterface.sendOdomUpdate(loopStart, drive.getTwist(), estimatedPoses.size() >= 1 ? new Pose3d(drive.getPose()) : null);
        
    }

    private void sortByStandardDeviation(ArrayList<BreakerEstimatedPose> poses) {
        int i, j;
        BreakerEstimatedPose temp;
        boolean swapped;

        for (i = 0; i < poses.size() - 1; i++) {
            swapped = false;
            for (j = 0; j < poses.size() - i - 1; j++) {
                BreakerEstimatedPose jEstpos = poses.get(j);
                BreakerEstimatedPose j1Estpos = poses.get(j);

                if (jEstpos.estimationStandardDevations.isPresent() && j1Estpos.estimationStandardDevations.isPresent()) {
                    if (jEstpos.estimationStandardDevations.get().get(2, 0) < j1Estpos.estimationStandardDevations.get().get(2, 0)) {
                        temp = poses.get(j);
                        poses.set(poses.indexOf(jEstpos), j1Estpos);
                        poses.set(poses.indexOf(j1Estpos), temp);
                        swapped = true;
                    }
                }
                // Nones get moved to front of list
                else if (jEstpos.estimationStandardDevations.isPresent() && !j1Estpos.estimationStandardDevations.isPresent()) {
                    temp = poses.get(j);
                    poses.set(poses.indexOf(jEstpos), j1Estpos);
                    poses.set(poses.indexOf(j1Estpos), temp);
                    swapped = true;
                }
            }
 
            if (swapped == false)
                break;
        }
    }

     public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
    
}
