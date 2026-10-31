package frc.robot.vision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;
    private final PhotonPipelineResult[] latestResults; // 每台相機最新一筆結果（給瞄準用）

    private boolean disabled = false;

    public Vision(VisionConsumer consumer) {
        this.consumer = consumer;

        int numCameras = CAMERA_NAMES.length;
        cameras = new PhotonCamera[numCameras];
        estimators = new PhotonPoseEstimator[numCameras];
        latestResults = new PhotonPipelineResult[numCameras];

        for (int i = 0; i < numCameras; i++) {
            cameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            estimators[i] = new PhotonPoseEstimator(
                APRIL_TAGS,
                MULTI_TAG_PNP_ON_COPROCESSOR,
                CAMERA_TRANSFORMS[i]);
        }
    }

    /** 目前聯盟色對應的瞄準 tag（依優先順序） */
    private List<Integer> getAimTags() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue ? BLUE_AIM_TAG_IDS : RED_AIM_TAG_IDS;
    }

    @Override
    public void periodic() {
        if (disabled) return;

        for (int i = 0; i < cameras.length; i++) {
            if (!cameras[i].isConnected()) continue;

            // 一次取出自上次以來所有新的幀，每一幀都處理，不會漏也不會重複
            for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
                latestResults[i] = result;

                Optional<EstimatedRobotPose> opt = estimators[i].update(result);
                if (opt.isEmpty()) continue;

                EstimatedRobotPose est = opt.get();
                Pose2d pose2d = est.estimatedPose.toPose2d();
                int tagCount = est.targetsUsed.size();

                // ---- 過濾不可信的結果 ----
                if (tagCount == 0) continue;
                if (tagCount == 1 && est.targetsUsed.get(0).getPoseAmbiguity() > MAX_AMBIGUITY) continue;
                if (est.estimatedPose.getZ() > MAX_Z_HEIGHT.in(Meters)) continue;
                if (pose2d.getX() < 0.0 || pose2d.getX() > APRIL_TAGS.getFieldLength()
                    || pose2d.getY() < 0.0 || pose2d.getY() > APRIL_TAGS.getFieldWidth()) continue;

                // ---- 標準差：越遠、tag 越少越不信任（所有 tag 一視同仁）----
                double totalDistance = 0.0;
                for (PhotonTrackedTarget t : est.targetsUsed) {
                    totalDistance += t.getBestCameraToTarget().getTranslation().getNorm();
                }
                double avgDistance = totalDistance / tagCount;
                double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;

                double linearStdDev = LINEAR_STD_DEV_BASELINES[i] * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                consumer.accept(
                    pose2d,
                    est.timestampSeconds,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

                SmartDashboard.putString("Vision/" + cameras[i].getName() + " Pose", pose2d.toString());
            }
        }
    }

    /**
     * 回傳瞄準 tag 的 yaw（度）。看不到、資料過期、或相機沒連線就回傳 empty。
     * 依 AIM_TAG 的優先順序挑（先找與 HUB 中心對齊的 tag）。
     */
    public OptionalDouble getHubTargetYaw(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameras.length) return OptionalDouble.empty();

        PhotonPipelineResult result = latestResults[cameraIndex];
        if (result == null) return OptionalDouble.empty();

        // 資料太舊（例如相機斷線）就不要用
        if (Timer.getFPGATimestamp() - result.getTimestampSeconds() > MAX_RESULT_AGE_SECONDS) {
            return OptionalDouble.empty();
        }

        for (int wantedId : getAimTags()) {
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.getFiducialId() == wantedId) {
                    return OptionalDouble.of(t.getYaw());
                }
            }
        }
        return OptionalDouble.empty();
    }

    public boolean hasHubTarget(int cameraIndex) {
        return getHubTargetYaw(cameraIndex).isPresent();
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}