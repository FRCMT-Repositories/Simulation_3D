package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    PIDController headingPID = new PIDController(0.028, 0.0001, 0.001);

    private static double OmegaCmd = 0;  
    private boolean ctrInit = true;

    public boolean shotOk = false;
    public boolean alinhoAuto = false;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public Field2d field = new Field2d();

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SwerveDriveBrake brakeX = new SwerveRequest.SwerveDriveBrake();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    
    public Command applyRequest(Supplier<SwerveRequest> request) { return run(() -> this.setControl(request.get())); }

    /**
     * Ativa o alinhamento em X das rodas.
     * @param null
     */
    public Command brakeX() {
        return applyRequest(() -> brakeX);
    }

    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation   
                        : kBlueAlliancePerspectiveRotation  
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // if(ctrInit){
        //     headingPID.enableContinuousInput(-180, 180);
        //     headingPID.setTolerance(2);

        //     ctrInit = false;
        // }

        double YawRaw = this.getPigeon2().getYaw().getValueAsDouble();
        double YawReal = YawRaw;
        double YawWrapping = MathUtil.inputModulus(YawReal, -180, 180);

        Pose2d currentPose = this.getState().Pose;

        Logger.recordOutput("ODOMETRY", currentPose);

        Logger.recordOutput("POSE/Odometry/Real", new double[] {currentPose.getX(), currentPose.getY(), Math.toRadians(YawWrapping)});
        Logger.recordOutput("POSE/Odometry/Estimate", new double[] {currentPose.getX(), currentPose.getY(), getPose().getRotation().getRadians()});

        Pose2d robot = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(getPose().getRotation().getRadians()));
        SmartDashboard.putData("FIELD", field);
        field.getObject("Robot").setPose(robot);
    }
    
    @Override
    public void simulationPeriodic()
    {
        if(ctrInit){
            if(!isRedAlliance()) this.resetPose(new Pose2d(2, 4, new Rotation2d(Math.PI)));
            else this.resetPose(new Pose2d(13.5, 4, new Rotation2d(0)));

            ctrInit = false;
        }
    }

    public void zeroGyroPigeon() {
        this.getPigeon2().setYaw(0);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }
    
    // Configura o PathPlanner
    private void configurePathPlanner() {
        try {
            //pega o arquivo .json com as configs do robo.
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(new PIDConstants(5, 0.0, 0.0), new PIDConstants(5, 0.0, 0.0)),
                config, 
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
                this
            ); 
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner Config Error: " + e.getMessage(), true);
        }
    }

    public void configAngleInit() {
        double newAngle = isRedAlliance() ? 0.0 : 180.0;

        this.getPigeon2().setYaw(newAngle);
        try { Thread.sleep(20); } catch (Exception e) {}

        Pose2d currentPose = this.getState().Pose;
        this.resetPose(new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(newAngle)));
        
        System.out.println("Reset Gyro: " + newAngle + " graus");
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

}