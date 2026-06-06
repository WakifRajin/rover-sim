using UnityEngine;
using UnityEngine.InputSystem;

public class ArticulationRoverController : MonoBehaviour
{
    [Header("Wheel Articulation Bodies")]
    [SerializeField] private ArticulationBody wheelFL;
    [SerializeField] private ArticulationBody wheelBL;
    [SerializeField] private ArticulationBody wheelFR;
    [SerializeField] private ArticulationBody wheelBR;

    [Header("Rover Parameters")]
    [SerializeField] private float wheelRadius = 0.15f;
    [SerializeField] private float wheelbase = 0.65f;
    [SerializeField] private float maxLinearSpeed = 2.0f;
    [SerializeField] private float maxAngularSpeed = 1.5f;

    [Header("Stability")]
    [SerializeField] private float centerOfMassYOffset = -0.2f;

    [Header("Drive Tuning")]
    [SerializeField] private float driveForceLimit = 500f;
    [SerializeField] private float driveDamping = 20f;
    [SerializeField] private float leftSideSpeedScale = 1f;
    [SerializeField] private float rightSideSpeedScale = 1f;

    [Header("Straight-Line Drift Compensation")]
    [SerializeField] private bool headingHoldWhenNoTurnInput = true;
    [SerializeField] private float headingRateDamping = 1.0f;
    [SerializeField] private float turnInputDeadzone = 0.05f;
    [SerializeField] private float linearSpeedThreshold = 0.05f;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionReference driveAction;
    [SerializeField] private bool invertLinear;
    [SerializeField] private bool invertAngular;

    [Header("External Control")]
    [SerializeField] private float externalCommandTimeout = 0.5f;

    private ArticulationBody rootBody;
    private float linearInput;
    private float angularInput;
    private float externalLinear;
    private float externalAngular;
    private float lastExternalCommandTime;
    private bool hasExternalCommand;

    private void OnEnable()
    {
        if (driveAction != null && driveAction.action != null)
        {
            driveAction.action.Enable();
        }
    }

    private void OnDisable()
    {
        if (driveAction != null && driveAction.action != null)
        {
            driveAction.action.Disable();
        }
    }

    private void Awake()
    {
        rootBody = GetComponent<ArticulationBody>();
        if (rootBody == null)
        {
            Debug.LogError("ArticulationRoverController requires an ArticulationBody on the root.");
            enabled = false;
            return;
        }

        if (!ValidateSetup())
        {
            enabled = false;
            return;
        }

        LowerCenterOfMass();
        ConfigureWheelDrive(wheelFL);
        ConfigureWheelDrive(wheelBL);
        ConfigureWheelDrive(wheelFR);
        ConfigureWheelDrive(wheelBR);
    }

    private void Update()
    {
        if (TryGetExternalInput(out float externalLinearInput, out float externalAngularInput))
        {
            linearInput = externalLinearInput;
            angularInput = externalAngularInput;
            return;
        }

        if (driveAction == null || driveAction.action == null)
        {
            linearInput = 0f;
            angularInput = 0f;
            return;
        }

        Vector2 input = driveAction.action.ReadValue<Vector2>();
        float linear = invertLinear ? -input.y : input.y;
        float angular = invertAngular ? -input.x : input.x;

        linearInput = Mathf.Clamp(linear, -1f, 1f);
        angularInput = Mathf.Clamp(angular, -1f, 1f);
    }

    private void FixedUpdate()
    {
        float targetLinear = linearInput * maxLinearSpeed;
        float targetAngular = angularInput * maxAngularSpeed;

        if (headingHoldWhenNoTurnInput && Mathf.Abs(angularInput) < turnInputDeadzone && Mathf.Abs(targetLinear) > linearSpeedThreshold)
        {
            float yawRate = rootBody != null ? rootBody.angularVelocity.y : 0f;
            targetAngular += -yawRate * headingRateDamping;
        }

        targetAngular = Mathf.Clamp(targetAngular, -maxAngularSpeed, maxAngularSpeed);

        float vLeft = targetLinear - (targetAngular * (wheelbase * 0.5f));
        float vRight = targetLinear + (targetAngular * (wheelbase * 0.5f));

        float omegaLeftRad = vLeft / wheelRadius;
        float omegaRightRad = vRight / wheelRadius;

        float omegaLeftDeg = (omegaLeftRad * Mathf.Rad2Deg) * leftSideSpeedScale;
        float omegaRightDeg = (omegaRightRad * Mathf.Rad2Deg) * rightSideSpeedScale;

        ApplyWheelVelocity(wheelFL, omegaLeftDeg);
        ApplyWheelVelocity(wheelBL, omegaLeftDeg);
        ApplyWheelVelocity(wheelFR, omegaRightDeg);
        ApplyWheelVelocity(wheelBR, omegaRightDeg);
    }

    private void LowerCenterOfMass()
    {
        Vector3 com = rootBody.centerOfMass;
        com.y += centerOfMassYOffset;
        rootBody.centerOfMass = com;
    }

    private void ConfigureWheelDrive(ArticulationBody wheel)
    {
        if (wheel == null)
        {
            return;
        }

        ArticulationDrive drive = wheel.xDrive;
        drive.driveType = ArticulationDriveType.Velocity;
        drive.stiffness = 0f;
        drive.damping = driveDamping;
        drive.forceLimit = driveForceLimit;
        wheel.xDrive = drive;
    }

    private void ApplyWheelVelocity(ArticulationBody wheel, float targetVelocityDeg)
    {
        if (wheel == null)
        {
            return;
        }

        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = targetVelocityDeg;
        wheel.xDrive = drive;
    }

    public void SetDriveInput(float linear, float angular)
    {
        linearInput = Mathf.Clamp(linear, -1f, 1f);
        angularInput = Mathf.Clamp(angular, -1f, 1f);
        externalLinear = linearInput;
        externalAngular = angularInput;
        lastExternalCommandTime = Time.time;
        hasExternalCommand = true;
    }

    public void SetCmdVel(float linearMetersPerSec, float angularRadPerSec)
    {
        float linear = maxLinearSpeed > 0f ? linearMetersPerSec / maxLinearSpeed : 0f;
        float angular = maxAngularSpeed > 0f ? angularRadPerSec / maxAngularSpeed : 0f;
        SetDriveInput(linear, angular);
    }

    private bool TryGetExternalInput(out float linear, out float angular)
    {
        if (!hasExternalCommand)
        {
            linear = 0f;
            angular = 0f;
            return false;
        }

        if (Time.time - lastExternalCommandTime > externalCommandTimeout)
        {
            hasExternalCommand = false;
            linear = 0f;
            angular = 0f;
            return false;
        }

        linear = externalLinear;
        angular = externalAngular;
        return true;
    }

    private bool ValidateSetup()
    {
        if (wheelFL == null || wheelBL == null || wheelFR == null || wheelBR == null)
        {
            Debug.LogError("All four wheel ArticulationBody references must be assigned.");
            return false;
        }

        if (driveAction == null || driveAction.action == null)
        {
            Debug.LogWarning("Drive InputActionReference not assigned — controller will rely on external input (e.g., ROS cmd_vel).");
        }

        if (wheelRadius <= 0f || wheelbase <= 0f)
        {
            Debug.LogError("Wheel radius and wheelbase must be greater than zero.");
            return false;
        }

        if (maxLinearSpeed <= 0f || maxAngularSpeed <= 0f)
        {
            Debug.LogError("Max linear and angular speeds must be greater than zero.");
            return false;
        }

        return true;
    }
}
