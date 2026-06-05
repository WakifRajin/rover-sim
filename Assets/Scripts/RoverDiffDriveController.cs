using UnityEngine;
using UnityEngine.InputSystem;

namespace RoverSim
{
    /// <summary>
    /// Physics-based differential (skid-steer) drive for the 4-wheel rover.
    ///
    /// This controller does NOT create or modify colliders or rigidbodies. You must
    /// have the following already on the rover in the scene/prefab:
    ///   - One <see cref="Rigidbody"/> on the rover root (mass ~25 kg is a good default).
    ///   - Four <see cref="WheelCollider"/>s, one per wheel link. Their center/radius/
    ///     suspension/friction values are configured from the public fields below on Awake.
    ///   - NO MeshColliders anywhere on the rover. URDF imports leave MeshColliders on
    ///     every link; they fight the wheels and launch the body into the sky. Disable or
    ///     remove them.
    ///   - A ground object with a collider for the wheels to sit on.
    ///
    /// Controls:
    ///   W/S or Up/Down  - throttle (forward/reverse)
    ///   A/D or L/R      - steering (left/right)
    ///   Left Shift      - boost
    ///   Left Ctrl       - precision
    ///   Space           - hard brake
    ///
    /// Drive model (true skid-steer):
    ///   - The commanded throttle is split into a LEFT multiplier and a RIGHT multiplier.
    ///   - On center (no steering): both sides get full throttle, rover goes straight.
    ///   - On full steering with no throttle: the controller injects a small "pivot" throttle,
    ///     so the inner side reverses and the outer side goes forward -> rover spins in place.
    ///   - On full steering WITH throttle: the inner side's multiplier moves toward
    ///     <see cref="innerWheelScale"/> (default -0.5), giving a tight arc turn.
    ///   - The max-linear-speed cap is enforced by reducing the commanded throttle as the
    ///     rover approaches the cap, not by fighting it with brakes. This gives natural
    ///     constant-speed feel without oscillation.
    ///   - Releasing throttle applies NO engine brake - the rover coasts to a stop. Space
    ///     is the hard brake.
    ///   - Visual wheel meshes spin from the actual WheelCollider.rpm.
    /// </summary>
    [DisallowMultipleComponent]
    public class RoverDiffDriveController : MonoBehaviour
    {
        [Header("Rover Body")]
        [Tooltip("Root of the rover. Must have a Rigidbody. Not auto-created.")]
        public Transform roverRoot;

        [Header("Wheels (visual spin only)")]
        public Transform wheelFL;
        public Transform wheelFR;
        public Transform wheelBL;
        public Transform wheelBR;

        [Header("Wheel Colliders (physics)")]
        [Tooltip("Front-left WheelCollider. Auto-found on wheelFL if left null.")]
        public WheelCollider colliderFL;
        public WheelCollider colliderFR;
        public WheelCollider colliderBL;
        public WheelCollider colliderBR;

        [Header("Apply Physics Defaults On Awake")]
        [Tooltip("If true, the values below are pushed onto the WheelColliders in Awake. " +
                 "If false, the controller uses whatever is configured on the colliders " +
                 "in the prefab/scene (only torque/brake inputs are applied per-frame).")]
        public bool applyWheelDefaults = true;

        [Header("Wheel Defaults (used when applyWheelDefaults is true)")]
        [Tooltip("Wheel mass in kg (unsprung mass per wheel). Small value (0.5-2).")]
        public float wheelMass = 2.0f;

        [Tooltip("Wheel radius in meters. URDF wheel diameter is 0.20 m -> radius 0.10.")]
        public float wheelRadius = 0.10f;

        [Tooltip("Suspension distance in meters (how far the wheel can travel up/down). " +
                 "Must be large enough that wheels can drop below the body. 0.15-0.25 is typical.")]
        public float suspensionDistance = 0.20f;

        [Tooltip("Suspension spring stiffness (N/m). For a ~20 kg rover, 20000-40000 is typical. " +
                 "Too low and the body sags onto the wheels; too high and bumps feel harsh.")]
        public float suspensionSpring = 25000f;

        [Tooltip("Suspension damper. ~0.7 * 2 * sqrt(spring * cornerMass) is a good critical-damping value. " +
                 "For spring=25000 and corner mass ~5 kg, ~700 is a starting point.")]
        public float suspensionDamper = 700f;

        [Tooltip("Force-app point distance (m). 0 = at the wheel center.")]
        public float forceAppPointDistance = 0f;

        [Header("Drive Limits")]
        [Tooltip("Maximum forward speed in m/s. The commanded throttle is reduced as the rover " +
                 "approaches this cap (so it naturally tops out without fighting itself).")]
        public float maxLinearSpeed = 5.0f;

        [Tooltip("Maximum per-wheel motor torque in N*m. Total tractive force = (4 * motorTorque) / wheelRadius.")]
        public float maxMotorTorque = 80f;

        [Tooltip("Hard brake torque (N*m) used only for the Space key.")]
        public float maxBrakeTorque = 300f;

        [Header("Steering")]
        [Tooltip("Multiplier applied to the inner side's motor torque at full steering WITH throttle. " +
                 "0 = inner wheels stop (tight turn). -0.5 = inner wheels reverse (pivot arc). " +
                 "-1 = inner wheels match outer in reverse (spin in place).")]
        [Range(-1f, 1f)]
        public float innerWheelScale = -0.5f;

        [Tooltip("Magnitude of the synthesized throttle (0..1) used to drive a pure pivot turn " +
                 "when only steering is pressed. 0.4-0.6 gives a natural in-place rotation rate.")]
        [Range(0f, 1f)]
        public float pivotThrottle = 0.5f;

        [Tooltip("Additional yaw torque (N*m) applied to the body around its local up. " +
                 "Helps the rover pivot when wheel friction alone isn't enough. " +
                 "Only applied when steering is non-zero.")]
        public float yawAssistTorque = 30f;

        [Header("Input")]
        public float boostMultiplier = 1.5f;
        public float precisionMultiplier = 0.35f;
        [Tooltip("Time (seconds) for throttle/steer to ramp to the commanded value. " +
                 "0.03 = very snappy, 0.1 = responsive, 0.3+ = sluggish. " +
                 "DO NOT set this above 0.5 - the rover will feel laggy.")]
        public float inputSmoothing = 0.03f;
        public bool invertForward = false;

        // --- internal state ---
        private float _throttle;       // -1..1
        private float _steer;          // -1..1
        private float _brakeHold;      // 0..1 (Space key)
        private float _throttleVel;
        private float _steerVel;
        private float _diagTimer;
        private Rigidbody _rb;

        private void Reset()
        {
            roverRoot = transform;
        }

        private void Awake()
        {
            if (roverRoot == null) roverRoot = transform;

            _rb = roverRoot.GetComponent<Rigidbody>();
            if (_rb == null)
            {
                Debug.LogError($"[RoverDiffDriveController] No Rigidbody on '{roverRoot.name}'. " +
                               "Add one to the rover root - this controller will not create it.", this);
                enabled = false;
                return;
            }

            // Find colliders on the wheel link transforms if not assigned.
            colliderFL = colliderFL ?? GetWheelCollider(wheelFL);
            colliderFR = colliderFR ?? GetWheelCollider(wheelFR);
            colliderBL = colliderBL ?? GetWheelCollider(wheelBL);
            colliderBR = colliderBR ?? GetWheelCollider(wheelBR);

            if (applyWheelDefaults)
            {
                ConfigureWheel(colliderFL);
                ConfigureWheel(colliderFR);
                ConfigureWheel(colliderBL);
                ConfigureWheel(colliderBR);
            }

            Debug.Log($"[Rover] Awake complete. " +
                      $"rbMass={_rb.mass:F1} kg, " +
                      $"FL=(r={colliderFL?.radius:F3}, sp={colliderFL?.suspensionSpring.spring:F0}, " +
                      $"da={colliderFL?.suspensionSpring.damper:F0}, " +
                      $"tp={colliderFL?.suspensionSpring.targetPosition:F2}, " +
                      $"sd={colliderFL?.suspensionDistance:F2}), " +
                      $"maxTorque={maxMotorTorque}, maxBrake={maxBrakeTorque}, " +
                      $"maxSpeed={maxLinearSpeed}, smoothing={inputSmoothing}, " +
                      $"pivotThrottle={pivotThrottle:F2}, innerScale={innerWheelScale:F2}, " +
                      $"yawAssist={yawAssistTorque:F0}", this);
        }

        private void Update()
        {
            // Visual wheel spin from the colliders' last-rpm sample.
            ApplyVisualSpin(wheelFL, colliderFL);
            ApplyVisualSpin(wheelFR, colliderFR);
            ApplyVisualSpin(wheelBL, colliderBL);
            ApplyVisualSpin(wheelBR, colliderBR);
        }

        private void FixedUpdate()
        {
            if (_rb == null) return;

            ReadInput();

            // Speed cap, modified by boost/precision modifiers.
            float speedCap = maxLinearSpeed;
            bool boost = Keyboard.current != null && Keyboard.current.shiftKey.isPressed;
            bool precision = Keyboard.current != null && Keyboard.current.leftCtrlKey.isPressed;
            if (boost) speedCap *= boostMultiplier;
            if (precision) speedCap *= precisionMultiplier;

            // Project current velocity onto the rover's local forward axis (local +Z is Unity's
            // default forward for an unrotated object; this matches the rover model in scene).
            Vector3 localVel = roverRoot.InverseTransformDirection(_rb.linearVelocity);
            float forwardSpeed = localVel.z;

            // ---- Compose the "drive command" ----
            float throttle = _throttle;
            float steer = Mathf.Clamp(_steer, -1f, 1f);
            const float deadZone = 0.05f;
            bool throttleActive = Mathf.Abs(throttle) > deadZone;
            bool steerActive = Mathf.Abs(steer) > deadZone;

            // When the user presses ONLY steering (no throttle), do a pure pivot: left wheels
            // and right wheels get MIRROR torques, not a scaled-down version of forward throttle.
            // The per-side scale (innerWheelScale) only applies when there's forward motion.
            //
            // Sign convention (verified by hand):
            //   D (steer > 0, right turn) -> left wheels go FORWARD (+), right wheels go BACKWARD (-)
            //   A (steer < 0, left  turn) -> left wheels go BACKWARD (-), right wheels go FORWARD (+)
            //   (Unity: positive motorTorque = wheel spins the way it rolls the body forward.)
            bool pivotMode = !throttleActive && steerActive;
            float leftSign = pivotMode ? -Mathf.Sign(steer) : 1f;
            float rightSign = pivotMode ? Mathf.Sign(steer) : 1f;

            // Effective drive magnitude. In pivot mode we use pivotThrottle (independent of
            // innerWheelScale) so the user gets a consistent spin rate. In drive mode we use
            // the actual throttle so W+anything steers.
            float driveMag = pivotMode ? pivotThrottle : Mathf.Abs(throttle);
            float driveSign = pivotMode ? 1f : Mathf.Sign(throttle);

            // ---- Speed cap via throttle falloff (no brake fight) ----
            // As the rover approaches the cap, the commanded throttle is scaled down smoothly.
            // Once at the cap, throttle is zero -> wheels coast. This is the natural feel.
            //
            // In pivot mode we cap on YAW rate (angular speed) instead, so a held D doesn't
            // keep accelerating the spin.
            if (!pivotMode)
            {
                float absSpeed = Mathf.Abs(forwardSpeed);
                float speedFraction = speedCap > 0.01f ? Mathf.Clamp01(absSpeed / speedCap) : 1f;
                float throttleScale = 1f - SmoothStep01(speedFraction);
                if (Mathf.Sign(driveSign) == Mathf.Sign(forwardSpeed) && Mathf.Abs(driveSign) > 0.01f)
                {
                    driveMag *= throttleScale;
                }
            }
            else
            {
                // Pivot: cap on |angular Y velocity| in rad/s.
                Vector3 angVelLocal = roverRoot.InverseTransformDirection(_rb.angularVelocity);
                float maxYawRate = 1.5f; // rad/s, ~86 deg/s, comfortable spin rate
                float yawFrac = Mathf.Clamp01(Mathf.Abs(angVelLocal.y) / maxYawRate);
                driveMag *= 1f - SmoothStep01(yawFrac);
            }

            // ---- Per-side torque split (skid-steer, drive mode) ----
            // In drive mode, scale the inner side's torque toward innerWheelScale at full steer.
            //   - right turn (steer > 0) -> right side is inner -> rightScale = Lerp(1, inner, steer).
            //   - left  turn (steer < 0) -> left  side is inner -> leftScale  = Lerp(1, inner, -steer).
            // In pivot mode, the per-side sign is already set above (mirror torques) and the
            // scale stays at 1 so the user gets the full pivotThrottle on both sides.
            float leftScale = 1f;
            float rightScale = 1f;
            if (!pivotMode)
            {
                if (steer > deadZone)
                {
                    rightScale = Mathf.Lerp(1f, innerWheelScale, steer);
                }
                else if (steer < -deadZone)
                {
                    leftScale = Mathf.Lerp(1f, innerWheelScale, -steer);
                }
            }

            float leftTorque = leftSign * driveSign * driveMag * maxMotorTorque * leftScale;
            float rightTorque = rightSign * driveSign * driveMag * maxMotorTorque * rightScale;

            // Hard brake only when Space is held. No engine brake, no speed-cap brake.
            float brake = _brakeHold * maxBrakeTorque;

            ApplyWheelTorque(colliderFL, leftTorque, brake);
            ApplyWheelTorque(colliderBL, leftTorque, brake);
            ApplyWheelTorque(colliderFR, rightTorque, brake);
            ApplyWheelTorque(colliderBR, rightTorque, brake);

            // Yaw assist: helps when wheel friction alone isn't enough. Skip in pure pivot
            // mode (the wheels are already doing the work via mirror torques).
            if (!pivotMode && Mathf.Abs(steer) > deadZone && Mathf.Abs(throttle) > 0.01f)
            {
                _rb.AddRelativeTorque(Vector3.up * (steer * yawAssistTorque * Mathf.Abs(throttle)), ForceMode.Force);
            }

            _diagTimer += Time.fixedDeltaTime;
            if (_diagTimer >= 1f)
            {
                _diagTimer = 0f;
                Debug.Log($"[Rover] mode={(pivotMode ? "PIVOT" : "DRIVE")} " +
                          $"thr={throttle:F2} cmd={_throttle:F2} steer={steer:F2} " +
                          $"L={leftTorque:F0}Nm R={rightTorque:F0}Nm " +
                          $"fwdSpd={forwardSpeed:F2} m/s cap={speedCap:F1}  " +
                          $"FL g={colliderFL?.isGrounded} rpm={(colliderFL?.rpm ?? 0):F0}  " +
                          $"FR g={colliderFR?.isGrounded} rpm={(colliderFR?.rpm ?? 0):F0}  " +
                          $"BL g={colliderBL?.isGrounded} rpm={(colliderBL?.rpm ?? 0):F0}  " +
                          $"BR g={colliderBR?.isGrounded} rpm={(colliderBR?.rpm ?? 0):F0}");
            }
        }

        private static float SmoothStep01(float x)
        {
            // Standard Hermite smoothstep on [0,1]. 0->0, 1->1, smooth in between.
            x = Mathf.Clamp01(x);
            return x * x * (3f - 2f * x);
        }

        private void ReadInput()
        {
            float throttleRaw = 0f;
            float steerRaw = 0f;
            float brakeRaw = 0f;

            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.wKey.isPressed || kb.upArrowKey.isPressed) throttleRaw += 1f;
                if (kb.sKey.isPressed || kb.downArrowKey.isPressed) throttleRaw -= 1f;
                if (kb.aKey.isPressed || kb.leftArrowKey.isPressed) steerRaw -= 1f;
                if (kb.dKey.isPressed || kb.rightArrowKey.isPressed) steerRaw += 1f;
                if (kb.spaceKey.isPressed) brakeRaw = 1f;
            }

            if (invertForward) throttleRaw = -throttleRaw;

            if (inputSmoothing <= 0f)
            {
                _throttle = Mathf.Clamp(throttleRaw, -1f, 1f);
                _steer = Mathf.Clamp(steerRaw, -1f, 1f);
            }
            else
            {
                _throttle = Mathf.SmoothDamp(_throttle, Mathf.Clamp(throttleRaw, -1f, 1f), ref _throttleVel, inputSmoothing);
                _steer = Mathf.SmoothDamp(_steer, Mathf.Clamp(steerRaw, -1f, 1f), ref _steerVel, inputSmoothing);
            }
            _brakeHold = Mathf.Clamp01(brakeRaw);
        }

        private static void ApplyWheelTorque(WheelCollider wc, float motorTorque, float brakeTorque)
        {
            if (wc == null) return;
            wc.motorTorque = motorTorque;
            wc.brakeTorque = brakeTorque;
        }

        private static void ApplyVisualSpin(Transform visual, WheelCollider wc)
        {
            if (visual == null || wc == null) return;
            // WheelCollider.rpm is revolutions per minute. 1 rpm = 360 deg / 60 s = 6 deg/s.
            float degPerSec = wc.rpm * 6f;
            visual.Rotate(Vector3.right, degPerSec * Time.deltaTime, Space.Self);
        }

        private static WheelCollider GetWheelCollider(Transform t)
        {
            if (t == null) return null;
            return t.GetComponentInChildren<WheelCollider>(true);
        }

        private void ConfigureWheel(WheelCollider wc)
        {
            if (wc == null) return;

            wc.mass = Mathf.Clamp(wheelMass, 0.05f, 50f);
            wc.radius = Mathf.Clamp(wheelRadius, 0.02f, 2f);
            wc.forceAppPointDistance = forceAppPointDistance;

            // Suspension
            var susp = wc.suspensionSpring;
            susp.spring = Mathf.Max(0f, suspensionSpring);
            susp.damper = Mathf.Max(0f, suspensionDamper);
            susp.targetPosition = 0.5f;
            wc.suspensionSpring = susp;
            wc.suspensionDistance = Mathf.Clamp(suspensionDistance, 0.01f, 1f);

            // Forward friction: a flat curve at the configured stiffness. The default WheelCollider
            // curve has 0 slope which makes the rover unable to find any longitudinal force at low
            // torque. Give it a real curve.
            wc.forwardFriction = BuildFrictionCurve(1.0f);
            wc.sidewaysFriction = BuildFrictionCurve(1.0f);
        }

        private static WheelFrictionCurve BuildFrictionCurve(float stiffness)
        {
            // A simple "rising to a plateau" curve. At low slip the friction force grows linearly
            // up to `extremumSlip` / `extremumValue`, then decays to `asymptoteSlip` / `asymptoteValue`.
            // The plateau value controls grip strength; stiffness is a global multiplier on top.
            var c = new WheelFrictionCurve
            {
                extremumSlip = 0.4f,
                extremumValue = 1.0f,
                asymptoteSlip = 0.8f,
                asymptoteValue = 0.75f,
                stiffness = Mathf.Clamp(stiffness, 0.01f, 10f),
            };
            return c;
        }
    }
}
