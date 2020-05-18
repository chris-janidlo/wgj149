using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
using crass;

public class Player : MonoBehaviour
{
    [Header("Physics")]
    public float Gravity;
    public float DragCoefficient;

    public float HalfHeight, GroundedFudge;
    public LayerMask GroundLayers;

    public float DeathCollisionSpeed;

    [Range(0, 1)]
    public float CriticalVerticality;

    public float FreefallBrakeAngle;

    public AnimationCurve TurnSpeedByTranslationalSpeed;
    public Vector2 PitchRange = new Vector2(-90, 90);

    [Tooltip("X is forward/backward, Y is side to side")]
    public Vector2 ManualFlyingAcceleration;
    public float ManualFlyingMaxSpeed, ManualFlyingDeceleration;

    [Header("Resources")]
    public float MaxStamina;
    public float StaminaChangePerSecondManualFlying, StaminaChangePerSecondResting;

    [Header("Controls")]
    public bool InversePitch;
    public string ForwardAxis, SidewaysAxis, PitchAxis, YawAxis;

    [Header("References")]
    public Rigidbody Rigidbody;
    public UnityEvent Died;

    public float Stamina { get; private set; }

    // x is forward/backward, y is side to side
    Vector2 translationalInput;
    // x is yaw, y is pitch
    Vector2 rotationalInput;

    public Vector3 automaticVelocityState, manualVelocityState;
    Vector2 targetRotation;

    void Start ()
    {
        Stamina = MaxStamina;
    }

    void Update ()
    {
        trackInput();
        manageStamina();
    }

    void FixedUpdate ()
    {
        rotate();
        translate();
    }

    void OnCollisionEnter (Collision collision)
    {
        if ((collision.relativeVelocity).sqrMagnitude >= DeathCollisionSpeed * DeathCollisionSpeed)
        {
            Died.Invoke();

            // ragdoll:
            Destroy(this); // remove the component
            Rigidbody.useGravity = true;
            Rigidbody.constraints = RigidbodyConstraints.None;
            // TODO: figure out camera following. maybe turn off rotation and switch to a lookat mode?
        }
    }

    void trackInput ()
    {
        translationalInput = new Vector2
        (
            Input.GetAxis(ForwardAxis),
            Input.GetAxis(SidewaysAxis)
        );

        rotationalInput = new Vector2
        (
            Input.GetAxis(YawAxis),
            Input.GetAxis(PitchAxis) * (InversePitch ? -1 : 1)
        );
    }

    void manageStamina ()
    {
        bool touchingGround = Physics.SphereCast(new Ray(transform.position, Vector3.down), GroundedFudge, HalfHeight, GroundLayers);
        bool resting = touchingGround && Rigidbody.velocity.sqrMagnitude == 0;

        bool flying = translationalInput != Vector2.zero;

        float stamChange = resting ? StaminaChangePerSecondResting
                         : flying ? StaminaChangePerSecondManualFlying
                         : 0;

        Stamina += stamChange * Time.deltaTime;
        Stamina = Mathf.Clamp(Stamina, 0, MaxStamina);
    }

    void rotate ()
    {
        targetRotation += rotationalInput * TurnSpeedByTranslationalSpeed.Evaluate(Rigidbody.velocity.magnitude);
        targetRotation = new Vector2
        (
            targetRotation.x % 360,
            Mathf.Clamp(targetRotation.y, PitchRange.x, PitchRange.y)
        );

        transform.localRotation = Quaternion.AngleAxis(-targetRotation.y, Vector3.right);
        transform.localRotation *= Quaternion.AngleAxis(targetRotation.x, transform.InverseTransformDirection(Vector3.up));
    }

    void translate ()
    {
        // AUTOMATIC

        automaticVelocityState += Vector3.down * Gravity * Time.deltaTime;

        if (Stamina > 0) glide();

        automaticVelocityState += -DragCoefficient * automaticVelocityState.normalized * automaticVelocityState.sqrMagnitude * Time.deltaTime;

        // MANUAL

        Vector3 manualVelocityChange;

        bool flyingManually = Stamina > 0 && translationalInput != Vector2.zero;

        if (!flyingManually)
        {
            manualVelocityChange = -manualVelocityState.normalized * ManualFlyingDeceleration;
            manualVelocityChange = Vector3.ClampMagnitude(manualVelocityChange, manualVelocityState.magnitude);
        }
        else
        {
            Vector2 direction = translationalInput.normalized;
            
            manualVelocityChange = new Vector3
            (
                direction.y * ManualFlyingAcceleration.y,
                0,
                direction.x * ManualFlyingAcceleration.x
            );
        }

        manualVelocityState += manualVelocityChange * Time.deltaTime;
        manualVelocityState = Vector3.ClampMagnitude(manualVelocityState, ManualFlyingMaxSpeed);

        // COMBINE

        if (flyingManually && targetRotation.y >= FreefallBrakeAngle)
        {
            automaticVelocityState = Vector3.ClampMagnitude(automaticVelocityState, ManualFlyingMaxSpeed);
        }

        Rigidbody.velocity = automaticVelocityState + transform.TransformDirection(manualVelocityState);

        if (flyingManually && targetRotation.y >= FreefallBrakeAngle)
        {
            Rigidbody.velocity = Vector3.ClampMagnitude(Rigidbody.velocity, ManualFlyingMaxSpeed);
        }
    }

    void glide ()
    {
        float verticality = Mathf.Abs(targetRotation.y) / 90;

        Vector3 moveDirection = new Vector3
        (
            (1 - verticality) * transform.forward.x,
            verticality * transform.forward.y,
            (1 - verticality) * transform.forward.z
        );

        if (verticality >= CriticalVerticality && Mathf.Sign(moveDirection.y) != Mathf.Sign(automaticVelocityState.y))
            moveDirection.y *= -1;

        automaticVelocityState = moveDirection.normalized * automaticVelocityState.magnitude;
    }
}
