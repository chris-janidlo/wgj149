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

    public float HalfHeight, GroundedFudge;
    public LayerMask GroundLayers;

    public float DeathCollisionSpeed;

    [Range(0, 1)]
    public float CriticalVerticality;

    [Tooltip("At any targetRotation value lower than this, you are considered to be in freefall")]
    public float FreefallAngle;

    public float GlideSpeedMaintenanceTime, GlideDeceleration;

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
    public string ForwardAxis, SidewaysAxis, GlideButton, PitchAxis, YawAxis;

    [Header("References")]
    public Rigidbody Rigidbody;
    public UnityEvent Died;

    public float Stamina { get; private set; }

    float verticality => Mathf.Abs(targetRotation.y) / 90;
    bool criticallyVertical => verticality >= CriticalVerticality;

    // x is forward/backward, y is side to side
    Vector2 translationalInput;
    // x is yaw, y is pitch
    Vector2 rotationalInput;
    bool glideInput;

    Vector2 targetRotation;

    float glideTimer;

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

        glideInput = Input.GetButton(GlideButton);
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
        if (Stamina > 0 && glideInput)
        {
            glide();
        }
        else if (Stamina > 0 && translationalInput != Vector2.zero)
        {
            glideTimer = GlideSpeedMaintenanceTime;
            flyManually();
        }
        else if (!criticallyVertical)
        {
            glideTimer = GlideSpeedMaintenanceTime;
            Rigidbody.velocity = Rigidbody.velocity.Decelerate(ManualFlyingDeceleration);
        }
        
        Rigidbody.velocity += Vector3.down * Gravity * Time.deltaTime;
    }

    void glide ()
    {
        Vector3 moveDirection = new Vector3
        (
            (1 - verticality) * transform.forward.x,
            verticality * transform.forward.y,
            (1 - verticality) * transform.forward.z
        );

        if (criticallyVertical && Mathf.Sign(moveDirection.y) != Mathf.Sign(Rigidbody.velocity.y))
            moveDirection.y *= -1;

        Rigidbody.velocity = moveDirection.normalized * Rigidbody.velocity.magnitude;

        if (targetRotation.y >= FreefallAngle)
        {
            glideTimer -= Time.deltaTime;

            if (glideTimer <= 0)
                Rigidbody.velocity = Rigidbody.velocity.Decelerate(GlideDeceleration);
        }
        else
        {
            glideTimer = GlideSpeedMaintenanceTime;
        }
    }

    void flyManually ()
    {
        Vector2 direction = translationalInput.normalized;

        if (targetRotation.y < FreefallAngle)
        {
            direction.x = 0;
        }
            
        Vector3 flyDirection = new Vector3
        (
            direction.y * ManualFlyingAcceleration.y,
            0,
            direction.x * ManualFlyingAcceleration.x
        );
        
        Rigidbody.velocity += transform.TransformDirection(flyDirection) * Time.deltaTime;

        if (!criticallyVertical && Rigidbody.velocity.y < 0)
        {
            Rigidbody.velocity += Vector3.up * Gravity * 2 * Time.deltaTime;
        }

        if (targetRotation.y >= FreefallAngle)
        {
            Rigidbody.velocity = Vector3.ClampMagnitude(Rigidbody.velocity, ManualFlyingMaxSpeed);
        }
    }
}
