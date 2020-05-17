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

    public AnimationCurve TurnSpeedByTranslationalSpeed;

    public float DeathCollisionSpeed;

    [Range(0, 1)]
    public float CriticalVerticality;

    public float FlapTime, FlapBurst, FlapHorizontalScale;

    public Vector2 PitchRange = new Vector2(-90, 90);

    public float DragCoefficientWhenFlapping;
    public float MaximumFallSpeedWhenBraking;

    public float HalfHeight, GroundedFudge;
    public LayerMask GroundLayers;

    [Header("Resources")]
    public float MaxStamina;

    public float StaminaChangePerSecondFlying, StaminaChangePerSecondResting, FlapStaminaCost;
    public float StaminaStunPeriod;

    public int BurstSlots;
    public float BurstStunPeriod;
    public AnimationCurve BurstRecoveryPerSecondByAvailableBurst; // where each slot is worth 100

    public float FallSlowAdditionalStaminaCostPerSecond;

    [Header("Controls")]
    public string FlapButton;
    public string TranslationalAxis, PitchAxis, YawAxis;
    public bool InversePitch;

    [Header("References")]
    public Rigidbody Rigidbody;

    public UnityEvent TriedToFlapWhenUnableTo, Died;

    public float Stamina { get; private set; }
    public float BurstStamina { get; private set; }

    bool flapInput { get; set; }
    // scalar for z force on flap (ie you move forward a little on flap when this is positive, or back a little when negative)
    float translationalInput { get; set; }
    // x is yaw, y is pitch
    Vector2 rotationalInput { get; set; }

    Vector2 targetRotation;

    float flapTimer;

    float staminaStunTimer, burstStaminaStunTimer;

    void Start ()
    {
        Stamina = MaxStamina;
        BurstStamina = BurstSlots * 100;
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
        if ((Rigidbody.velocity).sqrMagnitude >= DeathCollisionSpeed * DeathCollisionSpeed)
        {
            Died.Invoke();
            Destroy(this); // remove the component
        }
    }

    void trackInput ()
    {
        flapInput = Input.GetButton(FlapButton);
        translationalInput = Input.GetAxis(TranslationalAxis);
        rotationalInput = new Vector2
        (
            Input.GetAxis(YawAxis),
            Input.GetAxis(PitchAxis) * (InversePitch ? -1 : 1)
        );

        flapTimer -= Time.deltaTime;
    }

    void manageStamina ()
    {
        staminaStunTimer -= Time.deltaTime;
        if (staminaStunTimer <= 0)
        {
            bool touchingGround = Physics.SphereCast(new Ray(transform.position, Vector3.down), GroundedFudge, HalfHeight, GroundLayers);
            bool resting = touchingGround && Rigidbody.velocity.sqrMagnitude == 0;

            Stamina += (resting ? StaminaChangePerSecondResting : StaminaChangePerSecondFlying) * Time.deltaTime;
            Stamina = Mathf.Clamp(Stamina, 0, MaxStamina);
        }

        burstStaminaStunTimer -= Time.deltaTime;
        if (burstStaminaStunTimer <= 0)
        {
            BurstStamina += BurstRecoveryPerSecondByAvailableBurst.Evaluate(BurstStamina) * Time.deltaTime;
            BurstStamina = Mathf.Min(BurstStamina, BurstSlots * 100);
        }
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
        Rigidbody.AddForce(Vector3.down * Gravity, ForceMode.Acceleration);

        if (Stamina <= 0) return;

        bool flapping = flapTimer > 0;

        if (!flapping)
        {
            bool flappable = canFlap();

            if (!flapInput || !flappable)
            {
                glide();
            }
            else if (flapInput && !flappable)
            {
                TriedToFlapWhenUnableTo.Invoke();
            }
            else
            {
                flapping = true;
                flap();
            }
        }

        if (flapping)
        {
            Rigidbody.AddForce(-DragCoefficientWhenFlapping * Rigidbody.velocity.normalized * Rigidbody.velocity.sqrMagnitude, ForceMode.Acceleration);

            float staminaCostThisFrame = FallSlowAdditionalStaminaCostPerSecond * Time.deltaTime;

            // if we're braking and falling and have stamina
            if (translationalInput < 0 && Rigidbody.velocity.y < 0 && -Rigidbody.velocity.y > Mathf.Abs(Rigidbody.velocity.x) && -Rigidbody.velocity.y > Mathf.Abs(Rigidbody.velocity.z) && Stamina > staminaCostThisFrame)
            {
                Rigidbody.velocity = Vector3.ClampMagnitude(Rigidbody.velocity, MaximumFallSpeedWhenBraking);
                Stamina -= staminaCostThisFrame;
            }
        }
    }

    bool canFlap ()
    {
        return translationalInput < 0 || (Stamina >= FlapStaminaCost && BurstStamina >= 100);
    }

    void flap ()
    {
        flapTimer = FlapTime;

        var force = (Vector3.up + translationalInput * transform.forward * FlapHorizontalScale).normalized * FlapBurst;

        if (translationalInput < 0)
        {
            Vector2 ignoringGravity = new Vector2
            (
                Rigidbody.velocity.x,
                Rigidbody.velocity.z
            );

            force = Vector3.ClampMagnitude(force, ignoringGravity.magnitude);
        }

        Rigidbody.AddForce(force, ForceMode.VelocityChange);

        if (translationalInput >= 0)
        {
            consumeStamina(FlapStaminaCost);
            consumeBurstSlot();
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

        if (verticality >= CriticalVerticality && Mathf.Sign(moveDirection.y) != Mathf.Sign(Rigidbody.velocity.y))
            moveDirection.y *= -1;

        Rigidbody.velocity = moveDirection.normalized * Rigidbody.velocity.magnitude;
    }

    void consumeStamina (float amount)
    {
        Stamina -= amount;
        staminaStunTimer = StaminaStunPeriod;
    }

    void consumeBurstSlot ()
    {
        BurstStamina -= 100;
        burstStaminaStunTimer = BurstStunPeriod;
    }
}
