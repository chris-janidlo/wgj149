using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using crass;

public class Player : MonoBehaviour
{
    [Header("Physics")]
    public float Gravity;

    public float TurnSpeed;

    [Range(0, 1)]
    public float CriticalVerticality;

    public float FlapTime, FlapBurst, FlapHorizontalScale;

    public Vector2 PitchRange = new Vector2(-90, 90);

    public float DragCoefficientWhenFlapping;

    [Header("Controls")]
    public string FlapButton;
    public string TranslationalAxis, PitchAxis, YawAxis;
    public bool InversePitch;

    [Header("References")]
    public Rigidbody Rigidbody;

    bool flapInput { get; set; }
    // scalar for z force on flap (ie you move forward a little on flap when this is positive, or back a little when negative)
    float translationalInput { get; set; }
    // x is yaw, y is pitch
    Vector2 rotationalInput { get; set; }

    Vector2 targetRotation;

    float flapTimer;

    void Update ()
    {
        trackInput();
    }

    void FixedUpdate ()
    {
        rotate();
        translate();
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

    void rotate ()
    {
        targetRotation += rotationalInput * TurnSpeed;
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
        bool flapping = flapTimer > 0;

        if (!flapping)
        {
            if (flapInput) { flapping = true; flap(); }
            else glide();
        }

        Rigidbody.AddForce(Vector3.down * Gravity, ForceMode.Acceleration);

        if (flapping)
            Rigidbody.AddForce(-DragCoefficientWhenFlapping * Rigidbody.velocity.normalized * Rigidbody.velocity.sqrMagnitude, ForceMode.Acceleration);
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
}
