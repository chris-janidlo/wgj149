using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using crass;

public class Player : MonoBehaviour
{
    [Header("Physics")]
    public float Gravity;
    public float MaxSpeed, TurnSpeed;

    public float FlapTime, FlapBurst, FlapHorizontalScale;

    public float DragCoeff = 1;

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
        flapInput = Input.GetButton(FlapButton);
        translationalInput = Input.GetAxis(TranslationalAxis);
        rotationalInput = new Vector2
        (
            Input.GetAxis(YawAxis),
            Input.GetAxis(PitchAxis) * (InversePitch ? -1 : 1)
        );

        flapTimer -= Time.deltaTime;
    }

    void FixedUpdate ()
    {
        rotate();
        translate();
    }

    void rotate ()
    {
        targetRotation += rotationalInput * TurnSpeed;
        targetRotation = new Vector2
        (
            targetRotation.x % 360,
            Mathf.Clamp(targetRotation.y, -90, 90)
        );

        transform.localRotation = Quaternion.AngleAxis(targetRotation.y, Vector3.right);
        transform.localRotation *= Quaternion.AngleAxis(targetRotation.x, transform.InverseTransformDirection(Vector3.up));
    }

    void translate ()
    {
        if (flapTimer <= 0)
        {
            if (flapInput) flap();
            else glide();
        }

        Rigidbody.AddForce(Vector3.down * Gravity, ForceMode.Acceleration);
        Rigidbody.AddForce(-DragCoeff * Rigidbody.velocity.normalized * Rigidbody.velocity.sqrMagnitude);
    }

    void flap ()
    {
        flapTimer = FlapTime;

        var flapDir = (Vector3.up + translationalInput * transform.forward * FlapHorizontalScale).normalized;

        Rigidbody.AddForce(flapDir * FlapBurst, ForceMode.VelocityChange);
    }

    void glide ()
    {
        float verticality = Mathf.Abs(targetRotation.y) / 90;

        Rigidbody.velocity = Vector3.Scale
        (
            transform.forward,
            new Vector3
            (
                1 - verticality,
                verticality,
                1 - verticality
            )
        ).normalized * Rigidbody.velocity.magnitude;
    }
}
