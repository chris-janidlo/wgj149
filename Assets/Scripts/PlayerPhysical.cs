using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
using crass;

public class PlayerPhysical : MonoBehaviour
{
	[Header("Physics")]
	public float Gravity;
	public float FlapThrust; // TODO: some kind of acceleration over time as you reach high speeds

	public AnimationCurve DragCoefficientByResistanceFactor;

	public AnimationCurve LiftCoefficientByPitch;

	public float MaxTurnSpeed;
	public float TurnAcceleration, TurnDeceleration;

	public Vector2 PitchRange;

	[Header("Controls")]
	public string FlapButton;

	public bool InversePitch;
	public string PitchAxis, TurnAxis;

	[Header("References")]
	public Rigidbody Rigidbody;

	// input
	bool flapInput;
	Vector2 rotationalInput;

	// physical state
	Vector2 targetRotation; // y is pitch. x is a combination of roll and yaw called "turn," which can arbitrarily stand for yaw or roll as appropriate (ie, stands for yaw when rotating the body, but stands for roll in lift calculation)
	Vector2 turnVelocity;

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

        rotationalInput = new Vector2
        (
            Input.GetAxis(TurnAxis),
            Input.GetAxis(PitchAxis) * (InversePitch ? -1 : 1)
        );
	}

	void rotate ()
	{
		if (rotationalInput != Vector2.zero)
		{
			turnVelocity += rotationalInput * TurnAcceleration * Time.deltaTime;
			turnVelocity = Vector3.ClampMagnitude(turnVelocity, MaxTurnSpeed);
		}
		else
		{
			Vector2 deceleration = -turnVelocity.normalized * TurnDeceleration * Time.deltaTime;
			turnVelocity += Vector2.ClampMagnitude(deceleration, turnVelocity.magnitude);
		}

        targetRotation += turnVelocity * Time.deltaTime;

        targetRotation = new Vector2
        (
            Mathf.Repeat(targetRotation.x, 360),
            Mathf.Clamp(targetRotation.y, PitchRange.x, PitchRange.y)
        );

        transform.localRotation = Quaternion.AngleAxis(-targetRotation.y, Vector3.right);
        transform.localRotation *= Quaternion.AngleAxis(targetRotation.x, transform.InverseTransformDirection(Vector3.up));
	}

	void translate ()
	{
		Vector3 force =
			// forceFromGravity() +
			forceFromThrust() +
			forceFromDrag() +
			forceFromLift();
		
		Rigidbody.AddForce(force, ForceMode.Force);
	}

	Vector3 forceFromGravity ()
	{
		return Vector3.down * Gravity;
	}

	Vector3 forceFromThrust ()
	{
		return flapInput ? FlapThrust * transform.forward : Vector3.zero;
	}

	Vector3 forceFromDrag ()
	{
		// assuming that the airfoil is parallel to the ground when its up vector is equal to the world vector, this determines how much drag you'll get based on your rotation. if the airfoil is perfectly perpendicular to the direction of motion, then you get maximum drag. if the airfoil is travelling in parallel to the direction of motion, you get minimum drag. if it's somewhere in between, you get something betwen min and max
		float resistanceFactor = Mathf.Abs(Vector3.Dot(transform.up, Rigidbody.velocity.normalized));
		float dragCoefficient = DragCoefficientByResistanceFactor.Evaluate(resistanceFactor);

		Vector3 flowVelocityComponent = -Rigidbody.velocity.normalized * Rigidbody.velocity.sqrMagnitude;

		// treat the fluid density and reference area as constant factors included in the drag coefficient
		return dragCoefficient * flowVelocityComponent;
	}

	Vector3 forceFromLift ()
	{
		float liftCoefficient = LiftCoefficientByPitch.Evaluate(targetRotation.y);
		float forwardSpeed = Vector3.Dot(Rigidbody.velocity, transform.forward);
		
		// treat the air density and wing area as constant factors included in the lift coefficient
		return transform.up * liftCoefficient * forwardSpeed * forwardSpeed / 2;
	}
}
