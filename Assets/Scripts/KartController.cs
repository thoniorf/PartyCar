using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
    public float forwardStiffness;
    public float sidewaysStiffness;
}

[System.Serializable]
public class VisualAxleInfo
{
    public Transform leftWheel;
    public Transform rightWheel;
}

public class KartController : MonoBehaviour
{
    public Rigidbody rbody;
    public Transform centerOfMass;

    public List<AxleInfo> axleInfos;
    public List<VisualAxleInfo> visualaxleInfos;
    public bool isGrounded;

    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float jumpForce;

    public float inAirDrag;
    public float groundedDrag;

    [Header("Input values")]
    public float acceleration;
    public float steering;
    public int jumped;

    // Start is called before the first frame update
    void Start()
    {
        rbody = GetComponent<Rigidbody>();
        centerOfMass.position = rbody.centerOfMass;
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        float motor = maxMotorTorque * acceleration;
        float steer = maxSteeringAngle * steering;

        isGrounded = false;

        for (int i = 0; i < axleInfos.Count; i++)
        {
            AxleInfo axleInfo = axleInfos[i];

            isGrounded = axleInfo.leftWheel.isGrounded || axleInfo.rightWheel.isGrounded;

            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steer;
                axleInfo.rightWheel.steerAngle = steer;
            }

            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyWheelTransformToVisual(i);
            UpdateWheelsStiffness();
        }

        if (isGrounded)
        {
            rbody.drag = groundedDrag;
        }

        if (jumped > 0 && isGrounded)
        {
            rbody.drag = inAirDrag;
            rbody.AddForce(rbody.transform.up * jumpForce, ForceMode.Impulse);
        }

    }

    private void UpdateWheelsStiffness()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            // forward friction
            WheelFrictionCurve leftForwardFriction = axleInfo.leftWheel.forwardFriction;
            leftForwardFriction.stiffness = axleInfo.forwardStiffness;
            axleInfo.leftWheel.forwardFriction = leftForwardFriction;

            WheelFrictionCurve rightForwardFriction = axleInfo.rightWheel.forwardFriction;
            rightForwardFriction.stiffness = axleInfo.forwardStiffness;
            axleInfo.rightWheel.forwardFriction = rightForwardFriction;

            // sideway friction
            WheelFrictionCurve leftSidewayFriction = axleInfo.leftWheel.sidewaysFriction;
            leftSidewayFriction.stiffness = axleInfo.sidewaysStiffness;
            axleInfo.leftWheel.sidewaysFriction = leftSidewayFriction;

            WheelFrictionCurve rightSidewayFriction = axleInfo.rightWheel.sidewaysFriction;
            rightSidewayFriction.stiffness = axleInfo.sidewaysStiffness;
            axleInfo.rightWheel.sidewaysFriction = rightSidewayFriction;
        }
    }

    private void ApplyWheelTransformToVisual(int index)
    {
        Vector3 position;
        Quaternion rotation;

        axleInfos[index].leftWheel.GetWorldPose(out position, out rotation);

        visualaxleInfos[index].leftWheel.position = position;
        visualaxleInfos[index].leftWheel.rotation = rotation;

        axleInfos[index].rightWheel.GetWorldPose(out position, out rotation);

        visualaxleInfos[index].rightWheel.position = position;
        visualaxleInfos[index].rightWheel.rotation = rotation;
    }

    public void Move(InputAction.CallbackContext context)
    {
        Vector2 movement = context.ReadValue<Vector2>();
        acceleration = movement.y > 0 ? 1 : movement.y < 0 ? -1 : 0;
        steering = movement.x > 0 ? 1 : movement.x < 0 ? -1 : 0;
    }

    public void Jump(InputAction.CallbackContext context)
    {
        float jump = context.ReadValue<float>();
        jumped = Mathf.CeilToInt(jump);
    }

    private void OnGUI()
    {
        // Kart speed
        GUI.Label(new Rect(10, 10, 100, 20), rbody.velocity.magnitude.ToString());
    }
}


