using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class CarControllerTemp : MonoBehaviour
{
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have

    public Vector3 com = new Vector3(0,0,1.0f);
    public Rigidbody rb;

    public WheelCollider WheelL;
    public WheelCollider WheelR;
    public float antiRoll = 50000.0f;

    private WheelFrictionCurve forwardFriction, sidewaysFriction;
    private float driftFactor;
    public float handBrakeFrictionMultiplier = 2f;

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass += com;
    }

    public void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }

        WheelHit hit;
        float travelL = 1.0f;
        float travelR = 1.0f;

        bool groundedL = WheelL.GetGroundHit(out hit);
        if (groundedL)
            travelL = (-WheelL.transform.InverseTransformPoint(hit.point).y - WheelL.radius) / WheelL.suspensionDistance;

        bool groundedR = WheelR.GetGroundHit(out hit);
        if (groundedR)
            travelR = (-WheelR.transform.InverseTransformPoint(hit.point).y - WheelR.radius) / WheelR.suspensionDistance;

        float antiRollForce = (travelL - travelR) * antiRoll;

        if (groundedL)
            rb.AddForceAtPosition(WheelL.transform.up * -antiRollForce, WheelL.transform.position);
        if (groundedR)
            rb.AddForceAtPosition(WheelR.transform.up * antiRollForce, WheelR.transform.position);
    }
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F)){
            ResetCar();
        }
    }

    private void ResetCar()
    {
        Vector3 currPos = this.transform.position;
        currPos.y += 2;
        this.transform.position = currPos;
        this.transform.rotation = new Quaternion(0, 0, 0,0);
    }
    private void adjustTraction()
    {
        //tine it takes to go from normal drive to drift 
        float driftSmothFactor = .7f * Time.deltaTime;

        if (Input.GetKey(KeyCode.Space))
        {
            sidewaysFriction = WheelL.sidewaysFriction;
            forwardFriction = WheelL.forwardFriction;

            float velocity = 0;
            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = forwardFriction.extremumValue = forwardFriction.asymptoteValue =
                Mathf.SmoothDamp(forwardFriction.asymptoteValue, driftFactor * handBrakeFrictionMultiplier, ref velocity, driftSmothFactor);

            foreach (AxleInfo axleInfo in axleInfos)
            {
                axleInfo.leftWheel.sidewaysFriction = sidewaysFriction;
                axleInfo.leftWheel.forwardFriction = forwardFriction;
                axleInfo.rightWheel.sidewaysFriction = sidewaysFriction;
                axleInfo.rightWheel.forwardFriction = forwardFriction;
            }
            

            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = forwardFriction.extremumValue = forwardFriction.asymptoteValue = 1.1f;
            //extra grip for the front wheels
            foreach (AxleInfo axleInfo in axleInfos)
            {
                if (axleInfo.steering)
                {
                    axleInfo.leftWheel.sidewaysFriction = sidewaysFriction;
                    axleInfo.leftWheel.forwardFriction = forwardFriction;
                    axleInfo.rightWheel.sidewaysFriction = sidewaysFriction;
                    axleInfo.rightWheel.forwardFriction = forwardFriction;
                }
            }
            rb.AddForce(transform.forward * (rb.velocity.magnitude*3.6f / 400) * 40000);
        }
        //executed when handbrake is being held
        else
        {

            forwardFriction = WheelL.forwardFriction;
            sidewaysFriction = WheelL.sidewaysFriction;

            forwardFriction.extremumValue = forwardFriction.asymptoteValue = sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue =
                ((rb.velocity.magnitude * 3.6f * handBrakeFrictionMultiplier) / 300) + 1;

            foreach (AxleInfo axleInfo in axleInfos)
            {
                axleInfo.leftWheel.sidewaysFriction = sidewaysFriction;
                axleInfo.leftWheel.forwardFriction = forwardFriction;
                axleInfo.rightWheel.sidewaysFriction = sidewaysFriction;
                axleInfo.rightWheel.forwardFriction = forwardFriction;
            }
        }

        //checks the amount of slip to control the drift
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.motor)
            {
                WheelHit wheelHit;
                axleInfo.leftWheel.GetGroundHit(out wheelHit);
                axleInfo.rightWheel.GetGroundHit(out wheelHit);

                if (wheelHit.sidewaysSlip < 0) driftFactor = (1 + -Input.GetAxis("Horizontal")) * Mathf.Abs(wheelHit.sidewaysSlip);

                if (wheelHit.sidewaysSlip > 0) driftFactor = (1 + Input.GetAxis("Horizontal")) * Mathf.Abs(wheelHit.sidewaysSlip);
            }
        }

    }
}

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
}