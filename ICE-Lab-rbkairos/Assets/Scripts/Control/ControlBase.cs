using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;

public enum ControlMode { Keyboard, ROS };

public class ControlBase : MonoBehaviour
{
    public GameObject wheelFrontRight;
    public GameObject wheelFrontLeft;
    public GameObject wheelBackRight;
    public GameObject wheelBackLeft;
    public bool omniDrive = false;

    private ArticulationBody wA_fr;
    private ArticulationBody wA_fl;
    private ArticulationBody wA_br;
    private ArticulationBody wA_bl;

    public float wheelRadius = 0.127f; // m
    public float trackWidth = 0.535f; // m between tyres
    public float trackPace = 0.430f; // m between tyres
    public float rollerRadius = 0.041f; // m between tyres
    public float forceLimit = 10;
    public float damping = 10;

    public float maxLinearSpeed = 1.5f; //  m/s
    public float maxRotationalSpeed = 3.0f; // rad/s

    public ControlMode mode = ControlMode.ROS;
    public string topicName = "/cmd_vel";
    public float ROSTimeout = 0.5f;
    private float lastCmdReceived = 0f;

    ROSConnection ros;
    private RotationDirection direction;
    private float rosLinearX = 0f;
    private float rosLinearY = 0f;
    private float rosAngular = 0f;

    void Start()
    {
        wA_fr = wheelFrontRight.GetComponent<ArticulationBody>();
        wA_fl = wheelFrontLeft.GetComponent<ArticulationBody>();
        wA_br = wheelBackRight.GetComponent<ArticulationBody>();
        wA_bl = wheelBackLeft.GetComponent<ArticulationBody>();
        SetParameters(wA_fr);
        SetParameters(wA_fl);
        SetParameters(wA_br);
        SetParameters(wA_bl);

        if (omniDrive)
        {
            // FIXME these angles may be wrong
            wA_fr.anchorRotation = Quaternion.Euler(0,  45, 0);
            wA_fl.anchorRotation = Quaternion.Euler(0, 315, 0);
            wA_br.anchorRotation = Quaternion.Euler(0, 315, 0);
            wA_bl.anchorRotation = Quaternion.Euler(0,  45, 0);
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, _cb_ReceiveROSCmd);
    }

    void _cb_ReceiveROSCmd(TwistMsg cmdVel)
    {
        rosLinearX = (float)cmdVel.linear.x;
        rosLinearY = (float)cmdVel.linear.y;
        rosAngular = (float)cmdVel.angular.z;
        lastCmdReceived = Time.time;
    }

    private void SetParameters(ArticulationBody joint)
    {
        ArticulationDrive drive = joint.xDrive;
        drive.forceLimit = forceLimit;
        drive.damping = damping;
        joint.xDrive = drive;
    }

    private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
    {
        ArticulationDrive drive = joint.xDrive;
        if (float.IsNaN(wheelSpeed))
        {
            drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            print(drive.targetVelocity);
        }
        else
        {
            drive.targetVelocity = wheelSpeed;
        }
        joint.xDrive = drive;
    }


    void FixedUpdate()
    {
        if (mode == ControlMode.Keyboard)
        {
            KeyBoardUpdate();
        }
        else if (mode == ControlMode.ROS)
        {
            ROSUpdate();
        }
    }

    private void KeyBoardUpdate()
    {

        float moveDirectionX = Input.GetAxis("Vertical");
        float inputLinearSpeedX = maxLinearSpeed * Time.deltaTime * 143.4f * Math.Sign(moveDirectionX);
        
        float moveDirectionY = Input.GetAxis("Horizontal");
        float inputLinearSpeedY = maxLinearSpeed * Time.deltaTime * 143.4f * Math.Sign(moveDirectionY);
        
        float turnDirection = Convert.ToInt32(Input.GetKey(KeyCode.Q)) - Convert.ToInt32(Input.GetKey(KeyCode.E));
        float inputAngularSpeed = maxRotationalSpeed * Math.Sign(-turnDirection);

        RobotInputOmni(inputLinearSpeedX, inputLinearSpeedY, inputAngularSpeed);

    }

    private void ROSUpdate()
    {
        if (Time.time - lastCmdReceived > ROSTimeout)
        {
            rosLinearX = 0f;
            rosLinearY = 0f;
            rosAngular = 0f;
        }
        RobotInputOmni(rosLinearX, rosLinearY, -rosAngular);
    }

    private void RobotInputOmni(float linSpeedX, float linSpeedY, float angSpeedZ) // m/s, m/s, rad/s
    {
        linSpeedX = Mathf.Clamp(linSpeedX, -maxLinearSpeed, maxLinearSpeed);
        linSpeedY = Mathf.Clamp(linSpeedY, -maxLinearSpeed, maxLinearSpeed);
        linSpeedY = omniDrive ? linSpeedY : 0;
        angSpeedZ = Mathf.Clamp(angSpeedZ, -maxRotationalSpeed, maxRotationalSpeed);

        float lx = trackPace/2;
        float ly = trackWidth/2;

        float wheelFrontLeftRotation  = (linSpeedX - linSpeedY + (lx + ly)*angSpeedZ) / wheelRadius * Mathf.Rad2Deg;
        float wheelFrontRightRotation = (linSpeedX + linSpeedY - (lx + ly)*angSpeedZ) / wheelRadius * Mathf.Rad2Deg;
        float wheelBackLeftRotation   = (linSpeedX + linSpeedY + (lx + ly)*angSpeedZ) / wheelRadius * Mathf.Rad2Deg;
        float wheelBackRightRotation  = (linSpeedX - linSpeedY - (lx + ly)*angSpeedZ) / wheelRadius * Mathf.Rad2Deg;

        SetSpeed(wA_fr, wheelFrontRightRotation);
        SetSpeed(wA_fl, wheelFrontLeftRotation);
        SetSpeed(wA_br, wheelBackRightRotation);
        SetSpeed(wA_bl, wheelBackLeftRotation);
    }
}
