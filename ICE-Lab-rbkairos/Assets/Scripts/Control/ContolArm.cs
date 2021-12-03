using System.Collections;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;


public class ContolArm : MonoBehaviour
{
    private ArticulationBody[] jointArticulationBodies;
    public bool lockArm = true;
    public GameObject ur5;
    ArticulationReducedSpace joint1;
    ArticulationReducedSpace vel1;
    ArticulationReducedSpace joint2;
    ArticulationReducedSpace vel2;
    ArticulationReducedSpace joint3;
    ArticulationReducedSpace vel3;
    ArticulationReducedSpace joint4;
    ArticulationReducedSpace vel4;
    ArticulationReducedSpace joint5;
    ArticulationReducedSpace vel5;
    ArticulationReducedSpace joint6;
    ArticulationReducedSpace vel6;

    // Start is called before the first frame update
    void Start()
    {
        jointArticulationBodies = ur5.GetComponentsInChildren<ArticulationBody>();
        joint1 = jointArticulationBodies[0].jointPosition;  // shoulder_link
        joint2 = jointArticulationBodies[1].jointPosition;  // upper_arm_link
        joint3 = jointArticulationBodies[2].jointPosition;  // forearm_link
        joint4 = jointArticulationBodies[3].jointPosition;  // wrist_1
        joint5 = jointArticulationBodies[4].jointPosition;  // wrist_2
        joint6 = jointArticulationBodies[5].jointPosition;  // wrist_3
        vel1 = jointArticulationBodies[0].jointVelocity; vel1[0] = 0f;
        vel2 = jointArticulationBodies[1].jointVelocity; vel2[0] = 0f;
        vel3 = jointArticulationBodies[2].jointVelocity; vel3[0] = 0f;
        vel4 = jointArticulationBodies[3].jointVelocity; vel4[0] = 0f;
        vel5 = jointArticulationBodies[4].jointVelocity; vel5[0] = 0f;
        vel6 = jointArticulationBodies[5].jointVelocity; vel6[0] = 0f;

    }

    // Update is called once per frame
    void Update()
    {
        joint1[0] = 0f;
        joint2[0] = -3.14f / 4;
        joint3[0] = 3.14f / 4;
        joint4[0] = 0f;
        joint5[0] = 0f;
        joint6[0] = 0f;

        jointArticulationBodies[0].jointPosition = joint1;
        jointArticulationBodies[0].jointVelocity = vel1;
        jointArticulationBodies[1].jointPosition = joint2;
        jointArticulationBodies[1].jointVelocity = vel2;
        jointArticulationBodies[2].jointPosition = joint3;
        jointArticulationBodies[2].jointVelocity = vel3;
        jointArticulationBodies[3].jointPosition = joint4;
        jointArticulationBodies[3].jointVelocity = vel4;
        jointArticulationBodies[4].jointPosition = joint5;
        jointArticulationBodies[4].jointVelocity = vel5;
        jointArticulationBodies[5].jointPosition = joint6;
        jointArticulationBodies[5].jointVelocity = vel6;
    }
}