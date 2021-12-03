using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.Transforms;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class ROSRobotStatePublisher : MonoBehaviour
{
    const string k_TfTopic = "/tf";
    const string k_JointStatesTopic = "/joint_states";

    private enum ManagerMode
        {
            PublishTf,
            PublishJointStates
        }
    
    [SerializeField]
    ManagerMode managerMode = ManagerMode.PublishTf;
    [SerializeField]
    double m_PublishRateHz = 20f;
    [SerializeField]
    List<string> m_GlobalFrameIds = new List<string> { "map", "odom" };
    [SerializeField]
    GameObject m_RootGameObject;
    
    double m_LastPublishTimeSeconds;

    TransformTreeNode m_TransformRoot;
    ROSConnection m_ROS;

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    // Start is called before the first frame update
    void Start()
    {
        if (m_RootGameObject == null)
        {
            Debug.LogWarning($"No GameObject explicitly defined as {nameof(m_RootGameObject)}, so using {name} as root.");
            m_RootGameObject = gameObject;
        }

        m_TransformRoot = new TransformTreeNode(m_RootGameObject);
        
        m_ROS = ROSConnection.GetOrCreateInstance();
        switch (managerMode)
        {
            case ManagerMode.PublishTf:
                m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
                break;
            case ManagerMode.PublishJointStates:
                m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
                m_ROS.RegisterPublisher<JointStateMsg>(k_JointStatesTopic);
                break;
        }
        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
    }

    // il nodo /robot_state_publisher usa /robot_description (per conoscere la geometria del robot) e /joint_states (per impostarne la configurazione) per pubblicare la cinematica diretta su /tf 
    // il nodo /joint_state_publisher usa /robot_description (per conoscere i giunti non fissi) per pubblicare la configurazione del robot su /joint_states

    void Update()
    {
        if (managerMode == ManagerMode.PublishTf)
        {
            if (ShouldPublishMessage)
            {
                PublishTFMessage(true);
            }
        }
        else if (managerMode == ManagerMode.PublishJointStates) 
        {
            if (ShouldPublishMessage)
            {
                PublishTFMessage(false);
                PublishJSMessage();
            }
        }
    }

    void PublishTFMessage(bool publishEverything = false)
    {
        var tfMessageList = new List<TransformStampedMsg>();

        // add  global[last] -> root  transform
        if (m_GlobalFrameIds.Count > 0)
        {
            var tfRootToGlobal = new TransformStampedMsg(
                new HeaderMsg(
                    new TimeStamp(Clock.time), 
                    m_GlobalFrameIds.Last()
                ),
                m_TransformRoot.name,
                m_TransformRoot.Transform.To<FLU>()
            );
            tfMessageList.Add(tfRootToGlobal);
        }
        else
        {
            Debug.LogWarning($"No {m_GlobalFrameIds} specified, transform tree will be entirely local coordinates.");
        }

        // in case of multiple "global" frames, chain them together as the same coordinate frame, 
        // treating them as an ordered list, where the first entry is the "true" global frame
        for (var i = 1; i < m_GlobalFrameIds.Count; ++i)
        {
            var tfGlobalToGlobal = new TransformStampedMsg(
                new HeaderMsg(
                    new TimeStamp(Clock.time), 
                    m_GlobalFrameIds[i - 1]
                ),
                m_GlobalFrameIds[i],
                // Initializes to identity transform
                new TransformMsg()
            );
            tfMessageList.Add(tfGlobalToGlobal);
        }

        // avoid publishing all the tree if not needed
        if (publishEverything)
        {
            PopulateTFList(tfMessageList, m_TransformRoot);
        }

        var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
        m_ROS.Publish(k_TfTopic, tfMessage);
        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

    static void PopulateTFList(List<TransformStampedMsg> tfList, TransformTreeNode tfNode)
    {
        // TODO: Some of this could be done once and cached rather than doing from scratch every time
        // Only generate transform messages from the children, because This node will be parented to the global frame
        foreach (var childTf in tfNode.Children)
        {
            tfList.Add(TransformTreeNode.ToTransformStamped(childTf));

            if (!childTf.IsALeafNode)
            {
                PopulateTFList(tfList, childTf);
            }
        }
    }

    void PublishJSMessage()
    {
        List<string> name = new List<string>();
        List<double> position = new List<double>();
        List<double> velocity = new List<double>();
        List<double> effort = new List<double>();

        ArticulationBody[] jointArticulationBodies = m_TransformRoot.SceneObject.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody joint in jointArticulationBodies)
        {
            if (!joint.gameObject.activeInHierarchy)
            {
                continue;
            }

            string joint_name = joint.GetComponent<UrdfJoint>().jointName;
            switch (joint.jointType)
            {
                case ArticulationJointType.RevoluteJoint:
                case ArticulationJointType.PrismaticJoint:
                    name.Add(joint_name);
                    // [0] because these 1 DoF joints
                    position.Add(joint.jointPosition[0]);
                    velocity.Add(joint.jointVelocity[0]);
                    effort.Add(joint.jointForce[0]);
                    break;
                case ArticulationJointType.FixedJoint:
                case ArticulationJointType.SphericalJoint:
                    break;
            }
        }

        JointStateMsg msg = new JointStateMsg(
            new HeaderMsg(
                new TimeStamp(Clock.time), 
                m_TransformRoot.name
            ),
            name.ToArray(),
            position.ToArray(),
            velocity.ToArray(),
            effort.ToArray()
        );

        m_ROS.Publish(k_JointStatesTopic, msg);
        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

}
