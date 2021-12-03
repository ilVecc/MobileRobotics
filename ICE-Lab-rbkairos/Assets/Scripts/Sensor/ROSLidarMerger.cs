using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.Core;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;


public class ROSLidarMerger : MonoBehaviour
{
	public List<GameObject> sources;

    public string outputTopic = "/scan";
    public GameObject outputFrame;

	public float DesiredPublishPeriod = 0.1f;
	public float DesiredRangeMetersMin = 0.1f;
	public float DesiredRangeMetersMax = 8;
	public float DesiredScanAngleStartDegrees = 0;
	public float DesiredScanAngleEndDegrees = -359;
	public int NumMeasurementsPerScan = 360;

    List<ROSConnection> ros_laser_sources = new List<ROSConnection>();
	List<LaserScanMsg> ros_laser_msgs = new List<LaserScanMsg>();

    ROSConnection ros_laser_merged;
	Vector3 outputFramePos;
	Quaternion outputFrameRot;

	private static float delta = 1e-4f;
	private Func<Vector3, Vector3, bool> IsClose = (a, b) => (a - b).magnitude < delta;

    // Start is called before the first frame update
    void Start()
    {
		if (sources.Count == 0) {
			Debug.LogError("Must specify at least one sensor!");
		}

		// check scanners same plane
		outputFramePos = outputFrame.transform.position;
		outputFrameRot = outputFrame.transform.rotation;

		var output_y = outputFrameRot * Vector3.up;
		Func<Vector3, bool> IsOnPlane = (p) => Math.Abs(Vector3.Dot(outputFramePos - p, output_y)) < delta;
		for (int i = 0; i < sources.Count; i++) {
			var y_axis = sources[i].transform.rotation * Vector3.up;
			// we need to ensure y vectors have same direction
			if (!IsClose(output_y, y_axis) && !IsClose(output_y, -y_axis)) {
				Debug.LogError($"Frame {i+1} does not share the same axis as output frame");
			}
			// and that the origins lie on the same plane
			if (!IsOnPlane(sources[i].transform.position)) {
				Debug.LogError($"Frame {i+1} does not share the same plane as output frame");
			}
		}

		// IMPROVEMENT: may be useful when we don't care about a specific output frame
		// requires  https://github.com/GlitchEnzo/NuGetForUnity  to install  MathNet.Numerics

		// // calculate output frame pose
		// // average position: simply average the positions
		// outputFramePos = Vector3.zero;
		// foreach (GameObject source in sources) {
		// 	outputFramePos += source.transform.position;
		// }
		// outputFramePos /= sources.Count;
		// // average rotation: the average quaternion is the strongest eigenvector (i.e. with maximum eigenvalue) 
		// // https://core.ac.uk/download/pdf/10536576.pdf
		// var Q = Matrix<double>.Build.Dense(4, sources.Count);
		// for (int i = 0; i < sources.Count; i++) {
		// 	Quaternion q = sources[i].transform.rotation;
		// 	Q[0, i] = q.x;
		// 	Q[1, i] = q.y;
		// 	Q[2, i] = q.z;
		// 	Q[3, i] = q.w;
		// }
		// var avg_q = Q.TransposeAndMultiply(Q).Svd(true).VT.Row(0).Map(c => (float) c);  // stores the eigenvector with greatest eigenvalue
		// outputFrameRot = new Quaternion(avg_q[0], avg_q[1], avg_q[2], avg_q[3]);
		
		// register subscribers
		for (int i = 0; i < sources.Count; i++) {
			var index = i;  // this prevents sharing the same <i> variable in each callback
			var source_topic = sources[i].GetComponent<Lidar>().topic;
			var laser_source = ROSConnection.GetOrCreateInstance();
			laser_source.Subscribe<LaserScanMsg>(source_topic, msg => _cb_ROSStoreScan(index, msg));
			ros_laser_sources.Add(laser_source);
			ros_laser_msgs.Add(null);
		}
        ros_laser_merged = ROSConnection.GetOrCreateInstance();
        ros_laser_merged.RegisterPublisher<LaserScanMsg>(outputTopic);
    }

	void _cb_ROSStoreScan(int index, LaserScanMsg scan) 
	{
		ros_laser_msgs[index] = scan;
	}

	// TODO: we don't support intensity nor time_increment
	void RosPublisher()
	{
		float actual_PublishPeriod = DesiredPublishPeriod;
		float actual_RangeMetersMin = DesiredRangeMetersMin;
		float actual_RangeMetersMax = DesiredRangeMetersMax;

		float sec = 0;
		float nanosec = 0;
		
		// compose messages
		List<KeyValuePair<float, float>> scans = new List<KeyValuePair<float, float>>();
		for (int i = 0; i < ros_laser_msgs.Count; i++) {
			LaserScanMsg msg = ros_laser_msgs[i];
			if (msg == null) {
				continue;
			}
			// adjust settings
			actual_PublishPeriod = Mathf.Max(msg.scan_time, actual_PublishPeriod);
			actual_RangeMetersMin = Mathf.Max(msg.range_min, actual_RangeMetersMin);
			actual_RangeMetersMax = Mathf.Min(msg.range_max, actual_RangeMetersMax);
			sec += msg.header.stamp.sec;
			nanosec += msg.header.stamp.nanosec;
			// re-trace points on the new frame
			int n_scans = Mathf.CeilToInt((msg.angle_max - msg.angle_min) / msg.angle_increment);
			for (int j = 0; j < n_scans; j++) {
				var t = j / (float) n_scans;
				var yawSensorDegrees = Mathf.Lerp(msg.angle_min, msg.angle_max, t);
				var scanRotation = Quaternion.Euler(0f, yawSensorDegrees, 0f);
				var directionVector = sources[i].transform.rotation * scanRotation * Vector3.forward;
				var worldPoint = sources[i].transform.position + msg.ranges[j] * directionVector;
				var localPoint = Quaternion.Inverse(outputFrameRot) * (worldPoint - outputFramePos);
				var range = Mathf.Sqrt(localPoint.x * localPoint.x + localPoint.z * localPoint.z);
				var angle = Mathf.Atan2(localPoint.z, localPoint.x);
				scans.Add(new KeyValuePair<float, float>(angle, range));
			}
		}
		// awful way of doing a time mean, but it's ok
		sec /= (float) ros_laser_msgs.Count;
		nanosec /= (float) ros_laser_msgs.Count;
		scans.Sort((pa, pb) => pa.Key.CompareTo(pb.Key));

		// filter ranges
		List<float> ranges = new List<float>();
		float angleIncrement = (DesiredScanAngleEndDegrees - DesiredScanAngleStartDegrees) / (NumMeasurementsPerScan - 1);
		int NumMeasurementsTaken = 0;
		var NumMeasurementsExpected = NumMeasurementsPerScan;
		int lastMeasurementIndex = 0;
		while (NumMeasurementsTaken < NumMeasurementsExpected)
		{
			var t = NumMeasurementsTaken / (float)NumMeasurementsPerScan;
			var yawSensorDegrees = Mathf.Lerp(DesiredScanAngleStartDegrees, DesiredScanAngleEndDegrees, t);
			// average the measurements in that angle sector
			float measurement = 0;
			int firstMeasurementIndex = lastMeasurementIndex;
			while (lastMeasurementIndex < scans.Count && scans[lastMeasurementIndex].Key < yawSensorDegrees + angleIncrement) {
				measurement += scans[lastMeasurementIndex].Value;
				lastMeasurementIndex++;
			}
			if (lastMeasurementIndex != firstMeasurementIndex) {
				measurement /= (float) (lastMeasurementIndex - firstMeasurementIndex);
			} else {
				measurement = float.MaxValue;
			}
			
			if (measurement < actual_RangeMetersMin) {
				measurement = actual_RangeMetersMin;
			}
			if (measurement > actual_RangeMetersMax) {
				measurement = float.MaxValue;
			}
			ranges.Add(measurement);
			NumMeasurementsTaken++;
		}

		float angleStartRos = -DesiredScanAngleStartDegrees * Mathf.Deg2Rad;
		float angleEndRos = -DesiredScanAngleEndDegrees * Mathf.Deg2Rad;
		var out_msg = new LaserScanMsg
		{
			header = new HeaderMsg
			{
				frame_id = outputFrame.name,
				stamp = new TimeMsg
				{
					sec = Mathf.CeilToInt(sec),
					nanosec = (uint) Mathf.CeilToInt(nanosec)
				}
			},
			range_min = actual_RangeMetersMin,
			range_max = actual_RangeMetersMax,
			angle_min = angleStartRos,
			angle_max = angleEndRos,
			angle_increment = (angleEndRos - angleStartRos) / (NumMeasurementsPerScan - 1),
			time_increment = 0,
			scan_time = (float) DesiredPublishPeriod,
			intensities = new float[ranges.Count],
			ranges = ranges.ToArray()
		};
		ros_laser_merged.Publish(outputTopic, out_msg);
	}

    // Update is called once per frame
    void Update()
    {
        RosPublisher();
    }
}
