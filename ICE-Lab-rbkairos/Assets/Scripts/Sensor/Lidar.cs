using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.Core;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using System;
using UnityEngine.Serialization;

public class Lidar : MonoBehaviour
{
	class ScanMarker
	{
		public GameObject SceneObject;
		public float TimeCurrentStateSeconds;

		internal Renderer Renderer => m_Renderer ??= SceneObject?.GetComponent<Renderer>();
		internal MaterialPropertyBlock PropertyBlock => m_PropertyBlock ??= new MaterialPropertyBlock();

		private Renderer m_Renderer;
		private MaterialPropertyBlock m_PropertyBlock;

		public void SetColor(Color color)
		{
			PropertyBlock.SetColor("_BaseColor", color);
			Renderer.SetPropertyBlock(PropertyBlock);
		}
	}

	public string frameId = "base_scan";
	public string topic="/scan";
	public bool useRos;
	ROSConnection ros;
	
	[FormerlySerializedAs("TimeBetweenScansSeconds")]
	public double PublishPeriod = 0.01;
	public float RangeMetersMin = 0.1f;
	public float RangeMetersMax = 8;
	public float ScanAngleStartDegrees = -135;
	public float ScanAngleEndDegrees = 135;
	public int NumMeasurementsPerScan = 270;
	public float TimeBetweenMeasurementsSeconds = 0f;

	bool isScanning = false;
	double TimeNextScanSeconds = -1;
	double TimeLastScanBeganSeconds = -1;
	int NumMeasurementsTaken;
	List<float> ranges = new List<float>();

	[SerializeField]
	bool RenderDebugVisuals;
	public GameObject debugMarkerPrefab;
	Vector3 pointCollision = Vector3.zero;
	[SerializeField, FormerlySerializedAs("ActiveMarkerGradient")]
	Gradient ActiveMarkerGradient;
	[SerializeField, FormerlySerializedAs("InActiveMarkerGradient")]
	Gradient InactiveMarkerGradient;

	Queue<ScanMarker> MarkersActive = new Queue<ScanMarker>();
	Queue<ScanMarker> MarkersInactive = new Queue<ScanMarker>();

	void Start()
	{
		if (useRos)
		{
			ros = ROSConnection.GetOrCreateInstance();
			ros.RegisterPublisher<LaserScanMsg>(topic);
		}
		TimeNextScanSeconds = Clock.Now + PublishPeriod;
	}

	void ActivateMarker(ScanMarker marker)
	{
		marker.SetColor(ActiveMarkerGradient.Evaluate(0f));
		marker.TimeCurrentStateSeconds = 0f;
		//marker.SetActive(true);
		MarkersActive.Enqueue(marker);
	}

	void UpdateAllMarkers()
	{
		if (!RenderDebugVisuals)
		{
			ResetMarkers();
			return;
		}
		var timeDelta = Clock.deltaTime;
		foreach (var marker in MarkersActive)
		{
			marker.TimeCurrentStateSeconds += timeDelta;
			var fadeAmount = Mathf.Clamp01(marker.TimeCurrentStateSeconds / (float)PublishPeriod);
			marker.SetColor(ActiveMarkerGradient.Evaluate(fadeAmount));
		}

		foreach (var marker in MarkersInactive)
		{
			marker.TimeCurrentStateSeconds += timeDelta;
			var fadeAmount = Mathf.Clamp01(marker.TimeCurrentStateSeconds / (float)PublishPeriod);
			marker.SetColor(InactiveMarkerGradient.Evaluate(fadeAmount));
		}
	}

	void ResetMarkers()
	{
		var inactiveColor = InactiveMarkerGradient.Evaluate(0f);

		while (MarkersActive.Count > 0)
		{
			var marker = MarkersActive.Dequeue();
			//marker.SetActive(false);
			marker.SetColor(inactiveColor);
			marker.TimeCurrentStateSeconds = 0f;
			MarkersInactive.Enqueue(marker);
		}
	}

	void BeginScan()
	{
		isScanning = true;
		TimeLastScanBeganSeconds = Clock.Now;
		TimeNextScanSeconds = TimeLastScanBeganSeconds + PublishPeriod;
		NumMeasurementsTaken = 0;
		ResetMarkers();
	}
	
	void EndScan()
	{
		NumMeasurementsTaken = 0;
		ranges.Clear();
		isScanning = false;

		// take into account real-time issues
        var now = (float)Clock.time;
        if (now > TimeNextScanSeconds)
        {
            Debug.LogWarning($"Failed to complete scan started at {TimeLastScanBeganSeconds:F} before next scan was " +
                             $"scheduled to start: {TimeNextScanSeconds:F}, rescheduling to now ({now:F})");
            TimeNextScanSeconds = now;
        }
	}

	void RosPublisher()
	{
		if (ranges.Count == 0)
		{
			Debug.LogWarning($"Took {NumMeasurementsTaken} measurements but found no valid ranges");
		}
        else if (ranges.Count != NumMeasurementsTaken || ranges.Count != NumMeasurementsPerScan)
        {
            Debug.LogWarning($"Expected {NumMeasurementsPerScan} measurements. Actually took {NumMeasurementsTaken} " +
                             $"and recorded {ranges.Count} ranges.");
        }

		var timestamp = new TimeStamp(Clock.time);
		float angleStartRos = -ScanAngleStartDegrees * Mathf.Deg2Rad;
		float angleEndRos = -ScanAngleEndDegrees * Mathf.Deg2Rad;
		if (angleStartRos > angleEndRos)
		{
			Debug.LogWarning("LaserScan was performed in a clockwise direction but ROS expects a counter-clockwise scan, flipping the ranges...");
			var temp = angleEndRos;
			angleEndRos = angleStartRos;
			angleStartRos = temp;
			ranges.Reverse();
		}
		var msg = new LaserScanMsg
		{
			header = new HeaderMsg
			{
				frame_id = frameId,
				stamp = new TimeMsg
				{
					sec = timestamp.Seconds,
					nanosec = timestamp.NanoSeconds
				}
			},
			range_min = RangeMetersMin,
			range_max = RangeMetersMax,
			angle_min = angleStartRos,
			angle_max = angleEndRos,
			angle_increment = (angleEndRos - angleStartRos) / (NumMeasurementsPerScan - 1),
			time_increment = TimeBetweenMeasurementsSeconds,
			scan_time = (float)PublishPeriod,
			intensities = new float[ranges.Count],
			ranges = ranges.ToArray()
		};

		ros.Publish(topic, msg);
	}

	void Update()
	{
		if (!isScanning)
		{
			if (Clock.NowTimeInSeconds < TimeNextScanSeconds)
			{
				return;
			}
			BeginScan();
		}

		var NumMeasurementsExpected = 
			TimeBetweenMeasurementsSeconds == 0 
			? NumMeasurementsPerScan
			: 1 + Mathf.FloorToInt((float)(Clock.time - TimeLastScanBeganSeconds) / TimeBetweenMeasurementsSeconds);
		NumMeasurementsExpected = Mathf.Min(NumMeasurementsExpected, NumMeasurementsPerScan);

		var baseRotation = transform.rotation;
		while (NumMeasurementsTaken < NumMeasurementsExpected)
		{
			var t = NumMeasurementsTaken / (float)NumMeasurementsPerScan;
			var yawSensorDegrees = Mathf.Lerp(ScanAngleStartDegrees, ScanAngleEndDegrees, t);
			var scanRotation = Quaternion.Euler(0f, yawSensorDegrees, 0f);
			var directionVector = baseRotation * scanRotation * Vector3.forward;
			// Debug.DrawLine(transform.position, transform.position + directionVector, new Color(t, t, t, 1.0f), 1.0f);
			var measurementStart = transform.position + RangeMetersMin * directionVector;
			var measurementRay = new Ray(measurementStart, directionVector);
			var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);

            // Only record measurement if it's within the sensor's operating range
			if (foundValidMeasurement)
			{
				ranges.Add(hit.distance);
				if (RenderDebugVisuals)
				{
					if (MarkersInactive.Count > 0)
					{
						var marker = MarkersInactive.Dequeue();
						ActivateMarker(marker);
						marker.SceneObject.transform.position = hit.point;
					}
					else if (debugMarkerPrefab != null)
					{
						var scanMarker = new ScanMarker
						{
							SceneObject = Instantiate(debugMarkerPrefab, hit.point, Quaternion.identity),
						};
						ActivateMarker(scanMarker);
					}

				}
			}
			else
			{
				ranges.Add(float.MaxValue);  // FIXME Visualization Toolkit complaints about objects (markers) being out of the world with this
											 //       so we must add an high laser max range, like 100, to account for the whole room, or just
											 //       stop using the Visualization Toolkit
			}

            // Even if Raycast didn't find a valid hit, we still count it as a measurement
			++NumMeasurementsTaken;
		}
		
		UpdateAllMarkers();

		// this will never happen by design, but could arise from the slam_toolbox; no idea why it's here though...
		// https://github.com/SteveMacenski/slam_toolbox/issues/426
		if (NumMeasurementsTaken >= NumMeasurementsPerScan)
		{
			if (NumMeasurementsTaken > NumMeasurementsPerScan)
			{
				Debug.LogError($"LaserScan has {NumMeasurementsTaken} measurements but we expected {NumMeasurementsPerScan}");
			}
			if (useRos)
			{
				RosPublisher();
			}
			EndScan();
		}
	}
}
