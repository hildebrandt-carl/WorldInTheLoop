using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SecondDroneMovement : MonoBehaviour
{

    // [Range(0.0f, 1.0f)]
    // public float speed = 0.3f ;
    public bool slow;
    public bool fast;
    public Vector3 start_position = new Vector3(0.0f, 0.0f, 3.0f);
    public Vector3 end_position  = new Vector3(0.0f, 1.0f, -1.0f);
    [Range(0,60)]
    public int startDelay;

    private Vector3 takeoff_position;
    private bool takeoff_achieved;
    private double _start_time;
    private float total_distance = 0f;
    private float total_time = 0f;
    private float tilt_speed = 0f;
    private Vector3 velocity = Vector3.zero;


    // Start is called before the first frame update
    void Start()
    {
        // Set the start and takeoff positions
        transform.position = start_position ;
        takeoff_position = start_position;
        takeoff_position.y = 1.0f;

        // Init vairables
        takeoff_achieved = false;
        _start_time = Time.time;

        if ((slow == false) || (fast == false))
        {
            if (slow == true)
            {
                total_time = 5.0f;
                tilt_speed = 0.01f;
            }
            if (fast == true)
            {
                total_time = 2.5f;
                tilt_speed = 0.05f;
            }
        }

        // Compute the total distance
        total_distance = Vector3.Distance(end_position, takeoff_position);
    }

    // Update is called once per frame
    void Update()
    {
        if (Time.time - _start_time > startDelay)
        {
            // Compute distance between current position and takeoff
            float dist = Vector3.Distance(takeoff_position, transform.position);
            // While you havent taken off, takeoff.
            if((dist > 0.2) && (takeoff_achieved == false))
            {
                transform.position = Vector3.Lerp(transform.position, takeoff_position, 0.02f);
            }
            // Then fly to end position
            else
            {
                takeoff_achieved = true;
                transform.position = Vector3.SmoothDamp(transform.position, end_position, ref velocity, total_time);

                // Tilt the drone based on the distance
                float max_angle = -15.0f;
                float current_dist = Vector3.Distance(end_position, transform.position);
                float desired_angle = max_angle * (current_dist / total_distance);
                transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.Euler(desired_angle, 0, 0), tilt_speed);
            }
        }


        
    }

}
