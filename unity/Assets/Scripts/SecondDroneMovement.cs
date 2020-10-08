using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SecondDroneMovement : MonoBehaviour
{

    [Range(0.0f, 1.0f)]
    public float speed = 0.01f ;
    public Vector3 start_position = new Vector3(0.0f, 0.0f, -3.0f);
    public Vector3 end_position  = new Vector3(0.0f, 1.5f, 2.0f);

    private float true_speed ;
    private Vector3 takeoff_position;
    private bool takeoff_achieved;

    // Start is called before the first frame update
    void Start()
    {
        // Set the start and takeoff positions
        transform.position = start_position ;
        takeoff_position = start_position;
        takeoff_position.y = 1.0f;
        // Compute the speed
        true_speed = speed / 10.0f;
        // Init vairables
        takeoff_achieved = false;
    }

    // Update is called once per frame
    void Update()
    {
        // Compute distance between current position and takeoff
        float dist = Vector3.Distance(takeoff_position, transform.position);
        // While you havent taken off, takeoff.
        if((dist > 0.2) && (takeoff_achieved == false))
        {
            transform.position = Vector3.Lerp(transform.position, takeoff_position, true_speed);
        }
        // Then fly to end position
        else
        {
            takeoff_achieved = true;
            transform.position = Vector3.Lerp(transform.position, end_position, true_speed);
        }
        
    }

}
