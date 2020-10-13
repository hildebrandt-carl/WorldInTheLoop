using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DistanceToSecondDrone : MonoBehaviour
{

    public GameObject objectDrone;
    float smallest_distance = 100;

    // Update is called once per frame
    void Update()
    {
        Vector3 current_position = transform.position;
        Vector3 object_position = objectDrone.transform.position;
        float distance = Vector3.Distance(current_position, object_position);
        if (distance < smallest_distance)
        {
            smallest_distance = distance;
        }
        Debug.Log("Current Distance: " + distance.ToString() + " Min Distance: " + smallest_distance.ToString());
    }
}
