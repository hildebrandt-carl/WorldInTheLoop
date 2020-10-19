using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTilt : MonoBehaviour
{

    public GameObject Drone;

   // Update is called once per frame
    void Update()
    {
        // Match the transform to the drones transform
        transform.rotation = Drone.transform.rotation;
        // Get the current transform's yaw
        float yaw = transform.rotation.eulerAngles.y ;
        Quaternion target = Quaternion.Euler(0, yaw, 0) ;
        // Update the current transforms yaw
        transform.rotation = target ;
    }
}