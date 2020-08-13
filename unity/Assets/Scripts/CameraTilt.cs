using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTilt : MonoBehaviour
{

    // Update is called once per frame
    void Update()
    {
        float yaw = transform.rotation.eulerAngles.y ;
        Quaternion target = Quaternion.Euler(0, yaw, 0) ;
        transform.rotation = target ;
    }
}
