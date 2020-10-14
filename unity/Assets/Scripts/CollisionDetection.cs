using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetection : MonoBehaviour
{
    private int collision_number = 0;

    void OnTriggerEnter(Collider other)
    {
        Debug.Log("Collision Detected: " + collision_number.ToString());
        collision_number = collision_number + 1;
    }
}
