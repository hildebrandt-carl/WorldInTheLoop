using UnityEngine;
using System.IO;
using System.Collections;
using System.Linq;

// From: https://unitylist.com/p/hc8/Unity3D-Python-Communication


public class DroneHandler : MonoBehaviour
{

    // Have links to each of the objects
    public GameObject droneObject;
    public GameObject personObject;
    public Camera droneCamera;
    
    // Have the port number
    public int portNumber;

    // Check to enable camera
    public bool enableCamera = false;
    public bool updatePosition = true;

    // Create a TCP connection to ROS
    private TcpConnection _tcpConnection;

    // Keep track of the number of collisions
    private int collision_number = 0;

    private void Start()
    {
        // Create the TCP connection
        _tcpConnection = new TcpConnection(portNumber, updatePosition);

        // Update all the variables in the TCP node
        _tcpConnection.person_position      = personObject.transform.position;
        _tcpConnection.person_orientation   = personObject.transform.rotation.eulerAngles;

        // Set the collision as false
        _tcpConnection.collision = false;

        // Start the tcp connection
        _tcpConnection.Start();
    }

    // This happens on each frame
    private void FixedUpdate () 
    {
        Vector3 new_pose = _tcpConnection.drone_position;

        // Check if we want unity to use this position
        if (updatePosition)
        {   
            // Update the main drones position and orientation
            droneObject.transform.position = new_pose;
            Vector3 v = _tcpConnection.drone_orientation;
            droneObject.transform.rotation = Quaternion.Euler(v.x, v.y, v.z);
        }
        else
        {
            _tcpConnection.drone_position    = droneObject.transform.position;
            _tcpConnection.drone_orientation = droneObject.transform.rotation.eulerAngles;
        }

        // Update the person information
        _tcpConnection.person_position      = personObject.transform.position;
        _tcpConnection.person_orientation   = personObject.transform.rotation.eulerAngles;
    
        // Set the collision flag
        if (collision_number > 0)
        {
            _tcpConnection.collision = true;
        }
    }

    // Check for collisions
    void OnTriggerEnter(Collider other)
    {
        Debug.Log("Collision Detected: " + collision_number.ToString());
        collision_number = collision_number + 1;
    }

    // This happens on the late update
    private void LateUpdate()
    {
        // If the camera is enabled start the routine capture
        if (enableCamera)
        {
            StartCoroutine(Capture());
        }
    }

    // Capture the camera routine
    public IEnumerator Capture()
    {

        // Wait for the render to complete
        yield return new WaitForEndOfFrame();

        RenderTexture rt = RenderTexture.GetTemporary(480, 270);
        // RenderTexture rt = RenderTexture.GetTemporary(768, 576);
        // RenderTexture rt = RenderTexture.GetTemporary(1024, 768);
        // RenderTexture rt = RenderTexture.GetTemporary(2048, 1536);
        
        // This is the same dimension as sphinx
        // RenderTexture rt = RenderTexture.GetTemporary(1280, 720);

        // Render to RenderTexture
        droneCamera.targetTexture = rt;
        droneCamera.Render();
        RenderTexture.active = rt;

        // Read pixels to texture
        Texture2D image = new Texture2D(droneCamera.targetTexture.width, droneCamera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, droneCamera.targetTexture.width, droneCamera.targetTexture.height), 0, 0);
        // Possible cause of memory leak https://forum.unity.com/threads/texture2d-memory-leak.331895/
        image.Apply();

        // Encode texture into PNG
        byte[] image_bytes = image.EncodeToPNG();
        // byte[] image_bytes = image.GetRawTextureData();
        _tcpConnection.SetImage(image_bytes);

        // Clean up
        droneCamera.targetTexture = null;
        Object.Destroy(image);   

        // Save
        // File.WriteAllBytes(Application.dataPath + "/../Screenshots/SavedScreen.png", image_bytes);
    }

    // Destroy the TCP connection
    private void OnDestroy()
    {
        _tcpConnection.Stop();
    }
}