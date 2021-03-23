using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using System.Linq;

// Modified From: https://unitylist.com/p/hc8/Unity3D-Python-Communication

public class TcpConnection : RunAbleThread
{
    // Link to the current image
    private byte[] current_image = new byte[1] {0};
    // Port
    private int _port = 5555;
    // Incoming message
    private string message;

    // Public variables for the drone handler to use
    public Vector3 drone_position;
    public Vector3 drone_orientation;
    public Vector3 person_position;
    public Vector3 person_orientation;
    public bool collision;
    public string timestamp;

    private bool update_position = true;

    // Constructor
    public TcpConnection()
    {
        Debug.Log("Did no recieve the port, and update bool");
    }

    // Constructor
    public TcpConnection(int port, bool update_pos)
    {
        _port = port;
        update_position = update_pos;
    }

    // Saves the current image
    public void SetImage(byte[] imcoming_img)
    {
        current_image = imcoming_img.ToArray();
    }

    // Saves the current image
    public void SetTimeStamp(string imcoming_img)
    {
        timestamp = imcoming_img;
    }

    // Main method
    protected override void Run()
    {
        // this line is needed to prevent unity freeze after one use, not sure why yet
        ForceDotNet.Force(); 

        // Using the TCP connection
        using (RequestSocket client = new RequestSocket())
        {
            // Connect
            client.Connect("tcp://localhost:" + _port.ToString());

            // While running
            while(Running)
            {
                // Create the message
                RosUnityMsg msg = new RosUnityMsg();

                // Update the message parameters
                msg.image                   = current_image.ToArray();
                msg.drone_pose_x            = drone_position.x;
                msg.drone_pose_y            = drone_position.y;
                msg.drone_pose_z            = drone_position.z;
                msg.drone_orientation_x     = drone_orientation.x;
                msg.drone_orientation_y     = drone_orientation.y;
                msg.drone_orientation_z     = drone_orientation.z;
                msg.person_pose_x           = person_position.x;
                msg.person_pose_y           = person_position.y;
                msg.person_pose_z           = person_position.z;
                msg.person_orientation_x    = person_orientation.x;
                msg.person_orientation_y    = person_orientation.y;
                msg.person_orientation_z    = person_orientation.z;
                msg.collision               = collision;
                msg.timestamp               = timestamp;

                // Convert the message to JSON
                string msg_json = JsonUtility.ToJson(msg);

                // Send the package back
                client.SendFrame(msg_json);

                // Wait for a responce
                string message = null;
                bool gotMessage = false;
                while (Running)
                {
                     // Returns true if it's successful
                    gotMessage = client.TryReceiveFrameString(out message);
                    if (gotMessage) 
                    {
                        break;
                    }
                }
                
                // If we got a message
                if (gotMessage)
                {
                    RosUnityMsg msg_in = JsonUtility.FromJson<RosUnityMsg>(message);
                    // Get the new pose and orientation
                    Vector3 new_pose        = new Vector3(msg_in.drone_pose_x, msg_in.drone_pose_y, msg_in.drone_pose_z);
                    Vector3 new_orientation = new Vector3(msg_in.drone_orientation_x, msg_in.drone_orientation_y, msg_in.drone_orientation_z);

                    // Update the main_drones position
                    if (update_position)
                    {
                        drone_orientation = new_orientation;
                        drone_position = new_pose;
                    }
                }
            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }
}