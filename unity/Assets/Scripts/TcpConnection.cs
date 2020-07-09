using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using System.Linq;

// From: https://unitylist.com/p/hc8/Unity3D-Python-Communication

/// <summary>
///     Example of requester who only sends Hello. Very nice guy.
///     You can copy this class and modify Run() to suits your needs.
///     To use this class, you just instantiate, call Start() when you want to start and Stop() when you want to stop.
/// </summary>
public class TcpConnection : RunAbleThread
{
    private GameObject moveable_object;
    private byte[] current_image = new byte[1] {0};
    private int _port = 5555;

    public Vector3 position;
    public Vector3 orientation;
    public string message;
    public bool collision;

    public TcpConnection()
    {
        Debug.Log("Did no recieve the drone game object. The code will not work.");
    }

    public TcpConnection(GameObject game_obj, int port)
    {
        moveable_object = game_obj;
        _port = port;
    }

    public void SetImage(byte[] imcoming_img)
    {
        current_image = imcoming_img.ToArray();
    }

    /// <summary>
    ///     Request Hello message to server and receive message back. Do it 10 times.
    ///     Stop requesting when Running=false.
    /// </summary>
    protected override void Run()
    {
        ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet
        using (RequestSocket client = new RequestSocket())
        {
            client.Connect("tcp://localhost:" + _port.ToString());

            while(Running)
            {
                // Debug.Log("Sending Message");
                RosUnityMsg msg = new RosUnityMsg();
                // Debug.Log("Current Image: " + current_image);
                msg.image = current_image.ToArray();
                string msg_json = JsonUtility.ToJson(msg);

                client.SendFrame(msg_json);

                string message = null;
                bool gotMessage = false;
                while (Running)
                {
                    gotMessage = client.TryReceiveFrameString(out message); // this returns true if it's successful
                    if (gotMessage) 
                    {
                        break;
                    }
                }

                if (gotMessage)
                {
                    // Debug.Log("Received " + message);
                    RosUnityMsg received_msg = JsonUtility.FromJson<RosUnityMsg>(message);
                    // Get the pose
                    Vector3 new_pose = new Vector3(received_msg.pose_x, received_msg.pose_y, received_msg.pose_z);
                    position = new_pose;
                    // Get the orientation
                    Vector3 new_orientation = new Vector3(received_msg.orientation_x, received_msg.orientation_y, received_msg.orientation_z);
                    orientation = new_orientation;
                }
            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }
}