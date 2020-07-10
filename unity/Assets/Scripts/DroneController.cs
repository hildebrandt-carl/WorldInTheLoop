using UnityEngine;
using System.IO;
using System.Collections;
using System.Linq;

// From: https://unitylist.com/p/hc8/Unity3D-Python-Communication


public class DroneController : MonoBehaviour
{
    public GameObject droneObject;
    public Camera droneCamera;
    public int portNumber;
    public bool enableCamera;

    private TcpConnection _tcpConnection;

    private void Start()
    {
        _tcpConnection = new TcpConnection(droneObject, portNumber);
        _tcpConnection.position = droneObject.transform.position;
        _tcpConnection.orientation = droneObject.transform.rotation.eulerAngles;
        _tcpConnection.message = "";
        _tcpConnection.collision = false;
        _tcpConnection.Start();
    }

    private void FixedUpdate () 
    {
        droneObject.transform.position = _tcpConnection.position;
        Vector3 v = _tcpConnection.orientation;
        droneObject.transform.rotation = Quaternion.Euler(v.x, v.y, v.z);
    }

    private void LateUpdate()
    {
        if (enableCamera)
        {
            StartCoroutine(Capture());
        }
    }

    public IEnumerator Capture()
    {

        // Wait for the render to complete
        yield return new WaitForEndOfFrame();

        RenderTexture rt = RenderTexture.GetTemporary(512, 384);
        // RenderTexture rt = RenderTexture.GetTemporary(768, 576);
        // RenderTexture rt = RenderTexture.GetTemporary(1024, 768);
        // RenderTexture rt = RenderTexture.GetTemporary(2048, 1536);

        // Render to RenderTexture
        droneCamera.targetTexture = rt;
        droneCamera.Render();
        RenderTexture.active = rt;

        // Read pixels to texture
        Texture2D image = new Texture2D(droneCamera.targetTexture.width, droneCamera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, droneCamera.targetTexture.width, droneCamera.targetTexture.height), 0, 0);
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
    private void OnDestroy()
    {
        _tcpConnection.Stop();
    }
}