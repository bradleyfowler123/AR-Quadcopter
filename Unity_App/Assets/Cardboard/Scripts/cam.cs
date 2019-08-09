using UnityEngine;
using System.Collections;

public class cam : MonoBehaviour {
	// Use this for initialization
	void Start () {
        WebCamDevice[] devices = WebCamTexture.devices;
        for (var i = 0; i < devices.Length; i++) {
            Debug.Log(devices[i].name);
        }
        WebCamTexture webcamTexture = new WebCamTexture();
        Renderer renderer = GetComponent<Renderer>();
        renderer.material.mainTexture = webcamTexture;
        webcamTexture.Play();
    }
	
	// Update is called once per frame
	void Update () {
	
	}
}
