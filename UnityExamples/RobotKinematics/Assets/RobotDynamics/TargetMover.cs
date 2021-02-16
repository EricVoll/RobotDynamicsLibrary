using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetMover : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    public Vector3 defaultPos;
    public bool UseSinX = true;
    public bool UseSinY = true;
    public bool UseSinZ = true;
    public float frequency = 1f;
    public float amplitude = 100f;

    // Update is called once per frame
    void Update()
    {
        var t = Time.realtimeSinceStartup * frequency;
        var v = Mathf.Sin(t) * amplitude;
        this.transform.position = new Vector3(UseSinX ? v + defaultPos.x : defaultPos.x, UseSinY ? v + defaultPos.y : defaultPos.y, UseSinZ ? v + defaultPos.z : defaultPos.z);
    }
}
