using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Line : MonoBehaviour
{
    public GameObject _pos;
    public float lineLength = 1.0f;
    private Vector3 lineEnd, lineStart;

    // Start is called before the first frame update
    private void Start()
    {
        
    }

    // Update is called once per frame
    private void Update()
    {
        lineStart = _pos.transform.position;
        lineEnd = _pos.transform.position;
        lineEnd.z += lineLength;
        Debug.DrawLine(lineStart, lineEnd, Color.red);
    }
}
