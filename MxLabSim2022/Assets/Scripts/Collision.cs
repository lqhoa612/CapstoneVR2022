using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collision : MonoBehaviour
{
    [System.Serializable]
    public struct Tags
    {
        public string tagName;
    } 
    public Tags[] _tags;

    void OnCollisionEnter(UnityEngine.Collision collision)
    {
        for (int i = 0; i < _tags.Length; ++i)
        {
            string _tag = _tags[i].tagName;
            if (collision.collider.CompareTag(_tag))
            {
                Debug.LogWarning("Collision Detected!");
            }
        }    
    }
}
