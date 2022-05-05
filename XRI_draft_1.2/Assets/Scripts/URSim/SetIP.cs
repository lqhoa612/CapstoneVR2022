using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class SetIP : MonoBehaviour
    {
        private void Awake() {
            RosConnector rosConnector = GetComponent<RosConnector>();
            string ip = PlayerPrefs.GetString("ROSIP", "100.103.134.255");
            string fullIP = "ws://" + ip + ":9090";
            rosConnector.RosBridgeServerUrl = fullIP;
        }
    }
}