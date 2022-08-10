using UnityEngine;

public class URPosRot : MonoBehaviour
{
    [SerializeReference] private GameObject UR_TrackPoint;

    public Vector3 GetPosition()
    {
        return UR_TrackPoint.transform.position;
    }

    public Vector3 GetRotationEuler()
    {
        var eulerValue = UR_TrackPoint.transform.rotation.eulerAngles;
        for (int i = 0; i < 3; ++i)
        {
            eulerValue[i] %= 360;
            eulerValue[i] = eulerValue[i] > 180 ? eulerValue[i] - 360 : eulerValue[i];
        }
        return eulerValue;
    }

    public Quaternion GetRotationQuater()
    {
        return UR_TrackPoint.transform.rotation;
    }
}
