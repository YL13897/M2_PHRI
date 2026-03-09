using UnityEngine;

[DefaultExecutionOrder(-10000)]
public class GlobalRandomSeed : MonoBehaviour
{
    public int seed = 12345;
    public static int Seed { get; private set; } = 12345;
    public static bool IsInitialized { get; private set; } = false;

    void Awake()
    {
        Seed = seed;
        Random.InitState(Seed);
        IsInitialized = true;
        Debug.Log($"[Random] Global seed set to {Seed}");
    }
    
}
