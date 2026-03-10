using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndlessRoad : MonoBehaviour
{

    [SerializeField]
    GameObject[] sectionsPrefabs;

    GameObject[] sectionsPool = new GameObject[20];

    GameObject[] sections = new GameObject[10];

    Transform playerCarTransform;

    WaitForSeconds waitFor100ms = new WaitForSeconds(0.1f);

    const float sectionLength = 200;

    // public Material roadMat;
    // public float speed = 1;
    // public float Yoffset = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        playerCarTransform = GameObject.FindGameObjectWithTag("Player").transform;

        int prefabIndex = 0;

        // create a pool for our endless sections
        for(int i = 0; i < sectionsPool.Length; i++)
        {
            sectionsPool[i] = Instantiate(sectionsPrefabs[prefabIndex]);
            sectionsPool[i].SetActive(false);

            prefabIndex++;

            // Loop the prefab index if we run out of prefabs
            if (prefabIndex > sectionsPrefabs.Length - 1)
            {
                prefabIndex = 0;
            }
        }


        ResetRoadAroundPlayer();

        StartCoroutine(UpdateLessOftenCO());
    }

    // Rebuild visible road sections around current player position.
    public void ResetRoadAroundPlayer()
    {
        if (playerCarTransform == null)
        {
            var p = GameObject.FindGameObjectWithTag("Player");
            if (p == null) return;
            playerCarTransform = p.transform;
        }

        for (int i = 0; i < sectionsPool.Length; i++)
        {
            if (sectionsPool[i] != null)
                sectionsPool[i].SetActive(false);
        }

        float baseZ = playerCarTransform.position.z;
        for (int i = 0; i < sections.Length; i++)
        {
            GameObject randomSection = GetRandomSectionFromPool();
            randomSection.transform.position = new Vector3(randomSection.transform.position.x, 0, baseZ + (i + 1) * sectionLength);
            randomSection.SetActive(true);
            sections[i] = randomSection;
        }
    }

    IEnumerator UpdateLessOftenCO()
    {
        while (true)
        {
            UpdateSectionPositions();
            yield return waitFor100ms;
        }
    }

    void UpdateSectionPositions()
    {
        for(int i = 0; i < sections.Length; i++)
        {
            // Check if section is too far behind
            if (sections[i].transform.position.z - playerCarTransform.position.z < -sectionLength)
            {
                // Store the position of the section and disable it
                Vector3 lastSectionPosition = sections[i].transform.position;
                sections[i].SetActive(false);

                // Get new section and enable it and move it forward
                sections[i] = GetRandomSectionFromPool();

                //Move the new section into place and active it
                sections[i].transform.position = new Vector3(lastSectionPosition.x, 0, lastSectionPosition.z + sectionLength * sections.Length);
                // Debug.Log($"i={i} name={sections[i].name} pos={sections[i].transform.position} playerZ={playerCarTransform.position.z}");
                sections[i].SetActive(true);
            }
        }
    }


    GameObject GetRandomSectionFromPool()
    {

        // Pick a random index and hope that it is available
        int randomIndex = Random.Range(0, sectionsPool.Length);

        bool isNewSectionFound = false;

        while(!isNewSectionFound)
        {
            // Check if the section is not active, in that case we've found a section
            if(!sectionsPool[randomIndex].activeInHierarchy)
            {
                isNewSectionFound = true;}
            else
            {
                randomIndex++;
                if(randomIndex > sectionsPool.Length - 1)
                {randomIndex = 0;}
            }


        }

        return sectionsPool[randomIndex];

    }

    // Update is called once per frame
    // void Update()
    // {
    //     Yoffset += speed * Time.deltaTime;
    //     roadMat.SetTextureOffset("_MainTex", new Vector2(0,Yoffset));
    // }
}
