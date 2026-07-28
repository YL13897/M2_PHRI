using UnityEditor;
using UnityEngine;

// Temporary Editor-only validator for ExperimentBlockControl probability sequences.
// Run in Edit Mode: open the experiment scene, then select
// Tools > Experiment > Check Probability Sequence and read the Unity Console.
// It checks total trials and the probability/case counts for all normal and test sequence sets.

public static class ProbabilitySequenceChecker
{
    [MenuItem("Tools/Experiment/Check Probability Sequence")]
    static void Check()
    {
        var control = Object.FindFirstObjectByType<ExperimentBlockControl>();
        if (control == null)
        {
            Debug.LogError("[Probability] ExperimentBlockControl not found.");
            return;
        }

        // Read private serialized configuration without adding runtime accessors.
        var data = new SerializedObject(control);
        var probabilities = data.FindProperty("blockProbabilities");
        int[] expectedCases =
        {
            data.FindProperty("leftLfCount").intValue,
            data.FindProperty("rightRfCount").intValue,
            data.FindProperty("leftRfCount").intValue,
            data.FindProperty("rightLfCount").intValue
        };

        int setCount = Mathf.Min(
            ExperimentSequences.Random40.Length,
            ExperimentSequences.Case40.Length,
            ExperimentSequences.Random20.Length,
            ExperimentSequences.Case20.Length); // Length = 3; (3 sets for usage)

        for (int i = 0; i < setCount; i++)
        {
            int setNumber = i + 1;
            CheckSet($"normal set {setNumber}", ExperimentSequences.Random40[i], ExperimentSequences.Case40[i], probabilities, expectedCases);
            CheckSet($"test set {setNumber}", ExperimentSequences.Random20[i], ExperimentSequences.Case20[i], probabilities, new[] { 5, 5, 5, 5 });
        }
    }

    static void CheckSet(string label, string sequence, string cases, SerializedProperty probabilities, int[] expectedCases)
    {
        int sections = expectedCases[0] + expectedCases[1] + expectedCases[2] + expectedCases[3];
        int[] counts = new int[probabilities.arraySize];
        int[,] caseCounts = new int[probabilities.arraySize, 4];

        if (cases.Length != sequence.Length)
        {
            Debug.LogError($"[Probability] {label}: sequence lengths differ: probability={sequence.Length}, case={cases.Length}.");
            return;
        }

        // Count trials and cases for each probability.
        for (int i = 0; i < sequence.Length; i++)
        {
            int probability = sequence[i] - '0'; // Convert char to int.
            int caseIndex = cases[i] - '0';
            if (probability < 0 || probability >= counts.Length || caseIndex < 0 || caseIndex >= 4)
            {
                Debug.LogError($"[Probability] {label}: invalid value at trial {i + 1}.");
                return;
            }
            counts[probability]++;
            caseCounts[probability, caseIndex]++;
        }

        bool valid = sequence.Length == probabilities.arraySize * sections;
        for (int i = 0; i < counts.Length; i++)
        {
            valid &= counts[i] == sections; // Each probability should have the same number of trials as the total sections.
            for (int j = 0; j < expectedCases.Length; j++)
                valid &= caseCounts[i, j] == expectedCases[j];
            Debug.Log($"[Probability] {label} {probabilities.GetArrayElementAtIndex(i).floatValue:P3}: {counts[i]} trials; cases " +
                $"{caseCounts[i, 0]}/{caseCounts[i, 1]}/{caseCounts[i, 2]}/{caseCounts[i, 3]}");
        }

        if (valid)
            Debug.Log($"[Probability] {label}: sequence valid, {sequence.Length} trials.");
        else
            Debug.LogError($"[Probability] {label}: sequence invalid, expected {probabilities.arraySize * sections} trials, found {sequence.Length}.");
    }
}
