using UnityEngine;
using System.IO;

/// <summary>
/// Manages the retrieval of the API key from a text file in the Resources folder.
/// </summary>
public static class ApiKeyManager
{
    /// <summary>
    /// Retrieves the Gemini API key from a 'gemini_api_key.txt' file located in the Resources folder.
    /// </summary>
    /// <returns>The API key as a string, or an empty string if the file is not found.</returns>
    public static string GetApiKey()
    {
        // Attempt to load the text file containing the API key from the Resources folder.
        var textFile = Resources.Load<TextAsset>("gemini_api_key");

        if (textFile != null)
        {
            // If the file is found, read its content, trim any whitespace, and return the key.
            string apiKey = textFile.text.Trim();
            return apiKey;
        }
        else
        {
            // If the file is not found, log a warning to the console and return an empty string.
            Debug.LogWarning("API key file 'gemini_api_key.txt' not found in Resources folder.");
            return string.Empty;
        }
    }
}