using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

/// <summary>
/// Represents an object detected in an image, containing its coordinates and a descriptive label.
/// </summary>
[Serializable]
public struct DetectedObject
{
    public Vector2Int point;
    public string label;
}

/// <summary>
/// Custom JsonConverter for handling Vector2Int serialization and deserialization from a JSON array.
/// By default, JsonConvert expects an object for complex types, but the Gemini API returns an array for points.
/// </summary>
public class Vector2IntConverter : JsonConverter<Vector2Int>
{
    public override Vector2Int ReadJson(JsonReader reader, Type objectType, Vector2Int existingValue, bool hasExistingValue, JsonSerializer serializer)
    {
        if (reader.TokenType == JsonToken.StartArray)
        {
            var array = JArray.Load(reader);
            if (array.Count == 2 && array[0].Type == JTokenType.Integer && array[1].Type == JTokenType.Integer)
            {
                // The API returns points as [y, x] (row, column), so we swap them to [x, y] for Unity's Vector2Int.
                return new Vector2Int(array[1].Value<int>(), array[0].Value<int>());
            }
        }
        throw new JsonSerializationException("Expected a JSON array of two integers for Vector2Int.");
    }

    public override void WriteJson(JsonWriter writer, Vector2Int value, JsonSerializer serializer)
    {
        // Serialize Vector2Int as a JSON array [x, y]
        serializer.Serialize(writer, new[] { value.x, value.y });
    }
}

/// <summary>
/// Interacts with the Gemini API to perform robotics-related vision and language tasks.
/// </summary>
public class GeminiRoboticsApi
{
    Gemini gemini;

    // Initializes the Gemini API client with the API key and model.
    public GeminiRoboticsApi()
    {
        gemini = new Gemini(new Gemini.GeminiProps
        {
            GeminiApiKey = ApiKeyManager.GetApiKey(),
            GeminiModel = "gemini-robotics-er-1.5-preview",
        }, false);
    }

    /// <summary>
    /// Detects objects in a given base64 encoded image and returns their locations and labels.
    /// </summary>
    /// <param name="base64Image">The base64 encoded image string.</param>
    /// <returns>A task that resolves to an array of DetectedObject structs.</returns>
    public async Task<DetectedObject[]> DetectObjects(string base64Image)
    {
        // The prompt explicitly defines the expected JSON structure, so we pass null for the jsonSchema parameter.
        string query = "Point to no more than 10 items in the image. The label returned should be an identifying name for the object detected. The answer should follow the json format: [{\"point\": [y, x], \"label\": \"<label1>\"}, ...]. The points are in [y, x] format normalized to 0-1000.";

        var images = new List<string> { base64Image };
        DetectedObject[] detectedObjects = null;
        
        // Pass null for the jsonSchema parameter.
        await gemini.Chat(query, "", images, null, null, null, (responseText) =>
        {
            // The response from Gemini should be a JSON string that matches the schema.
            if (!string.IsNullOrEmpty(responseText))
            {
                Debug.Log("Gemini Response: " + responseText);
                string jsonResponse = responseText.Trim();

                // The model sometimes returns the JSON wrapped in a markdown code block.
                // We need to remove the markers before deserializing.
                if (jsonResponse.StartsWith("```json"))
                {
                    jsonResponse = jsonResponse.Substring("```json".Length).Trim();
                }
                jsonResponse = jsonResponse.Trim('`');
                
                // Use the custom converter to handle Vector2Int deserialization from an array.
                detectedObjects = JsonConvert.DeserializeObject<DetectedObject[]>(jsonResponse, new Vector2IntConverter());
            }
        });

        // Return the array of detected objects, or an empty array if none were found.
        return detectedObjects ?? Array.Empty<DetectedObject>();
    }
}
