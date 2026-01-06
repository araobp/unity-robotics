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

[Serializable]
public struct Move
{
    public Vector2Int from;
    public Vector2Int to;
    public string label;
}

/// <summary>
/// Provides a custom JsonConverter for Vector2Int. This is necessary because the Gemini API
/// returns point coordinates as a JSON array (e.g., [y, x]), whereas the default JSON.NET
/// serializer expects a JSON object for complex types like Vector2Int. This converter
/// correctly handles the array-to-Vector2Int mapping and also swaps the y, x order to
/// Unity's standard x, y format.
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
/// Provides a high-level interface for interacting with the Gemini API to perform
/// robotics-related vision and language tasks. This class handles the construction of
/// API requests, sending them to the Gemini service, and parsing the JSON responses
/// into structured data that can be used by the robot.
/// </summary>
public class GeminiRoboticsApi
{
    Gemini gemini;

    /// <summary>
    /// Initializes a new instance of the GeminiRoboticsApi class, setting up the
    /// connection to the Gemini service with the necessary API key and model identifier.
    /// </summary>
    public GeminiRoboticsApi()
    {
        gemini = new Gemini(new Gemini.GeminiProps
        {
            GeminiApiKey = ApiKeyManager.GetApiKey(),
            GeminiModel = "gemini-robotics-er-1.5-preview",
        }, false);
    }

    /// <summary>
    /// Sends an image to the Gemini API to detect objects within it. The API returns a list
    /// of detected objects, each with a label and a 2D point coordinate (normalized to 0-1000)
    /// indicating its center.
    /// </summary>
    /// <param name="base64Image">The base64 encoded image string.</param>
    /// <returns>A task that resolves to an array of DetectedObject structs.</returns>
    public async Task<DetectedObject[]> DetectObjects(string base64Image)
    {
        // The prompt explicitly defines the expected JSON structure.
        string query = "Point to no more than 10 items in the image. The label returned should be an identifying name for the object detected. The answer should follow the json format: [{\"point\": [y, x], \"label\": \"<label1>\"}, ...]. The points are in [y, x] format normalized to 0-1000.";

        var images = new List<string> { base64Image };
        DetectedObject[] detectedObjects = null;
        
        await gemini.Chat(query, "", images, null, null, null, (responseText) =>
        {
            // The response from Gemini should be a JSON string that matches the schema.
            if (!string.IsNullOrEmpty(responseText))
            {
                // Log the raw response from Gemini for debugging.
                Debug.Log("Gemini Raw Response (DetectObjects): " + responseText);
                string jsonResponse = responseText.Trim();

                // The model sometimes returns the JSON wrapped in a markdown code block.
                // We need to remove the markers before deserializing.
                if (jsonResponse.StartsWith("```json"))
                {
                    jsonResponse = jsonResponse.Substring("```json".Length).Trim();
                }
                jsonResponse = jsonResponse.Trim('`');

                // Log the cleaned JSON for debugging before deserialization.
                Debug.Log("Cleaned JSON response (DetectObjects): " + jsonResponse);

                try
                {
                    // Use the custom converter to handle Vector2Int deserialization from an array.
                    detectedObjects = JsonConvert.DeserializeObject<DetectedObject[]>(jsonResponse, new Vector2IntConverter());
                    if (detectedObjects != null)
                    {
                        Debug.Log($"Successfully deserialized {detectedObjects.Length} detected objects.");
                    }
                    else
                    {
                        Debug.LogWarning("JSON deserialization resulted in a null 'detectedObjects' array.");
                    }
                }
                catch (JsonException ex)
                {
                    // Log any errors that occur during JSON deserialization.
                    Debug.LogError($"Failed to deserialize JSON response (DetectObjects): {ex.Message}\nJSON: {jsonResponse}");
                }
            }
        });

        // Return the array of detected objects, or an empty array if none were found.
        return detectedObjects ?? Array.Empty<DetectedObject>();
    }

    /// <summary>
    /// Sends an image and a natural language instruction to the Gemini API. The API analyzes
    /// the image to identify objects and interprets the instruction to determine a sequence of
    /// 'move' operations. Each move consists of a source point, a destination point, and the
    /// label of the object to be moved.
    /// </summary>
    /// <param name="base64Image">The base64 encoded image string.</param>
    /// <param name="instruction">The user's instruction for moving the objects.</param>
    /// <returns>A task that resolves to an array of Move structs, indicating from and to locations.</returns>
    public async Task<Move[]> DetectAndMoveObjects(string base64Image, string instruction)
    {
        // Construct the query for the Gemini API, asking it to identify objects and generate move instructions.
        // The query specifies the desired JSON output format.
        string query = $"Analyze the image to identify objects and follow the user's instruction for moving them. The user instruction is: '{instruction}'. For each object to be moved, provide its current location ('from'), its new location ('to'), and its label. The response should be in JSON format: [{{\"from\": [y1, x1], \"to\": [y2, x2], \"label\": \"<label>\"}}, ...]. The points are in [y, x] format normalized to 0-1000.";

        var images = new List<string> { base64Image };
        Move[] moves = null;

        await gemini.Chat(query, "", images, null, null, null, (responseText) =>
        {
            if (!string.IsNullOrEmpty(responseText))
            {
                // Log the raw response from Gemini for debugging.
                Debug.Log("Gemini Raw Response: " + responseText);
                string jsonResponse = responseText.Trim();

                // The model sometimes returns the JSON wrapped in a markdown code block.
                // This part of the code cleans up the response to extract the pure JSON string.
                if (jsonResponse.StartsWith("```json"))
                {
                    jsonResponse = jsonResponse.Substring("```json".Length).Trim();
                }
                jsonResponse = jsonResponse.Trim('`');

                // Log the cleaned JSON for debugging before deserialization.
                Debug.Log("Cleaned JSON response: " + jsonResponse);

                try
                {
                    // Deserialize the JSON string into an array of Move objects.
                    // A custom Vector2IntConverter is used to handle the [y, x] array format.
                    moves = JsonConvert.DeserializeObject<Move[]>(jsonResponse, new Vector2IntConverter());
                    if (moves != null)
                    {
                        Debug.Log($"Successfully deserialized {moves.Length} move operations.");
                    }
                    else
                    {
                        Debug.LogWarning("JSON deserialization resulted in a null 'moves' array.");
                    }
                }
                catch (JsonException ex)
                {
                    // Log any errors that occur during JSON deserialization.
                    Debug.LogError($"Failed to deserialize JSON response: {ex.Message}\nJSON: {jsonResponse}");
                }
            }
        });

        // Return the array of moves, or an empty array if deserialization failed or no moves were found.
        return moves ?? Array.Empty<Move>();
    }
}
