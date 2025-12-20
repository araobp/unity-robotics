using UnityEngine;
using UnityEngine.Networking;
using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json.Serialization;

/// <summary>
/// This class provides a client for interacting with the Google Gemini API,
/// including chat completion, function calling, and text-to-speech capabilities.
/// </summary>
public class Gemini
{
    /// <summary>
    /// Configuration properties for the Gemini client.
    /// </summary>
    public struct GeminiProps
    {
        /// <summary>
        /// The specific Gemini model to use (e.g., "gemini-2.5-flash").
        /// </summary>
        public string GeminiModel;
        /// <summary>
        /// The API key for authenticating with the Google Gemini API.
        /// </summary>
        public string GeminiApiKey;
    }

    // Private fields for API endpoints, history management, and logging.
    private string _apiEndpoint;
    private string _ttsApiEndpoint;
    private bool _enableHistory = false;
    private List<Content> _chatHistory = new List<Content>();
    private const int MAX_CHAT_HISTORY_LENGTH = 16;

    private string CHAT_HISTORY_LOG_PATH;

    /// <summary>
    /// A default callback function for logging Gemini's text output to the Unity console.
    /// </summary>
    /// <param name="text">The text to log.</param>
    private void DefaultOutputText(string text)
    {
        Debug.Log($"DEFAULT OUTPUT: {text}\n\n");
    }

    #region Public Methods
    #endregion
    #region Public Methods

    /// <summary>
    /// Initializes a new instance of the Gemini client.
    /// </summary>
    /// <param name="props">Configuration properties including the model and API key.</param>
    /// <param name="enableHistory">Whether to enable conversation history.</param>
    public Gemini(GeminiProps props, bool enableHistory = false)
    {
        if (string.IsNullOrEmpty(props.GeminiModel))
        {
            props.GeminiModel = "gemini-2.5-flash";
        }
        string api = $"https://generativelanguage.googleapis.com/v1beta/models/{props.GeminiModel}:generateContent"; // Base API for content generation
        Debug.Log($"Using Gemini API Endpoint: {api}");
        _apiEndpoint = $"{api}?key={props.GeminiApiKey}";
        _ttsApiEndpoint = $"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash-preview-tts:generateContent?key={props.GeminiApiKey}";
        _enableHistory = enableHistory;
        CHAT_HISTORY_LOG_PATH = Path.Combine(Application.persistentDataPath, "chat_history.txt");
    }

    /// <summary>
    /// Sends a request to the Gemini API and handles the conversation flow, including function calls.
    /// </summary>
    /// <param name="query">The user's text prompt.</param>
    /// <param name="systemInstruction">Instructions for the model on how to behave.</param>
    /// <param name="base64Images">A list of Base64-encoded images to include in the prompt.</param>
    /// <param name="jsonSchema">A JSON schema to enforce structured output from the model.</param>
    /// <param name="functionDeclarations">Declarations of functions the model can call.</param>
    /// <param name="functionHandlers">A dictionary of objects that contain the methods to handle function calls.</param>
    /// <param name="callback">A callback action to handle the model's final text response.</param>
    /// <returns>The final text response from the model after all function calls are resolved.</returns>
    public async Task<string> Chat(
        string query, 
        string systemInstruction, 
        List<string> base64Images = null,
        JObject jsonSchema = null,
        JObject functionDeclarations = null,
        Dictionary<string, object> functionHandlers = null,
        Action<string> callback = null)
    {
        // Use the default logger if no callback is provided.
        callback ??= DefaultOutputText;

        // Construct the conversation history, including the new user prompt.
        var conversation = new List<Content>(_chatHistory)
        {
            CreateUserContent(query, base64Images)
        };

        string latestResponseText = "";

        // Loop to handle multi-turn function calling.
        while (true)
        {
            var request = BuildRequest(conversation, systemInstruction, jsonSchema, functionDeclarations);
            var response = await SendRequest(request);

            if (response?.Candidates == null || !response.Candidates.Any())
            {
                Debug.LogError("Invalid response from Gemini API.");
                if (_enableHistory) _chatHistory.Clear();
                return null;
            }

            var modelResponseContent = response.Candidates[0].Content;
            // Add the model's response to the conversation history for the next turn.
            conversation.Add(modelResponseContent);

            // Extract any function calls requested by the model.
            var functionCalls = modelResponseContent.Parts
                .Where(p => p.FunctionCall != null)
                .Select(p => p.FunctionCall)
                .ToList();

            // Process and invoke the callback for any text parts in the response.
            foreach (var part in modelResponseContent.Parts.Where(p => !string.IsNullOrEmpty(p.Text)))
            {
                latestResponseText = part.Text;
                if (!latestResponseText.StartsWith("**")) // Don't output "thought" blocks
                {
                    callback(latestResponseText);
                }
            }

            // If there are no more function calls, the conversation turn is over.
            if (!functionCalls.Any())
            {
                break; // End of conversation turn
            }

            // Execute all function calls and gather results
            foreach (var funcCall in functionCalls)
            {
                var functionResponseContent = await HandleFunctionCall(funcCall, functionHandlers);
                conversation.Add(functionResponseContent);
            }
        }

        // Manage chat history if enabled.
        if (_enableHistory)
        {
            _chatHistory = conversation;
            if (_chatHistory.Count > MAX_CHAT_HISTORY_LENGTH)
            {
                _chatHistory.RemoveRange(0, _chatHistory.Count - MAX_CHAT_HISTORY_LENGTH);
            }
        }

        // Log the full conversation to a file for debugging.
        LogHistory(conversation);

        return latestResponseText;
    }

    /// <summary>
    /// Synthesizes speech from text.
    /// </summary>
    /// <param name="text">The text to synthesize.</param>
    /// <param name="voiceName">The name of the voice to use.</param>
    /// <returns>A byte array containing the synthesized audio data.</returns>
    public async Task<byte[]> SynthesizeSpeech(string text, string voiceName = "Leda")
    {
        // Construct the JSON payload for the Text-to-Speech API.
        var payload = new JObject();
        payload["contents"] = new JArray {
                new JObject {
                    ["parts"] = new JArray {
                        new JObject { ["text"] = text }
                    }
                }
            };
        payload["generationConfig"] = new JObject {
                ["responseModalities"] = new JArray { "AUDIO" },
                ["speechConfig"] = new JObject {
                    ["voiceConfig"] = new JObject {
                        ["prebuiltVoiceConfig"] = new JObject { ["voiceName"] = voiceName }
                    }
                }
            };
        //payload["model"] = "gemini-1.5-flash-preview-tts";
        
        // Send the request to the TTS API endpoint.
        string responseBody = await SendWebRequest(_ttsApiEndpoint, payload.ToString());
        if (string.IsNullOrEmpty(responseBody)) return null;

        // Parse the response and extract the Base64-encoded audio data.
        try
        {
            var jsonResponse = JObject.Parse(responseBody);
            var audioData = jsonResponse["candidates"]?[0]?["content"]?["parts"]?[0]?["inlineData"]?["data"]?.ToString();

            if (string.IsNullOrEmpty(audioData)) return null;

            return Convert.FromBase64String(audioData);
        }
        catch (Exception ex)
        {
            Debug.LogError($"JSON Parse Error or Base64 Decode Error: {ex.Message}\nResponse Body: {responseBody}");
            return null;
        }
    }

    /// <summary>
    /// Identifies objects in a scene from an image and a text prompt.
    /// </summary>
    /// <param name="prompt">The text prompt describing what to find.</param>
    /// <param name="base64Image">A Base64-encoded image of the scene.</param>
    /// <returns>A JObject containing the identified objects and their 2D coordinates.</returns>
    public async Task<JObject> FindObjectsInScene(string prompt, string base64Image)
    {
        // Define the JSON schema for the expected output of object detection.
        JObject jsonSchema = JObject.Parse(@"
        {
            'type': 'object',
            'properties': {
                'objects': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'label': { 'type': 'string' },
                            'point': {
                                'type': 'array',
                                'items': { 'type': 'number' },
                                'minItems': 2,
                                'maxItems': 2
                            }
                        },
                        'required': ['label', 'point']
                    }
                }
            },
            'required': ['objects']
        }");

        string jsonResponse = await Chat(prompt, 
                                         "You are a robotics assistant. Your task is to identify objects in the provided image based on the prompt and return their labels and 2D coordinates.", 
                                         new List<string> { base64Image }, 
                                         jsonSchema, 
                                         null, null, null);
        if (jsonResponse != null)
        {
            return JObject.Parse(jsonResponse);
        }
        return null;
    }

    /// <summary>
    /// Generates a step-by-step plan for a robot to execute a high-level command.
    /// </summary>
    /// <param name="command">The high-level natural language command for the robot.</param>
    /// <returns>A JObject containing the sequence of steps for the robot to perform.</returns>
    public async Task<JObject> GenerateRobotPlan(string command)
    {
        // Define the JSON schema for the expected output of a robot plan.
        JObject jsonSchema = JObject.Parse(@"
        {
            'type': 'object',
            'properties': {
                'plan': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'step': { 'type': 'string', 'description': 'A single, actionable step for the robot.' }
                        },
                        'required': ['step']
                    }
                }
            },
            'required': ['plan']
        }");

        string jsonResponse = await Chat(command, "You are a robotics planner. Deconstruct the user's command into a series of simple, actionable steps for a robot to follow.", null, jsonSchema, null, null, null);
        if (jsonResponse != null)
        {
            return JObject.Parse(jsonResponse);
        }
        return null;
    }

    #endregion

    #region Private Helpers

    /// <summary>
    /// Creates a user content object for the conversation history.
    /// </summary>
    /// <param name="query">The user's text prompt.</param>
    /// <param name="base64Images">A list of Base64-encoded images.</param>
    /// <returns>A content object representing the user's turn.</returns>
    private Content CreateUserContent(string query, List<string> base64Images)
    {
        var parts = new List<Part> { new Part { Text = query } };
        if (base64Images != null)
        {
            parts.AddRange(base64Images.Select(img => new Part
            {
                InlineData = new InlineData { MimeType = "image/jpeg", Data = img }
            }));
        }
        return new Content { Role = "user", Parts = parts.ToArray() };
    }

    /// <summary>
    /// Builds the full request object to be sent to the Gemini API.
    /// </summary>
    /// <param name="conversation">The current conversation history.</param>
    /// <param name="systemInstruction">The system-level instructions for the model.</param>
    //  <param name="jsonSchema">The JSON schema for structured responses.</param>
    /// <param name="functionDeclarations">The declarations of available functions.</param>
    /// <returns>A fully constructed GeminiRequest object.</returns>
    private GeminiRequest BuildRequest(List<Content> conversation, string systemInstruction, JObject jsonSchema, JObject functionDeclarations)
    {
        var request = new GeminiRequest
        {
            SystemInstruction = new Content { Parts = new[] { new Part { Text = systemInstruction } } },
            Contents = conversation.ToArray(),
            GenerationConfig = new GenerationConfig()
        };

        // Configure for structured JSON output if a schema is provided.
        if (jsonSchema != null)
        {
            request.GenerationConfig.ResponseMimeType = "application/json";
            request.GenerationConfig.ResponseSchema = jsonSchema;
        }

        if (functionDeclarations != null)
        {
            // Add tool configurations for function calling.
            request.Tools = new[] { new Tool { FunctionDeclarations = functionDeclarations["functions"] } };
        }

        return request;
    }

    private async Task<GeminiResponse> SendRequest(GeminiRequest request)
    {
        // Serialize the request object to JSON with camelCase naming convention.
        var settings = new JsonSerializerSettings { ContractResolver = new CamelCasePropertyNamesContractResolver(), NullValueHandling = NullValueHandling.Ignore };
        string payload = JsonConvert.SerializeObject(request, settings);

        // Send the web request and get the response body.
        string responseBody = await SendWebRequest(_apiEndpoint, payload);
        if (string.IsNullOrEmpty(responseBody)) return null;

        // Deserialize the JSON response into a GeminiResponse object.
        try
        {
            return JsonConvert.DeserializeObject<GeminiResponse>(responseBody);
        }
        catch (Exception ex)
        {
            Debug.LogError($"JSON Parse Error: {ex.Message}\nResponse Body: {responseBody}");
            return null;
        }
    }

    /// <summary>
    /// Handles a function call request from the model by executing the corresponding C# method.
    /// </summary>
    /// <param name="functionCall">The function call details from the model.</param>
    /// <param name="functionHandlers">A dictionary of objects containing the handler methods.</param>
    /// <returns>A content object containing the result of the function execution.</returns>
    private async Task<Content> HandleFunctionCall(FunctionCall functionCall, Dictionary<string, object> functionHandlers)
    {
        string funcName = functionCall.Name;
        JObject args = JObject.FromObject(functionCall.Args);

        Debug.Log($"Calling: {funcName} with args: {args}\n\n");

        // Execute the function using reflection.
        JObject functionResult = await ExecuteFunction(functionHandlers, funcName, args);

        // Create a function response content object to send back to the model.
        return new Content
        {
            Role = "function",
            Parts = new[]
            {
                new Part
                {
                    FunctionResponse = new FunctionResponse
                    {
                        Name = funcName,
                        Response = functionResult
                    }
                }
            }
        };
    }

    /// <summary>
    /// Logs the entire conversation to a text file for debugging purposes.
    /// </summary>
    /// <param name="conversation">The list of content objects representing the conversation.</param>
    private void LogHistory(List<Content> conversation)
    {
        try
        {
            Debug.Log($"Writing chat history log to: {CHAT_HISTORY_LOG_PATH}");
            string historyJson = JsonConvert.SerializeObject(conversation, Formatting.Indented);
            File.WriteAllText(CHAT_HISTORY_LOG_PATH, historyJson);
        }
        catch (Exception e)
        {
            Debug.LogError($"Cannot write log: {e.Message}");
        }
    }

    /// <summary>
    /// A generic helper to send a POST web request with a JSON payload.
    /// </summary>
    /// <param name="url">The URL to send the request to.</param>
    /// <param name="jsonPayload">The JSON payload as a string.</param>
    /// <returns>The response body as a string, or null if an error occurred.</returns>
    private async Task<string> SendWebRequest(string url, string jsonPayload)
    {
        using (UnityWebRequest request = new UnityWebRequest(url, "POST"))
        {
            byte[] bodyRaw = Encoding.UTF8.GetBytes(jsonPayload);
            request.uploadHandler = new UploadHandlerRaw(bodyRaw);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");
            // request.SetRequestHeader("Accept-Encoding", "identity"); // Unity handles this automatically usually

            var operation = request.SendWebRequest();

            while (!operation.isDone) await Task.Yield();

            if (request.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"Gemini Error: {request.error}\nResponse: {request.downloadHandler.text}");
                return null;
            }

            return request.downloadHandler.text;
        }
    }

    /// <summary>
    /// Executes a method on a referenced object within functionHandlers using Reflection.
    /// </summary>
    /// <param name="functionHandlers">A dictionary of objects containing handler methods.</param>
    /// <param name="funcName">The name of the function to execute.</param>
    /// <param name="args">The arguments for the function as a JObject.</param>
    /// <returns>The result of the function execution as a JObject.</returns>
    private async Task<JObject> ExecuteFunction(Dictionary<string, object> functionHandlers, string funcName, JObject args)
    {
        if (functionHandlers == null || functionHandlers.Count == 0)
        {
            Debug.LogError("No function handlers provided.");
            return new JObject { ["error"] = "No function handlers available" };
        }

        // Find the method across all handler objects
        var (targetObject, method) = FindMethod(functionHandlers, funcName);

        if (method == null)
        {
            Debug.LogError($"Function '{funcName}' not found on any of the provided handler objects.");
            return new JObject { ["error"] = $"Function {funcName} not found" };
        }

        try
        {            
            object result = null;
            // Check if the method returns a Task for async handling.
            bool isAwaitable = method.ReturnType.GetMethod(nameof(Task.GetAwaiter)) != null;

            if (method.GetParameters().Length > 0)
            {
                result = method.Invoke(targetObject, new object[] { args });
            }
            else
            {
                result = method.Invoke(targetObject, null);
            }

            // If the method is awaitable, wait for it to complete.
            if (isAwaitable && result is Task task)
            {
                await task;
                if (task.GetType().IsGenericType)
                {
                    result = task.GetType().GetProperty("Result").GetValue(task, null);
                }
                else { result = null; } // Task with no result
            }
            
            // Package the result into a JObject.
            return result is JObject jResult ? jResult : new JObject { ["result"] = JToken.FromObject(result) };
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error executing function {funcName}: {ex}");
            return new JObject { ["error"] = ex.Message };
        }
    }

    /// <summary>
    /// Finds a public method with the given name in any of the handler objects.
    /// </summary>
    /// <param name="functionHandlers">The dictionary of handler objects.</param>
    /// <param name="funcName">The name of the method to find.</param>
    /// <returns>A tuple containing the target object and the MethodInfo, or (null, null) if not found.</returns>
    private (object, MethodInfo) FindMethod(Dictionary<string, object> functionHandlers, string funcName)
    {
        foreach (var handler in functionHandlers.Values)
        {
            var method = handler.GetType().GetMethod(funcName, BindingFlags.Public | BindingFlags.Instance);
            if (method != null)
            {
                return (handler, method);
            }
        }
        return (null, null);
    }

    #endregion

    #region API Data Structures

    /// <summary>
    /// Represents the overall request sent to the Gemini API.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class GeminiRequest
    {
        public Content SystemInstruction { get; set; }
        public Content[] Contents { get; set; }
        public Tool[] Tools { get; set; }
        public GenerationConfig GenerationConfig { get; set; }
    }

    /// <summary>
    /// Represents the overall response from the Gemini API.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class GeminiResponse
    {
        public Candidate[] Candidates { get; set; }
    }

    /// <summary>
    /// Represents the response from the Text-to-Speech API.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class SynthesizeSpeechResponse
    {
        public Candidate[] Candidates { get; set; }
    }

    /// <summary>
    /// A candidate response generated by the model.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class Candidate
    {
        public Content Content { get; set; }
    }

    /// <summary>
    /// A piece of content in the conversation, associated with a role (user, model, or function).
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class Content
    {
        public string Role { get; set; }
        public Part[] Parts { get; set; }
    }

    /// <summary>
    /// A part of a multi-modal content message. Can be text, image data, a function call, etc.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class Part
    {
        public string Text { get; set; }
        public InlineData InlineData { get; set; }
        public FunctionCall FunctionCall { get; set; }
        public FunctionResponse FunctionResponse { get; set; }
        public AudioData AudioData { get; set; }
    }

    /// <summary>
    /// Represents inline data, such as a Base64-encoded image.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class InlineData
    {
        public string MimeType { get; set; }
        public string Data { get; set; }
    }

    /// <summary>
    /// Represents inline audio data from the TTS API.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class AudioData
    {
        public string MimeType { get; set; }
        public string Data { get; set; }
    }

    /// <summary>
    /// Represents a function call requested by the model.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class FunctionCall
    {
        public string Name { get; set; }
        public object Args { get; set; }
    }

    /// <summary>
    /// Represents the response from a function execution, to be sent back to the model.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class FunctionResponse
    {
        public string Name { get; set; }
        public object Response { get; set; }
    }

    /// <summary>
    /// A container for function declarations provided to the model.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class Tool
    {
        public JToken FunctionDeclarations { get; set; }
    }

    /// <summary>
    /// Configuration options for content generation.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class GenerationConfig
    {
        public string ResponseMimeType { get; set; }
        public JObject ResponseSchema { get; set; }
        public ThinkingConfig ThinkingConfig { get; set; }
    }

    /// <summary>
    /// Configuration for enabling the model's "thinking" or chain-of-thought process in the response.
    /// </summary>
    [JsonObject(NamingStrategyType = typeof(CamelCaseNamingStrategy))]
    public class ThinkingConfig
    {
        public bool IncludeThoughts { get; set; } = true;
    }

    #endregion
}