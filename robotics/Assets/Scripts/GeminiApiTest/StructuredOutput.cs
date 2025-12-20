using UnityEngine;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;

[AddComponentMenu("Structured Output Test")]
public class StructuredOutput : MonoBehaviour
{
    private Gemini gemini;

    void Awake()
    {
        string apiKey = ApiKeyManager.GetApiKey();
        Gemini.GeminiProps geminiProps = new Gemini.GeminiProps
        {
            GeminiApiKey = apiKey,
            GeminiModel = "gemini-2.5-flash",
        };
        
        // Instantiate the Gemini class directly
        gemini = new Gemini(geminiProps, false);
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    async void Start()
    {        
        string query = "Give me a recipe for a chocolate cake.";
        string systemInstruction = "You are a helpful assistant that provides recipes in a structured format.";
        
        // Define the JSON schema for the expected output
        JObject jsonSchema = JObject.Parse(@"
        {
            'type': 'object',
            'properties': {
                'recipe_name': {
                    'type': 'string'
                },
                'ingredients': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'name': { 'type': 'string' },
                            'quantity': { 'type': 'string' }
                        },
                        'required': ['name', 'quantity']
                    }
                },
                'instructions': {
                    'type': 'array',
                    'items': { 'type': 'string' }
                }
            },
            'required': ['recipe_name', 'ingredients', 'instructions']
        }");
        
        await gemini.Chat(query, systemInstruction, null, jsonSchema, null, null, (responseText) =>
        {
            Debug.Log("Gemini JSON Response: " + responseText);
        });
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
