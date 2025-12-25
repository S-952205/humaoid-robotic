# OpenRouter Configuration

## Setup Instructions

1. Sign up at [OpenRouter](https://openrouter.ai/) and get your API key
2. Replace the placeholder in the `.env` file with your actual OpenRouter API key:
   ```
   OPENROUTER_API_KEY=your_actual_api_key_here
   ```
3. The system is configured to use the free model `qwen/qwen-2.5-72b-instruct:free`
4. Optionally, you can switch to `google/gemini-2.0-flash-exp:free` by changing the model name in `agent.py`

## Required Headers

OpenRouter requires the following headers for proper attribution:
- HTTP-Referer: Your application's URL
- X-Title: Your application's name

These should be configured in the underlying API client library.

## Model Options

The system currently uses `qwen/qwen-2.5-72b-instruct:free` which is a free model option.
Alternative free model: `google/gemini-2.0-flash-exp:free`

## Fallback Configuration

If OPENROUTER_API_KEY is not set, the system will use a dummy configuration for testing.

## Testing

After setting up your API key, restart the backend server to load the new configuration.