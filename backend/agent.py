from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
from agents import function_tool
import logging
import os
from typing import Dict, List

logger = logging.getLogger(__name__)

# Initialize vector store (will be set from main.py)
vector_store = None

def set_vector_store(vs):
    """Set the vector store instance to be used by the agent"""
    global vector_store
    vector_store = vs

@function_tool
def retrieve_context_tool(query: str, top_k: int = 5) -> List[Dict]:
    """Tool to retrieve relevant context from knowledge base"""
    from embedder import CohereEmbedder

    # Check if vector store is initialized
    if vector_store is None:
        logger.warning("Vector store is not initialized in retrieve_context_tool")
        return []

    try:
        embedder = CohereEmbedder()
        query_embedding = embedder.embed_text(query)
        results = vector_store.search_similar(query_embedding, top_k=top_k)

        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "title": result.get('title', ''),
                "url": result.get('url', ''),
                "content": result.get('content', ''),
                "relevance_score": result.get('score', 0.0)
            })
        return formatted_results
    except Exception as e:
        logger.error(f"Error in retrieve_context_tool: {e}")
        return []

# Configure OpenRouter with Gemini model
open_router = os.getenv('OPENROUTER_API_KEY')
if not open_router:
    logger.warning("OPENROUTER_API_KEY not found in environment variables")
    open_router = "sk-or-v1-dummy-key-for-testing"  # Use dummy key if not set

# Configure AsyncOpenAI for OpenRouter with required headers
import openai
client = AsyncOpenAI(
    api_key=open_router,
    base_url="https://openrouter.ai/api/v1",
    default_headers={
        "HTTP-Referer": os.getenv('DEPLOYED_BOOK_URL', 'http://localhost:3000'),
        "X-Title": "Humanoid Robotics Course Book Chatbot"
    }
)

model = OpenAIChatCompletionsModel(
    model = 'mistralai/devstral-2512:free',
    openai_client=client
)


def answer_query(query: str, top_k: int = 5, session_id: str = "default", selected_text: str = None):
    """Function to answer queries using the agent with OpenRouter Key"""
    try:
        # Check if vector store is initialized
        if vector_store is None:
            logger.warning("Vector store is not initialized in answer_query")
            # Try to return a response without RAG
            import openai
            client = openai.OpenAI(
                api_key=os.getenv('OPENROUTER_API_KEY') or "sk-or-v1-dummy-key-for-testing",
                base_url="https://openrouter.ai/api/v1",
                default_headers={
                    "HTTP-Referer": os.getenv('DEPLOYED_BOOK_URL', 'http://localhost:3000'),
                    "X-Title": "Humanoid Robotics Course Book Chatbot"
                }
            )

            # Prepare the prompt with selected text context if available
            if selected_text:
                full_prompt = f"User has selected this text: '{selected_text}'\n\nOriginal query: {query}\n\nPlease provide relevant information or answer based on the selected text."
            else:
                full_prompt = query

            response = client.chat.completions.create(
                model="mistralai/devstral-2512:free",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for the Humanoid Robotics Course Book. The knowledge base is currently unavailable, but please try to answer general questions based on the provided context."},
                    {"role": "user", "content": full_prompt}
                ],
                temperature=0.7,
                max_tokens=500
            )

            return response.choices[0].message.content

        # Prepare the query with selected text context if available
        if selected_text:
            enhanced_query = f"User has selected this text: '{selected_text}'\n\nOriginal query: {query}\n\nPlease provide relevant information or answer based on the selected text and use the retrieve_context tool to get additional relevant information."
        else:
            enhanced_query = query

        # Create a simple agent
        agent = Agent(
            name="RAG Assistant",
            instructions=f"""You are a helpful assistant for the Humanoid Robotics Course Book. Use the retrieve_context tool to get relevant information, then answer the user's question. Retrieve {top_k} documents for context. Pay special attention to any selected text provided by the user.""",
            tools=[retrieve_context_tool],
            model=model,
        )

        # Run the agent
        result = Runner.run_sync(agent, enhanced_query)
        response = result.final_output if hasattr(result, 'final_output') and result.final_output else str(result)

        return response
    except Exception as e:
        logger.error(f"Agent failed: {e}")
        return f"Sorry, I'm unable to process your request at the moment. Error: {str(e)}"