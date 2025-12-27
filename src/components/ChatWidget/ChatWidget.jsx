import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

const ChatWidget = ({
  backendUrl = 'https://sufyanalisyed-deploy-robotics-book.hf.space',
  initialOpen = false,
  position = 'bottom-right'
}) => {
  const [isOpen, setIsOpen] = useState(initialOpen);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isHistoryLoading, setIsHistoryLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const chatContainerRef = useRef(null);

  // Initialize session ID from localStorage or create a new one
  useEffect(() => {
    let storedSessionId = localStorage.getItem('chat_session_id');
    if (!storedSessionId) {
      storedSessionId = generateSessionId();
      localStorage.setItem('chat_session_id', storedSessionId);
    }
    setSessionId(storedSessionId);
  }, []);

  // Fetch chat history when component mounts and sessionId is available
  useEffect(() => {
    if (sessionId) {
      fetchChatHistory();
    }
  }, [sessionId]);

  const generateSessionId = () => {
    // Generate a proper UUID v4 format
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
  };

  // Function to get selected text from the page
  const getSelectedText = () => {
    return window.getSelection ? window.getSelection().toString().trim() : '';
  };

  // Function to handle sending selected text to the chat
  const sendSelectedText = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      // Add selected text as a user message
      const userMessage = {
        id: Date.now(),
        role: 'user',
        content: `Regarding this selected text: "${selectedText}". Please explain or answer questions about it.`,
        timestamp: new Date().toISOString()
      };

      // Add user message to the chat
      setMessages(prev => [...prev, userMessage]);

      // Send to backend
      sendTextToBackend(selectedText);
    } else {
      alert('Please select some text on the page first.');
    }
  };

  // Function to send text to backend - backend is deployed on hugging-face
  const sendTextToBackend = async (selectedText) => {
    setIsLoading(true);

    try {
      const response = await fetch(`${backendUrl}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: `Regarding this selected text: "${selectedText}". Please explain or answer questions about it.`,
          session_id: sessionId,
          selected_text: selectedText  // Include selected text as context
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.answer || data.response, // Handle both possible response formats
        timestamp: new Date().toISOString()
      };

      // Add assistant message to the chat
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending selected text:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your selected text. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const fetchChatHistory = async () => {
    setIsHistoryLoading(true);
    try {
      const response = await fetch(`${backendUrl}/chat/history/${sessionId}`);
      if (response.ok) {
        const data = await response.json();
        // Format the messages to match our internal structure
        const formattedMessages = data.messages.map(msg => ({
          id: msg.id,
          role: msg.role,
          content: msg.content,
          timestamp: msg.timestamp
        }));
        setMessages(formattedMessages);
      } else {
        console.error('Failed to fetch chat history:', response.status);
        // Continue without history if fetch fails, don't break the UI
      }
    } catch (error) {
      console.error('Error fetching chat history:', error);
      // Continue without history if fetch fails, don't break the UI
    } finally {
      setIsHistoryLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    // Add user message to the chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send message to backend - use the new chat API endpoint
      const response = await fetch(`${backendUrl}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          session_id: sessionId
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.answer || data.response, // Handle both possible response formats
        timestamp: new Date().toISOString()
      };

      // Add assistant message to the chat
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className={`chat-widget ${position}`}>
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <div className="chat-title">Humanoid Robot Book</div>
            <button className="chat-close" onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className="chat-messages">
            {isHistoryLoading && (
              <div className="message assistant">
                <div className="message-content">Loading chat history...</div>
              </div>
            )}
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
              >
                <div className="message-content">{message.content}</div>
                <div className="message-timestamp">
                  {new Date(message.timestamp).toLocaleTimeString()}
                </div>
              </div>
            ))}
            {isLoading && !isHistoryLoading && (
              <div className="message assistant">
                <div className="message-content">Thinking...</div>
              </div>
            )}
          </div>
          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              disabled={isLoading}
              rows="1"
            />
            <div className="input-buttons">
              <button
                className="selected-text-button"
                onClick={sendSelectedText}
                title="Send selected text to chat"
                type="button"
                disabled={isLoading}
              >
                Selected Text
              </button>
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim()}
                className="send-button"
              >
                Send
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button className="chat-toggle" onClick={toggleChat}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM9.5 16.5C8.5 16.5 7.5 16 6.7 15.2C5.9 14.4 5.5 13.4 5.5 12.5C5.5 11.5 5.9 10.5 6.7 9.7C7.5 8.9 8.5 8.5 9.5 8.5C11.5 8.5 13.5 10.5 13.5 12.5C13.5 14.5 11.5 16.5 9.5 16.5ZM17.5 16.5C16.5 16.5 15.5 16 14.7 15.2C13.9 14.4 13.5 13.4 13.5 12.5C13.5 11.5 13.9 10.5 14.7 9.7C15.5 8.9 16.5 8.5 17.5 8.5C19.5 8.5 21.5 10.5 21.5 12.5C21.5 14.5 19.5 16.5 17.5 16.5Z" fill="currentColor"/>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;