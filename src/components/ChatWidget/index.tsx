import React, { useState } from 'react';
import ReactMarkdown from 'react-markdown';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator,
} from '@chatscope/chat-ui-kit-react';
import { useTextSelection } from '../../hooks/useTextSelection';
import ContextSelector from '../ContextSelector';
import CitationLink from '../CitationLink';
import './styles.css';

const ChatWidget = ({ apiUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isTyping, setIsTyping] = useState(false);
  const [sessionId] = useState(() => `sess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`);
  const [error, setError] = useState(null);
  const [contextMode, setContextMode] = useState('full');
  
  // Text selection hook for User Story 2
  const { selectedText, selectedContext, clearSelection } = useTextSelection();
  
  const API_URL = apiUrl || 'http://localhost:8000';

  const handleSendMessage = async (text) => {
    if (!text.trim()) return;

    // Add user message
    const userMessage = {
      role: 'user',
      content: text,
      timestamp: new Date().toISOString(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsTyping(true);
    setError(null);

    // Create placeholder for assistant message
    const assistantMessageId = Date.now();
    const assistantMessage = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      timestamp: new Date().toISOString(),
    };
    setMessages((prev) => [...prev, assistantMessage]);

    try {
      // Prepare request body with optional selected_context
      const requestBody = {
        session_id: sessionId,
        question_text: text,
        context_mode: contextMode,
        ...(contextMode === 'selected' && selectedContext ? {
          selected_context: selectedContext
        } : {})
      };

      const response = await fetch(`${API_URL}/v1/chat/ask/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Rate limit exceeded. Please wait a moment and try again.');
        }
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      // Process SSE stream
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      let streamedContent = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const eventData = JSON.parse(line.slice(6));
              
              if (eventData.type === 'delta') {
                // Append text delta to content (each delta is a token)
                streamedContent += eventData.data;
                setMessages((prev) =>
                  prev.map((msg) =>
                    msg.id === assistantMessageId
                      ? { ...msg, content: streamedContent }
                      : msg
                  )
                );
              } else if (eventData.type === 'done') {
                // Stream completed
                console.log('Streaming completed');
              } else if (eventData.type === 'error') {
                throw new Error(eventData.data);
              }
            } catch (parseError) {
              console.error('Error parsing SSE event:', parseError);
            }
          }
        }
      }

      setIsTyping(false);
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err instanceof Error ? err.message : 'Failed to send message');
      
      // Update assistant message with error
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                content: `Sorry, I encountered an error: ${err instanceof Error ? err.message : 'Unknown error'}. Please try again.`,
              }
            : msg
        )
      );
      setIsTyping(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className="chat-toggle-button"
        onClick={toggleChat}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-widget-container">
          <div className="chat-widget-header">
            <h3>Textbook Assistant</h3>
            <button onClick={toggleChat} className="chat-close-button">
              ✕
            </button>
          </div>
          
          {/* Context Selector for User Story 2 */}
          {selectedContext && (
            <ContextSelector
              selectedContext={selectedContext}
              contextMode={contextMode}
              onModeChange={setContextMode}
              onClear={() => {
                clearSelection();
                setContextMode('full');
              }}
            />
          )}

          <MainContainer>
            <ChatContainer>
              <MessageList
                scrollBehavior="smooth"
                typingIndicator={isTyping ? <TypingIndicator content="Assistant is thinking..." /> : null}
              >
                {messages.length === 0 && (
                  <div className="chat-welcome">
                    <div className="welcome-header">
                      <span className="welcome-icon">🤖</span>
                      <h4>Welcome! I'm here to help you learn Physical AI & Humanoid Robotics</h4>
                    </div>
                    
                    <div className="quick-actions">
                      <button 
                        className="quick-action-btn"
                        onClick={() => handleSendMessage("What are the key components of ROS2?")}
                      >
                        <span className="action-icon">🔧</span>
                        <span>What are the key components of ROS2?</span>
                      </button>
                      
                      <button 
                        className="quick-action-btn"
                        onClick={() => handleSendMessage("Explain forward kinematics")}
                      >
                        <span className="action-icon">📐</span>
                        <span>Explain forward kinematics</span>
                      </button>
                      
                      <button 
                        className="quick-action-btn"
                        onClick={() => handleSendMessage("How does sensor fusion work?")}
                      >
                        <span className="action-icon">📡</span>
                        <span>How does sensor fusion work?</span>
                      </button>
                      
                      <button 
                        className="quick-action-btn"
                        onClick={() => handleSendMessage("What is trajectory planning?")}
                      >
                        <span className="action-icon">📊</span>
                        <span>What is trajectory planning?</span>
                      </button>
                    </div>
                    
                    {selectedContext && (
                      <div className="context-notice">
                        📌 You have text selected. Switch to "Selected Text Only" mode to focus your questions!
                      </div>
                    )}
                  </div>
                )}
                
                {messages.map((msg, index) => (
                  <div key={index}>
                    {msg.role === 'user' ? (
                      <Message
                        model={{
                          message: msg.content,
                          sentTime: msg.timestamp || '',
                          sender: 'You',
                          direction: 'outgoing',
                          position: 'single',
                        }}
                      />
                    ) : (
                      <div className="assistant-message-wrapper">
                        <div className="assistant-message">
                          <ReactMarkdown>{msg.content}</ReactMarkdown>
                        </div>
                      </div>
                    )}
                  </div>
                ))}
              </MessageList>
              
              <MessageInput
                placeholder="Type your question here..."
                onSend={handleSendMessage}
                attachButton={false}
              />
            </ChatContainer>
          </MainContainer>

          {error && (
            <div className="chat-error">
              ⚠️ {error}
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default ChatWidget;
