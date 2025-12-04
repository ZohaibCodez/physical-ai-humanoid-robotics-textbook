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
      
      // Better error messages
      let errorMessage = 'Failed to send message';
      if (err instanceof Error) {
        if (err.message.includes('422')) {
          errorMessage = 'Your selection is too short. Please select at least 20 characters (about 4-5 words) to use selected-text mode.';
        } else if (err.message.includes('429')) {
          errorMessage = 'Rate limit exceeded. Please wait a moment and try again.';
        } else if (err.message.includes('Failed to fetch')) {
          errorMessage = 'Cannot connect to server. Please check if the backend is running.';
        } else {
          errorMessage = err.message;
        }
      }
      
      setError(errorMessage);
      
      // Update assistant message with user-friendly error
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                content: `⚠️ ${errorMessage}\n\nTip: Make sure your selection is at least 20 characters if using selected-text mode.`,
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
      {/* Chat Toggle Button with selection badge */}
      <button
        className="chat-toggle-button"
        onClick={toggleChat}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
        {selectedContext && !isOpen && (
          <span className="selection-badge" title="Text selected">
            📌
          </span>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-widget-container">
          <div className="chat-widget-header">
            <div className="header-left">
              <h3>Textbook Assistant</h3>
            </div>
            <button onClick={toggleChat} className="chat-close-button">
              ✕
            </button>
          </div>
          
          {/* Integrated Selection Controls */}
          {selectedContext && (
            <div className="selection-controls">
              <div className="selection-info">
                <span className="selection-icon">📌</span>
                <div className="selection-text">
                  <strong>Text Selected</strong>
                  <span className="selection-length">{selectedContext.text.length} characters</span>
                </div>
              </div>
              
              <div className="mode-toggle">
                <button
                  className={`mode-btn ${contextMode === 'full' ? 'active' : ''}`}
                  onClick={() => setContextMode('full')}
                  title="Search entire textbook"
                >
                  <span className="mode-icon">📚</span>
                  Full Textbook
                </button>
                <button
                  className={`mode-btn ${contextMode === 'selected' ? 'active' : ''}`}
                  onClick={() => setContextMode('selected')}
                  title="Search only selected text"
                >
                  <span className="mode-icon">🎯</span>
                  Selected Text
                </button>
              </div>
              
              <button
                className="clear-selection-btn"
                onClick={() => {
                  clearSelection();
                  setContextMode('full');
                }}
                title="Clear selection"
              >
                ✕
              </button>
            </div>
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
                        💡 You've selected text! Use the controls above to switch between <strong>Full Textbook</strong> or <strong>Selected Text</strong> mode.
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
