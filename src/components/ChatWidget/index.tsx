import React, { useState } from 'react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator,
} from '@chatscope/chat-ui-kit-react';
import CitationLink from '../CitationLink';
import './styles.css';

const ChatWidget = ({ apiUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isTyping, setIsTyping] = useState(false);
  const [sessionId] = useState(() => `sess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`);
  const [error, setError] = useState(null);
  
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
      citations: [],
    };
    setMessages((prev) => [...prev, assistantMessage]);

    try {
      const response = await fetch(`${API_URL}/v1/chat/ask/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          session_id: sessionId,
          question_text: text,
          context_mode: 'full',
        }),
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
      let citations = [];

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const eventData = JSON.parse(line.slice(6));
              
              if (eventData.type === 'delta') {
                // Append text delta to content
                streamedContent += eventData.data;
                setMessages((prev) =>
                  prev.map((msg) =>
                    msg.id === assistantMessageId
                      ? { ...msg, content: streamedContent }
                      : msg
                  )
                );
              } else if (eventData.type === 'citations') {
                // Store citations
                citations = eventData.data;
              } else if (eventData.type === 'done') {
                // Update final message with citations
                setMessages((prev) =>
                  prev.map((msg) =>
                    msg.id === assistantMessageId
                      ? { ...msg, citations }
                      : msg
                  )
                );
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
          
          <MainContainer>
            <ChatContainer>
              <MessageList
                scrollBehavior="smooth"
                typingIndicator={isTyping ? <TypingIndicator content="Assistant is thinking..." /> : null}
              >
                {messages.length === 0 && (
                  <div className="chat-welcome">
                    <p>👋 Hi! I'm your textbook assistant.</p>
                    <p>Ask me anything about Physical AI and Humanoid Robotics!</p>
                  </div>
                )}
                
                {messages.map((msg, index) => (
                  <div key={index}>
                    <Message
                      model={{
                        message: msg.content,
                        sentTime: msg.timestamp || '',
                        sender: msg.role === 'user' ? 'You' : 'Assistant',
                        direction: msg.role === 'user' ? 'outgoing' : 'incoming',
                        position: 'single',
                      }}
                    />
                    
                    {/* Display citations for assistant messages */}
                    {msg.role === 'assistant' && msg.citations && msg.citations.length > 0 && (
                      <div className="chat-citations">
                        <p className="citations-label">📚 Sources:</p>
                        {msg.citations.map((citation, idx) => (
                          <CitationLink
                            key={idx}
                            text={citation.text}
                            url={citation.url}
                            relevance_score={citation.relevance_score}
                            snippet={citation.snippet}
                          />
                        ))}
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
