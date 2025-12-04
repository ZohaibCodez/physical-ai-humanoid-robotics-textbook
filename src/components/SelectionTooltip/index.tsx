import React, { useState, useEffect, useRef } from 'react';
import './SelectionTooltip.css';

interface SelectionTooltipProps {
  selectedText: string;
  onAsk: (question: string) => void;
  onClose: () => void;
}

const SelectionTooltip: React.FC<SelectionTooltipProps> = ({
  selectedText,
  onAsk,
  onClose,
}) => {
  const [showInput, setShowInput] = useState(false);
  const [question, setQuestion] = useState('');
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const tooltipRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    // Get selection position
    const selection = window.getSelection();
    if (selection && selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      
      // Position tooltip above selection
      setPosition({
        top: rect.top + window.scrollY - 60,
        left: rect.left + window.scrollX + (rect.width / 2),
      });
    }
  }, [selectedText]);

  useEffect(() => {
    if (showInput && inputRef.current) {
      inputRef.current.focus();
    }
  }, [showInput]);

  const handleAskClick = () => {
    setShowInput(true);
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (question.trim()) {
      onAsk(question.trim());
      setQuestion('');
      setShowInput(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Escape') {
      onClose();
    }
  };

  return (
    <div
      ref={tooltipRef}
      className="selection-tooltip"
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
      }}
      onKeyDown={handleKeyDown}
    >
      <div className="tooltip-content">
        {!showInput ? (
          <div className="tooltip-buttons">
            <button
              className="tooltip-btn ask-btn"
              onClick={handleAskClick}
              title="Ask a question about this text"
            >
              <span className="btn-icon">💬</span>
              Ask Question
            </button>
            <button
              className="tooltip-btn close-btn"
              onClick={onClose}
              title="Clear selection"
            >
              ✕
            </button>
          </div>
        ) : (
          <form className="tooltip-input-form" onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              value={question}
              onChange={(e) => setQuestion(e.target.value)}
              placeholder="Ask about this text..."
              className="tooltip-input"
              maxLength={200}
            />
            <div className="tooltip-input-actions">
              <button
                type="submit"
                className="tooltip-submit-btn"
                disabled={!question.trim()}
                title="Send question"
              >
                Send
              </button>
              <button
                type="button"
                className="tooltip-cancel-btn"
                onClick={() => {
                  setShowInput(false);
                  setQuestion('');
                }}
                title="Cancel"
              >
                Cancel
              </button>
            </div>
          </form>
        )}
      </div>
      
      <div className="tooltip-arrow" />
    </div>
  );
};

export default SelectionTooltip;
