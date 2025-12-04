import React from 'react';
import './ContextSelector.css';

export interface SelectedContext {
  text: string;
  metadata?: {
    chapter?: number;
    section?: string;
    heading?: string;
  };
}

export interface ContextSelectorProps {
  selectedContext: SelectedContext | null;
  contextMode: 'full' | 'selected';
  onModeChange: (mode: 'full' | 'selected') => void;
  onClear: () => void;
}

/**
 * Component to display selected text context and allow mode switching.
 * 
 * Shows:
 * - Selected text snippet with character count
 * - Toggle between full textbook and selected text modes
 * - Clear selection button
 * - Visual indicator of current mode
 */
const ContextSelector: React.FC<ContextSelectorProps> = ({
  selectedContext,
  contextMode,
  onModeChange,
  onClear,
}) => {
  if (!selectedContext) {
    return null;
  }

  const textLength = selectedContext.text.length;
  const snippet = textLength > 100 
    ? `${selectedContext.text.substring(0, 100)}...` 
    : selectedContext.text;

  const isValidSelection = textLength >= 50;

  return (
    <div className="context-selector">
      <div className="context-selector-header">
        <span className="context-selector-title">📌 Selected Context</span>
        <button
          className="context-clear-btn"
          onClick={onClear}
          aria-label="Clear selection"
        >
          ✕
        </button>
      </div>

      <div className="context-snippet">
        <p>{snippet}</p>
        <span className="context-length">
          {textLength} characters
          {!isValidSelection && ' (minimum 50 required)'}
        </span>
      </div>

      {selectedContext.metadata && (
        <div className="context-metadata">
          {selectedContext.metadata.chapter && (
            <span className="metadata-tag">Chapter {selectedContext.metadata.chapter}</span>
          )}
          {selectedContext.metadata.section && (
            <span className="metadata-tag">{selectedContext.metadata.section}</span>
          )}
        </div>
      )}

      <div className="context-mode-toggle">
        <button
          className={`mode-btn ${contextMode === 'full' ? 'active' : ''}`}
          onClick={() => onModeChange('full')}
          disabled={!isValidSelection}
        >
          📚 Full Textbook
        </button>
        <button
          className={`mode-btn ${contextMode === 'selected' ? 'active' : ''}`}
          onClick={() => onModeChange('selected')}
          disabled={!isValidSelection}
        >
          📄 Selected Text Only
        </button>
      </div>

      {!isValidSelection && (
        <div className="context-warning">
          ⚠️ Selection too short. Select at least 50 characters to enable "Selected Text Only" mode.
        </div>
      )}
    </div>
  );
};

export default ContextSelector;
