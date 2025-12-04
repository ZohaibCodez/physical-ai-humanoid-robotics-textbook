import { useState, useEffect, useCallback } from 'react';

export interface TextRange {
  startOffset: number;
  endOffset: number;
}

export interface ContextMetadata {
  chapter?: number;
  section?: string;
  filePath?: string;
  heading?: string;
}

export interface SelectedContext {
  text: string;
  range?: TextRange;
  metadata?: ContextMetadata;
}

export interface UseTextSelectionReturn {
  selectedText: string;
  selectedContext: SelectedContext | null;
  clearSelection: () => void;
}

/**
 * Hook to capture and manage user text selections in the document.
 * 
 * Listens for mouseup events and extracts:
 * - Selected text content
 * - Position range (start/end offsets)
 * - Metadata from parent elements (chapter, section, file path)
 * 
 * @returns {UseTextSelectionReturn} Selected text state and clear function
 */
export function useTextSelection(): UseTextSelectionReturn {
  const [selectedContext, setSelectedContext] = useState<SelectedContext | null>(null);
  const [selectedText, setSelectedText] = useState<string>('');

  const extractMetadata = useCallback((element: Element): ContextMetadata => {
    const metadata: ContextMetadata = {};

    // Traverse up the DOM to find metadata attributes
    let currentElement: Element | null = element;
    while (currentElement && currentElement !== document.body) {
      // Look for data attributes
      if (currentElement.hasAttribute('data-chapter')) {
        const chapter = parseInt(currentElement.getAttribute('data-chapter') || '0', 10);
        if (chapter > 0 && chapter <= 13) {
          metadata.chapter = chapter;
        }
      }
      
      if (currentElement.hasAttribute('data-section')) {
        metadata.section = currentElement.getAttribute('data-section') || undefined;
      }
      
      if (currentElement.hasAttribute('data-file-path')) {
        metadata.filePath = currentElement.getAttribute('data-file-path') || undefined;
      }

      // Try to extract from heading elements
      if (!metadata.heading && (currentElement.tagName === 'H1' || currentElement.tagName === 'H2' || currentElement.tagName === 'H3')) {
        metadata.heading = currentElement.textContent?.trim();
      }

      currentElement = currentElement.parentElement;
    }

    return metadata;
  }, []);

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    
    if (!selection || selection.isCollapsed || selection.rangeCount === 0) {
      // No selection or selection is collapsed
      return;
    }

    const text = selection.toString().trim();
    
    // Minimum 50 characters for valid selection
    if (text.length < 50) {
      return;
    }

    const range = selection.getRangeAt(0);
    const startOffset = range.startOffset;
    const endOffset = range.endOffset;

    // Extract metadata from the selection's parent element
    const container = range.commonAncestorContainer;
    const element = container.nodeType === Node.ELEMENT_NODE 
      ? container as Element
      : container.parentElement;

    const metadata = element ? extractMetadata(element) : {};

    const context: SelectedContext = {
      text,
      range: {
        startOffset,
        endOffset,
      },
      metadata: Object.keys(metadata).length > 0 ? metadata : undefined,
    };

    setSelectedText(text);
    setSelectedContext(context);
  }, [extractMetadata]);

  const clearSelection = useCallback(() => {
    setSelectedText('');
    setSelectedContext(null);
    
    // Clear browser selection
    const selection = window.getSelection();
    if (selection) {
      selection.removeAllRanges();
    }
  }, []);

  useEffect(() => {
    // Listen for mouseup events to capture selection
    document.addEventListener('mouseup', handleSelection);
    
    // Also listen for selection change events
    document.addEventListener('selectionchange', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('selectionchange', handleSelection);
    };
  }, [handleSelection]);

  return {
    selectedText,
    selectedContext,
    clearSelection,
  };
}
