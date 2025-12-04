import React, { useState } from 'react';
import './CitationLink.css';

const CitationLink = ({ text, url, relevance_score, snippet }) => {
  const [showTooltip, setShowTooltip] = useState(false);

  const handleClick = (e) => {
    e.preventDefault();
    
    // Navigate to the URL (within Docusaurus)
    window.location.hash = '';
    window.location.href = url;
    
    // Smooth scroll to target if it's an anchor link
    if (url.includes('#')) {
      setTimeout(() => {
        const anchor = url.split('#')[1];
        const element = document.getElementById(anchor);
        if (element) {
          element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        }
      }, 100);
    }
  };

  return (
    <div
      className="citation-link-container"
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
    >
      <a
        href={url}
        onClick={handleClick}
        className="citation-link"
      >
        <span className="citation-icon">🔗</span>
        <span className="citation-text">{text}</span>
        <span className="citation-score">
          {Math.round(relevance_score * 100)}%
        </span>
      </a>
      
      {showTooltip && snippet && (
        <div className="citation-tooltip">
          <p className="citation-snippet">{snippet}</p>
          <p className="citation-hint">Click to view in textbook</p>
        </div>
      )}
    </div>
  );
};

export default CitationLink;
