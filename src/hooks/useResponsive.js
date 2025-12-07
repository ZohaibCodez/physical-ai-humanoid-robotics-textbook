import { useState, useEffect } from 'react';

/**
 * Breakpoints aligned with design tokens
 */
const BREAKPOINTS = {
  sm: 320,
  md: 768,
  lg: 1024,
  xl: 1440,
};

/**
 * useResponsive hook - Detect current breakpoint and screen size
 * @returns {Object} { isMobile, isTablet, isDesktop, isWide, width, breakpoint }
 */
export function useResponsive() {
  const [dimensions, setDimensions] = useState(() => {
    if (typeof window === 'undefined') {
      return {
        width: 1024,
        height: 768,
      };
    }
    return {
      width: window.innerWidth,
      height: window.innerHeight,
    };
  });

  useEffect(() => {
    if (typeof window === 'undefined') return;

    let timeoutId = null;

    const handleResize = () => {
      // Debounce resize events
      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => {
        setDimensions({
          width: window.innerWidth,
          height: window.innerHeight,
        });
      }, 150);
    };

    window.addEventListener('resize', handleResize);
    return () => {
      window.removeEventListener('resize', handleResize);
      clearTimeout(timeoutId);
    };
  }, []);

  const width = dimensions.width;
  const height = dimensions.height;

  // Determine breakpoint
  let breakpoint = 'sm';
  if (width >= BREAKPOINTS.xl) {
    breakpoint = 'xl';
  } else if (width >= BREAKPOINTS.lg) {
    breakpoint = 'lg';
  } else if (width >= BREAKPOINTS.md) {
    breakpoint = 'md';
  }

  return {
    // Convenience booleans
    isMobile: width < BREAKPOINTS.md,
    isTablet: width >= BREAKPOINTS.md && width < BREAKPOINTS.lg,
    isDesktop: width >= BREAKPOINTS.lg && width < BREAKPOINTS.xl,
    isWide: width >= BREAKPOINTS.xl,
    
    // Raw dimensions
    width,
    height,
    
    // Current breakpoint
    breakpoint,
    
    // Breakpoint values for reference
    breakpoints: BREAKPOINTS,
  };
}
