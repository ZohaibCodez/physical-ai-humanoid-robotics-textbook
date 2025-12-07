# Quickstart Guide: UI/UX Redesign Implementation

**Feature**: 005-ui-ux-redesign  
**Date**: 2025-12-07  
**Audience**: Developers implementing the comprehensive UI/UX redesign

---

## Prerequisites

✅ Node.js 18+ installed  
✅ Docusaurus 3.x project set up  
✅ Git repository initialized  
✅ Better-auth authentication implemented (Feature 003)  
✅ RAG chatbot backend running (Feature 002)  

---

## Phase 1: Setup Design System Foundation (P1)

### Step 1: Create Design Token File

Create `src/css/design-tokens.css`:

```css
/**
 * Design Tokens - Physical AI Textbook
 * Single source of truth for all design values
 */

:root {
  /* === Colors: Primary (Blue) === */
  --color-primary-50: #eff6ff;
  --color-primary-100: #dbeafe;
  --color-primary-200: #bfdbfe;
  --color-primary-300: #93c5fd;
  --color-primary-400: #60a5fa;
  --color-primary-500: #3b82f6;
  --color-primary-600: #2563eb;
  --color-primary-700: #1d4ed8;
  --color-primary-800: #1e40af;
  --color-primary-900: #1e3a8a;
  
  /* === Colors: Secondary (Purple) === */
  --color-secondary-50: #faf5ff;
  --color-secondary-100: #f3e8ff;
  --color-secondary-200: #e9d5ff;
  --color-secondary-300: #d8b4fe;
  --color-secondary-400: #c084fc;
  --color-secondary-500: #a855f7;
  --color-secondary-600: #9333ea;
  --color-secondary-700: #7e22ce;
  --color-secondary-800: #6b21a8;
  --color-secondary-900: #581c87;
  
  /* === Colors: Accent (Green) === */
  --color-accent-50: #ecfdf5;
  --color-accent-100: #d1fae5;
  --color-accent-200: #a7f3d0;
  --color-accent-300: #6ee7b7;
  --color-accent-400: #34d399;
  --color-accent-500: #10b981;
  --color-accent-600: #059669;
  --color-accent-700: #047857;
  --color-accent-800: #065f46;
  --color-accent-900: #064e3b;
  
  /* === Colors: Neutral (Gray) === */
  --color-neutral-50: #f9fafb;
  --color-neutral-100: #f3f4f6;
  --color-neutral-200: #e5e7eb;
  --color-neutral-300: #d1d5db;
  --color-neutral-400: #9ca3af;
  --color-neutral-500: #6b7280;
  --color-neutral-600: #4b5563;
  --color-neutral-700: #374151;
  --color-neutral-800: #1f2937;
  --color-neutral-900: #111827;
  
  /* === Semantic Colors === */
  --color-success: #10b981;
  --color-warning: #f59e0b;
  --color-error: #ef4444;
  --color-info: #3b82f6;
  
  /* === Typography === */
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
  --font-mono: "SF Mono", Monaco, Inconsolata, "Fira Code", Consolas, monospace;
  
  --text-xs: 0.75rem;
  --text-sm: 0.875rem;
  --text-base: 1rem;
  --text-lg: 1.125rem;
  --text-xl: 1.25rem;
  --text-2xl: 1.5rem;
  --text-3xl: 1.875rem;
  --text-4xl: 2.25rem;
  --text-5xl: 3rem;
  
  --font-normal: 400;
  --font-medium: 500;
  --font-semibold: 600;
  --font-bold: 700;
  
  --leading-tight: 1.2;
  --leading-normal: 1.5;
  --leading-relaxed: 1.6;
  --leading-loose: 1.8;
  
  /* === Spacing (8px grid) === */
  --spacing-xs: 0.25rem;
  --spacing-sm: 0.5rem;
  --spacing-md: 1rem;
  --spacing-lg: 1.5rem;
  --spacing-xl: 2rem;
  --spacing-2xl: 3rem;
  --spacing-3xl: 4rem;
  --spacing-4xl: 6rem;
  
  /* === Shadows === */
  --shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
  --shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
  --shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05);
  --shadow-xl: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04);
  --shadow-2xl: 0 25px 50px -12px rgba(0, 0, 0, 0.25);
  
  /* === Borders === */
  --radius-sm: 0.25rem;
  --radius-md: 0.375rem;
  --radius-lg: 0.5rem;
  --radius-xl: 0.75rem;
  --radius-2xl: 1rem;
  --radius-full: 9999px;
  
  /* === Transitions === */
  --transition-fast: 150ms;
  --transition-normal: 300ms;
  --transition-slow: 500ms;
  --ease-default: cubic-bezier(0.4, 0, 0.2, 1);
  
  /* === Z-Index Layers === */
  --z-dropdown: 1000;
  --z-sticky: 1020;
  --z-fixed: 1030;
  --z-modal-backdrop: 1040;
  --z-modal: 1050;
  --z-popover: 1060;
  --z-tooltip: 1070;
}

/* Dark mode overrides */
[data-theme='dark'] {
  --color-primary-50: #1e3a8a;
  --color-primary-900: #eff6ff;
  
  /* Invert neutral scale for dark backgrounds */
  --color-neutral-50: #111827;
  --color-neutral-100: #1f2937;
  --color-neutral-200: #374151;
  --color-neutral-300: #4b5563;
  --color-neutral-400: #6b7280;
  --color-neutral-500: #9ca3af;
  --color-neutral-600: #d1d5db;
  --color-neutral-700: #e5e7eb;
  --color-neutral-800: #f3f4f6;
  --color-neutral-900: #f9fafb;
  
  --shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.3);
  --shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.4), 0 2px 4px -1px rgba(0, 0, 0, 0.3);
  --shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.5), 0 4px 6px -2px rgba(0, 0, 0, 0.4);
}

/* Accessibility: Reduced motion */
@media (prefers-reduced-motion: reduce) {
  :root {
    --transition-fast: 0ms;
    --transition-normal: 0ms;
    --transition-slow: 0ms;
  }
}
```

### Step 2: Update Docusaurus Custom CSS

Edit `src/css/custom.css`:

```css
/* Import design tokens first */
@import './design-tokens.css';

/* Override Docusaurus defaults with design system */
:root {
  /* Use our primary color as Docusaurus primary */
  --ifm-color-primary: var(--color-primary-600);
  --ifm-color-primary-dark: var(--color-primary-700);
  --ifm-color-primary-darker: var(--color-primary-800);
  --ifm-color-primary-darkest: var(--color-primary-900);
  --ifm-color-primary-light: var(--color-primary-500);
  --ifm-color-primary-lighter: var(--color-primary-400);
  --ifm-color-primary-lightest: var(--color-primary-300);
  
  /* Typography */
  --ifm-font-family-base: var(--font-sans);
  --ifm-font-family-monospace: var(--font-mono);
  --ifm-font-size-base: var(--text-base);
  --ifm-line-height-base: var(--leading-relaxed);
  
  /* Spacing */
  --ifm-spacing-horizontal: var(--spacing-lg);
  --ifm-spacing-vertical: var(--spacing-md);
  
  /* Borders */
  --ifm-global-radius: var(--radius-md);
}

[data-theme='dark'] {
  --ifm-background-color: var(--color-neutral-50);
  --ifm-color-primary: var(--color-primary-400);
}

/* Global resets for consistency */
* {
  box-sizing: border-box;
}

body {
  font-family: var(--font-sans);
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}
```

### Step 3: Configure Dark Mode in Docusaurus

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    // ... rest of config
  },
};
```

**✅ Checkpoint**: Run `npm start` and verify:
- Design tokens are loaded
- Dark mode toggle works in navbar
- Colors are consistent across pages

---

## Phase 2: Implement Core Components (P1)

### Step 4: Create LoadingButton Component

Create `src/components/LoadingButton.jsx`:

```jsx
import React, { useState } from 'react';
import styles from './LoadingButton.module.css';

export default function LoadingButton({
  children,
  loading = false,
  disabled = false,
  variant = 'primary',
  size = 'md',
  icon,
  onClick,
  type = 'button',
  className = '',
  fullWidth = false,
  'aria-label': ariaLabel,
}) {
  const [internalLoading, setInternalLoading] = useState(false);
  
  const isLoading = loading || internalLoading;
  const isDisabled = disabled || isLoading;
  
  const handleClick = async (e) => {
    if (!onClick || isDisabled) return;
    
    const result = onClick(e);
    
    // If onClick returns a Promise, set internal loading state
    if (result instanceof Promise) {
      setInternalLoading(true);
      try {
        await result;
      } finally {
        setInternalLoading(false);
      }
    }
  };
  
  const classNames = [
    styles.button,
    styles[variant],
    styles[size],
    isLoading && styles.loading,
    isDisabled && styles.disabled,
    fullWidth && styles.fullWidth,
    className,
  ].filter(Boolean).join(' ');
  
  return (
    <button
      type={type}
      className={classNames}
      onClick={handleClick}
      disabled={isDisabled}
      aria-busy={isLoading}
      aria-disabled={isDisabled}
      aria-label={ariaLabel}
    >
      {isLoading ? (
        <span className={styles.spinner} />
      ) : icon ? (
        <span className={styles.icon}>{icon}</span>
      ) : null}
      <span className={styles.text}>{children}</span>
    </button>
  );
}
```

Create `src/components/LoadingButton.module.css`:

```css
.button {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: var(--spacing-sm);
  font-family: var(--font-sans);
  font-weight: var(--font-medium);
  border: none;
  border-radius: var(--radius-md);
  cursor: pointer;
  transition: all var(--transition-fast) var(--ease-default);
}

.button:focus-visible {
  outline: 2px solid var(--color-primary-600);
  outline-offset: 2px;
}

/* Variants */
.primary {
  background: var(--color-primary-600);
  color: white;
}

.primary:hover:not(.disabled) {
  background: var(--color-primary-700);
}

/* Sizes */
.sm {
  padding: var(--spacing-sm) var(--spacing-md);
  font-size: var(--text-sm);
}

.md {
  padding: var(--spacing-md) var(--spacing-lg);
  font-size: var(--text-base);
}

.lg {
  padding: var(--spacing-lg) var(--spacing-xl);
  font-size: var(--text-lg);
}

/* States */
.loading, .disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.fullWidth {
  width: 100%;
}

/* Spinner animation */
.spinner {
  width: 1em;
  height: 1em;
  border: 2px solid currentColor;
  border-right-color: transparent;
  border-radius: var(--radius-full);
  animation: spin 0.6s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}
```

**✅ Checkpoint**: Test LoadingButton:
```jsx
<LoadingButton variant="primary" loading={true}>
  Save Changes
</LoadingButton>
```

### Step 5: Create Profile Dropdown (see full implementation in contracts)

Create `src/components/ProfileDropdown.jsx` and `ProfileDropdown.module.css` following the contract specification.

**✅ Checkpoint**: Verify dropdown shows user info and closes on outside click.

---

## Phase 3: Mobile Responsive Layout (P2)

### Step 6: Add Responsive Meta Tag

Verify in `static/index.html` or `docusaurus.config.js`:

```html
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
```

### Step 7: Update Landing Page for Mobile

Edit `src/pages/index.module.css`:

```css
/* Mobile-first approach */
.hero {
  padding: var(--spacing-2xl) var(--spacing-lg);
  text-align: center;
}

.heroTitle {
  font-size: var(--text-3xl);
  line-height: var(--leading-tight);
  margin-bottom: var(--spacing-lg);
}

/* Tablet */
@media (min-width: 768px) {
  .hero {
    padding: var(--spacing-3xl) var(--spacing-2xl);
  }
  
  .heroTitle {
    font-size: var(--text-4xl);
  }
}

/* Desktop */
@media (min-width: 1024px) {
  .hero {
    padding: var(--spacing-4xl) var(--spacing-3xl);
  }
  
  .heroTitle {
    font-size: var(--text-5xl);
  }
}

/* Feature cards: mobile-first grid */
.featureGrid {
  display: grid;
  grid-template-columns: 1fr;
  gap: var(--spacing-lg);
}

@media (min-width: 768px) {
  .featureGrid {
    grid-template-columns: repeat(2, 1fr);
  }
}

@media (min-width: 1024px) {
  .featureGrid {
    grid-template-columns: repeat(3, 1fr);
  }
}
```

**✅ Checkpoint**: Test on mobile devices (320px, 375px, 768px widths).

---

## Phase 4: Content Cleanup (P3)

### Step 8: Remove Week Folders

```bash
# Backup first
git checkout -b backup-before-cleanup

# Remove week folders
rm -rf docs/week-01-02
rm -rf docs/week-03-05
rm -rf docs/week-06-07
rm -rf docs/week-08-10
rm -rf docs/week-11-12
rm -rf docs/week-13
```

### Step 9: Update Sidebar Configuration

Edit `sidebars.js`:

```javascript
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part 1: Foundations',
      link: { type: 'doc', id: 'part-01-foundations/index' },
      items: [
        'part-01-foundations/chapter-01-introduction-to-physical-ai/index',
        // ... more chapters
      ],
    },
    // Remove any week-XX references
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/hardware-guide',
        'appendices/resources',
      ],
    },
  ],
};
```

### Step 10: Add Redirects

Install redirect plugin:

```bash
npm install @docusaurus/plugin-client-redirects
```

Update `docusaurus.config.js`:

```javascript
plugins: [
  [
    '@docusaurus/plugin-client-redirects',
    {
      redirects: [
        {
          from: '/docs/week-01-02',
          to: '/docs/part-01-foundations',
        },
        {
          from: '/docs/week-03-05',
          to: '/docs/part-02-ros2-ecosystem',
        },
        // ... more redirects
      ],
    },
  ],
],
```

**✅ Checkpoint**: Run broken link checker:
```bash
npx broken-link-checker http://localhost:3000
```

---

## Phase 5: Chatbot Integration (P3)

### Step 11: Create Chatbot FAB

Create `src/components/ChatbotFAB.jsx` (see contracts for full spec).

**✅ Checkpoint**: FAB appears in bottom-right, pulse animation on first visit.

---

## Testing Checklist

### Accessibility
- [ ] Run Lighthouse accessibility audit (score ≥ 95)
- [ ] Test with screen reader (NVDA/JAWS)
- [ ] Verify keyboard navigation (Tab, Enter, Escape)
- [ ] Check color contrast with WebAIM tool
- [ ] Test with `prefers-reduced-motion` enabled

### Responsive
- [ ] Test on iPhone SE (375px)
- [ ] Test on iPad (768px)
- [ ] Test on Desktop (1440px)
- [ ] Verify no horizontal scroll
- [ ] Check touch target sizes (≥ 44px)

### Cross-Browser
- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)

### Performance
- [ ] Lighthouse performance score ≥ 80
- [ ] First Contentful Paint < 1.5s
- [ ] No layout shift (CLS < 0.1)

---

## Deployment

### Build for Production

```bash
npm run build
```

### Deploy to GitHub Pages

```bash
npm run deploy
```

Or configure GitHub Actions (see `.github/workflows/deploy.yml`).

---

## Common Issues & Solutions

**Issue**: Dark mode doesn't persist  
**Solution**: Check localStorage is enabled and not blocked by browser

**Issue**: Components don't update on theme change  
**Solution**: Use `useColorMode()` hook from `@docusaurus/theme-common`

**Issue**: Mobile layout breaks  
**Solution**: Verify viewport meta tag and test in actual devices, not just DevTools

**Issue**: Profile dropdown conflicts with navbar  
**Solution**: Check z-index values, ensure dropdown has `z-index: var(--z-dropdown)`

---

**Quickstart Complete**: Follow these steps sequentially to implement the UI/UX redesign. Each phase builds on the previous one.
