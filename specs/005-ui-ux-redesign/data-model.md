# Data Model: UI/UX Redesign

**Feature**: 005-ui-ux-redesign  
**Date**: 2025-12-07  
**Purpose**: Define data structures for UI state, user preferences, and component interfaces

---

## 1. Design Tokens (CSS Custom Properties)

### Color Palette

```typescript
interface ColorPalette {
  primary: ColorShades;
  secondary: ColorShades;
  accent: ColorShades;
  neutral: ColorShades;
  semantic: {
    success: ColorShades;
    warning: ColorShades;
    error: ColorShades;
    info: ColorShades;
  };
}

interface ColorShades {
  50: string;   // Lightest
  100: string;
  200: string;
  300: string;
  400: string;
  500: string;
  600: string;  // Base
  700: string;
  800: string;
  900: string;  // Darkest
}
```

**CSS Variables**:
```css
--color-primary-50: #eff6ff;
--color-primary-600: #2563eb;  /* Base */
--color-primary-900: #1e3a8a;

--color-secondary-600: #7c3aed;
--color-accent-600: #10b981;
--color-neutral-600: #6b7280;
```

### Typography

```typescript
interface TypographySystem {
  fontFamilies: {
    sans: string;
    mono: string;
  };
  scale: {
    xs: string;    // 0.75rem (12px)
    sm: string;    // 0.875rem (14px)
    base: string;  // 1rem (16px)
    lg: string;    // 1.125rem (18px)
    xl: string;    // 1.25rem (20px)
    '2xl': string; // 1.5rem (24px)
    '3xl': string; // 1.875rem (30px)
    '4xl': string; // 2.25rem (36px)
    '5xl': string; // 3rem (48px)
  };
  weights: {
    normal: number;    // 400
    medium: number;    // 500
    semibold: number;  // 600
    bold: number;      // 700
  };
  lineHeights: {
    tight: number;   // 1.2
    normal: number;  // 1.5
    relaxed: number; // 1.6
    loose: number;   // 1.8
  };
}
```

**CSS Variables**:
```css
--font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
--font-mono: "SF Mono", Monaco, Consolas, monospace;

--text-xs: 0.75rem;
--text-base: 1rem;
--text-5xl: 3rem;

--font-normal: 400;
--font-bold: 700;

--leading-tight: 1.2;
--leading-relaxed: 1.6;
```

### Spacing

```typescript
interface SpacingSystem {
  xs: string;   // 0.25rem (4px)
  sm: string;   // 0.5rem (8px)
  md: string;   // 1rem (16px)
  lg: string;   // 1.5rem (24px)
  xl: string;   // 2rem (32px)
  '2xl': string; // 3rem (48px)
  '3xl': string; // 4rem (64px)
  '4xl': string; // 6rem (96px)
}
```

**CSS Variables**:
```css
--spacing-xs: 0.25rem;
--spacing-sm: 0.5rem;
--spacing-md: 1rem;
--spacing-lg: 1.5rem;
--spacing-xl: 2rem;
--spacing-2xl: 3rem;
--spacing-3xl: 4rem;
```

### Shadows

```typescript
interface ShadowSystem {
  sm: string;
  md: string;
  lg: string;
  xl: string;
  '2xl': string;
  inner: string;
}
```

**CSS Variables**:
```css
--shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
--shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
--shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
--shadow-xl: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
```

### Breakpoints

```typescript
interface BreakpointSystem {
  mobile: string;   // 320px
  tablet: string;   // 768px
  desktop: string;  // 1024px
  wide: string;     // 1440px
}
```

**CSS Media Queries**:
```css
@media (min-width: 320px) { /* mobile */ }
@media (min-width: 768px) { /* tablet */ }
@media (min-width: 1024px) { /* desktop */ }
@media (min-width: 1440px) { /* wide */ }
```

---

## 2. Theme Configuration

### Theme Mode

```typescript
type ThemeMode = 'light' | 'dark';

interface ThemeConfig {
  mode: ThemeMode;
  respectSystemPreference: boolean;
  persistPreference: boolean;
}
```

**localStorage Key**: `theme`

**State Management**:
```typescript
// Managed by Docusaurus colorMode API
// Access via: useColorMode() hook from @docusaurus/theme-common
```

---

## 3. User Preferences (localStorage)

### UserPreferences Model

```typescript
interface UserPreferences {
  // Theme
  theme: ThemeMode;
  
  // Sidebar
  sidebarCollapsed: boolean;
  
  // Chatbot
  chatbotSeen: boolean;        // Has user seen chatbot before?
  chatbotMinimized: boolean;   // Is chatbot currently minimized?
  
  // Accessibility
  reducedMotion: boolean;      // Override for prefers-reduced-motion
  highContrast: boolean;       // Enhanced contrast mode
  
  // Language
  preferredLanguage: 'en' | 'ur' | 'both';
  
  // Timestamps
  lastUpdated: string;         // ISO 8601 date
}
```

**localStorage Keys**:
```typescript
const STORAGE_KEYS = {
  THEME: 'theme',                           // Managed by Docusaurus
  SIDEBAR: 'docusaurus.sidebar.collapsed',  // Custom
  CHATBOT_SEEN: 'chatbot.seen',             // Custom
  CHATBOT_MINIMIZED: 'chatbot.minimized',   // Custom
  REDUCED_MOTION: 'a11y.reducedMotion',     // Custom
  HIGH_CONTRAST: 'a11y.highContrast',       // Custom
  LANGUAGE: 'user.language',                // Custom
} as const;
```

**Default Values**:
```typescript
const DEFAULT_PREFERENCES: UserPreferences = {
  theme: 'light',                // Default to light, or use system preference
  sidebarCollapsed: false,       // Desktop: expanded by default
  chatbotSeen: false,            // Show pulse animation
  chatbotMinimized: true,        // Start minimized
  reducedMotion: false,          // Respect system preference by default
  highContrast: false,           // Standard contrast by default
  preferredLanguage: 'en',       // English default
  lastUpdated: new Date().toISOString(),
};
```

**Validation Rules**:
- `theme`: Must be 'light' or 'dark'
- `sidebarCollapsed`: Boolean
- `chatbotSeen`: Boolean, set to true after first interaction
- `preferredLanguage`: Must be 'en', 'ur', or 'both'
- `lastUpdated`: Valid ISO 8601 string

---

## 4. User Profile (Backend API)

### UserProfile Model

```typescript
interface UserProfile {
  // Identity
  id: string;                   // UUID
  email: string;                // Required, unique
  name: string;                 // Display name
  
  // Status
  isActive: boolean;
  isVerified: boolean;
  
  // Background Profiling (from Feature 003)
  softwareLevel: 'beginner' | 'intermediate' | 'advanced';
  hardwareAccess: 'none' | 'basic' | 'full_lab';
  preferredLanguage: 'en' | 'ur' | 'both';
  
  // Timestamps
  createdAt: string;            // ISO 8601
  lastLogin: string | null;     // ISO 8601 or null
  updatedAt: string;            // ISO 8601
}
```

**API Endpoint**: `GET /v1/user/profile`

**Validation Rules**:
- `email`: Valid email format, max 255 chars
- `name`: Min 2 chars, max 100 chars
- `softwareLevel`: Enum validation
- `hardwareAccess`: Enum validation
- `preferredLanguage`: Enum validation

**State Transitions**:
```
New User:
  isActive: true, isVerified: false
  
After Email Verification:
  isVerified: true
  
After Deactivation:
  isActive: false
```

---

## 5. Component Props Interfaces

### ProfileDropdown Component

```typescript
interface ProfileDropdownProps {
  user: UserProfile | null;
  onLogout: () => Promise<void>;
  onSettingsClick: () => void;
  onLanguageChange: (lang: 'en' | 'ur' | 'both') => void;
}

interface ProfileDropdownState {
  isOpen: boolean;
  isLoggingOut: boolean;
}
```

### Sidebar Component

```typescript
interface SidebarProps {
  items: SidebarItem[];
  collapsed: boolean;
  onToggle: (collapsed: boolean) => void;
  autoCollapseOnMobile: boolean;
}

interface SidebarItem {
  type: 'doc' | 'category' | 'link';
  label: string;
  href?: string;           // For 'link' type
  docId?: string;          // For 'doc' type
  items?: SidebarItem[];   // For 'category' type (recursive)
  collapsed?: boolean;     // For 'category' type
}

interface SidebarState {
  collapsed: boolean;
  activeDocId: string | null;
}
```

### ChatbotFAB Component

```typescript
interface ChatbotFABProps {
  onOpen: () => void;
  hasUnreadMessages: boolean;
  seen: boolean;               // Controls pulse animation
  position?: 'bottom-right' | 'bottom-left';
}

interface ChatbotPanelProps {
  isOpen: boolean;
  onClose: () => void;
  onMinimize: () => void;
  user: UserProfile | null;    // For personalized responses
}

interface ChatbotState {
  isOpen: boolean;
  minimized: boolean;
  messages: ChatMessage[];
  isTyping: boolean;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: string;           // ISO 8601
}

interface Citation {
  text: string;
  url: string;
  relevanceScore: number;
}
```

### LoadingButton Component

```typescript
interface LoadingButtonProps {
  loading: boolean;
  disabled?: boolean;
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost';
  size?: 'sm' | 'md' | 'lg';
  children: React.ReactNode;
  onClick?: () => void | Promise<void>;
  type?: 'button' | 'submit' | 'reset';
  icon?: React.ReactNode;
  'aria-label'?: string;
}

interface LoadingButtonState {
  internalLoading: boolean;    // For async onClick handlers
}
```

### SkeletonLoader Component

```typescript
interface SkeletonLoaderProps {
  variant: 'text' | 'circular' | 'rectangular' | 'card';
  width?: string | number;
  height?: string | number;
  animation?: 'pulse' | 'wave' | 'none';
  count?: number;              // Render multiple skeletons
}
```

### FeatureCard Component

```typescript
interface FeatureCardProps {
  icon: React.ReactNode;
  title: string;
  description: string;
  href?: string;
  badge?: {
    text: string;
    color: 'primary' | 'secondary' | 'accent';
  };
  variant?: 'default' | 'elevated' | 'outlined';
}
```

---

## 6. Navigation State

### Navbar State

```typescript
interface NavbarState {
  isSticky: boolean;           // Is navbar stuck to top?
  isMobileMenuOpen: boolean;   // Mobile hamburger menu state
  activeItem: string | null;   // Currently active nav item
}
```

### Sidebar Navigation State

```typescript
interface SidebarNavigationState {
  currentPath: string;         // Current doc path
  openCategories: string[];    // IDs of expanded categories
  scrollPosition: number;      // For restoring scroll on navigation
}
```

---

## 7. Form State (Profile Update)

### ProfileFormState

```typescript
interface ProfileFormState {
  // Form fields
  name: string;
  softwareLevel: 'beginner' | 'intermediate' | 'advanced';
  hardwareAccess: 'none' | 'basic' | 'full_lab';
  preferredLanguage: 'en' | 'ur' | 'both';
  
  // Validation
  errors: {
    name?: string;
  };
  
  // Submission state
  isSubmitting: boolean;
  submitSuccess: boolean;
  submitError: string | null;
  
  // Dirty tracking
  isDirty: boolean;
}
```

**Validation Rules**:
```typescript
const validateProfileForm = (state: ProfileFormState): boolean => {
  const errors: ProfileFormState['errors'] = {};
  
  if (state.name.trim().length < 2) {
    errors.name = 'Name must be at least 2 characters';
  }
  
  if (state.name.length > 100) {
    errors.name = 'Name must not exceed 100 characters';
  }
  
  return Object.keys(errors).length === 0;
};
```

---

## 8. Loading States

### Global Loading State

```typescript
interface LoadingState {
  page: boolean;               // Page-level loading
  components: Record<string, boolean>; // Component-specific loading
}

// Example usage
interface AppState {
  loading: LoadingState;
}

// Setting component loading
dispatch({
  type: 'SET_LOADING',
  payload: { component: 'chatbot', loading: true }
});
```

---

## 9. Error State

### Error Model

```typescript
interface AppError {
  id: string;                  // Unique error ID
  type: 'network' | 'validation' | 'auth' | 'server' | 'unknown';
  message: string;             // User-friendly message
  details?: string;            // Technical details (for logging)
  timestamp: string;           // ISO 8601
  dismissible: boolean;
  action?: {
    label: string;
    handler: () => void;
  };
}

interface ErrorState {
  errors: AppError[];
}
```

**Error Display Priority**:
1. Auth errors (immediate, blocking)
2. Network errors (prominent, dismissible)
3. Validation errors (inline, contextual)
4. Server errors (toast notification)

---

## 10. Responsive State

### Device Detection

```typescript
type DeviceType = 'mobile' | 'tablet' | 'desktop' | 'wide';

interface ResponsiveState {
  deviceType: DeviceType;
  breakpoint: number;          // Current width in pixels
  isMobile: boolean;           // < 768px
  isTablet: boolean;           // 768px - 1023px
  isDesktop: boolean;          // >= 1024px
  orientation: 'portrait' | 'landscape';
}
```

**Breakpoint Detection**:
```typescript
const getDeviceType = (width: number): DeviceType => {
  if (width < 768) return 'mobile';
  if (width < 1024) return 'tablet';
  if (width < 1440) return 'desktop';
  return 'wide';
};
```

---

## 11. Animation State

### Animation Configuration

```typescript
interface AnimationConfig {
  enabled: boolean;            // Respect prefers-reduced-motion
  duration: {
    fast: number;              // 150ms
    normal: number;            // 300ms
    slow: number;              // 500ms
  };
  easing: {
    default: string;           // ease-in-out
    spring: string;            // cubic-bezier
  };
}
```

---

## State Management Strategy

### Context Providers

```typescript
// Theme Context (provided by Docusaurus)
const ThemeContext = {
  colorMode: ThemeMode;
  setColorMode: (mode: ThemeMode) => void;
};

// Auth Context (from Feature 003)
const AuthContext = {
  user: UserProfile | null;
  isAuthenticated: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (data: Partial<UserProfile>) => Promise<void>;
};

// Preferences Context (custom)
const PreferencesContext = {
  preferences: UserPreferences;
  updatePreference: <K extends keyof UserPreferences>(
    key: K,
    value: UserPreferences[K]
  ) => void;
  resetPreferences: () => void;
};

// Chatbot Context (custom)
const ChatbotContext = {
  state: ChatbotState;
  openChatbot: () => void;
  closeChatbot: () => void;
  sendMessage: (message: string) => Promise<void>;
  clearHistory: () => void;
};
```

### Hook Patterns

```typescript
// Custom hooks for state management
function usePreferences(): [
  UserPreferences,
  (key: keyof UserPreferences, value: any) => void
];

function useChatbot(): {
  isOpen: boolean;
  open: () => void;
  close: () => void;
  sendMessage: (msg: string) => Promise<void>;
};

function useResponsive(): ResponsiveState;

function useTheme(): {
  mode: ThemeMode;
  toggle: () => void;
  set: (mode: ThemeMode) => void;
};
```

---

## Data Flow

```
User Action (e.g., toggle theme)
  ↓
Component Event Handler
  ↓
Context Action/Hook
  ↓
Update State (React State + localStorage)
  ↓
Re-render Components
  ↓
CSS Variables Updated (for theme)
  ↓
Visual Change Applied
```

---

## Persistence Strategy

| Data Type | Storage Location | Sync Strategy |
|-----------|------------------|---------------|
| Theme Mode | localStorage + Context | Immediate |
| Sidebar State | localStorage | Debounced (300ms) |
| User Profile | Backend API + Context | On login, Manual sync |
| Chatbot History | SessionStorage | Per session |
| Preferences | localStorage + Context | Immediate |

---

**Data Model Complete**: All entities, interfaces, and state management patterns defined. Ready for contracts generation.
