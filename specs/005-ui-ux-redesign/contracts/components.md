# Component Contracts: Core UI Components

**Feature**: 005-ui-ux-redesign  
**Date**: 2025-12-07  
**Purpose**: Define interfaces for reusable UI components

---

## 1. ProfileDropdown Component

### Interface

```typescript
interface ProfileDropdownProps {
  /** Current authenticated user, null if not logged in */
  user: UserProfile | null;
  
  /** Callback when user clicks logout */
  onLogout: () => Promise<void>;
  
  /** Callback when user clicks settings */
  onSettingsClick: () => void;
  
  /** Callback when user changes language preference */
  onLanguageChange: (lang: 'en' | 'ur' | 'both') => void;
  
  /** Optional className for styling */
  className?: string;
}

interface UserProfile {
  id: string;
  name: string;
  email: string;
  preferredLanguage: 'en' | 'ur' | 'both';
}
```

### Behavior Contract

**When not authenticated** (`user === null`):
- Display "Sign Up" and "Login" buttons
- No dropdown menu
- Buttons link to `/signup` and `/login` pages

**When authenticated** (`user !== null`):
- Display user avatar/initials in navbar
- Click opens dropdown menu with:
  - User name and email (read-only)
  - "Settings" link
  - "Language" toggle (EN/UR/Both)
  - "Logout" button
- Click outside dropdown closes it
- Escape key closes dropdown
- Tab navigation works through dropdown items

**Logout flow**:
1. User clicks "Logout"
2. Button shows loading spinner
3. `onLogout()` called (async)
4. On success: dropdown closes, user redirected
5. On error: show error message, keep dropdown open

### Accessibility Requirements

- Dropdown button has `aria-expanded` attribute
- Dropdown menu has `role="menu"`
- Menu items have `role="menuitem"`
- Focus trap when dropdown open
- Keyboard navigation: Tab, Enter, Escape

### CSS Module Structure

```css
/* ProfileDropdown.module.css */
.container { /* Dropdown container */ }
.trigger { /* Button that opens dropdown */ }
.menu { /* Dropdown menu */ }
.userInfo { /* User name/email section */ }
.menuItem { /* Individual menu items */ }
.logoutButton { /* Logout button with loading state */ }
```

### Usage Example

```jsx
import ProfileDropdown from '@site/src/components/ProfileDropdown';
import { useAuth } from '@site/src/contexts/AuthContext';

function Navbar() {
  const { user, logout } = useAuth();
  
  return (
    <ProfileDropdown
      user={user}
      onLogout={logout}
      onSettingsClick={() => navigate('/settings')}
      onLanguageChange={(lang) => updatePreference('language', lang)}
    />
  );
}
```

---

## 2. LoadingButton Component

### Interface

```typescript
interface LoadingButtonProps {
  /** Button content */
  children: React.ReactNode;
  
  /** Show loading spinner */
  loading: boolean;
  
  /** Disable button */
  disabled?: boolean;
  
  /** Visual style variant */
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost' | 'danger';
  
  /** Size variant */
  size?: 'sm' | 'md' | 'lg';
  
  /** Optional icon (shown before text) */
  icon?: React.ReactNode;
  
  /** Click handler (can be async) */
  onClick?: (event: React.MouseEvent<HTMLButtonElement>) => void | Promise<void>;
  
  /** Button type */
  type?: 'button' | 'submit' | 'reset';
  
  /** Accessibility label */
  'aria-label'?: string;
  
  /** Optional className */
  className?: string;
  
  /** Full width button */
  fullWidth?: boolean;
}
```

### Behavior Contract

**Loading state** (`loading === true`):
- Button is disabled
- Spinner replaces icon (if present)
- Text remains visible
- Cursor shows `not-allowed`
- `aria-busy="true"`

**Disabled state** (`disabled === true`):
- Button is not clickable
- Opacity reduced to 0.5
- Cursor shows `not-allowed`
- `aria-disabled="true"`

**Async onClick**:
- If `onClick` returns Promise, automatically set loading state
- Re-enable button when Promise resolves/rejects
- Parent can override with `loading` prop

**Variants**:
- `primary`: Filled button with primary color
- `secondary`: Filled button with secondary color
- `outline`: Border-only with transparent background
- `ghost`: No border, transparent background (hover: light bg)
- `danger`: Red color for destructive actions

**Sizes**:
- `sm`: padding 8px 16px, text 14px
- `md`: padding 12px 24px, text 16px (default)
- `lg`: padding 16px 32px, text 18px

### Accessibility Requirements

- `aria-busy` when loading
- `aria-disabled` when disabled
- `aria-label` for icon-only buttons
- Focus visible on keyboard navigation
- Min touch target: 44x44px (on mobile)

### CSS Module Structure

```css
/* LoadingButton.module.css */
.button { /* Base button styles */ }
.primary { /* Primary variant */ }
.secondary { /* Secondary variant */ }
.outline { /* Outline variant */ }
.ghost { /* Ghost variant */ }
.danger { /* Danger variant */ }
.sm { /* Small size */ }
.md { /* Medium size */ }
.lg { /* Large size */ }
.loading { /* Loading state styles */ }
.disabled { /* Disabled state styles */ }
.fullWidth { /* Full width modifier */ }
.spinner { /* Loading spinner */ }
.icon { /* Icon styles */ }
```

### Usage Example

```jsx
import LoadingButton from '@site/src/components/LoadingButton';

function LoginForm() {
  const [loading, setLoading] = useState(false);
  
  const handleSubmit = async () => {
    setLoading(true);
    try {
      await login(email, password);
    } finally {
      setLoading(false);
    }
  };
  
  return (
    <LoadingButton
      variant="primary"
      size="lg"
      loading={loading}
      onClick={handleSubmit}
      fullWidth
    >
      Log In
    </LoadingButton>
  );
}
```

---

## 3. SkeletonLoader Component

### Interface

```typescript
interface SkeletonLoaderProps {
  /** Skeleton shape variant */
  variant: 'text' | 'circular' | 'rectangular' | 'card';
  
  /** Width (CSS value or number in px) */
  width?: string | number;
  
  /** Height (CSS value or number in px) */
  height?: string | number;
  
  /** Animation type */
  animation?: 'pulse' | 'wave' | 'none';
  
  /** Number of skeleton elements to render */
  count?: number;
  
  /** Optional className */
  className?: string;
}
```

### Behavior Contract

**Variants**:
- `text`: Single line of text (default height: 1em)
- `circular`: Circle (equal width/height)
- `rectangular`: Rectangle with rounded corners
- `card`: Predefined card layout (image + text lines)

**Animations**:
- `pulse`: Opacity fade in/out (default)
- `wave`: Shimmer effect moving left to right
- `none`: Static (respects prefers-reduced-motion)

**Count** (`count > 1`):
- Renders multiple skeleton elements with gap between
- Useful for lists

**Accessibility**:
- `aria-busy="true"` on container
- `aria-label="Loading content"`

### CSS Module Structure

```css
/* SkeletonLoader.module.css */
.skeleton { /* Base skeleton styles */ }
.text { /* Text variant */ }
.circular { /* Circular variant */ }
.rectangular { /* Rectangular variant */ }
.card { /* Card variant (composite) */ }
.pulse { /* Pulse animation */ }
.wave { /* Wave animation */ }
.container { /* Container for multiple skeletons */ }
```

### Usage Example

```jsx
import SkeletonLoader from '@site/src/components/SkeletonLoader';

function ProfilePage() {
  const { user, loading } = useAuth();
  
  if (loading) {
    return (
      <div>
        <SkeletonLoader variant="circular" width={80} height={80} />
        <SkeletonLoader variant="text" width="60%" count={3} />
      </div>
    );
  }
  
  return <UserProfile user={user} />;
}
```

---

## 4. ChatbotFAB Component

### Interface

```typescript
interface ChatbotFABProps {
  /** Callback when FAB clicked */
  onOpen: () => void;
  
  /** Show red dot for unread messages */
  hasUnreadMessages?: boolean;
  
  /** Has user seen chatbot before? (controls pulse animation) */
  seen: boolean;
  
  /** Position on screen */
  position?: 'bottom-right' | 'bottom-left';
  
  /** Optional className */
  className?: string;
}
```

### Behavior Contract

**Default state**:
- Fixed position on screen (bottom-right by default)
- Chat icon visible
- Z-index: 1000 (above most content, below modals)

**First-time user** (`seen === false`):
- Subtle pulse animation to draw attention
- After first click, set `seen: true` in localStorage

**Unread messages** (`hasUnreadMessages === true`):
- Small red dot badge on top-right of FAB
- Badge disappears when chatbot opened

**Click behavior**:
- Calls `onOpen()` callback
- No built-in navigation (parent controls chatbot panel)

**Accessibility**:
- `aria-label="Open chatbot"`
- `role="button"`
- Keyboard accessible (Tab, Enter, Space)
- Min touch target: 56x56px (Material Design FAB standard)

### CSS Module Structure

```css
/* ChatbotFAB.module.css */
.fab { /* Floating action button */ }
.bottomRight { /* Position variant */ }
.bottomLeft { /* Position variant */ }
.pulse { /* Pulse animation (first-time users) */ }
.badge { /* Unread messages badge */ }
.icon { /* Chat icon */ }
```

### Usage Example

```jsx
import ChatbotFAB from '@site/src/components/ChatbotFAB';
import { usePreferences } from '@site/src/hooks/usePreferences';

function Layout() {
  const [chatbotOpen, setChatbotOpen] = useState(false);
  const [preferences, updatePreference] = usePreferences();
  
  const handleOpen = () => {
    setChatbotOpen(true);
    if (!preferences.chatbotSeen) {
      updatePreference('chatbotSeen', true);
    }
  };
  
  return (
    <>
      <ChatbotFAB
        onOpen={handleOpen}
        seen={preferences.chatbotSeen}
        hasUnreadMessages={false}
      />
      {chatbotOpen && <ChatbotPanel onClose={() => setChatbotOpen(false)} />}
    </>
  );
}
```

---

## 5. FeatureCard Component

### Interface

```typescript
interface FeatureCardProps {
  /** Icon element (React component or SVG) */
  icon: React.ReactNode;
  
  /** Card title */
  title: string;
  
  /** Card description */
  description: string;
  
  /** Optional link (makes card clickable) */
  href?: string;
  
  /** Optional badge */
  badge?: {
    text: string;
    color: 'primary' | 'secondary' | 'accent' | 'success' | 'warning';
  };
  
  /** Visual variant */
  variant?: 'default' | 'elevated' | 'outlined';
  
  /** Optional className */
  className?: string;
}
```

### Behavior Contract

**Without href**:
- Static card (not interactive)
- No hover effects beyond subtle shadow

**With href**:
- Card is clickable (entire card is link)
- Hover: lift effect (transform: translateY(-4px))
- Focus: visible focus ring
- Opens link on click/Enter

**Variants**:
- `default`: White background, no border
- `elevated`: White background, shadow elevation
- `outlined`: Border, no shadow

**Badge** (if present):
- Positioned top-right corner
- Colored background based on `badge.color`
- Small text (12px)

**Responsive**:
- Mobile: Full width, stack vertically
- Tablet: 2 columns
- Desktop: 3-4 columns (parent grid controls)

### CSS Module Structure

```css
/* FeatureCard.module.css */
.card { /* Base card styles */ }
.default { /* Default variant */ }
.elevated { /* Elevated variant */ }
.outlined { /* Outlined variant */ }
.clickable { /* When href present */ }
.icon { /* Icon container */ }
.badge { /* Badge styles */ }
.badgePrimary { /* Primary badge color */ }
.title { /* Title styles */ }
.description { /* Description styles */ }
```

### Usage Example

```jsx
import FeatureCard from '@site/src/components/FeatureCard';
import { RobotIcon } from '@site/src/components/icons';

function FeaturesSection() {
  return (
    <div className={styles.grid}>
      <FeatureCard
        icon={<RobotIcon />}
        title="ROS 2 Fundamentals"
        description="Master the robotic nervous system with hands-on ROS 2 programming"
        href="/docs/part-02-ros2-ecosystem"
        variant="elevated"
        badge={{ text: "3 weeks", color: "primary" }}
      />
      {/* More cards... */}
    </div>
  );
}
```

---

## 6. Toast Notification Component

### Interface

```typescript
interface ToastProps {
  /** Toast ID (for dismissal tracking) */
  id: string;
  
  /** Notification message */
  message: string;
  
  /** Notification type (affects color and icon) */
  type: 'success' | 'error' | 'warning' | 'info';
  
  /** Auto-dismiss duration in ms (0 = manual dismiss only) */
  duration?: number;
  
  /** Show close button */
  dismissible?: boolean;
  
  /** Callback when dismissed */
  onDismiss?: (id: string) => void;
  
  /** Optional action button */
  action?: {
    label: string;
    onClick: () => void;
  };
}

interface ToastContainerProps {
  /** Position on screen */
  position?: 'top-right' | 'top-center' | 'bottom-right' | 'bottom-center';
  
  /** Maximum number of toasts to show */
  maxToasts?: number;
}
```

### Behavior Contract

**Auto-dismiss** (`duration > 0`):
- Toast disappears after specified milliseconds
- Progress bar shows remaining time
- Hover pauses auto-dismiss timer

**Manual dismiss**:
- X button visible if `dismissible === true`
- Click X or Escape key dismisses

**Action button** (if present):
- Button shown next to message
- Click calls `action.onClick`
- Toast remains open after action (unless action dismisses)

**Types** (with icons):
- `success`: Green, ✓ icon
- `error`: Red, ✕ icon
- `warning`: Yellow, ⚠ icon
- `info`: Blue, ℹ icon

**Stacking**:
- New toasts appear at top of stack
- Oldest toasts dismissed first if `maxToasts` exceeded
- Smooth enter/exit animations

### CSS Module Structure

```css
/* Toast.module.css */
.container { /* Toast container (fixed position) */ }
.toast { /* Individual toast */ }
.success { /* Success type */ }
.error { /* Error type */ }
.warning { /* Warning type */ }
.info { /* Info type */ }
.icon { /* Type icon */ }
.message { /* Message text */ }
.closeButton { /* Dismiss button */ }
.actionButton { /* Action button */ }
.progressBar { /* Auto-dismiss progress bar */ }
```

### Usage Example

```jsx
import { useToast } from '@site/src/hooks/useToast';

function Form() {
  const { showToast } = useToast();
  
  const handleSubmit = async () => {
    try {
      await saveData();
      showToast({
        type: 'success',
        message: 'Profile updated successfully!',
        duration: 3000,
      });
    } catch (error) {
      showToast({
        type: 'error',
        message: 'Failed to save changes',
        dismissible: true,
        action: {
          label: 'Retry',
          onClick: handleSubmit,
        },
      });
    }
  };
}
```

---

## Component Dependencies

```
ProfileDropdown
  └─ LoadingButton (for logout button)

ChatbotFAB
  └─ (no dependencies)

LoadingButton
  └─ Spinner component (internal)

SkeletonLoader
  └─ (no dependencies)

FeatureCard
  └─ (no dependencies)

Toast
  └─ (no dependencies)
```

---

## Global Component Props

All components support these common props:

```typescript
interface CommonProps {
  /** Custom className for styling overrides */
  className?: string;
  
  /** Custom inline styles (use sparingly) */
  style?: React.CSSProperties;
  
  /** Data attributes for testing */
  'data-testid'?: string;
}
```

---

**Contracts Complete**: All component interfaces defined with behavior contracts, accessibility requirements, and usage examples.
