# Contract: docusaurus.config.js Configuration Schema

**Purpose**: Defines the structure and required fields for the Docusaurus site configuration.  
**Location**: `docusaurus.config.js` (root directory)  
**Feature**: 001-docusaurus-textbook-site  
**Last Updated**: 2025-12-03

---

## Required Configuration Structure

```javascript
// @ts-check
const {themes} = require('prism-react-renderer');
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive guide to building intelligent physical agents',
  favicon: 'img/favicon.ico',
  
  // Site deployment configuration
  url: 'https://[username].github.io',
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',
  organizationName: '[username]',
  projectName: 'physical-ai-humanoid-robotics-textbook',
  
  // Build behavior
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,
  
  // Internationalization (future-ready)
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          editUrl: 'https://github.com/[username]/physical-ai-humanoid-robotics-textbook/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Textbook Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/[username]/physical-ai-humanoid-robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Resources',
            items: [
              {
                label: 'Getting Started',
                to: '/',
              },
              {
                label: 'ROS 2 Humble Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub Discussions',
                href: 'https://github.com/[username]/physical-ai-humanoid-robotics-textbook/discussions',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook Project. Built with Docusaurus.`,
      },
      prism: {
        theme: lightTheme,
        darkTheme: darkTheme,
        additionalLanguages: ['python', 'xml', 'yaml', 'bash', 'cpp'],
      },
    }),
};

export default config;
```

---

## Field Specifications

### Core Metadata

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `title` | string | ✅ | Site title (appears in browser tab) | "Physical AI & Humanoid Robotics Textbook" |
| `tagline` | string | ✅ | Short description | "A comprehensive guide to building intelligent physical agents" |
| `favicon` | string | ✅ | Path to favicon (from `static/`) | "img/favicon.ico" |

### Deployment Settings

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `url` | string | ✅ | Production URL (no trailing slash) | Must start with `https://` |
| `baseUrl` | string | ✅ | Repository path (with trailing slash) | Must start and end with `/` |
| `organizationName` | string | ✅ | GitHub username or org | Alphanumeric, hyphens allowed |
| `projectName` | string | ✅ | GitHub repository name | Must match repo exactly |

**Example**:
```javascript
url: 'https://john-doe.github.io',
baseUrl: '/physical-ai-humanoid-robotics-textbook/',
organizationName: 'john-doe',
projectName: 'physical-ai-humanoid-robotics-textbook',
```

### Build Configuration

| Field | Type | Required | Description | Default |
|-------|------|----------|-------------|---------|
| `onBrokenLinks` | 'throw' \| 'warn' | ✅ | Behavior for broken links | 'throw' (fail build) |
| `onBrokenMarkdownLinks` | 'throw' \| 'warn' | ✅ | Behavior for broken MD links | 'warn' |
| `trailingSlash` | boolean | ✅ | URL trailing slash behavior | `false` |

**Rationale**: `throw` on broken links ensures high-quality content; `warn` on Markdown links allows flexibility during development.

### Preset: Classic

```javascript
presets: [
  [
    'classic',
    {
      docs: {
        sidebarPath: './sidebars.js',  // Navigation structure
        routeBasePath: '/',             // Docs at root (no /docs prefix)
        editUrl: 'https://github.com/[username]/[repo]/tree/main/',
      },
      blog: false,                      // No blog functionality
      theme: {
        customCss: './src/css/custom.css',
      },
    },
  ],
]
```

**Key Decisions**:
- `routeBasePath: '/'` → Landing page is the intro (no separate homepage)
- `blog: false` → Pure textbook, no blog posts
- `editUrl` → Enables "Edit this page" button on each chapter

### Theme Configuration

#### Navbar

```javascript
navbar: {
  title: 'Physical AI Textbook',
  logo: {
    alt: 'Textbook Logo',
    src: 'img/logo.svg',  // From static/img/
  },
  items: [
    {
      type: 'docSidebar',
      sidebarId: 'tutorialSidebar',  // References sidebars.js
      position: 'left',
      label: 'Textbook',
    },
    {
      href: 'https://github.com/[username]/[repo]',
      label: 'GitHub',
      position: 'right',
    },
  ],
}
```

**Navbar Items**:
- **Textbook** (left): Dropdown showing sidebar navigation
- **GitHub** (right): Link to repository

#### Footer

```javascript
footer: {
  style: 'dark',
  links: [
    {
      title: 'Learning Resources',
      items: [
        { label: 'Getting Started', to: '/' },
        { label: 'ROS 2 Humble Docs', href: 'https://docs.ros.org/en/humble/' },
      ],
    },
    {
      title: 'Community',
      items: [
        { label: 'GitHub Discussions', href: 'https://github.com/[username]/[repo]/discussions' },
      ],
    },
  ],
  copyright: `Copyright © ${new Date().getFullYear()} ...`,
}
```

**Footer Structure**:
- **2 columns**: "Learning Resources" and "Community"
- Dynamic copyright year with `new Date().getFullYear()`

#### Prism Syntax Highlighting

```javascript
prism: {
  theme: lightTheme,        // GitHub theme for light mode
  darkTheme: darkTheme,     // Dracula theme for dark mode
  additionalLanguages: ['python', 'xml', 'yaml', 'bash', 'cpp'],
}
```

**Supported Languages**:
- `python`: ROS 2 nodes, scripts
- `xml`: URDF, launch files
- `yaml`: Configuration files
- `bash`: Terminal commands
- `cpp`: Advanced ROS 2 nodes

**Default Included**: JavaScript, JSX, Markdown

---

## Validation Rules

### Pre-Commit Checks

1. **URL Format**:
   ```javascript
   // ✅ Valid
   url: 'https://username.github.io'
   
   // ❌ Invalid
   url: 'http://username.github.io'  // Must be HTTPS
   url: 'https://username.github.io/' // No trailing slash
   ```

2. **baseUrl Consistency**:
   ```javascript
   // ✅ Valid
   baseUrl: '/repo-name/'
   
   // ❌ Invalid
   baseUrl: 'repo-name'   // Missing slashes
   baseUrl: '/repo-name'  // Missing trailing slash
   ```

3. **sidebarPath File Exists**:
   - Must point to existing `sidebars.js` file
   - Build fails if file not found

4. **Logo/Favicon Files Exist**:
   - `static/img/logo.svg` must exist
   - `static/img/favicon.ico` must exist

### Build-Time Validation

Run before deploying:

```bash
npm run build
```

**Expected Behavior**:
- ✅ Build succeeds → All links valid, config correct
- ❌ Build fails → Fix errors (Docusaurus reports exact line/file)

---

## Extension Points (Future)

### Plugins

Add custom plugins in `plugins` array:

```javascript
plugins: [
  [
    '@docusaurus/plugin-content-docs',
    {
      id: 'community',
      path: 'community',
      routeBasePath: 'community',
      sidebarPath: './sidebarsCommunity.js',
    },
  ],
],
```

**Use Case**: Separate "Community Guides" section

### Custom Components

Register custom React components:

```javascript
// In src/components/
export { default as CustomButton } from './CustomButton';
```

Use in MDX:

```mdx
import CustomButton from '@site/src/components/CustomButton';

<CustomButton label="Try Simulation" />
```

### Search Integration

Add Algolia DocSearch (recommended for production):

```javascript
themeConfig: {
  algolia: {
    appId: 'YOUR_APP_ID',
    apiKey: 'YOUR_SEARCH_API_KEY',
    indexName: 'physical-ai-textbook',
  },
}
```

---

## Environment-Specific Overrides

Use environment variables for deployment targets:

```javascript
const isProd = process.env.NODE_ENV === 'production';

const config = {
  url: isProd ? 'https://[username].github.io' : 'http://localhost',
  baseUrl: isProd ? '/physical-ai-humanoid-robotics-textbook/' : '/',
  // ...
};
```

**Benefit**: Seamless local development without changing config

---

## Dependencies

**Required npm Packages**:
```json
{
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "prism-react-renderer": "^2.1.0",
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  }
}
```

**Update Strategy**:
- Major version upgrades: Test locally, check migration guides
- Minor/patch: Safe to update anytime

---

## References

- **Docusaurus Configuration API**: https://docusaurus.io/docs/api/docusaurus-config
- **Preset Classic Options**: https://docusaurus.io/docs/api/plugins/@docusaurus/preset-classic
- **Theme Configuration**: https://docusaurus.io/docs/api/themes/configuration
- **Prism Languages**: https://prismjs.com/#supported-languages

---

## Change Log

| Date | Version | Change | Author |
|------|---------|--------|--------|
| 2025-12-03 | 1.0.0 | Initial contract definition | Spec Phase |

---

**Status**: ✅ Validated - All fields verified against Docusaurus 3.x API
