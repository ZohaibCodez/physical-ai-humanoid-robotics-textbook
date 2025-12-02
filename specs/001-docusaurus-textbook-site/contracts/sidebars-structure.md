# Contract: sidebars.js Navigation Structure Schema

**Purpose**: Defines the sidebar navigation hierarchy for the Docusaurus textbook.  
**Location**: `sidebars.js` (root directory)  
**Feature**: 001-docusaurus-textbook-site  
**Last Updated**: 2025-12-03

---

## Required Structure

```javascript
// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    
    {
      type: 'category',
      label: 'Weeks 1-2: Foundations of Physical AI',
      collapsed: false,
      items: [
        'week-01-02/01-introduction-to-physical-ai',
        'week-01-02/02-ai-fundamentals-review',
        'week-01-02/03-robotics-hardware-overview',
        'week-01-02/04-sensors-and-actuators',
        'week-01-02/05-coordinate-systems-and-kinematics',
      ],
    },
    
    {
      type: 'category',
      label: 'Weeks 3-5: ROS 2 and Robot Programming',
      collapsed: false,
      items: [
        'week-03-05/01-ros2-architecture',
        'week-03-05/02-nodes-topics-services',
        'week-03-05/03-robot-state-publisher',
        'week-03-05/04-tf2-transformations',
        'week-03-05/05-parameter-server',
        'week-03-05/06-actionlib-and-goals',
      ],
    },
    
    {
      type: 'category',
      label: 'Weeks 6-7: Perception Systems',
      collapsed: false,
      items: [
        'week-06-07/01-computer-vision-for-robotics',
        'week-06-07/02-depth-cameras-and-lidar',
        'week-06-07/03-object-detection',
        'week-06-07/04-point-cloud-processing',
        'week-06-07/05-sensor-fusion',
      ],
    },
    
    {
      type: 'category',
      label: 'Weeks 8-10: Motion Planning and Control',
      collapsed: false,
      items: [
        'week-08-10/01-motion-planning-fundamentals',
        'week-08-10/02-path-planning-algorithms',
        'week-08-10/03-trajectory-generation',
        'week-08-10/04-pid-control',
        'week-08-10/05-moveit2-integration',
        'week-08-10/06-navigation-stack',
      ],
    },
    
    {
      type: 'category',
      label: 'Weeks 11-12: Humanoid Robotics',
      collapsed: false,
      items: [
        'week-11-12/01-humanoid-kinematics',
        'week-11-12/02-bipedal-locomotion',
        'week-11-12/03-balance-and-stability',
        'week-11-12/04-manipulation-and-grasping',
        'week-11-12/05-human-robot-interaction',
      ],
    },
    
    {
      type: 'category',
      label: 'Week 13: Simulation and Integration',
      collapsed: false,
      items: [
        'week-13/01-gazebo-simulation',
        'week-13/02-nvidia-isaac-sim',
        'week-13/03-digital-twins',
        'week-13/04-sim-to-real-transfer',
        'week-13/05-final-project-guidelines',
      ],
    },
    
    {
      type: 'category',
      label: 'Appendix',
      collapsed: true,
      items: [
        'appendix/setup-instructions',
        'appendix/troubleshooting',
        'appendix/ros2-cheatsheet',
        'appendix/hardware-recommendations',
        'appendix/further-reading',
      ],
    },
  ],
};

export default sidebars;
```

---

## Schema Specification

### Top-Level Object

```typescript
type SidebarsConfig = {
  [sidebarId: string]: SidebarItem[];
};
```

**Required Sidebar ID**: `tutorialSidebar` (referenced in `docusaurus.config.js`)

### SidebarItem Types

#### 1. Document Reference (String)

```javascript
'intro'  // Points to docs/intro.md
'week-01-02/01-introduction-to-physical-ai'  // Points to docs/week-01-02/01-introduction-to-physical-ai.md
```

**Rules**:
- Path relative to `docs/` directory
- Omit `.md` extension
- Use forward slashes `/` for subdirectories

#### 2. Category Object

```typescript
{
  type: 'category',
  label: string,        // Display name in sidebar
  collapsed: boolean,   // Initial state (true = collapsed, false = expanded)
  items: SidebarItem[], // Nested items (documents or subcategories)
}
```

**Example**:
```javascript
{
  type: 'category',
  label: 'Weeks 1-2: Foundations of Physical AI',
  collapsed: false,
  items: [
    'week-01-02/01-introduction-to-physical-ai',
    'week-01-02/02-ai-fundamentals-review',
  ],
}
```

---

## Curriculum Mapping

### Module Structure

| Weeks | Category Label | Directory | Chapter Count | Collapsed |
|-------|----------------|-----------|---------------|-----------|
| 1-2 | Foundations of Physical AI | `week-01-02/` | 5 | ❌ false |
| 3-5 | ROS 2 and Robot Programming | `week-03-05/` | 6 | ❌ false |
| 6-7 | Perception Systems | `week-06-07/` | 5 | ❌ false |
| 8-10 | Motion Planning and Control | `week-08-10/` | 6 | ❌ false |
| 11-12 | Humanoid Robotics | `week-11-12/` | 5 | ❌ false |
| 13 | Simulation and Integration | `week-13/` | 5 | ❌ false |
| - | Appendix | `appendix/` | 5 | ✅ true |

**Collapsed Strategy**:
- Core curriculum modules: `collapsed: false` (immediate visibility)
- Appendix: `collapsed: true` (reference material, accessed as needed)

### Chapter Naming Convention

Format: `<directory>/<number>-<slug>`

**Examples**:
- ✅ `week-01-02/01-introduction-to-physical-ai`
- ✅ `week-03-05/06-actionlib-and-goals`
- ✅ `appendix/ros2-cheatsheet`

**Rules**:
1. **Prefix number**: `01`, `02`, ..., `10` (zero-padded for sorting)
2. **Slug format**: Lowercase, hyphens, descriptive
3. **No file extension**: Docusaurus adds `.md` automatically

---

## Validation Rules

### Build-Time Checks

Docusaurus validates:

1. **File Existence**: Every sidebar item must point to an existing `.md` file in `docs/`
   ```bash
   # ❌ Error if missing:
   Error: Docs markdown file not found for "week-01-02/01-introduction-to-physical-ai"
   ```

2. **No Duplicate IDs**: Each document can only appear once in the sidebar
   ```javascript
   // ❌ Invalid
   items: [
     'intro',
     'week-01-02/01-introduction',
     'intro',  // Duplicate!
   ]
   ```

3. **Valid Category Structure**: Categories must have `type`, `label`, and `items`
   ```javascript
   // ❌ Missing required field
   {
     type: 'category',
     // ❌ No label!
     items: ['week-01-02/01-introduction'],
   }
   ```

### Accessibility Requirements

- **Clear Labels**: Category labels should be descriptive and unique
- **Logical Hierarchy**: No more than 2 levels of nesting (category → document)
- **Sequential Order**: Chapters numbered in learning order

---

## Progressive Delivery Strategy

### Phase 1: Core Weeks 1-5

```javascript
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Weeks 1-2: Foundations of Physical AI',
      items: ['week-01-02/01-introduction-to-physical-ai'],  // Only Chapter 1
    },
    // Week 3-5 coming soon...
  ],
};
```

### Phase 2: Add "Coming Soon" Placeholders

```javascript
{
  type: 'category',
  label: 'Weeks 8-10: Motion Planning and Control (Coming Soon: Jan 15)',
  collapsed: true,
  items: [
    'week-08-10/placeholder',  // Placeholder page
  ],
}
```

**Placeholder File** (`docs/week-08-10/placeholder.md`):
```markdown
---
sidebar_position: 1
title: Coming Soon
---

# Motion Planning and Control

This module is currently under development and will be available on **January 15, 2025**.

## What to Expect
- Motion planning fundamentals
- Path planning algorithms
- MoveIt2 integration
- Navigation stack

Check back soon or [watch this repository](https://github.com/[username]/[repo]) for updates!
```

### Phase 3: Complete Curriculum

All categories populated with final chapters (as shown in full structure above).

---

## Dynamic Sidebar Features

### Autogenerated Sidebars (Future Enhancement)

Instead of manually listing every file, use directory-based generation:

```javascript
{
  type: 'autogenerated',
  dirName: 'week-01-02',  // Auto-include all .md files in this directory
}
```

**Benefit**: Reduces maintenance when adding new chapters

**Trade-off**: Less control over order (relies on filesystem sorting)

### Sidebar Item Metadata

Add descriptions or icons:

```javascript
{
  type: 'doc',
  id: 'week-01-02/01-introduction-to-physical-ai',
  label: 'Introduction to Physical AI',
  customProps: {
    description: 'Learn the fundamentals of embodied intelligence',
    icon: '🤖',
  },
}
```

**Usage**: Custom components can read `customProps` for enhanced UI

---

## File System Constraints

### Directory Structure Enforcement

Sidebar paths **must** match file system structure:

```
docs/
├── intro.md              → 'intro'
├── week-01-02/
│   ├── 01-introduction-to-physical-ai.md  → 'week-01-02/01-introduction-to-physical-ai'
│   └── 02-ai-fundamentals-review.md       → 'week-01-02/02-ai-fundamentals-review'
├── week-03-05/
│   └── ...
└── appendix/
    └── ...
```

**Mismatches**:
```javascript
// ❌ Error: Path mismatch
items: [
  'week-1-2/intro',  // File is actually at week-01-02/01-introduction...
]
```

---

## Sidebar Behavior

### Navigation Flow

1. **Homepage**: User lands on `intro.md`
2. **Sidebar**: Shows all categories (collapsed/expanded per config)
3. **Click Category**: Expands to show chapters
4. **Click Chapter**: Navigates to chapter page; sidebar highlights active item
5. **Next/Previous**: Buttons at bottom of each page based on sidebar order

### Mobile Responsiveness

- **Mobile**: Sidebar hidden by default; toggle with hamburger menu
- **Tablet**: Sidebar visible, narrower width
- **Desktop**: Full sidebar (250px width)

Docusaurus handles this automatically—no custom CSS needed.

---

## Testing Checklist

### Pre-Commit

- [ ] All sidebar paths point to existing `.md` files in `docs/`
- [ ] No duplicate document IDs
- [ ] Category labels are descriptive and unique
- [ ] `collapsed` state set intentionally for each category
- [ ] Chapter order matches curriculum sequence

### Build Test

```bash
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build".
```

**If Errors**:
- Fix broken sidebar references
- Verify file paths are correct
- Check for syntax errors in `sidebars.js`

### Manual Testing

1. Run `npm start`
2. Navigate through sidebar categories
3. Verify:
   - ✅ All links work
   - ✅ Active page is highlighted
   - ✅ Collapse/expand animations smooth
   - ✅ Next/Previous buttons appear correctly

---

## References

- **Docusaurus Sidebar API**: https://docusaurus.io/docs/sidebar
- **Sidebar Items Reference**: https://docusaurus.io/docs/sidebar/items
- **Autogenerated Sidebars**: https://docusaurus.io/docs/sidebar/autogenerated

---

## Change Log

| Date | Version | Change | Author |
|------|---------|--------|--------|
| 2025-12-03 | 1.0.0 | Initial sidebar contract with 6 modules + appendix | Spec Phase |

---

**Status**: ✅ Validated - Structure aligns with 13-week curriculum
