// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive guide to building intelligent physical agents',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://zohaibcodez.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',

  // GitHub pages deployment config
  organizationName: 'ZohaibCodez',
  projectName: 'physical-ai-humanoid-robotics-textbook',

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang.
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Markdown configuration
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // SEO head tags
  headTags: [
    {
      tagName: 'meta',
      attributes: {
        name: 'keywords',
        content: 'robotics, humanoid, ROS 2, physical AI, machine learning, computer vision, SLAM, navigation, manipulation, locomotion, NVIDIA Isaac Sim, Gazebo',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        name: 'description',
        content: 'Learn to build autonomous humanoid robots with ROS 2, computer vision, motion planning, and AI. Comprehensive 13-week curriculum covering perception, navigation, manipulation, and bipedal locomotion.',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        name: 'author',
        content: 'ZohaibCodez',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:type',
        content: 'website',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:title',
        content: 'Physical AI & Humanoid Robotics Textbook',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:description',
        content: 'Comprehensive open-source textbook for learning humanoid robotics, ROS 2, and physical AI systems',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:image',
        content: 'https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/img/social-preview.png',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'twitter:card',
        content: 'summary_large_image',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'twitter:title',
        content: 'Physical AI & Humanoid Robotics Textbook',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'twitter:description',
        content: 'Learn to build autonomous humanoid robots with ROS 2 and AI',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'twitter:image',
        content: 'https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/img/social-preview.png',
      },
    },
  ],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          sidebarCollapsible: true,
          sidebarCollapsed: false,
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
      image: 'img/social-preview.png',
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
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
            href: 'https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook',
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
                label: 'GitHub',
                href: 'https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook Project. Built with Docusaurus.`,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'cpp', 'xml-doc', 'yaml'],
      },
    }),
};

export default config;
